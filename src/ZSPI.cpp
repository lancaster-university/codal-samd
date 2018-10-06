/*
The MIT License (MIT)

Copyright (c) 2017 Lancaster University.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/

#include "CodalConfig.h"
#include "ZSPI.h"
#include "ErrorNo.h"
#include "CodalDmesg.h"
#include "codal-core/inc/driver-models/Timer.h"
#include "MessageBus.h"
#include "Event.h"
#include "CodalFiber.h"

extern "C" {
#include "sercom.h"
}

#include "pinmap.h"

#include "parts.h"

//#define LOG DMESG
#define LOG(...) ((void)0)

namespace codal
{

#define ZERO(f) memset(&f, 0, sizeof(f))

void ZSPI::dmaTransferComplete(DmaCode)
{
    LOG("SPI complete D=%p", doneHandler);

    // Wait for the last byte of the SPI transfer to complete.
    while (sercom->SPI.INTFLAG.bit.TXC == 0)
        ;

    // In case we ignored input, clear the garbage
    while (sercom->SPI.INTFLAG.bit.RXC == 1)
    {
        sercom->SPI.DATA.reg;
    }
    sercom->SPI.STATUS.bit.BUFOVF = 1;
    sercom->SPI.INTFLAG.reg = SERCOM_SPI_INTFLAG_ERROR;

    LOG("SPI TXC done");

    if (doneHandler)
    {
        PVoidCallback done = doneHandler;
        doneHandler = NULL;
        done(doneHandlerArg);
    }
    else
    {
        Event(DEVICE_ID_NOTIFY_ONE, transferCompleteEventCode);
    }
}

void ZSPI::init()
{
    if (!needsInit)
        return;

    needsInit = false;

    if (!sercom)
    {
        auto sclk_mcu = find_mcu_pin(sclk->name);
        auto miso_mcu = find_mcu_pin(miso ? miso->name : PIN_NONE);
        auto mosi_mcu = find_mcu_pin(mosi ? mosi->name : PIN_NONE);

        int dopo = 10;
        int sercomIdx;

        for (sercomIdx = 0; sercomIdx < SERCOM_INST_NUM; sercomIdx++)
        {
            if (used_sercoms[sercomIdx])
                continue;

            int sclk_si = find_sercom(sclk_mcu, sercomIdx);
            if (sclk_si == -1)
                continue;
            int miso_si = find_sercom(miso_mcu, sercomIdx);
            if (miso_si == -1)
                continue;
            int mosi_si = find_sercom(mosi_mcu, sercomIdx);
            if (mosi_si == -1)
                continue;

            int sclk_pad = sclk_mcu->sercom[sclk_si].pad;
            int mosi_pad = mosi_mcu ? mosi_mcu->sercom[mosi_si].pad : 0;
            int miso_pad = miso_mcu ? miso_mcu->sercom[miso_si].pad : 2;

            if (!mosi_mcu && miso_mcu && miso_pad == mosi_pad)
            {
                mosi_pad = sclk_pad == 3 ? 2 : 3;
            }

            if (!samd_peripherals_valid_spi_clock_pad(sclk_pad))
                continue;

            dopo = samd_peripherals_get_spi_dopo(sclk_pad, mosi_pad);
            if (dopo > 3)
                continue;

            sercom = sercom_insts[sercomIdx];
            used_sercoms[sercomIdx] = 1;

            // Set up SPI clocks on SERCOM.
            samd_peripherals_sercom_clock_init(sercom, sercomIdx);

            if (spi_m_sync_init(&spi_desc, sercom) != ERR_NONE)
            {
                target_panic(901);
            }

            // Pads must be set after spi_m_sync_init(), which uses default values from
            // the prototypical SERCOM.
            hri_sercomspi_write_CTRLA_DOPO_bf(sercom, dopo);
            hri_sercomspi_write_CTRLA_DIPO_bf(sercom, miso_pad);

            // Always start at 250khz which is what SD cards need. They are sensitive to
            // SPI bus noise before they are put into SPI mode.
            uint8_t baud_value = samd_peripherals_spi_baudrate_to_baud_reg_value(250000);
            if (spi_m_sync_set_baudrate(&spi_desc, baud_value) != ERR_NONE)
            {
                // spi_m_sync_set_baudrate does not check for validity, just whether the device is
                // busy or not
                target_panic(902);
            }

#define MUX(si) ((si == 0) ? MUX_C : MUX_D)
            sclk->_setMux(MUX(sclk_si));

            if (mosi)
                mosi->_setMux(MUX(mosi_si));

            if (miso)
                miso->_setMux(MUX(miso_si), true);

            break;
        }

        if (!sercom)
            target_panic(903);

        DmaFactory factory;

        if (miso)
        {
            dmaRxCh = factory.allocate();
            dmaRxCh->configure(sercom_trigger_src(sercomIdx, false), BeatByte, &sercom->SPI.DATA.reg, NULL);
        }
        else
            dmaRxCh = NULL;

        if (mosi)
        {
            dmaTxCh = factory.allocate();
            dmaTxCh->configure(sercom_trigger_src(sercomIdx, true), BeatByte, NULL, &sercom->SPI.DATA.reg);
            dmaTxCh->onTransferComplete(this);
        }
        else
        {
            CODAL_ASSERT(0);
            dmaTxCh = NULL;
        }
    }

    LOG("SPI instance %p", sercom);

    uint8_t baud_reg_value = samd_peripherals_spi_baudrate_to_baud_reg_value(freq);

    void *hw = spi_desc.dev.prvt;
    // Disable, set values (most or all are enable-protected), and re-enable.
    spi_m_sync_disable(&spi_desc);
    hri_sercomspi_wait_for_sync(hw, SERCOM_SPI_SYNCBUSY_MASK);

    hri_sercomspi_write_CTRLA_CPHA_bit(hw, _mode & 1);
    hri_sercomspi_write_CTRLA_CPOL_bit(hw, (_mode >> 1) & 1);
    hri_sercomspi_write_CTRLB_CHSIZE_bf(hw, _bits - 8);
    hri_sercomspi_write_BAUD_BAUD_bf(hw, baud_reg_value);
    hri_sercomspi_wait_for_sync(hw, SERCOM_SPI_SYNCBUSY_MASK);

    spi_m_sync_enable(&spi_desc);
    hri_sercomspi_wait_for_sync(hw, SERCOM_SPI_SYNCBUSY_MASK);
}

ZSPI::ZSPI(Pin &mosi, Pin &miso, Pin &sclk) : codal::SPI()
{
    this->mosi = (ZPin*)&mosi;
    this->miso = (ZPin*)&miso;
    this->sclk = (ZPin*)&sclk;


    this->transferCompleteEventCode = codal::allocateNotifyEvent();

    _mode = 0;
    _bits = 8;
    freq = 250000;
    sercom = NULL;

    needsInit = true;

    ZERO(spi_desc);
}

int ZSPI::setFrequency(uint32_t frequency)
{
    freq = frequency;
    needsInit = true;
    return DEVICE_OK;
}

int ZSPI::setMode(int mode, int bits)
{
    _mode = mode;
    _bits = bits;
    needsInit = true;

    CODAL_ASSERT(bits == 8);

    return DEVICE_OK;
}

int ZSPI::write(int data)
{
    rxCh = 0;
    txCh = data;
    if (transfer(&txCh, 1, &rxCh, 1) < 0)
        return DEVICE_SPI_ERROR;
    return rxCh;
}

int ZSPI::transfer(const uint8_t *txBuffer, uint32_t txSize, uint8_t *rxBuffer, uint32_t rxSize)
{
    fiber_wake_on_event(DEVICE_ID_NOTIFY, transferCompleteEventCode);
    auto res = startTransfer(txBuffer, txSize, rxBuffer, rxSize, NULL, NULL);
    LOG("SPI ->");
    schedule();
    LOG("SPI <-");
    return res;
}

int ZSPI::startTransfer(const uint8_t *txBuffer, uint32_t txSize, uint8_t *rxBuffer,
                        uint32_t rxSize, PVoidCallback doneHandler, void *arg)
{
    init();

    LOG("SPI start %p/%d %p/%d D=%p", txBuffer, txSize, rxBuffer, rxSize, doneHandler);

    if (txSize == 0 && rxSize == 0)
        return 0; // nothing to do

    this->doneHandler = doneHandler;
    this->doneHandlerArg = arg;

    sercom->SPI.INTFLAG.reg = SERCOM_SPI_INTFLAG_RXC | SERCOM_SPI_INTFLAG_DRE;

    CODAL_ASSERT(txSize > 0);
    CODAL_ASSERT(rxSize == 0 || txSize == rxSize);

    if (rxSize)
        dmaRxCh->transfer(NULL, rxBuffer, rxSize);

    dmaTxCh->transfer(txBuffer, NULL, txSize);

    return 0;
}

} // namespace codal
