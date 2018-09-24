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

#include "Timer.h"
#include "Event.h"
#include "CodalCompat.h"
#include "SAMDDMAC.h"

#undef ENABLE

static DmaComponent *apps[DMA_DESCRIPTOR_COUNT];

#ifdef SAMD21
extern "C" void DMAC_Handler(void)
{
    uint32_t oldChannel = DMAC->CHID.bit.ID;

    int channel = DMAC->INTPEND.bit.ID;
    DMAC->CHID.bit.ID = channel;
    DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TCMPL;

    if (apps[channel] != NULL)
        apps[channel]->dmaTransferComplete();

    DMAC->CHID.bit.ID = oldChannel;
}
#else
static void dmac_irq_handler()
{
    uint8_t channel = hri_dmac_get_INTPEND_reg(DMAC, DMAC_INTPEND_ID_Msk);

    if (hri_dmac_get_CHINTFLAG_TERR_bit(DMAC, channel))
    {
        hri_dmac_clear_CHINTFLAG_TERR_bit(DMAC, channel);
        // tmp_resource->dma_cb.error(tmp_resource);
    }
    else if (hri_dmac_get_CHINTFLAG_TCMPL_bit(DMAC, channel))
    {
        hri_dmac_clear_CHINTFLAG_TCMPL_bit(DMAC, channel);
        if (apps[channel] != NULL)
            apps[channel]->dmaTransferComplete();
    }
}

extern "C" void DMAC_0_Handler(void)
{
    dmac_irq_handler();
}

extern "C" void DMAC_1_Handler(void)
{
    dmac_irq_handler();
}

extern "C" void DMAC_2_Handler(void)
{
    dmac_irq_handler();
}

extern "C" void DMAC_3_Handler(void)
{
    dmac_irq_handler();
}

extern "C" void DMAC_4_Handler(void)
{
    dmac_irq_handler();
}
#endif

/**
 * Base implementation of a DMA callback
 */
void DmaComponent::dmaTransferComplete() {}

SAMDDMAC::SAMDDMAC()
{
    uint32_t ptr = (uint32_t)descriptorsBuffer;
    while (ptr & (DMA_DESCRIPTOR_ALIGNMENT - 1))
        ptr++;
    descriptors = (DmacDescriptor *)ptr;

    memclr(descriptors, sizeof(DmacDescriptor) * (DMA_DESCRIPTOR_COUNT * 2));

    // Set up to DMA Controller
    this->disable();

// Turn on the clocks
#ifdef SAMD21
    PM->AHBMASK.reg |= PM_AHBMASK_DMAC;
    PM->APBBMASK.reg |= PM_APBBMASK_DMAC;
#else
    MCLK->AHBMASK.reg |= MCLK_AHBMASK_DMAC;
#endif

    DMAC->CTRL.reg = DMAC_CTRL_SWRST;

    // Allow all DMA priorities.
    DMAC->CTRL.reg = DMAC_CTRL_LVLEN0 | DMAC_CTRL_LVLEN1 | DMAC_CTRL_LVLEN2 | DMAC_CTRL_LVLEN3;

    DMAC->CRCCTRL.reg = 0; // Disable all CRC expectations

    DMAC->BASEADDR.reg =
        (uint32_t)&descriptors[DMA_DESCRIPTOR_COUNT]; // Initialise Descriptor table
    DMAC->WRBADDR.reg = (uint32_t)&descriptors[0];    // initialise Writeback table

    this->enable();

#ifdef SAMD21
    NVIC_EnableIRQ(DMAC_IRQn);
    NVIC_SetPriority(DMAC_IRQn, 1);
#else
    for (int i = 0; i < 5; ++i)
    {
        NVIC_EnableIRQ(DMAC_0_IRQn + i);
        NVIC_SetPriority(DMAC_0_IRQn + i, 1);
    }
#endif
}

void SAMDDMAC::enable()
{
    DMAC->CTRL.bit.DMAENABLE = 1; // Enable controller.
    DMAC->CTRL.bit.CRCENABLE = 1; // Disable CRC checking.
}

void SAMDDMAC::disable()
{
    DMAC->CTRL.bit.DMAENABLE = 0; // Diable controller, just while we configure it.
    DMAC->CTRL.bit.CRCENABLE = 0; // Disable CRC checking.
}

void SAMDDMAC::startTransfer(int channel_number, void *src_addr, void *dst_addr, uint32_t len)
{
    CODAL_ASSERT(channel_number >= 0);
    target_disable_irq();
    DmacDescriptor &descriptor = dmac.getDescriptor(channel_number);
    descriptor.BTCNT.bit.BTCNT = len;
    if (src_addr)
        descriptor.SRCADDR.reg = (uint32_t)src_addr;
    if (dst_addr)
        descriptor.DSTADDR.reg = (uint32_t)dst_addr;

#ifdef SAMD21
    /** Select the DMA channel and clear software trigger */
    DMAC->CHID.bit.ID = channel_number;
    // Clear any previous interrupts.
    DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_MASK;
    DMAC->CHCTRLA.bit.ENABLE = true;
#else
    DmacChannel *channel = &DMAC->Channel[channel_number];
    // Clear any previous interrupts.
    channel->CHINTFLAG.reg = DMAC_CHINTFLAG_MASK;
    channel->CHCTRLA.bit.ENABLE = true;
#endif

    target_enable_irq();
}

void SAMDDMAC::configureChannel(int channel_number, uint8_t trig_src, uint8_t beat_size,
                                  void *src_addr, void *dst_addr)
{
    CODAL_ASSERT(channel_number >= 0);
    target_disable_irq();

#ifdef SAMD21
    DMAC->CHID.bit.ID = channel_number; // Select our allocated channel

    DMAC->CHCTRLA.bit.ENABLE = 0;
    DMAC->CHCTRLA.bit.SWRST = 1;

    while (DMAC->CHCTRLA.bit.SWRST)
        ;

    DMAC->SWTRIGCTRL.reg &= (uint32_t)(~(1 << channel_number));

    // DMAC->CHCTRLB.bit.CMD = 0;     // No Command (yet)
    DMAC->CHCTRLB.bit.TRIGACT = 2; // One trigger per beat transfer
    DMAC->CHCTRLB.bit.TRIGSRC = trig_src;
    DMAC->CHCTRLB.bit.LVL = 0;   // Low priority transfer
    DMAC->CHCTRLB.bit.EVOE = 0;  // Disable output event on every BEAT
    DMAC->CHCTRLB.bit.EVIE = 0;  // Disable input event
    DMAC->CHCTRLB.bit.EVACT = 0; // Trigger DMA transfer on BEAT

    DMAC->CHINTENSET.bit.TCMPL = 1; // Enable interrupt on completion.
#else
    DmacChannel *channel = &DMAC->Channel[channel_number];

    channel->CHCTRLA.bit.ENABLE = 0;
    channel->CHCTRLA.bit.SWRST = 1;

    while (channel->CHCTRLA.bit.SWRST)
        ;

    DMAC->SWTRIGCTRL.reg &= (uint32_t)(~(1 << channel_number));

    channel->CHCTRLA.bit.TRIGACT = 2; // One trigger per beat transfer
    channel->CHCTRLA.bit.TRIGSRC = trig_src;
    channel->CHCTRLA.bit.LVL = 0;   // Low priority transfer
    channel->CHCTRLA.bit.EVOE = 0;  // Disable output event on every BEAT
    channel->CHCTRLA.bit.EVIE = 0;  // Disable input event
    channel->CHCTRLA.bit.EVACT = 0; // Trigger DMA transfer on BEAT

    channel->CHINTENSET.bit.TCMPL = 1; // Enable interrupt on completion.
#endif

    DmacDescriptor &descriptor = dmac.getDescriptor(channel_number);

    descriptor.BTCTRL.reg = 0;

    bool isTx = dst_addr != NULL;

    descriptor.BTCTRL.bit.STEPSIZE = 0;           // Auto increment address by 1 after each beat
    descriptor.BTCTRL.bit.STEPSEL = isTx ? 0 : 1; // increment applies to SOURCE address
    descriptor.BTCTRL.bit.DSTINC = isTx ? 0 : 1;  // increment does not apply to destintion address
    descriptor.BTCTRL.bit.SRCINC = isTx ? 1 : 0;  // increment does apply to source address
    descriptor.BTCTRL.bit.BEATSIZE = beat_size;   // 16 bit wide transfer.
    descriptor.BTCTRL.bit.BLOCKACT = 0;           // No action when transfer complete.
    descriptor.BTCTRL.bit.EVOSEL = 3;             // Strobe events after every BEAT transfer

    descriptor.BTCNT.bit.BTCNT = 0;
    descriptor.SRCADDR.reg = (uint32_t)src_addr;
    descriptor.DSTADDR.reg = (uint32_t)dst_addr;
    descriptor.DESCADDR.reg = 0;

    descriptor.BTCTRL.bit.VALID = 1; // Enable the descritor

    target_enable_irq();
}

/**
 * Provides the SAMD21 specific DMA descriptor for the given channel number
 * @return a valid DMA decriptor, matching a previously allocated channel.
 */
DmacDescriptor &SAMDDMAC::getDescriptor(int channel)
{
    if (channel < DMA_DESCRIPTOR_COUNT)
        return descriptors[channel + DMA_DESCRIPTOR_COUNT];

    return descriptors[0];
}

/**
 * Allocates an unused DMA channel, if one is available.
 * @return a valid channel descriptor in the range 1..DMA_DESCRIPTOR_COUNT, or DEVICE_NO_RESOURCES
 * otherwise.
 */
int SAMDDMAC::allocateChannel()
{
    for (int i = 0; i < DMA_DESCRIPTOR_COUNT; i++)
    {
        if (!descriptors[i + DMA_DESCRIPTOR_COUNT].BTCTRL.bit.VALID)
        {
            descriptors[i + DMA_DESCRIPTOR_COUNT].BTCTRL.bit.VALID = 1;
            return i;
        }
    }

    return DEVICE_NO_RESOURCES;
}

/**
 * Registers a component to receive low level, hardware interrupt upon DMA transfer completion
 *
 * @param channel the DMA channel that the component is interested in.
 * @param component the component that wishes to receive the interrupt.
 *
 * @return DEVICE_OK on success, or DEVICE_INVALID_PARAMETER if the channel number is invalid.
 */
int SAMDDMAC::onTransferComplete(int channel, DmaComponent *component)
{
    if (channel >= DMA_DESCRIPTOR_COUNT)
        return DEVICE_INVALID_PARAMETER;

    apps[channel] = component;
    return DEVICE_OK;
}

#if CONFIG_ENABLED(DEVICE_DBG)

void SAMDDMAC::showDescriptor(DmacDescriptor *desc)
{
    SERIAL_DEBUG->printf("DESC: %p\n", desc);
    SERIAL_DEBUG->printf("DESC->SRCADDR: %p\n", &desc->SRCADDR);
    SERIAL_DEBUG->printf("DESC->DSTADDR: %p\n", &desc->DSTADDR);

    SERIAL_DEBUG->printf("STESIZE: %d\n", desc->BTCTRL.bit.STEPSIZE);
    SERIAL_DEBUG->printf("DSTINC: %d\n", desc->BTCTRL.bit.DSTINC);
    SERIAL_DEBUG->printf("SRCINC: %d\n", desc->BTCTRL.bit.SRCINC);
    SERIAL_DEBUG->printf("BEATSIZE: %d\n", desc->BTCTRL.bit.BEATSIZE);
    SERIAL_DEBUG->printf("BLOCKACT: %d\n", desc->BTCTRL.bit.BLOCKACT);
    SERIAL_DEBUG->printf("EVOSEL: %d\n", desc->BTCTRL.bit.EVOSEL);
    SERIAL_DEBUG->printf("VALID: %d\n", desc->BTCTRL.bit.VALID);

    SERIAL_DEBUG->printf("BTCNT: %d\n", desc->BTCNT.bit.BTCNT);
    SERIAL_DEBUG->printf("SRCADDR: %p\n", desc->SRCADDR.bit.SRCADDR);
    SERIAL_DEBUG->printf("DSTADDR: %p\n", desc->DSTADDR.bit.DSTADDR);
    SERIAL_DEBUG->printf("DESCADDR: %p\n", desc->DESCADDR.bit.DESCADDR);
}

void SAMDDMAC::showRegisters()
{
    SERIAL_DEBUG->printf("BASEADDR: 0x%.8x[%p]\n", DMAC->BASEADDR.reg, &descriptors[1]);
    SERIAL_DEBUG->printf("WRBADDR: 0x%.8x [%p]\n", DMAC->WRBADDR.reg, &descriptors[0]);
    SERIAL_DEBUG->printf("CTRL: 0x%.2x\n", DMAC->CTRL.reg);
    SERIAL_DEBUG->printf("PENDCH: 0x%.8x\n", DMAC->PENDCH.reg);
    DMAC->CHID.bit.ID = 0; // Select channel 0
    SERIAL_DEBUG->printf("CHCTRLA: 0x%.8x\n", DMAC->CHCTRLA.reg);
    SERIAL_DEBUG->printf("CHCTRLB: 0x%.8x\n", DMAC->CHCTRLB.reg);
    SERIAL_DEBUG->printf("INTPEND: 0x%.8x\n", DMAC->INTPEND.reg);
    SERIAL_DEBUG->printf("ACTIVE: 0x%.8x\n", DMAC->ACTIVE.reg);
    SERIAL_DEBUG->printf("BUSYCH: 0x%.8x\n", DMAC->BUSYCH.reg);

    SERIAL_DEBUG->printf("APBBMASK: 0x%.8x\n", PM->APBBMASK.reg);
    SERIAL_DEBUG->printf("AHBMASK: 0x%.2x\n", PM->AHBMASK.reg);
}

#endif
