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

#include "dma.h"
#include "pinmap.h"
#include "PeripheralPins.h"

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

//#define LOG DMESG
#define LOG(...) ((void)0)

namespace codal
{

static const uint32_t baudprescaler[] = //
    {SPI_BAUDRATEPRESCALER_2,
     2,
     SPI_BAUDRATEPRESCALER_4,
     4,
     SPI_BAUDRATEPRESCALER_8,
     8,
     SPI_BAUDRATEPRESCALER_16,
     16,
     SPI_BAUDRATEPRESCALER_32,
     32,
     SPI_BAUDRATEPRESCALER_64,
     64,
     SPI_BAUDRATEPRESCALER_128,
     128,
     SPI_BAUDRATEPRESCALER_256,
     256,
     0,
     0};

static ZSPI *instances[4];

#define ZERO(f) memset(&f, 0, sizeof(f))

uint32_t codal_setup_pin(Pin *p, uint32_t prev, const PinMap *map)
{
    if (!p)
        return 0;
    auto pin = p->name;
    uint32_t tmp = pinmap_peripheral(pin, map);
    pin_function(pin, pinmap_function(pin, map));
    pin_mode(pin, PullNone);
    CODAL_ASSERT(!prev || prev == tmp);
    return tmp;
}

static int enable_clock(uint32_t instance)
{
    switch (instance)
    {
    case SPI1_BASE:
        __HAL_RCC_SPI1_CLK_ENABLE();
        NVIC_EnableIRQ(SPI1_IRQn);
        return HAL_RCC_GetPCLK2Freq();
    case SPI2_BASE:
        __HAL_RCC_SPI2_CLK_ENABLE();
        NVIC_EnableIRQ(SPI2_IRQn);
        return HAL_RCC_GetPCLK1Freq();
#ifdef SPI3_BASE
    case SPI3_BASE:
        __HAL_RCC_SPI3_CLK_ENABLE();
        NVIC_EnableIRQ(SPI3_IRQn);
        return HAL_RCC_GetPCLK1Freq();
#endif
#ifdef SPI4_BASE
    case SPI4_BASE:
        __HAL_RCC_SPI4_CLK_ENABLE();
        NVIC_EnableIRQ(SPI4_IRQn);
        return HAL_RCC_GetPCLK2Freq();
#endif
#ifdef SPI5_BASE
    case SPI5_BASE:
        __HAL_RCC_SPI5_CLK_ENABLE();
        NVIC_EnableIRQ(SPI5_IRQn);
        return HAL_RCC_GetPCLK2Freq();
#endif
#ifdef SPI6_BASE
    case SPI6_BASE:
        __HAL_RCC_SPI6_CLK_ENABLE();
        NVIC_EnableIRQ(SPI6_IRQn);
        return HAL_RCC_GetPCLK2Freq();
#endif

    default:
        CODAL_ASSERT(0);
        return 0;
    }
    return 0;
}

void ZSPI::complete()
{
    LOG("SPI complete D=%p", doneHandler);
    if (doneHandler)
    {
        PVoidCallback done = doneHandler;
        doneHandler = NULL;
        //create_fiber(done, doneHandlerArg);
        done(doneHandlerArg);
    }
    else
    {
        Event(DEVICE_ID_NOTIFY_ONE, transferCompleteEventCode);
    }
}

void ZSPI::_complete(uint32_t instance)
{
    LOG("SPI complete %p", instance);
    for (unsigned i = 0; i < ARRAY_SIZE(instances); ++i)
    {
        if (instances[i] && (uint32_t)instances[i]->spi.Instance == instance)
        {
            instances[i]->complete();
            break;
        }
    }
}

void ZSPI::_irq(uint32_t instance)
{
    LOG("SPI IRQ %p", instance);
    for (unsigned i = 0; i < ARRAY_SIZE(instances); ++i)
    {
        if (instances[i] && (uint32_t)instances[i]->spi.Instance == instance)
        {
            HAL_SPI_IRQHandler(&instances[i]->spi);
            break;
        }
    }
}

extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    ZSPI::_complete((uint32_t)hspi->Instance);
}

extern "C" void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    ZSPI::_complete((uint32_t)hspi->Instance);
}

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    ZSPI::_complete((uint32_t)hspi->Instance);
}

#define DEFIRQ(nm, id)                                                                             \
    extern "C" void nm() { ZSPI::_irq(id); }

DEFIRQ(SPI1_IRQHandler, SPI1_BASE)
DEFIRQ(SPI2_IRQHandler, SPI2_BASE)
#ifdef SPI3_BASE
DEFIRQ(SPI3_IRQHandler, SPI3_BASE)
#endif
#ifdef SPI4_BASE
DEFIRQ(SPI4_IRQHandler, SPI4_BASE)
#endif
#ifdef SPI5_BASE
DEFIRQ(SPI5_IRQHandler, SPI5_BASE)
#endif
#ifdef SPI6_BASE
DEFIRQ(SPI6_IRQHandler, SPI6_BASE)
#endif

void ZSPI::init()
{
    if (!needsInit)
        return;

    needsInit = false;

    if (!spi.Instance)
    {
        uint32_t instance = codal_setup_pin(sclk, 0, PinMap_SPI_SCLK);
        instance = codal_setup_pin(miso, 0, PinMap_SPI_MISO);
        instance = codal_setup_pin(mosi, 0, PinMap_SPI_MOSI);

        spi.Instance = (SPI_TypeDef *)instance;
    }

    LOG("SPI instance %p", spi.Instance);

    spi.Init.Direction = SPI_DIRECTION_2LINES;
    spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spi.Init.CRCPolynomial = 7;
    spi.Init.DataSize = SPI_DATASIZE_8BIT;
    spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spi.Init.NSS = SPI_NSS_SOFT;
    spi.Init.TIMode = SPI_TIMODE_DISABLE;
    spi.Init.Mode = SPI_MODE_MASTER;

    if (mosi && !hdma_tx.Instance)
    {
        dma_init((uint32_t)spi.Instance, DMA_TX, &hdma_tx, 0);
        __HAL_LINKDMA(&spi, hdmatx, hdma_tx);
    }

    if (miso && !hdma_rx.Instance)
    {
        dma_init((uint32_t)spi.Instance, DMA_RX, &hdma_rx, 0);
        __HAL_LINKDMA(&spi, hdmarx, hdma_rx);
    }

    auto pclkHz = enable_clock((uint32_t)spi.Instance);

    for (int i = 0; baudprescaler[i]; i += 2)
    {
        spi.Init.BaudRatePrescaler = baudprescaler[i];
        if (pclkHz / baudprescaler[i + 1] <= freq)
            break;
    }

    spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;

    auto res = HAL_SPI_Init(&spi);
    CODAL_ASSERT(res == HAL_OK);
}

ZSPI::ZSPI(Pin &mosi, Pin &miso, Pin &sclk) : codal::SPI()
{
    this->mosi = &mosi;
    this->miso = &miso;
    this->sclk = &sclk;

    ZERO(spi);
    ZERO(hdma_tx);
    ZERO(hdma_rx);

    this->needsInit = true;
    this->transferCompleteEventCode = codal::allocateNotifyEvent();

    for (unsigned i = 0; i < ARRAY_SIZE(instances); ++i)
    {
        if (instances[i] == NULL) {
            instances[i] = this;
            break;
        }
    }
}

int ZSPI::setFrequency(uint32_t frequency)
{
    freq = frequency;
    needsInit = true;
    return DEVICE_OK;
}

int ZSPI::setMode(int mode, int bits)
{
    spi.Init.CLKPhase = mode & 1 ? SPI_PHASE_2EDGE : SPI_PHASE_1EDGE;
    spi.Init.CLKPolarity = mode & 2 ? SPI_POLARITY_HIGH : SPI_POLARITY_LOW;
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
    int res;

    init();

    LOG("SPI start %p/%d %p/%d D=%p", txBuffer, txSize, rxBuffer, rxSize, doneHandler);

    this->doneHandler = doneHandler;
    this->doneHandlerArg = arg;

    if (txSize && rxSize)
    {
        CODAL_ASSERT(txSize == rxSize); // we could support this if needed
        res = HAL_SPI_TransmitReceive_DMA(&spi, (uint8_t *)txBuffer, rxBuffer, txSize);
    }
    else if (txSize)
    {
        res = HAL_SPI_Transmit_DMA(&spi, (uint8_t *)txBuffer, txSize);
    }
    else if (rxSize)
    {
        res = HAL_SPI_Receive_DMA(&spi, rxBuffer, rxSize);
    }
    else
    {
        return 0; // nothing to do
    }

    CODAL_ASSERT(res == HAL_OK);

    return 0;
}

} // namespace codal
