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
#include "codal_target_hal.h"
#include <parts.h>
#include "CodalDmesg.h"


#undef ENABLE

using namespace codal;

DmaFactory* DmaFactory::instance = NULL;
DmaInstance* DmaFactory::apps[DMA_DESCRIPTOR_COUNT];

#ifdef SAMD21
extern "C" void DMAC_Handler(void)
{
    uint32_t oldChannel = DMAC->CHID.bit.ID;

    int channel = DMAC->INTPEND.bit.ID;
    DMAC->CHID.bit.ID = channel;

    bool err = (DMAC->CHINTFLAG.bit.ERROR > 0) ? true : false;
    DMAC->CHINTFLAG.reg= DMAC_CHINTENCLR_TCMPL;

    if (DmaFactory::instance->apps[channel] != NULL && DmaFactory::instance->apps[channel]->cb)
    {
        if (err)
            DmaFactory::instance->apps[channel]->trigger(DMA_ERROR);
        else
            DmaFactory::instance->apps[channel]->trigger(DMA_COMPLETE);
    }

    DMAC->CHID.bit.ID = oldChannel;
}
#else
static void dmac_irq_handler()
{
    uint8_t channel = hri_dmac_get_INTPEND_reg(DMAC, DMAC_INTPEND_ID_Msk);

    if (hri_dmac_get_CHINTFLAG_TERR_bit(DMAC, channel))
    {
        hri_dmac_clear_CHINTFLAG_TERR_bit(DMAC, channel);
        if (DmaFactory::instance->apps[channel] != NULL && DmaFactory::instance->apps[channel]->cb)
            DmaFactory::instance->apps[channel]->trigger(DMA_ERROR);
        // tmp_resource->dma_cb.error(tmp_resource);
    }
    else if (hri_dmac_get_CHINTFLAG_TCMPL_bit(DMAC, channel))
    {
        hri_dmac_clear_CHINTFLAG_TCMPL_bit(DMAC, channel);

        if (DmaFactory::instance->apps[channel] != NULL && DmaFactory::instance->apps[channel]->cb)
            DmaFactory::instance->apps[channel]->trigger(DMA_COMPLETE);
    }

    hri_dmac_clear_CHINTFLAG_reg(DMAC, channel, 0);
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
void DmaComponent::dmaTransferComplete(DmaCode) {}

DmaFactory::DmaFactory()
{
    if (instance == NULL)
        DmaFactory::instance = this;
    else
        return;

    uint32_t ptr = (uint32_t)descriptorsBuffer;
    while (ptr & (DMA_DESCRIPTOR_ALIGNMENT - 1))
        ptr++;
    descriptors = (DmacDescriptor *)ptr;

    memclr(descriptors, sizeof(DmacDescriptor) * (DMA_DESCRIPTOR_COUNT * 2));
    memclr(apps, sizeof(DmaInstance*) * DMA_DESCRIPTOR_COUNT);

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

    DMAC->BASEADDR.reg = (uint32_t)&descriptors[DMA_DESCRIPTOR_COUNT]; // Initialise Descriptor table
    DMAC->WRBADDR.reg = (uint32_t)&descriptors[0];    // initialise Writeback table

    this->enable();

#ifdef SAMD21
    NVIC_EnableIRQ(DMAC_IRQn);
    NVIC_SetPriority(DMAC_IRQn, 1);
#else
    for (int i = 0; i < 5; ++i)
    {
        auto irq = (IRQn_Type)(DMAC_0_IRQn + i);
        NVIC_EnableIRQ(irq);
        NVIC_SetPriority(irq, 1);
    }
#endif
}

void DmaFactory::enable()
{
    DMAC->CTRL.bit.DMAENABLE = 1; // Enable controller.
}

void DmaFactory::disable()
{
    DMAC->CTRL.bit.DMAENABLE = 0; // Disable controller, just while we configure it.
}

/**
 * Provides the SAMD21 specific DMA descriptor for the given channel number
 * @return a valid DMA decriptor, matching a previously allocated channel.
 */
DmacDescriptor &DmaFactory::getDescriptor(int channel)
{
    if (channel < DMA_DESCRIPTOR_COUNT)
        return DmaFactory::instance->descriptors[channel + DMA_DESCRIPTOR_COUNT];

    return DmaFactory::instance->descriptors[0];
}

/**
 * Allocates an unused DMA channel, if one is available.
 * @return a valid channel descriptor in the range 1..DMA_DESCRIPTOR_COUNT, or DEVICE_NO_RESOURCES
 * otherwise.
 */
DmaInstance* DmaFactory::allocate()
{
    for (int i = 0; i < DMA_DESCRIPTOR_COUNT; i++)
    {
        if (!DmaFactory::instance->descriptors[i + DMA_DESCRIPTOR_COUNT].BTCTRL.bit.VALID)
        {
            DmaFactory::instance->descriptors[i + DMA_DESCRIPTOR_COUNT].BTCTRL.bit.VALID = 1;
            DmaFactory::instance->apps[i] = new DmaInstance(i);
            return DmaFactory::instance->apps[i];
        }
    }

    return NULL;
}

/**
 * free's an unused DMA channel
 */
void DmaFactory::free(DmaInstance* dmaInstance)
{
    for (int i = 0; i < DMA_DESCRIPTOR_COUNT; i++)
    {
        if (DmaFactory::instance->apps[i] == dmaInstance)
        {
            DmaFactory::instance->descriptors[i + DMA_DESCRIPTOR_COUNT].BTCTRL.bit.VALID = 0;
            DmaFactory::instance->apps[i] = NULL;
            return;
        }
    }
}




// ingore for now.
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
