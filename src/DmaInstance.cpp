#include "Timer.h"
#include "Event.h"
#include "CodalCompat.h"
#include "SAMDDMAC.h"
#include "codal_target_hal.h"
#include <parts.h>
#include "CodalDmesg.h"

#undef ENABLE

using namespace codal;

DmaInstance::DmaInstance(int channel)
{
    this->channel_number = channel;
    this->cb = NULL;
}

/**
 * Disables all confgures DMA activity.
 * Typically required before configuring DMA descriptors and DMA channels.
 */
void DmaInstance::disable()
{
#ifdef SAMD21
    DMAC->CHID.bit.ID = channel_number; // Select our allocated channel
    DMAC->CHCTRLA.bit.ENABLE = 0;
#else
    DmacChannel *channel = &DMAC->Channel[channel_number];
    channel->CHCTRLA.bit.ENABLE = 0;
#endif
}

/**
 * Enables all confgures DMA activity
 */
void DmaInstance::enable()
{
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
}

void DmaInstance::trigger(DmaCode c)
{
    disable();

    if (this->cb)
        this->cb->dmaTransferComplete(c);
}


void DmaInstance::abort()
{
    disable();
}

/**
 * Registers a component to receive low level, hardware interrupt upon DMA transfer completion
 *
 * @param channel the DMA channel that the component is interested in.
 * @param component the component that wishes to receive the interrupt.
 *
 * @return DEVICE_OK on success, or DEVICE_INVALID_PARAMETER if the channel number is invalid.
 */
int DmaInstance::onTransferComplete(DmaComponent *component)
{
    cb = component;
    return DEVICE_OK;
}

DmacDescriptor& DmaInstance::getDescriptor()
{
    return DmaFactory::instance->getDescriptor(channel_number);
}

void DmaInstance::setDescriptor(DmacDescriptor* desc)
{
    DmaFactory::instance->setDescriptor(channel_number, desc);
}

void DmaInstance::transfer(const void *src_addr, void *dst_addr, uint32_t len)
{
    CODAL_ASSERT(channel_number >= 0, DEVICE_HARDWARE_CONFIGURATION_ERROR);
    target_disable_irq();
    DmacDescriptor &descriptor = DmaFactory::instance->getDescriptor(channel_number);

    descriptor.BTCNT.bit.BTCNT = len >> descriptor.BTCTRL.bit.BEATSIZE;

    this->bufferSize = len >> descriptor.BTCTRL.bit.BEATSIZE;

    if (src_addr)
        descriptor.SRCADDR.reg = (uint32_t)src_addr + len;
    if (dst_addr)
        descriptor.DSTADDR.reg = (uint32_t)dst_addr + len;

    enable();

    target_enable_irq();
}

int DmaInstance::getBytesTransferred()
{
    DmacDescriptor &descriptor = DmaFactory::instance->getDescriptor(channel_number);
    return (-1 * descriptor.BTCNT.bit.BTCNT) + this->bufferSize;
}

void DmaInstance::configure(uint8_t trig_src, DmaBeatSize beat_size, volatile void *src_addr, volatile void *dst_addr)
{
    CODAL_ASSERT(channel_number >= 0, DEVICE_HARDWARE_CONFIGURATION_ERROR);
    target_disable_irq();

#ifdef SAMD21
    DMAC->CHID.bit.ID = channel_number; // Select our allocated channel

    DMAC->CHCTRLA.bit.ENABLE = 0;
    DMAC->CHCTRLA.bit.SWRST = 1;

    while (DMAC->CHCTRLA.bit.SWRST);

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

    while (channel->CHCTRLA.bit.SWRST);

    DMAC->SWTRIGCTRL.reg &= (uint32_t)(~(1 << channel_number));

    channel->CHCTRLA.bit.TRIGACT = 2; // One trigger per beat transfer
    channel->CHCTRLA.bit.TRIGSRC = trig_src;
    /*
    channel->CHCTRLA.bit.LVL = 0;   // Low priority transfer
    channel->CHCTRLA.bit.EVOE = 0;  // Disable output event on every BEAT
    channel->CHCTRLA.bit.EVIE = 0;  // Disable input event
    channel->CHCTRLA.bit.EVACT = 0; // Trigger DMA transfer on BEAT
    */

    channel->CHINTENSET.bit.TERR = 1; // Enable interrupt on error.
    channel->CHINTENSET.bit.TCMPL = 1; // Enable interrupt on completion.
#endif

    DmacDescriptor &descriptor = DmaFactory::instance->getDescriptor(channel_number);

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

DmaInstance::~DmaInstance()
{
    DmaFactory::free(this);
}