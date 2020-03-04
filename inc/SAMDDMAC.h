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
#include "Timer.h"
#include "Pin.h"
#include "sam.h"

#ifndef SAMDDMAC_H
#define SAMDDMAC_H

#define DMA_DESCRIPTOR_ALIGNMENT 16 // SAMD21 Datasheet 20.8.15 and 20.8.16
#define DMA_DESCRIPTOR_COUNT 8

namespace codal
{

enum DmaCode
{
    DMA_COMPLETE,
    DMA_ERROR
};

enum DmaBeatSize
{
    BeatByte = 0,
    BeatHalfWord,
    BeatWord
};

class DmaInstance;

class DmaComponent
{
public:
    virtual void dmaTransferComplete(DmaInstance *dma, DmaCode c) = 0;
};

static inline int sercom_trigger_src(int sercomIdx, bool tx)
{
    return SERCOM0_DMAC_ID_RX + sercomIdx * 2 + (tx ? 1 : 0);
}

class DmaInstance
{
    uint32_t bufferSize;
    public:
    int channel_number;
    DmaComponent* cb;

    DmaInstance(int channel);

    /**
     * Disables this DMA instance.
     * Typically required before configuring DMA descriptors and DMA channels.
     */
    void disable();

    /**
     * enables this DMA instance
     */
    void enable();

    /**
     * Registers a component to receive low level, hardware interrupt upon DMA transfer completion
     *
     * @param channel the DMA channel that the component is interested in.
     * @param component the component that wishes to receive the interrupt.
     *
     * @return DEVICE_OK on success, or DEVICE_INVALID_PARAMETER if the channel number is invalid.
     */
    int onTransferComplete(DmaComponent *component);

    void abort();

    void transfer(const void *from, void *to, uint32_t len);

    void configure(uint8_t trig_src, DmaBeatSize beat_size, volatile void *src_addr, volatile void *dst_addr);

    DmacDescriptor& getDescriptor();
    DmacDescriptor& getWriteBackDescriptor();

    void setDescriptor(DmacDescriptor* d);

    int getBytesTransferred();

    void trigger(DmaCode c);

    ~DmaInstance();
};

class DmaControllerInstance
{
    // descriptors have to be 128 bit aligned - we allocate 16 more bytes, and set descriptors
    // at the right offset in descriptorsBuffer
    uint8_t descriptorsBuffer[sizeof(DmacDescriptor) * (DMA_DESCRIPTOR_COUNT * 2) +
                              DMA_DESCRIPTOR_ALIGNMENT];
    DmacDescriptor *descriptors;

    protected:
    DmaControllerInstance();

    public:

    void enable();

    void disable();

    void setDescriptor(int channel, DmacDescriptor*);

    /**
     * Provides the SAMD21 specific DMA descriptor for the given channel number
     * @return a valid DMA decriptor, matching a previously allocated channel.
     */
    DmacDescriptor& getDescriptor(int channel);

    DmacDescriptor& getWriteBackDescriptor(int channel);

    friend class DmaFactory;
};

class DmaFactory
{
    static void instantiate();

    public:
    static DmaControllerInstance* instance;
    static DmaInstance* apps[DMA_DESCRIPTOR_COUNT];

    /**
     * Disables all confgures DMA activity.
     * Typically required before configuring DMA descriptors and DMA channels.
     */
    static void disable();

    /**
     * Enables all confgures DMA activity
     */
    static void enable();

    static DmaInstance* allocate();

    static void setDescriptor(int channel, DmacDescriptor*);

    /**
     * Provides the SAMD21 specific DMA descriptor for the given channel number
     * @return a valid DMA decriptor, matching a previously allocated channel.
     */
    static DmacDescriptor& getDescriptor(int channel);

    static DmacDescriptor& getWriteBackDescriptor(int channel);

    static void free(DmaInstance*);
};

} // namespace codal

#endif
