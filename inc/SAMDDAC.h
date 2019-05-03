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
#include "ZPin.h"
#include "SAMDDMAC.h"
#include "DataStream.h"

#ifndef SAMDDAC_H
#define SAMDDAC_H

#ifndef SAMDDAC_DEFAULT_FREQUENCY
#define SAMDDAC_DEFAULT_FREQUENCY 44100
#endif

using namespace codal;

class SAMDDAC : public CodalComponent, public DmaComponent, public DataSink
{

private:
    bool        active;
    int         dataReady;
    int         sampleRate;
    Tc*         tc;

    DmaInstance* dmaInstance0;
#ifdef SAMD51
    DmaInstance* dmaInstance1;
#endif
    void prefill();

public:

    // The stream component that is serving our data
    DataSource  &upstream;
    ManagedBuffer buffer, nextBuffer;

    /**
      * Constructor for an instance of a DAC,
      *
      * @param pin The pin this DAC shoudl output to.
      * @param id The id to use for the message bus when transmitting events.
      */
    SAMDDAC(ZPin &pin, DataSource &source, int sampleRate = SAMDDAC_DEFAULT_FREQUENCY, uint16_t id = DEVICE_ID_SYSTEM_DAC);

    /**
     * Callback provided when data is ready.
     */
	virtual int pullRequest();

    /**
     * Pull down a buffer from upstream, and schedule a DMA transfer from it.
     */
    int pull();

    void setValue(int value);
    int getValue();

    /**
     * Change the DAC playback sample rate to the given frequency.
     * n.b. Only sample periods that are a multiple of 125nS are supported.
     * Frequencies mathcing other sample periods will be rounded down to the next lowest supported frequency.
     *
     * @param frequency The new sample playback frequency.
     */
    int getSampleRate();

    /**
     * Change the DAC playback sample rate to the given frequency.
     * n.b. Only sample periods that are a multiple of 125nS are supported.
     * Frequencies mathcing other sample periods will be rounded to the next highest supported frequency.
     *
     * @param frequency The new sample playback frequency.
     */
    int setSampleRate(int frequency);

    /**
     * Interrupt callback when playback of DMA buffer has completed
     */
    virtual void dmaTransferComplete(DmaCode c);
};

#endif
