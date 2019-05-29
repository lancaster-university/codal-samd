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

#define EIC_CHANNEL_COUNT           16

#ifndef SAMDEIC_H
#define SAMDEIC_H

namespace codal
{

enum EICEventType
{
    EICEventsNone = 0,
    EICEventsRise,
    EICEventsFall,
    EICEventsRiseFall,
    EICEventsHigh,
    EICEventsLo
};

class EICInterface
{
    public:
    virtual void pinEventDetected();
};

class EICChannel
{
    int channel_number; // the channel
    EICEventType configuration;

    public:
    EICInterface* cb;

    /**
     * Create an EIC channel given a channel number
     *
     * @param channel the channel to use
     **/
    EICChannel(int channel);

    /**
     * Configures this instance given a type.
     *
     * @param t the EICEventType configuration for the channel.
     **/
    void configure(EICEventType t);

    /**
     * Retrieves mode of this channel.
     **/
    EICEventType getConfiguration();

    /**
     * Disables this channel
     **/
    void disable();

    /**
     * enables this channel with the given event configuration
     *
     * @param t the event type.
     **/
    void enable(EICEventType t);

    /**
     * Called by the interrupt handler when an interrupt is received.
     **/
    void trigger();

    /**
     * Set the event handling instance.
     *
     * @param interface the EICinterface to invoke.
     **/
    void setChangeCallback(EICInterface* interface);
};

class EICFactory
{
    /**
     * A management instance for the external interrupt controller.
     *
     * Returns instances of EICChannels if the channel is not already in use.
     */
    EICFactory();

    public:

    static EICChannel* instances[EIC_CHANNEL_COUNT]; // EIC channel instances.
    static EICFactory* instance;    // singleton reference

    static EICFactory* getInstance();

    /**
     * Disables all external interrupt channels
     */
    void disable();

    /**
     * Enables all external interrupt channels
     */
    void enable();

    /**
     * Returns an EIC channel given a channel number if the channel is unused.
     *
     * @param channel the channel number to use
     *
     * @returns an eic channel instance, or NULL if already in use.
     **/
    EICChannel* getChannel(int channel);

    /**
     * Free's an EICChannel instance.
     *
     * @param instance the instance to free.
     **/
    void free(int channel);
};

} // namespace codal

#endif
