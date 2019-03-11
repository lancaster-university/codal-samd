#ifndef SAMD_TC_TIMER_H
#define SAMD_TC_TIMER_H

#include "LowLevelTimer.h"

#include "sam.h"
extern "C"
{
    #include "clocks.h"
    #include "timers.h"
}

namespace codal
{

class SAMDTCTimer : public LowLevelTimer
{
    uint8_t irqN;

    public:
    Tc* tc;

    SAMDTCTimer(Tc* tc, uint8_t irqN);

    virtual int setIRQPriority(int priority) override;

    virtual int enable();

    virtual int enableIRQ();

    virtual int disable();

    virtual int disableIRQ();

    virtual int reset();

    virtual int setMode(TimerMode t);

    virtual int setCompare(uint8_t channel, uint32_t value);

    virtual int offsetCompare(uint8_t channel, uint32_t value);

    virtual int clearCompare(uint8_t channel);

    virtual uint32_t captureCounter();

    virtual int setClockSpeed(uint32_t speedKHz);

    virtual int setBitMode(TimerBitMode t);
};

}

#endif