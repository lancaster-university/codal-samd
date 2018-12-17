#include "LowLevelTimer.h"

#ifndef SAMD_TCC_TIMER_H
#define SAMD_TCC_TIMER_H

namespace codal
{

class SAMDTCCTimer : public LowLevelTimer
{
    uint8_t irqN;

    public:
    Tcc* tcc;

    SAMDTCCTimer(Tcc* tcc, uint8_t irqN);

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