#include "LowLevelTimer.h"

#ifndef SAMD_TC_TIMER_H
#define SAMD_TC_TIMER_H

namespace codal
{

struct TCTimerStruct
{
    Tc* tc;
    int countReg;
}

class SAMDTCTimer : public LowLevelTimer
{
    Tc* tc;
    uint8_t irqN;
    TimerBitMode bm;

    public:

    SAMDTCTimer(Tc* tc);

    virtual int enable();

    virtual int disable();

    virtual int reset();

    virtual int setMode(TimerMode t);

    virtual int setCompare(uint8_t channel, uint32_t value);

    virtual int offsetCompare(uint8_t channel, uint32_t value);

    virtual int clearCompare(uint8_t channel);

    virtual uint32_t captureCounter(uint8_t channel);

    virtual int setPrescaler(uint16_t prescaleValue);

    virtual int setBitMode(TimerBitMode t);
};

}

#endif