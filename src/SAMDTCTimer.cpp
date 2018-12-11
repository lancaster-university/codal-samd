#include "SAMDTCTimer.h"
#include "CodalDmesg.h"
#include "sam.h"
#include "SAMDTimer.h"
extern "C"
{
    #include "clocks.h"
    #include "timers.h"
}

using namespace codal;

static SAMDTCTimer *instances[TC_INST_NUM] = { 0 };

void tc_irq_handler(uint8_t index)
{
    if (instances[index] == NULL)
        return;

    if (instances[index]->timer_pointer)
    {
        if (instances[index]->tc->COUNT32.INTFLAG.bit.MC0 && instances[index]->tc->COUNT32.INTENSET.reg & (1 << TC_INTENSET_MC0_Pos))
        {
            switch (instances[index]->bm)
            {
                case BitMode8:
                    instances[index]->tc->COUNT8.INTFLAG.bit.MC0 = 1;
                    break;
                case BitMode16:
                    instances[index]->tc->COUNT16.INTFLAG.bit.MC0 = 1;
                    break;
                case BitMode24:
                    // Take a break compiler...
                    break;
                case BitMode32:
                    instances[index]->tc->COUNT32.INTFLAG.bit.MC0 = 1;
                    break;
            }
            instances[index]->timer_pointer(0);
        }

        if (instances[index]->tc->COUNT32.INTFLAG.bit.MC1 && instances[index]->tc->COUNT32.INTENSET.reg & (1 << TC_INTENSET_MC1_Pos))
        {
            instances[index]->tc->COUNT32.INTFLAG.bit.MC1 = 1;
            instances[index]->timer_pointer(1);
        }
    }
}


SAMDTCTimer::SAMDTCTimer(Tc* tc, uint8_t irqn) : LowLevelTimer(2)
{
    // TODO: neaten up constructor, look up tc and irqn given tc number.
    this->tc= tc;
    this->irqN = irqn;

    // 48MHz / 6 == 8MhZ
    enable_clock_generator(CLK_GEN_8MHZ, CLOCK_48MHZ, 6);

    // find the tx index in the insts array
    uint8_t tc_index = 0;
    while (tc_index < TC_INST_NUM)
    {
        if (tc_insts[tc_index] == this->tc)
            break;
        tc_index++;
    }

    CODAL_ASSERT(tc_index < TC_INST_NUM);
    CODAL_ASSERT(instances[tc_index] == NULL);

    DMESG("tc_ind: %d clk_index: %d", tc_index, CLK_GEN_8MHZ);

    // configure the clks for the current timer.
    turn_on_clocks(true, tc_index, CLK_GEN_8MHZ);

    // disable the timer
    disable();

    bool inited = false;

    for (int index = 0; index < TC_INST_NUM; index++)
        if (instances[index])
            inited = true;

    if (!inited)
        tc_set_app_handler(tc_irq_handler);

    setBitMode(BitMode32);

    // configure
    switch (bm)
    {
        case BitMode8:
            tc->COUNT8.CTRLBCLR.bit.DIR = 1; // count up
#ifdef SAMD21
            while (tc->COUNT8.STATUS.bit.SYNCBUSY);
#endif
            break;
        case BitMode16:
            tc->COUNT16.CTRLBCLR.bit.DIR = 1; // count up
#ifdef SAMD21
            while (tc->COUNT16.STATUS.bit.SYNCBUSY);
#endif
            break;
        case BitMode24:
            // make the compiler shut up
            break;
        case BitMode32:
            tc->COUNT32.CTRLBCLR.bit.DIR = 1; // count up
#ifdef SAMD21
            while (tc->COUNT32.STATUS.bit.SYNCBUSY);
#endif
            break;
    }

    setPrescaler(3);
}

int SAMDTCTimer::enable()
{
    NVIC_SetPriority((IRQn_Type)this->irqN, 2);
    NVIC_ClearPendingIRQ((IRQn_Type)this->irqN);
    NVIC_EnableIRQ((IRQn_Type)this->irqN);
    tc_set_enable(tc, true);
    return DEVICE_OK;
}

int SAMDTCTimer::disable()
{
    NVIC_DisableIRQ((IRQn_Type)this->irqN);
    tc_set_enable(tc, false);
    return DEVICE_OK;
}

int SAMDTCTimer::reset()
{
    NVIC_DisableIRQ((IRQn_Type)this->irqN);
    switch (bm)
    {
        case BitMode8:
            tc->COUNT8.COUNT.reg = 0;
#ifdef SAMD21
            while (tc->COUNT8.STATUS.bit.SYNCBUSY);
#endif
            break;
        case BitMode16:
            tc->COUNT16.COUNT.reg = 0;
#ifdef SAMD21
            while (tc->COUNT16.STATUS.bit.SYNCBUSY);
#endif
            break;
        case BitMode24:
            // make the compiler shut up
            break;
        case BitMode32:
            tc->COUNT32.COUNT.reg = 0;
#ifdef SAMD21
            while (tc->COUNT32.STATUS.bit.SYNCBUSY);
#endif
            break;
    }
    NVIC_EnableIRQ((IRQn_Type)this->irqN);
    return DEVICE_OK;
}

int SAMDTCTimer::setMode(TimerMode t)
{
    // only support timer mode.
    return DEVICE_OK;
}

int SAMDTCTimer::setCompare(uint8_t channel, uint32_t value)
{
    if (channel > getChannelCount())
        return DEVICE_INVALID_PARAMETER;

    switch (bm)
    {
        case BitMode8:
            tc->COUNT8.CC[channel].reg = value;
#ifdef SAMD21
            while (tc->COUNT8.STATUS.bit.SYNCBUSY);
#endif
            // add channel to MC0, MC0 is 4, MC1 is 5
            tc->COUNT8.INTENSET.reg = (1 << (TC_INTENSET_MC0_Pos + channel));
            break;
        case BitMode16:
            tc->COUNT16.CC[channel].reg = value;
#ifdef SAMD21
            while (tc->COUNT16.STATUS.bit.SYNCBUSY);
#endif

            // add channel to MC0, MC0 is 4, MC1 is 5
            tc->COUNT16.INTENSET.reg = (1 << (TC_INTENSET_MC0_Pos + channel));
            break;
        case BitMode24:
            // make the compiler shut up
            break;
        case BitMode32:
            tc->COUNT32.CC[channel].reg = value;
#ifdef SAMD21
            while (tc->COUNT32.STATUS.bit.SYNCBUSY);
#endif

            // add channel to MC0, MC0 is 4, MC1 is 5
            tc->COUNT32.INTENSET.reg = (1 << (TC_INTENSET_MC0_Pos + channel));
            break;
    }
    return DEVICE_OK;
}

int SAMDTCTimer::offsetCompare(uint8_t channel, uint32_t value)
{
    if (channel > getChannelCount())
        return DEVICE_INVALID_PARAMETER;

    switch (bm)
    {
        case BitMode8:
            tc->COUNT8.CC[channel].reg += value;
#ifdef SAMD21
            while (tc->COUNT8.STATUS.bit.SYNCBUSY);
#endif
            // add channel to MC0, MC0 is 4, MC1 is 5
            tc->COUNT8.INTENSET.reg = (1 << (TC_INTENSET_MC0_Pos + channel));
            break;
        case BitMode16:
            tc->COUNT16.CC[channel].reg += value;
#ifdef SAMD21
            while (tc->COUNT16.STATUS.bit.SYNCBUSY);
#endif

            // add channel to MC0, MC0 is 4, MC1 is 5
            tc->COUNT16.INTENSET.reg = (1 << (TC_INTENSET_MC0_Pos + channel));
            break;
        case BitMode24:
            // make the compiler shut up
            break;
        case BitMode32:
            tc->COUNT32.CC[channel].reg += value;
#ifdef SAMD21
            while (tc->COUNT32.STATUS.bit.SYNCBUSY);
#endif

            // add channel to MC0, MC0 is 4, MC1 is 5
            tc->COUNT32.INTENSET.reg = (1 << (TC_INTENSET_MC0_Pos + channel));
            break;
    }

    return DEVICE_OK;
}

int SAMDTCTimer::clearCompare(uint8_t channel)
{
    if (channel > getChannelCount())
        return DEVICE_INVALID_PARAMETER;

    switch (bm)
    {
        case BitMode8:
            // add channel to MC0, MC0 is 4, MC1 is 5
            tc->COUNT8.INTENCLR.reg = (1 << (TC_INTENCLR_MC0_Pos + channel));
            break;
        case BitMode16:
            // add channel to MC0, MC0 is 4, MC1 is 5
            tc->COUNT16.INTENCLR.reg = (1 << (TC_INTENCLR_MC0_Pos + channel));
            break;
        case BitMode24:
            // make the compiler shut up
            break;
        case BitMode32:
            // add channel to MC0, MC0 is 4, MC1 is 5
            tc->COUNT32.INTENCLR.reg = (1 << (TC_INTENCLR_MC0_Pos + channel));
            break;
    }

    return DEVICE_OK;
}

uint32_t SAMDTCTimer::captureCounter(uint8_t)
{
    uint32_t elapsed = 0;

    NVIC_DisableIRQ((IRQn_Type)this->irqN);
#ifdef SAMD51
    switch (bm)
    {
        case BitMode8:
            tc->COUNT8.CTRLBSET.bit.CMD = 0x04;
            while (tc->COUNT8.SYNCBUSY.bit.CTRLB);
            elapsed = tc->COUNT8.COUNT.reg;
            break;
        case BitMode16:
            tc->COUNT16.CTRLBSET.bit.CMD = 0x04;
            while (tc->COUNT16.SYNCBUSY.bit.CTRLB);
            elapsed = tc->COUNT16.COUNT.reg;
            break;
        case BitMode24:
            // make the compiler shut up
            break;
        case BitMode32:
            tc->COUNT32.CTRLBSET.bit.CMD = 0x04;
            while (tc->COUNT32.SYNCBUSY.bit.CTRLB);
            elapsed = tc->COUNT32.COUNT.reg;
            break;
    }
#elif SAMD21

    switch (bm)
    {
        case BitMode8:
            tc->COUNT8.READREQ.bit.ADDR = 0x10;
            tc->COUNT8.READREQ.bit.RREQ = 1;
            while (tc->COUNT8.STATUS.bit.SYNCBUSY);
            elapsed = tc->COUNT8.COUNT.reg;
            break;
        case BitMode16:
            tc->COUNT16.READREQ.bit.ADDR = 0x10;
            tc->COUNT16.READREQ.bit.RREQ = 1;
            while (tc->COUNT16.STATUS.bit.SYNCBUSY);
            elapsed = tc->COUNT16.COUNT.reg;
            break;
        case BitMode24:
            // make the compiler shut up
            break;
        case BitMode32:
            tc->COUNT32.READREQ.bit.ADDR = 0x10;
            tc->COUNT32.READREQ.bit.RREQ = 1;
            while (tc->COUNT32.STATUS.bit.SYNCBUSY);
            elapsed = tc->COUNT32.COUNT.reg;
            break;
    }
#else
    #error TC sync needs to be implemented
#endif

    NVIC_DisableIRQ((IRQn_Type)this->irqN);
    return elapsed;
}

int SAMDTCTimer::setPrescaler(uint16_t prescaleValue)
{
    if (prescaleValue > 0x7)
        return DEVICE_INVALID_PARAMETER;

    switch (bm)
    {
        case BitMode8:
            tc->COUNT8.CTRLA.bit.PRESCALER = prescaleValue;
            break;
        case BitMode16:
            tc->COUNT16.CTRLA.bit.PRESCALER = prescaleValue;
            break;
        case BitMode24:
            // make the compiler shut up
            break;
        case BitMode32:
            tc->COUNT32.CTRLA.bit.PRESCALER = prescaleValue;
            break;
    }

    return DEVICE_OK;
}

int SAMDTCTimer::setBitMode(TimerBitMode t)
{
    switch (t)
    {
        case BitMode8:
            tc->COUNT8.CTRLA.bit.MODE = 0x1; // 8 bit operation
            break;
        case BitMode16:
            tc->COUNT16.CTRLA.bit.MODE = 0x0; // 16 bit operation
            break;
        case BitMode24:
            return DEVICE_INVALID_PARAMETER;
        case BitMode32:
            tc->COUNT32.CTRLA.bit.MODE = 0x2; // 32 bit operation
            break;
    }

    bm = t;

    return DEVICE_OK;
}