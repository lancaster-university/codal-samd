#include "SAMDTCCTimer.h"


static SAMDTCTimer *instances[TC_INST_NUM] = { 0 };

void tc_irq_handler(uint8_t index)
{
    if (instances[i] == NULL)
        return;

    if (instances[i]->timer_pointer)
    {
        if (instances[i]->tc->COUNT32.INTFLAG.bit.MC0 && instances[i]->tc->COUNT32.INTENSET.reg & (1 << TC_INTENSET_MC0_Pos))
        {
            SAMDTimer::instance->tc->COUNT32.INTFLAG.bit.MC0 = 1;
            instances[i]->timer_pointer(0);
        }

        if (instances[i]->tc->COUNT32.INTFLAG.bit.MC1 && instances[i]->tc->COUNT32.INTENSET.reg & (1 << TC_INTENSET_MC1_Pos))
        {
            instances[i]->tc->COUNT32.INTFLAG.bit.MC1 = 1;
            instances[i]->timer_pointer(1);
        }
    }
}


SAMDTCTimer::SAMDTCTimer(Tcc* tcc, uint8_t irqn) : LowLevelTimer(2)
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
    DMESG("tc_ind: %d clk_index: %d", tc_index, CLK_GEN_8MHZ);

    // configure the clks for the current timer.
    turn_on_clocks(true, tc_index, CLK_GEN_8MHZ);

    // disable the timer
    disable();
    tc_set_app_handler(tc_irq_handler);
    setBitMode(BitMode32);

    // configure
    counterPtr->CTRLBCLR.bit.DIR = 1; // count up
#ifdef SAMD21
    while (counterPtr->STATUS.bit.SYNCBUSY);
#endif

    setPrescaler(3);
}

int SAMDTCTimer::enable()
{
    NVIC_SetPriority((IRQn_Type)this->irqN, 2);
    NVIC_ClearPendingIRQ((IRQn_Type)this->irqN);
    NVIC_EnableIRQ((IRQn_Type)this->irqN);
    tc_set_enable(tc, true);
}

int SAMDTCTimer::disable()
{
    NVIC_DisableIRQ((IRQn_Type)this->irqN);
    tc_set_enable(tc, false);
}

int SAMDTCTimer::reset()
{
    NVIC_DisableIRQ((IRQn_Type)this->irqN);
    counterPtr->COUNT.reg = 0;
#ifdef SAMD21
    while (counterPtr->STATUS.bit.SYNCBUSY);
#endif
    NVIC_EnableIRQ((IRQn_Type)this->irqN);
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

    counterPtr->CC[channel].reg = value;
#ifdef SAMD21
    while (tc->COUNT32.STATUS.bit.SYNCBUSY);
#endif

    // add channel to MC0, MC0 is 4, MC1 is 5
    counterPtr->INTENSET.reg = (1 << (TC_INTENSET_MC0_Pos + channel));
}

int SAMDTCTimer::offsetCompare(uint8_t channel, uint32_t value)
{
    if (channel > getChannelCount())
        return DEVICE_INVALID_PARAMETER;

    counterPtr->CC[channel].reg += value;
#ifdef SAMD21
    while (tc->COUNT32.STATUS.bit.SYNCBUSY);
#endif
}

int SAMDTCTimer::clearCompare(uint8_t channel)
{
    if (channel > getChannelCount())
        return DEVICE_INVALID_PARAMETER;

    counterPtr->CC[channel].reg += value;
#ifdef SAMD21
    while (tc->COUNT32.STATUS.bit.SYNCBUSY);
#endif

    // add channel to MC0, MC0 is 4, MC1 is 5
    counterPtr->INTENCLR.reg = (1 << (TC_INTENCLR_MC0_Pos + channel));
}

uint32_t SAMDTCTimer::captureCounter(uint8_t channel)
{
    (uint8_t)channel;

    NVIC_DisableIRQ((IRQn_Type)this->irqN);
#ifdef SAMD51
    counterPtr->CTRLBSET.bit.CMD = 0x04;
    while (counterPtr->SYNCBUSY.bit.CTRLB);
#elif SAMD21
    counterPtr->READREQ.bit.ADDR = 0x10;
    counterPtr->READREQ.bit.RREQ = 1;
    while (counterPtr->STATUS.bit.SYNCBUSY);
#else
    #error TC sync needs to be implemented
#endif

    uint32_t elapsed = counterPtr->COUNT.reg;
    NVIC_DisableIRQ((IRQn_Type)this->irqN);
    return elapsed;
}

int SAMDTCTimer::setPrescaler(uint16_t prescaleValue)
{
    if (prescaleValue > 0x7)
        return DEVICE_INVALID_PARAMETER;

    counterPtr->CTRLA.bit.PRESCALER = prescaleValue;

    return DEVICE_OK;
}

int SAMDTCTimer::setBitMode(TimerBitMode t)
{
    if (t == BitMode8)
    {
        countPtr = &tc->COUNT8;
        tc->COUNT8.CTRLA.bit.MODE = 0x1; // 8 bit operation
    }

    if (t == BitMode16)
    {
        counterPtr = &tc->COUNT16;
        tc->COUNT16.CTRLA.bit.MODE = 0x0; // 16 bit operation
    }

    if (t == BitMode24)
        return DEVICE_INVALID_PARAMETER;

    if (t == BitMode32)
    {
        counterPtr = &tc->COUNT32;
        tc->COUNT32.CTRLA.bit.MODE = 0x2; // 32 bit operation
    }

    bm = t;

    return DEVICE_OK;
}