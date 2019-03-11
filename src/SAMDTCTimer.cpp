#include "SAMDTCTimer.h"
#include "CodalDmesg.h"

#ifdef SAMD21
    #define SAMD_SYNC_BUSY_8() while (tc->COUNT8.STATUS.bit.SYNCBUSY)
    #define SAMD_SYNC_BUSY_16() while (tc->COUNT16.STATUS.bit.SYNCBUSY)
    #define SAMD_SYNC_BUSY_32() while (tc->COUNT32.STATUS.bit.SYNCBUSY)
#else
    #define SAMD_SYNC_BUSY_8() ((void)0)
    #define SAMD_SYNC_BUSY_16() ((void)0)
    #define SAMD_SYNC_BUSY_32() ((void)0)
#endif

#define PRESCALE_VALUE_MAX          8

const static uint16_t prescalerDivison[PRESCALE_VALUE_MAX] = { 1, 2, 4, 8, 16, 64, 256, 1024};

using namespace codal;

static SAMDTCTimer *instances[TC_INST_NUM] = { 0 };

void tc_irq_handler(uint8_t index)
{
    if (instances[index] == NULL)
        return;

    uint16_t channel_bitmsk = 0;

    if (instances[index]->timer_pointer)
    {
        switch (instances[index]->getBitMode())
        {
            case BitMode8:
                if (instances[index]->tc->COUNT8.INTFLAG.bit.MC0 && instances[index]->tc->COUNT8.INTENSET.reg & (1 << TC_INTENSET_MC0_Pos))
                {
                    instances[index]->tc->COUNT8.INTFLAG.bit.MC0 = 1;
                    channel_bitmsk |= (1 << 0);
                }

                if (instances[index]->tc->COUNT8.INTFLAG.bit.MC1 && instances[index]->tc->COUNT8.INTENSET.reg & (1 << TC_INTENSET_MC1_Pos))
                {
                    instances[index]->tc->COUNT8.INTFLAG.bit.MC1 = 1;
                    channel_bitmsk |= (1 << 1);
                }
                break;
            case BitMode16:
                if (instances[index]->tc->COUNT16.INTFLAG.bit.MC0 && instances[index]->tc->COUNT16.INTENSET.reg & (1 << TC_INTENSET_MC0_Pos))
                {
                    instances[index]->tc->COUNT16.INTFLAG.bit.MC0 = 1;
                    channel_bitmsk |= (1 << 0);
                }

                if (instances[index]->tc->COUNT16.INTFLAG.bit.MC1 && instances[index]->tc->COUNT16.INTENSET.reg & (1 << TC_INTENSET_MC1_Pos))
                {
                    instances[index]->tc->COUNT16.INTFLAG.bit.MC1 = 1;
                    channel_bitmsk |= (1 << 1);
                }
                break;
            case BitMode24:
                // Take a break compiler...
                break;
            case BitMode32:
                if (instances[index]->tc->COUNT32.INTFLAG.bit.MC0 && instances[index]->tc->COUNT32.INTENSET.reg & (1 << TC_INTENSET_MC0_Pos))
                {
                    instances[index]->tc->COUNT32.INTFLAG.bit.MC0 = 1;
                    channel_bitmsk |= (1 << 0);
                }

                if (instances[index]->tc->COUNT32.INTFLAG.bit.MC1 && instances[index]->tc->COUNT32.INTENSET.reg & (1 << TC_INTENSET_MC1_Pos))
                {
                    instances[index]->tc->COUNT32.INTFLAG.bit.MC1 = 1;
                    channel_bitmsk |= (1 << 1);
                }
                break;
        }

        instances[index]->timer_pointer(channel_bitmsk);
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
    // we should do a singleton here...
    CODAL_ASSERT(instances[tc_index] == NULL);

    DMESG("tc_ind: %d clk_index: %d", tc_index, CLK_GEN_8MHZ);

    // 32 bit mode does not work, but don't have the time to figure out why
    // I think i've tracked it down to TC1 (slave to TC0) not being enabled.
    // configure the clks for the current timer.
    turn_on_clocks(true, tc_index, CLK_GEN_8MHZ);

    // disable the timer
    disable();
    tc_reset(tc);

    bool inited = false;

    for (int index = 0; index < TC_INST_NUM; index++)
        if (instances[index])
            inited = true;

    if (!inited)
        tc_set_app_handler(tc_irq_handler);

    instances[tc_index] = this;

    setBitMode(BitMode16);

    // configure
    switch (bitMode)
    {
        case BitMode8:
            tc->COUNT8.CTRLBCLR.bit.DIR = 1; // count up
            SAMD_SYNC_BUSY_8();
            break;
        case BitMode16:
            tc->COUNT16.CTRLBCLR.bit.DIR = 1; // count up
            SAMD_SYNC_BUSY_16();
            break;
        case BitMode24:
            // make the compiler shut up
            break;
        case BitMode32:
            tc->COUNT32.CTRLBCLR.bit.DIR = 1; // count up
            SAMD_SYNC_BUSY_32();
            break;
    }

    // 1000 khz == 1 mhz
    setClockSpeed(1000);

    setIRQPriority(2);
}

int SAMDTCCTimer::setIRQPriority(int priority)
{
    NVIC_SetPriority((IRQn_Type)this->irqN, priority);
    return DEVICE_OK;
}

int SAMDTCTimer::enable()
{
    NVIC_ClearPendingIRQ((IRQn_Type)this->irqN);
    enableIRQ();
    tc_set_enable(tc, true);
    return DEVICE_OK;
}

int SAMDTCTimer::enableIRQ()
{
    NVIC_EnableIRQ((IRQn_Type)this->irqN);
    return DEVICE_OK;
}

int SAMDTCTimer::disable()
{
    disableIRQ();
    tc_set_enable(tc, false);
    return DEVICE_OK;
}

int SAMDTCTimer::disableIRQ()
{
    NVIC_DisableIRQ((IRQn_Type)this->irqN);
    return DEVICE_OK;
}

int SAMDTCTimer::reset()
{
    NVIC_DisableIRQ((IRQn_Type)this->irqN);
    switch (bitMode)
    {
        case BitMode8:
            tc->COUNT8.COUNT.reg = 0;
            SAMD_SYNC_BUSY_8();
            break;
        case BitMode16:
            tc->COUNT16.COUNT.reg = 0;
            SAMD_SYNC_BUSY_16();
            break;
        case BitMode24:
            // make the compiler shut up
            break;
        case BitMode32:
            tc->COUNT32.COUNT.reg = 0;
            SAMD_SYNC_BUSY_32();
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

    switch (bitMode)
    {
        case BitMode8:
            tc->COUNT8.CC[channel].reg = value;
            SAMD_SYNC_BUSY_8();

            // add channel to MC0, MC0 is 4, MC1 is 5
            tc->COUNT8.INTENSET.reg = (1 << (TC_INTENSET_MC0_Pos + channel));
            break;
        case BitMode16:
            tc->COUNT16.CC[channel].reg = value;
            SAMD_SYNC_BUSY_16();

            // add channel to MC0, MC0 is 4, MC1 is 5
            tc->COUNT16.INTENSET.reg = (1 << (TC_INTENSET_MC0_Pos + channel));
            break;
        case BitMode24:
            // make the compiler shut up
            break;
        case BitMode32:
            tc->COUNT32.CC[channel].reg = value;
            SAMD_SYNC_BUSY_32();

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

    switch (bitMode)
    {
        case BitMode8:
            tc->COUNT8.CC[channel].reg += value;
            SAMD_SYNC_BUSY_8();

            // add channel to MC0, MC0 is 4, MC1 is 5
            tc->COUNT8.INTENSET.reg = (1 << (TC_INTENSET_MC0_Pos + channel));
            break;
        case BitMode16:
            tc->COUNT16.CC[channel].reg += value;
            SAMD_SYNC_BUSY_16();

            // add channel to MC0, MC0 is 4, MC1 is 5
            tc->COUNT16.INTENSET.reg = (1 << (TC_INTENSET_MC0_Pos + channel));
            break;
        case BitMode24:
            // make the compiler shut up
            break;
        case BitMode32:
            tc->COUNT32.CC[channel].reg += value;
            SAMD_SYNC_BUSY_32();

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

    setCompare(channel, 0);

    switch (bitMode)
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

uint32_t SAMDTCTimer::captureCounter()
{
    uint32_t elapsed = 0;

    NVIC_DisableIRQ((IRQn_Type)this->irqN);
#ifdef SAMD51
    switch (bitMode)
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

    switch (bitMode)
    {
        case BitMode8:
            tc->COUNT8.READREQ.bit.ADDR = 0x10;
            tc->COUNT8.READREQ.bit.RREQ = 1;
            SAMD_SYNC_BUSY_8();
            elapsed = tc->COUNT8.COUNT.reg;
            break;
        case BitMode16:
            tc->COUNT16.READREQ.bit.ADDR = 0x10;
            tc->COUNT16.READREQ.bit.RREQ = 1;
            SAMD_SYNC_BUSY_16();
            elapsed = tc->COUNT16.COUNT.reg;
            break;
        case BitMode24:
            // make the compiler shut up
            break;
        case BitMode32:
            tc->COUNT32.READREQ.bit.ADDR = 0x10;
            tc->COUNT32.READREQ.bit.RREQ = 1;
            SAMD_SYNC_BUSY_32();
            elapsed = tc->COUNT32.COUNT.reg;
            break;
    }
#else
    #error TC sync needs to be implemented
#endif

    NVIC_EnableIRQ((IRQn_Type)this->irqN);
    return elapsed;
}

int SAMDTCTimer::setClockSpeed(uint32_t speedKHz)
{
    // 8000 khz
    // TODO: Reconfigure clocks if resolution is greater than 8khz
    if (speedKHz > 8000)
        return DEVICE_INVALID_PARAMETER;

    // clock is 8khz
    uint32_t clockSpeed = 8000;
    uint8_t prescaleValue = 0;

    // snap to the lowest
    for (prescaleValue = 0; prescaleValue < PRESCALE_VALUE_MAX; prescaleValue++)
    {
        if (speedKHz < (clockSpeed / prescalerDivison[prescaleValue]))
            continue;

        break;
    }

    // set prescaler
    switch (bitMode)
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

    bitMode = t;

    return DEVICE_OK;
}