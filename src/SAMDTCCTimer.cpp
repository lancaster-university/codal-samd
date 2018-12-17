#include "SAMDTCCTimer.h"
#include "CodalDmesg.h"
#include "sam.h"
extern "C"
{
    #include "clocks.h"
    #include "timers.h"
}


#define PRESCALE_VALUE_MAX          8

const static uint16_t prescalerDivison[PRESCALE_VALUE_MAX] = { 1, 2, 4, 8, 16, 64, 256, 1024};

using namespace codal;

static SAMDTCCTimer *instances[TCC_INST_NUM] = { 0 };

void tcc_irq_handler(uint8_t index)
{
    if (instances[index] == NULL)
        return;

    uint16_t channel_bitmsk = 0;

    if (instances[index]->timer_pointer)
    {
        if (instances[index]->tcc->INTFLAG.bit.MC0 && instances[index]->tcc->INTENSET.reg & (1 << TCC_INTENSET_MC0_Pos))
        {
            instances[index]->tcc->INTFLAG.bit.MC0 = 1;
            channel_bitmsk |= (1 << 0);
        }

        if (instances[index]->tcc->INTFLAG.bit.MC1 && instances[index]->tcc->INTENSET.reg & (1 << TCC_INTENSET_MC1_Pos))
        {
            instances[index]->tcc->INTFLAG.bit.MC1 = 1;
            channel_bitmsk |= (1 << 1);
        }

        if (instances[index]->tcc->INTFLAG.bit.MC2 && instances[index]->tcc->INTENSET.reg & (1 << TCC_INTENSET_MC2_Pos))
        {
            instances[index]->tcc->INTFLAG.bit.MC2 = 1;
            channel_bitmsk |= (1 << 2);
        }

        if (instances[index]->tcc->INTFLAG.bit.MC3 && instances[index]->tcc->INTENSET.reg & (1 << TCC_INTENSET_MC3_Pos))
        {
            instances[index]->tcc->INTFLAG.bit.MC3 = 1;
            channel_bitmsk |= (1 << 3);
        }

        instances[index]->timer_pointer(channel_bitmsk);
    }
}


SAMDTCCTimer::SAMDTCCTimer(Tcc* tcc, uint8_t irqn) : LowLevelTimer(4)
{
    // TODO: neaten up constructor, look up tcc and irqn given tcc number.
    this->tcc= tcc;
    this->irqN = irqn;

    // 48MHz / 6 == 8MhZ
    enable_clock_generator(CLK_GEN_8MHZ, CLOCK_48MHZ, 6);

    // find the tx index in the insts array
    uint8_t tcc_index = 0;
    while (tcc_index < TCC_INST_NUM)
    {
        if (tcc_insts[tcc_index] == this->tcc)
            break;
        tcc_index++;
    }

    CODAL_ASSERT(tcc_index < TCC_INST_NUM);
    // we should do a singleton here...
    CODAL_ASSERT(instances[tcc_index] == NULL);

    DMESG("tcc_ind: %d clk_index: %d", tcc_index, CLK_GEN_8MHZ);

    // configure the clks for the current timer.
    turn_on_clocks(false, tcc_index, CLK_GEN_8MHZ);

    // disable the timer
    disable();

    bool inited = false;

    for (int index = 0; index < TCC_INST_NUM; index++)
        if (instances[index])
            inited = true;

    if (!inited)
    {
        DMESG("SET APP");
        tcc_set_app_handler(tcc_irq_handler);
    }

    instances[tcc_index] = this;

    setBitMode(BitMode24);

    tcc->CTRLBCLR.bit.DIR = 1; // count up
#ifdef SAMD21
    while (tcc->SYNCBUSY.bit.CTRLB);
#endif

    // 1000 khz == 1 mhz
    setClockSpeed(1000);
}

int SAMDTCCTimer::enable()
{
    NVIC_SetPriority((IRQn_Type)this->irqN, 2);
    NVIC_ClearPendingIRQ((IRQn_Type)this->irqN);
    enableIRQ();
    tcc_set_enable(tcc, true);
    return DEVICE_OK;
}

int SAMDTCCTimer::enableIRQ()
{
    NVIC_EnableIRQ((IRQn_Type)this->irqN);
    return DEVICE_OK;
}

int SAMDTCCTimer::disable()
{
    disableIRQ();
    tcc_set_enable(tcc, false);
    return DEVICE_OK;
}

int SAMDTCCTimer::disableIRQ()
{
    NVIC_DisableIRQ((IRQn_Type)this->irqN);
    return DEVICE_OK;
}

int SAMDTCCTimer::reset()
{
    NVIC_DisableIRQ((IRQn_Type)this->irqN);

    tcc->COUNT.reg = 0;
#ifdef SAMD21
    while (tcc->SYNCBUSY.bit.COUNT);
#endif

    NVIC_EnableIRQ((IRQn_Type)this->irqN);
    return DEVICE_OK;
}

int SAMDTCCTimer::setMode(TimerMode t)
{
    // only support timer mode.
    return DEVICE_OK;
}

int SAMDTCCTimer::setCompare(uint8_t channel, uint32_t value)
{
    if (channel > getChannelCount())
        return DEVICE_INVALID_PARAMETER;

    tcc->CC[channel].reg = value;
#ifdef SAMD21
    switch (channel)
    {
        case 0:
            while (tcc->SYNCBUSY.bit.CC0);
            break;
        case 1:
            while (tcc->SYNCBUSY.bit.CC1);
            break;
        case 2:
            while (tcc->SYNCBUSY.bit.CC2);
            break;
        case 3:
            while (tcc->SYNCBUSY.bit.CC3);
            break;
    }
#endif

    // add channel to MC0, MC0 is 4, MC1 is 5
    tcc->INTENSET.reg = (1 << (TCC_INTENSET_MC0_Pos + channel));
    return DEVICE_OK;
}

int SAMDTCCTimer::offsetCompare(uint8_t channel, uint32_t value)
{
    if (channel > getChannelCount())
        return DEVICE_INVALID_PARAMETER;

    tcc->CC[channel].reg += value;

#ifdef SAMD21
    switch (channel)
    {
        case 0:
            while (tcc->SYNCBUSY.bit.CC0);
            break;
        case 1:
            while (tcc->SYNCBUSY.bit.CC1);
            break;
        case 2:
            while (tcc->SYNCBUSY.bit.CC2);
            break;
        case 3:
            while (tcc->SYNCBUSY.bit.CC3);
            break;
    }
#endif

    // add channel to MC0, MC0 is 4, MC1 is 5
    tcc->INTENSET.reg = (1 << (TCC_INTENSET_MC0_Pos + channel));

    return DEVICE_OK;
}

int SAMDTCCTimer::clearCompare(uint8_t channel)
{
    if (channel > getChannelCount())
        return DEVICE_INVALID_PARAMETER;

    // add channel to MC0, MC0 is 4, MC1 is 5
    tcc->INTENCLR.reg = (1 << (TCC_INTENCLR_MC0_Pos + channel));
    setCompare(channel, 0);

    return DEVICE_OK;
}

uint32_t SAMDTCCTimer::captureCounter()
{
    uint32_t elapsed = 0;

    NVIC_DisableIRQ((IRQn_Type)this->irqN);
#ifdef SAMD51
    #warning "not sure if this works on samd 51"
    tcc->CTRLBSET.bit.CMD = 0x04;
    while (tcc->SYNCBUSY.bit.CTRLB);
    elapsed = tcc->COUNT.reg;
#elif SAMD21
    tcc->CTRLBSET.bit.CMD = 0x04;
    while (tcc->SYNCBUSY.bit.CTRLB);
    elapsed = tcc->COUNT.reg;
#else
    #error TC sync needs to be implemented
#endif

    NVIC_DisableIRQ((IRQn_Type)this->irqN);
    return elapsed;
}

int SAMDTCCTimer::setClockSpeed(uint32_t speedKHz)
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

    tcc->CTRLA.bit.PRESCALER = prescaleValue;

    return DEVICE_OK;
}

int SAMDTCCTimer::setBitMode(TimerBitMode t)
{
    bitMode = t;
    return DEVICE_NOT_IMPLEMENTED;
}