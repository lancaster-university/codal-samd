#include "SAMDTimer.h"
extern "C"
{
    #include "clocks.h"
    #include "timers.h"
}
#include "CodalDmesg.h"

using namespace codal;

#define MINIMUM_PERIOD      1

SAMDTimer *SAMDTimer::instance = NULL;

void tc_irq_handler(uint8_t index)
{
    bool isFallback = false;

    if (SAMDTimer::instance->tc->COUNT32.INTFLAG.bit.MC0 && SAMDTimer::instance->tc->COUNT32.INTENSET.reg & (1 << TC_INTENSET_MC0_Pos))
    {
        isFallback = true;
        SAMDTimer::instance->tc->COUNT32.INTFLAG.bit.MC0 = 1;
    }

    if (SAMDTimer::instance->tc->COUNT32.INTFLAG.bit.MC1 && SAMDTimer::instance->tc->COUNT32.INTENSET.reg & (1 << TC_INTENSET_MC1_Pos))
        SAMDTimer::instance->tc->COUNT32.INTFLAG.bit.MC1 = 1;

    if (SAMDTimer::instance)
    {
        SAMDTimer::instance->syncRequest();

        if (isFallback)
        {
            SAMDTimer::instance->sigma = 0;
            SAMDTimer::instance->tc->COUNT32.COUNT.reg = 0;
        }

        SAMDTimer::instance->trigger();
    }
}

SAMDTimer::SAMDTimer(Tc* tc, uint8_t irqn)
{
    // TODO: neaten up constructor, look up tc and irqn given tc number.
    this->tc= tc;
    this->irqN = irqn;
    this->period = 10000000; // 10s fallback timer
    this->sigma = 0;

    instance = this;

    // find the first available clock and configure for one mhz generation
    uint8_t clk_index = 3;
    while (gclk_enabled(clk_index))
        clk_index++;

    // 48MHz / 48 == 1MhZ
    enable_clock_generator(clk_index, CLOCK_48MHZ, 48);


    // configure the clks for the current timer.
    turn_on_clocks(true, 0, clk_index);

    // disable the timer
    tc_set_enable(tc, false);

    tc_set_app_handler(tc_irq_handler);

    // configure
    tc->COUNT32.CTRLA.bit.MODE = 0x2; // 32 bit operation
    tc->COUNT32.CTRLBCLR.bit.DIR = 1; // count up

    tc->COUNT32.CTRLA.bit.PRESCALER = 0;

    // configure our well defined period for definitive interrupts.
    tc->COUNT32.CC[0].reg = this->period;

    // re-enable
    // enable compare for channels 0
    tc->COUNT32.INTENSET.reg = (1 << TC_INTENSET_MC0_Pos);

    NVIC_SetPriority((IRQn_Type)this->irqN, 2);
    NVIC_ClearPendingIRQ((IRQn_Type)this->irqN);
    NVIC_EnableIRQ((IRQn_Type)this->irqN);

    tc_set_enable(tc, true);
}

void SAMDTimer::triggerIn(CODAL_TIMESTAMP t)
{
    if (t < MINIMUM_PERIOD)
        t = MINIMUM_PERIOD;

    NVIC_DisableIRQ((IRQn_Type)this->irqN);

    tc->COUNT32.CC[1].reg = t;
    // intenset mc1
    tc->COUNT32.INTENSET.reg = (1 << TC_INTENSET_MC1_Pos);
    sigma = 0;
    tc->COUNT32.COUNT.reg = 0;
    // we may need to sync here
    NVIC_EnableIRQ((IRQn_Type)this->irqN);
}

void SAMDTimer::syncRequest()
{
    __disable_irq();

#ifdef SAMD51
    tc->COUNT32.CTRLBSET.bit.CMD = 0x04;
    while (tc->COUNT32.SYNCBUSY.bit.CTRLB);
#elif SAMD21
    tc->COUNT32.READREQ.bit.RREQ = 1;
    while (tc->COUNT32.STATUS.bit.SYNCBUSY);
#else
    #error TC sync needs to be implemented
#endif

    uint32_t snapshot = tc->COUNT32.COUNT.reg;
    uint32_t elapsed = snapshot - sigma;
    sigma = snapshot;
    this->sync(elapsed);
    __enable_irq();
}