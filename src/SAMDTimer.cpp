#include "SAMDTimer.h"
extern "C"
{
    #include "clocks.h"
    #include "timers.h"
}
#include "CodalDmesg.h"
#include "codal_target_hal.h"
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
}

void SAMDTimer::enable()
{
    // find the first available clock and configure for one mhz generation
    uint8_t clk_index = find_free_gclk(48);

    // 48MHz / 48 == 1MhZ
    enable_clock_generator(clk_index, CLOCK_48MHZ, 48);

    // find the tx index in the insts array
    uint8_t tc_index = 0;
    while (tc_index < TC_INST_NUM)
    {
        if (tc_insts[tc_index] == this->tc)
            break;
        tc_index++;
    }

    CODAL_ASSERT(tc_index < TC_INST_NUM);

    DMESG("tc_ind: %d clk_index: %d",tc_index, clk_index);

    // configure the clks for the current timer.
    turn_on_clocks(true, tc_index, clk_index);

    // disable the timer
    tc_set_enable(tc, false);

    tc_set_app_handler(tc_irq_handler);

    // configure
    tc->COUNT32.CTRLA.bit.MODE = 0x2; // 32 bit operation

    tc->COUNT32.CTRLBCLR.bit.DIR = 1; // count up
#ifdef SAMD21
    while (tc->COUNT32.STATUS.bit.SYNCBUSY);
#endif

    tc->COUNT32.CTRLA.bit.PRESCALER = 0;

    // configure our well defined period for definitive interrupts.
    tc->COUNT32.CC[0].reg = this->period;
#ifdef SAMD21
    while (tc->COUNT32.STATUS.bit.SYNCBUSY);
#endif

    // re-enable
    // enable compare for channels 0
    tc->COUNT32.INTENSET.reg = (1 << TC_INTENSET_MC0_Pos);
    tc->COUNT32.INTENSET.reg = (1 << TC_INTENSET_MC1_Pos);

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
#ifdef SAMD21
    while (tc->COUNT32.STATUS.bit.SYNCBUSY);
#endif

    sigma = 0;

    tc->COUNT32.COUNT.reg = 0;
#ifdef SAMD21
    while (tc->COUNT32.STATUS.bit.SYNCBUSY);
#endif

    NVIC_EnableIRQ((IRQn_Type)this->irqN);
}

void SAMDTimer::syncRequest()
{
    NVIC_DisableIRQ((IRQn_Type)this->irqN);

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
    NVIC_EnableIRQ((IRQn_Type)this->irqN);
}
