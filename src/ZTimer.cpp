#include "ZTimer.h"
#include "CodalCompat.h"
#include "CodalConfig.h"
#include "codal_target_hal.h"

#include "CodalDmesg.h"


// TODO get rid of division on SAMD21 - it's slow


namespace codal
{

// Global millisecond tick count
static volatile uint64_t ticks_ms = 0;
static volatile uint64_t nextTrigger = 0;
ZTimer *ZTimer::instance;

extern "C" void SysTick_Handler(void) {
    int shouldTrigger = 0;

    // SysTick interrupt handler called when the SysTick timer reaches zero
    // (every millisecond).
    target_disable_irq();
    ticks_ms += 1;

    // Read the control register to reset the COUNTFLAG.
    (void) SysTick->CTRL;

    if (nextTrigger && ticks_ms >= nextTrigger) {
        nextTrigger = 0;
        shouldTrigger = 1;
    }
    target_enable_irq();

    if (shouldTrigger)
        ZTimer::instance->trigger();
}

static void tick_init() {
    if (ticks_ms)
        return;
    ticks_ms = 1;
    uint32_t ticks_per_ms = CODAL_CPU_MHZ * 1000;
    SysTick_Config(ticks_per_ms-1);
    NVIC_EnableIRQ(SysTick_IRQn);
    // Set all peripheral interrupt priorities to the lowest priority by default.
    for (uint16_t i = 0; i < PERIPH_COUNT_IRQn; i++) {
        NVIC_SetPriority((IRQn_Type)i, (1UL << __NVIC_PRIO_BITS) - 1UL);
    }
    // Bump up the systick interrupt so nothing else interferes with timekeeping.
    NVIC_SetPriority(SysTick_IRQn, 0);
    #ifdef SAMD21
    NVIC_SetPriority(USB_IRQn, 1);
    #endif

    #ifdef SAMD51
    NVIC_SetPriority(USB_0_IRQn, 1);
    NVIC_SetPriority(USB_1_IRQn, 1);
    NVIC_SetPriority(USB_2_IRQn, 1);
    NVIC_SetPriority(USB_3_IRQn, 1);
    #endif
}

void target_wait_us(uint32_t us) {
    tick_init();

    uint32_t ticks_per_us = CODAL_CPU_MHZ;
    uint32_t us_until_next_tick = SysTick->VAL / ticks_per_us;
    uint32_t start_tick;
    while (us >= us_until_next_tick) {
        start_tick = SysTick->VAL;  // wait for SysTick->VAL to  RESET
        while (SysTick->VAL < start_tick) {}
        us -= us_until_next_tick;
        us_until_next_tick = 1000;
    }
    while (SysTick->VAL > ((us_until_next_tick - us) * ticks_per_us)) {}
}

ZTimer::ZTimer() : codal::Timer()
{
    instance = this;
}

void ZTimer::init()
{
    tick_init();
}

void ZTimer::triggerIn(CODAL_TIMESTAMP t)
{
    target_disable_irq();
    nextTrigger = ticks_ms + t / 1000;
    target_enable_irq();
}

void ZTimer::syncRequest()
{
    target_disable_irq();

    uint32_t ticks_per_us = CODAL_CPU_MHZ;

    uint32_t tick_status = SysTick->CTRL;
    uint32_t current_us = SysTick->VAL;
    uint32_t tick_status2 = SysTick->CTRL;
    uint64_t current_ms = ticks_ms;
    // The second clause ensures our value actually rolled over. Its possible it hit zero between
    // the VAL read and CTRL read.
    if ((tick_status & SysTick_CTRL_COUNTFLAG_Msk) != 0 ||
        ((tick_status2 & SysTick_CTRL_COUNTFLAG_Msk) != 0 && current_us > ticks_per_us)) {
        current_ms++;
    }

    currentTime = current_ms;
    currentTimeUs = current_ms * 1000 + (1000 - (current_us / ticks_per_us));

    target_enable_irq();
}

} // namespace codal
