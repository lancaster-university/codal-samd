#include "stm32.h"
#include "codal_target_hal.h"
#include "CodalDmesg.h"
#include "CodalCompat.h"

void target_enable_irq()
{
    __enable_irq();
}

void target_disable_irq()
{
    __disable_irq();
}

void target_wait_for_event()
{
    __WFE();
}

void target_wait(uint32_t milliseconds)
{
    target_wait_us(milliseconds * 1000);
}

int target_seed_random(uint32_t rand)
{
    return codal::seed_random(rand);
}

int target_random(int max)
{
    return codal::random(max);
}

void target_reset()
{
    // TODO skip bootloader
    NVIC_SystemReset();
}

extern "C" void assert_failed(uint8_t* file, uint32_t line)
{
    target_panic(920);
}

void target_panic(int statusCode)
{
    target_disable_irq();

    DMESG("*** CODAL PANIC : [%d]", statusCode);
    while (1)
    {
    }
}

void target_init()
{
    // TODO
}

/**
 *  Thread Context for an ARM Cortex core.
 *
 * This is probably overkill, but the ARMCC compiler uses a lot register optimisation
 * in its calling conventions, so better safe than sorry!
 */
struct PROCESSOR_TCB
{
    uint32_t R0;
    uint32_t R1;
    uint32_t R2;
    uint32_t R3;
    uint32_t R4;
    uint32_t R5;
    uint32_t R6;
    uint32_t R7;
    uint32_t R8;
    uint32_t R9;
    uint32_t R10;
    uint32_t R11;
    uint32_t R12;
    uint32_t SP;
    uint32_t LR;
    uint32_t stack_base;
};

PROCESSOR_WORD_TYPE fiber_initial_stack_base()
{
    uint32_t mbed_stack_base;

    mbed_stack_base = DEVICE_STACK_BASE;

    return mbed_stack_base;
}

void *tcb_allocate()
{
    return (void *)malloc(sizeof(PROCESSOR_TCB));
}

/**
 * Configures the link register of the given tcb to have the value function.
 *
 * @param tcb The tcb to modify
 * @param function the function the link register should point to.
 */
void tcb_configure_lr(void *tcb, PROCESSOR_WORD_TYPE function)
{
    PROCESSOR_TCB *tcbPointer = (PROCESSOR_TCB *)tcb;
    tcbPointer->LR = function;
}

/**
 * Configures the link register of the given tcb to have the value function.
 *
 * @param tcb The tcb to modify
 * @param function the function the link register should point to.
 */
void tcb_configure_sp(void *tcb, PROCESSOR_WORD_TYPE sp)
{
    PROCESSOR_TCB *tcbPointer = (PROCESSOR_TCB *)tcb;
    tcbPointer->SP = sp;
}

void tcb_configure_stack_base(void *tcb, PROCESSOR_WORD_TYPE stack_base)
{
    PROCESSOR_TCB *tcbPointer = (PROCESSOR_TCB *)tcb;
    tcbPointer->stack_base = stack_base;
}

PROCESSOR_WORD_TYPE tcb_get_stack_base(void *tcb)
{
    PROCESSOR_TCB *tcbPointer = (PROCESSOR_TCB *)tcb;
    return tcbPointer->stack_base;
}

PROCESSOR_WORD_TYPE get_current_sp()
{
    return __get_MSP();
}

PROCESSOR_WORD_TYPE tcb_get_sp(void *tcb)
{
    PROCESSOR_TCB *tcbPointer = (PROCESSOR_TCB *)tcb;
    return tcbPointer->SP;
}

void tcb_configure_args(void *tcb, PROCESSOR_WORD_TYPE ep, PROCESSOR_WORD_TYPE cp,
                        PROCESSOR_WORD_TYPE pm)
{
    PROCESSOR_TCB *tcbPointer = (PROCESSOR_TCB *)tcb;
    tcbPointer->R0 = (uint32_t)ep;
    tcbPointer->R1 = (uint32_t)cp;
    tcbPointer->R2 = (uint32_t)pm;
}

extern PROCESSOR_WORD_TYPE __end__;
PROCESSOR_WORD_TYPE codal_heap_start = (PROCESSOR_WORD_TYPE)(&__end__);
