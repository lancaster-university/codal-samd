#ifndef DEVICE_FIBER_IMP_H
#define DEVICE_FIBER_IMP_H

#include "device_types.h"
#include "mbed.h"

/**
  *  Thread Context for an ARM Cortex M0 core.
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

inline PROCESSOR_WORD_TYPE fiber_initial_stack_base()
{
    uint32_t mbed_stack_base;

#ifdef MBED_CONF_RTOS_PRESENT
    extern osThreadDef_t os_thread_def_main;
    mbed_stack_base = (uint32_t)os_thread_def_main.stack_pointer + os_thread_def_main.stacksize;
#else
    mbed_stack_base = DEVICE_STACK_BASE;
#endif

    return mbed_stack_base;
}

/**
  * Configures the link register of the given tcb to have the value function.
  *
  * @param tcb The tcb to modify
  * @param function the function the link register should point to.
  */
inline void tcb_configure_lr(PROCESSOR_TCB* tcb, PROCESSOR_WORD_TYPE function)
{
    tcb->LR = function;
}

/**
  * Configures the link register of the given tcb to have the value function.
  *
  * @param tcb The tcb to modify
  * @param function the function the link register should point to.
  */
inline void tcb_configure_sp(PROCESSOR_TCB* tcb, PROCESSOR_WORD_TYPE sp)
{
    tcb->SP = sp;
}

inline void tcb_configure_stack_base(PROCESSOR_TCB* tcb, PROCESSOR_WORD_TYPE stack_base)
{
    tcb->stack_base = stack_base;
}

inline PROCESSOR_WORD_TYPE tcb_get_stack_base(PROCESSOR_TCB* tcb)
{
    return tcb->stack_base;
}

inline PROCESSOR_WORD_TYPE get_current_sp()
{
#ifdef MBED_CONF_RTOS_PRESENT
    return __get_PSP();
#else
    return __get_MSP();
#endif
}

inline PROCESSOR_WORD_TYPE tcb_get_sp(PROCESSOR_TCB* tcb)
{
    return tcb->SP;
}

inline void tcb_configure_args(PROCESSOR_TCB* tcb, PROCESSOR_WORD_TYPE ep, PROCESSOR_WORD_TYPE cp, PROCESSOR_WORD_TYPE pm)
{
    tcb->R0 = (uint32_t) ep;
    tcb->R1 = (uint32_t) cp;
    tcb->R2 = (uint32_t) pm;
}

#endif
