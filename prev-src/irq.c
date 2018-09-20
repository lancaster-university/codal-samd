#include "stm32.h"

void init_irqs() {}

void NMI_Handler(void) {}

void SVC_Handler(void) {}

void DebugMon_Handler(void) {}

void PendSV_Handler(void) {}

void SysTick_Handler(void)
{
    HAL_IncTick();
}