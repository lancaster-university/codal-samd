/* mbed Microcontroller Library
 *******************************************************************************
 * Copyright (c) 2017, STMicroelectronics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************
 */
#include "cmsis.h"
#include "pwmout_api.h"
#include "pwmout_device.h"

#if !defined(STM32F4)
#error "Please make sure this list is correct for your MCU and modify the line above"
#endif

const pwm_apb_map_t pwm_apb_map_table[] =
{
#if defined(TIM2_BASE)
    {TIM2_BASE, PWMOUT_ON_APB1},
#endif
#if defined(TIM3_BASE)
    {TIM3_BASE, PWMOUT_ON_APB1},
#endif
#if defined(TIM4_BASE)
    {TIM4_BASE, PWMOUT_ON_APB1},
#endif
#if defined(TIM5_BASE)
    {TIM5_BASE, PWMOUT_ON_APB1},
#endif
#if defined(TIM12_BASE)
    {TIM12_BASE, PWMOUT_ON_APB1},
#endif
#if defined(TIM13_BASE)
    {TIM13_BASE, PWMOUT_ON_APB1},
#endif
#if defined(TIM14_BASE)
    {TIM14_BASE, PWMOUT_ON_APB1},
#endif
#if defined(TIM1_BASE)
    {TIM1_BASE, PWMOUT_ON_APB2},
#endif
#if defined(TIM8_BASE)
    {TIM8_BASE, PWMOUT_ON_APB2},
#endif
#if defined(TIM9_BASE)
    {TIM9_BASE, PWMOUT_ON_APB2},
#endif
#if defined(TIM10_BASE)
    {TIM10_BASE, PWMOUT_ON_APB2},
#endif
#if defined(TIM11_BASE)
    {TIM11_BASE, PWMOUT_ON_APB2},
#endif
    {0, PWMOUT_UNKNOWN}
};
