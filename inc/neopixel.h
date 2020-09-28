/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Scott Shawcroft for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "hpl_gpio.h"
#include "hal_gpio.h"

// Ensure this code is compiled with -Os. Any other optimization level may change the timing of it
// and break neopixels.
#pragma GCC push_options
#pragma GCC optimize ("Os")

#define ASM asm

void neopixel_send_buffer(Pin& pin, const uint8_t *data, uint32_t length) {
    // This is adapted directly from the Adafruit NeoPixel library SAMD21G18A code:
    // https://github.com/adafruit/Adafruit_NeoPixel/blob/master/Adafruit_NeoPixel.cpp
    gpio_set_pin_direction(pin.name, GPIO_DIRECTION_OUT);

    // Need to wait at least Treset (50uS) after pulling the pin low
    system_timer_wait_us(52);

    uint8_t  *ptr, *end, p, bitMask;
    uint32_t  pinMask;
    PortGroup* port;

  

    uint32_t pin_name = pin.name;
    port    =  &PORT->Group[GPIO_PORT(pin_name)];  // Convert GPIO # to port register
    pinMask =  (1UL << (pin_name % 32));  // From port_pin_set_output_level ASF code.
    ptr     =  (uint8_t*)data;
    end     =  ptr + length;
    bitMask =  0x80;

    volatile uint32_t *set = &(port->OUTSET.reg),
                      *clr = &(port->OUTCLR.reg);

    // Turn off interrupts of any kind during timing-sensitive code.
    target_disable_irq();

    #ifdef SAMD51
    // the M4 code is not from MicroPython
    // WS2812B timings, +-0.15uS
    // 0 - 0.40uS hi 0.85uS low
    // 1 - 0.80uS hi 0.45uS low
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk))
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    uint32_t phase = DWT->CYCCNT;
    for (;;)
    {
        *set = pinMask;

        // phase += CPU_MHZ / 0.8
        // the other numbers are relative to phase increment
        uint32_t change = *ptr & bitMask ? phase + 97 : phase + 47;
        phase += 150;

        bitMask = bitMask >> 1;
        if (bitMask == 0)
        {
            bitMask = 0x80;
            ptr++;
        }

        while (DWT->CYCCNT < change)
            ;

        *clr = pinMask;

        if (ptr >= end)
            break;

        while (DWT->CYCCNT < phase)
            ;
    }


    #elif defined(SAMD21)
    // Make sure the NVM cache is consistently timed.
    NVMCTRL->CTRLB.bit.READMODE = NVMCTRL_CTRLB_READMODE_DETERMINISTIC_Val;
    p       = *ptr++;
    for(;;) {
        *set = pinMask;
        // This is the time where the line is always high regardless of the bit.
        // For the SK6812 its 0.3us +- 0.15us
        ASM("nop; nop;");
        if((p & bitMask) != 0) {
            // This is the high delay unique to a one bit.
            // For the SK6812 its 0.3us
            ASM("nop; nop; nop; nop; nop; nop; nop;");
            *clr = pinMask;
        } else {
            *clr = pinMask;
            // This is the low delay unique to a zero bit.
            // For the SK6812 its 0.3us
            ASM("nop; nop;");
        }
        if((bitMask >>= 1) != 0) {
            // This is the delay between bits in a byte and is the 1 code low
            // level time from the datasheet.
            // For the SK6812 its 0.6us +- 0.15us
            ASM("nop; nop; nop; nop; nop;");
        } else {
            if(ptr >= end) break;
            p       = *ptr++;
            bitMask = 0x80;
            // This is the delay between bytes. It's similar to the other branch
            // in the if statement except its tuned to account for the time the
            // above operations take.
            // For the SK6812 its 0.6us +- 0.15us
        }
    }
    // Speed up! (But inconsistent timing.)
    NVMCTRL->CTRLB.bit.READMODE = NVMCTRL_CTRLB_READMODE_NO_MISS_PENALTY_Val;
    #else
    #error "MCU not defined"
    #endif

    // Turn on interrupts after timing-sensitive code.
    target_enable_irq();
}

#pragma GCC pop_options