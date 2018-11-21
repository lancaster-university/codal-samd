/*
The MIT License (MIT)

Copyright (c) 2017 Lancaster University.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/

#include "Timer.h"
#include "Event.h"
#include "CodalCompat.h"
#include "CodalDmesg.h"
#include "SAMDDAC.h"
#include "SAMDTimer.h"

#include "pins.h"
#include "tc.h"
#include "parts.h"
#include "hpl_gclk_base.h"
#include "peripheral_clk_config.h"
#ifdef SAMD21
#include "hpl/pm/hpl_pm_base.h"
#endif

extern "C"
{
#include "clocks.h"
#include "timers.h"
}

#define MUX_B 1

#ifdef SAMD21
#define DAC_DATA DAC->DATA.reg
#endif
#ifdef SAMD51
#define DAC_DATA DAC->DATA[0].reg
#endif

#undef ENABLE

SAMDDAC::SAMDDAC(ZPin &pin, DataSource &source, int sampleRate, uint16_t id) : upstream(source)
{
    this->id = id;
    this->active = false;
    this->dataReady = 0;
    this->sampleRate = sampleRate;

    // Put the pin into output mode.
    pin.setDigitalValue(0);

    CODAL_ASSERT(pin.name == PIN_PA02);

#ifdef SAMD51
    hri_mclk_set_APBDMASK_DAC_bit(MCLK);
#endif

#ifdef SAMD21
    _pm_enable_bus_clock(PM_BUS_APBC, DAC);
#endif

    // SAMD51: This clock should be <= 12 MHz, per datasheet section 47.6.3.
    // SAMD21: This clock is 48mhz despite the datasheet saying it must only be <= 350kHz, per
    // datasheet table 37-6. It's incorrect because the max output rate is 350ksps and is only
    // achieved when the GCLK is more than 8mhz.
    _gclk_enable_channel(DAC_GCLK_ID, CLK_GEN_8MHZ);

    DAC->CTRLA.bit.SWRST = 1;
    while (DAC->CTRLA.bit.SWRST == 1)
        ;
// Make sure there are no outstanding access errors. (Reading DATA can cause this.)
#ifdef SAMD51
    PAC->INTFLAGD.reg = PAC_INTFLAGD_DAC;
#endif

#ifdef SAMD21
    DAC->EVCTRL.reg |= DAC_EVCTRL_STARTEI;
    // We disable the voltage pump because we always run at 3.3v.
    DAC->CTRLB.reg = DAC_CTRLB_REFSEL_AVCC | DAC_CTRLB_EOEN | DAC_CTRLB_VPD; //  | DAC_CTRLB_LEFTADJ
#endif
#ifdef SAMD51
    DAC->EVCTRL.reg |= DAC_EVCTRL_STARTEI0;
    DAC->DACCTRL[0].reg = DAC_DACCTRL_CCTRL_CC1M | DAC_DACCTRL_ENABLE; // | DAC_DACCTRL_LEFTADJ;
    DAC->CTRLB.reg = DAC_CTRLB_REFSEL_VREFPU;
#endif

    // Re-enable the DAC
    DAC->CTRLA.bit.ENABLE = 1;
#ifdef SAMD21
    while (DAC->STATUS.bit.SYNCBUSY == 1)
        ;
#endif
#ifdef SAMD51
    while (DAC->SYNCBUSY.bit.ENABLE == 1)
        ;
    while (DAC->STATUS.bit.READY0 == 0)
        ;
#endif

    // Use a timer to coordinate when DAC conversions occur.
    this->tc = TC3;
    uint8_t tc_index = 0;
    while (tc_index < TC_INST_NUM)
    {
        if (tc_insts[tc_index] == this->tc)
            break;
        tc_index++;
    }
    CODAL_ASSERT(tc_index < TC_INST_NUM);

    tc_set_enable(tc, false);
    turn_on_clocks(true, tc_index, CLK_GEN_8MHZ);

    // Don't bother setting the period. We set it before you playback anything.
    tc_set_enable(tc, false);
    tc_reset(tc);

    tc->COUNT16.CTRLA.bit.MODE = 0;
#ifdef SAMD51
    tc->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;
#endif
#ifdef SAMD21
    tc->COUNT16.CTRLA.bit.WAVEGEN = TC_CTRLA_WAVEGEN_MFRQ_Val;
#endif
    tc->COUNT16.CTRLA.bit.RUNSTDBY = 1;
    tc->COUNT16.CTRLA.bit.PRESCALER = 0;
    tc->COUNT16.EVCTRL.reg = TC_EVCTRL_OVFEO;
    // t->COUNT16.CTRLC.reg = 0x00;     // compare mode
    tc->COUNT16.CTRLBCLR.bit.DIR = 1; // count up

    //tc_set_enable(tc, true);
    //tc->COUNT16.CTRLBSET.reg = TC_CTRLBSET_CMD_STOP;

    setSampleRate(sampleRate);

    pin._setMux(MUX_B);

    DmaFactory factory;
    dmaInstance = factory.allocate();
    CODAL_ASSERT(dmaInstance != NULL);

    dmaInstance->onTransferComplete(this);
    dmaInstance->configure(TC3_DMAC_ID_OVF, BeatHalfWord, NULL, (volatile void *)&DAC_DATA);

    // Register with our upstream component
    source.connect(*this);
}

/**
 * Change the DAC playback sample rate to the given frequency.
 * n.b. Only sample periods that are a multiple of 125nS are supported.
 * Frequencies mathcing other sample periods will be rounded down to the next lowest supported
 * frequency.
 *
 * @param frequency The new sample playback frequency.
 */
int SAMDDAC::getSampleRate()
{
    return sampleRate;
}

/**
 * Change the DAC playback sample rate to the given frequency.
 * n.b. Only sample periods that are a multiple of 125nS are supported.
 * Frequencies mathcing other sample periods will be rounded to the next highest supported
 * frequency.
 *
 * @param frequency The new sample playback frequency.
 */
int SAMDDAC::setSampleRate(int frequency)
{
    uint32_t period = 8000000 / frequency;
    sampleRate = 8000000 / period;

    tc_set_enable(tc, false);
    tc->COUNT16.CC[0].reg = period; // Set period
    tc_set_enable(tc, true);        // Restart the timer

    return DEVICE_OK;
}

/**
 * Callback provided when data is ready.
 */
int SAMDDAC::pullRequest()
{
    dataReady++;

    if (!active)
        pull();

    return DEVICE_OK;
}

/**
 * Pull down a buffer from upstream, and schedule a DMA transfer from it.
 */
int SAMDDAC::pull()
{
    output = upstream.pull();
    dataReady--;

    if (output.length() == 0)
    {
        dataReady = 0;
        active = false;
        return DEVICE_OK;
    }

    active = true;

    DMESG("DAC %d bytes", output.length());

    dmaInstance->transfer((const void *)&output[0], NULL, output.length());


    return DEVICE_OK;
}

void SAMDDAC::setValue(int value)
{
    DAC_DATA = value;
}

int SAMDDAC::getValue()
{
    return DAC_DATA;
}

extern void debug_flip();

/**
 * Base implementation of a DMA callback
 */
void SAMDDAC::dmaTransferComplete(DmaCode c)
{
    if (dataReady == 0)
    {
        active = false;
        return;
    }

    pull();
}
