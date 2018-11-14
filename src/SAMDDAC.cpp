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
    _gclk_enable_channel(DAC_GCLK_ID, CONF_GCLK_DAC_SRC);

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
    DAC->CTRLB.reg = DAC_CTRLB_REFSEL_AVCC | DAC_CTRLB_LEFTADJ | DAC_CTRLB_EOEN | DAC_CTRLB_VPD;
#endif
#ifdef SAMD51
    DAC->EVCTRL.reg |= DAC_EVCTRL_STARTEI0;
    DAC->DACCTRL[0].reg = DAC_DACCTRL_CCTRL_CC1M | DAC_DACCTRL_ENABLE | DAC_DACCTRL_LEFTADJ;
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
    Tc *t = TC3;
    uint8_t tc_index = 0;
    while (tc_index < TC_INST_NUM)
    {
        if (tc_insts[tc_index] == t)
            break;
        tc_index++;
    }
    CODAL_ASSERT(tc_index < TC_INST_NUM);

    // Use the 48mhz clocks on both the SAMD21 and 51 because we will be going much slower.
    uint8_t tc_gclk = 0;
#ifdef SAMD51
    tc_gclk = 1;
#endif

    turn_on_clocks(true, tc_index, tc_gclk);

    DMESG("clock: %d %d", tc_index, tc_gclk);

    // Don't bother setting the period. We set it before you playback anything.
    tc_set_enable(t, false);
    // tc_reset(t);

    t->COUNT16.CTRLA.bit.MODE = 0;
#ifdef SAMD51
    t->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;
#endif
#ifdef SAMD21
    t->COUNT16.CTRLA.bit.WAVEGEN = TC_CTRLA_WAVEGEN_MFRQ_Val;
#endif
    t->COUNT16.CTRLA.bit.RUNSTDBY = 1;
    t->COUNT16.CTRLA.bit.PRESCALER = 0;
    t->COUNT16.EVCTRL.reg = TC_EVCTRL_OVFEO;
    // t->COUNT16.CTRLC.reg = 0x00;     // compare mode
    t->COUNT16.CTRLBCLR.bit.DIR = 1; // count up

    // tc_set_enable(t, true);
    // t->COUNT16.CTRLBSET.reg = TC_CTRLBSET_CMD_STOP;
    /*
        {
            CTRLA = {bit = {SWRST = 0, ENABLE = 1, MODE = 0, PRESCSYNC = 0, RUNSTDBY = 1, ONDEMAND =
       0, PRESCALER = 0, ALOCK = 0, CAPTEN0 = 0, CAPTEN1 = 0, COPEN0 = 0, COPEN1 = 0, CAPTMODE0 = 0,
       CAPTMODE1 = 0}, vec = {CAPTEN = 0, COPEN = 0}, reg = 66}, CTRLBCLR = {bit = {DIR = 0 '\000',
       LUPD = 0 '\000', ONESHOT = 0 '\000', CMD = 0 '\000'}, reg = 0 '\000'}, CTRLBSET = {bit = {DIR
       = 0 '\000', LUPD = 0 '\000', ONESHOT = 0 '\000', CMD = 0 '\000'}, reg = 0 '\000'}, EVCTRL =
       {bit = {EVACT = 0, TCINV = 0, TCEI = 0, OVFEO = 1, MCEO0 = 0, MCEO1 = 0}, vec = {MCEO = 0},
       reg = 256}, INTENCLR = {bit = {OVF = 0 '\000', ERR = 0 '\000', MC0 = 0 '\000', MC1 = 0
       '\000'}, vec = {MC = 0 '\000'}, reg = 0 '\000'}, INTENSET = {bit = {OVF = 0 '\000', ERR = 0
       '\000', MC0 = 0 '\000', MC1 = 0 '\000'}, vec = {MC = 0 '\000'}, reg = 0 '\000'}, INTFLAG =
       {bit = {OVF = 0 '\000', ERR = 0 '\000', MC0 = 0 '\000', MC1 = 1 '\001'}, vec = {MC = 2
       '\002'}, reg = 32 ' '}, STATUS = {bit = {STOP = 0 '\000', SLAVE = 0 '\000', PERBUFV = 0
       '\000', CCBUFV0 = 0 '\000', CCBUFV1 = 0 '\000'}, vec = {CCBUFV = 0 '\000'}, reg = 0 '\000'},
            WAVE = {bit = {WAVEGEN = 1 '\001'}, reg = 1 '\001'},
            DRVCTRL = {bit = {INVEN0 = 0 '\000', INVEN1 = 0 '\000'}, vec = {INVEN = 0 '\000'},
                       reg = 0 '\000'},
            Reserved1 = "", DBGCTRL = {bit = {DBGRUN = 0 '\000'}, reg = 0 '\000'},
            SYNCBUSY = {bit = {SWRST = 0, ENABLE = 0, CTRLB = 0, STATUS = 0, COUNT = 0, PER = 0,
                               CC0 = 0, CC1 = 0},
                        vec = {CC = 0}, reg = 0},
            COUNT = {bit = {COUNT = 0}, reg = 0}, Reserved2 = "\000\000\000\000\000",
            CC = {{bit = {CC = 1088}, reg = 1088}, {bit = {CC = 0}, reg = 0}},
            Reserved3 = '\000' < repeats 15 times >, CCBUF = {
                {bit = {CCBUF = 1088}, reg = 1088},
                {bit = {CCBUF = 0}, reg = 0}
            }
        }
    */

    pin._setMux(MUX_B);

    DmaFactory factory;
    dmaInstance = factory.allocate();
    CODAL_ASSERT(dmaInstance != NULL);

    dmaInstance->onTransferComplete(this);
    dmaInstance->configure(TC3_DMAC_ID_OVF, BeatHalfWord, NULL, (volatile void *)&DAC_DATA);

    setSampleRate(sampleRate);

    // Register with our upstream component
    source.connect(*this);

    target_wait_us(50);
    DMESG("cl: %d", TC3->COUNT16.COUNT.reg);
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
    uint32_t period = CONF_GCLK_DAC_FREQUENCY / frequency;
    sampleRate = CONF_GCLK_DAC_FREQUENCY / period;

    tc_set_enable(TC3, false);
    TC3->COUNT16.CC[0].reg = period; // Set period
    tc_set_enable(TC3, true);        // Restart the timer

    DMESG("cl: %d", TC3->COUNT16.COUNT.reg);

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
void SAMDDAC::dmaTransferComplete()
{
    DMESG("DAC DMA");

    if (dataReady == 0)
    {
        active = false;
        return;
    }

    pull();
}
