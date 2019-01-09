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

#include "sam.h"

#ifdef I2S

#include "Event.h"
#include "CodalCompat.h"
#include "SAMDPDM.h"
#include "Pin.h"
#include "CodalDmesg.h"

extern "C"
{
    #include "clocks.h"
    #include "i2s.h"
}

#undef ENABLE

/**
 * An 8 bit PDM lookup table, used to reduce processing time.
 */
const int8_t pdmDecode[256] = {
#   define S(n) (2*(n)-8)
#   define B2(n) S(n),  S(n+1),  S(n+1),  S(n+2)
#   define B4(n) B2(n), B2(n+1), B2(n+1), B2(n+2)
#   define B6(n) B4(n), B4(n+1), B4(n+1), B4(n+2)
B6(0), B6(1), B6(1), B6(2)
};

#ifdef SAMD21
#define DATAREG DATA[1]
#else
#define DATAREG RXDATA
#endif

// a windowed sinc filter for 44 khz, 64 samples
const uint16_t sincfilter[SAMD21_PDM_DECIMATION] = {0, 2, 9, 21, 39, 63, 94, 132, 179, 236, 302, 379, 467, 565, 674, 792, 920, 1055, 1196, 1341, 1487, 1633, 1776, 1913, 2042, 2159, 2263, 2352, 2422, 2474, 2506, 2516, 2506, 2474, 2422, 2352, 2263, 2159, 2042, 1913, 1776, 1633, 1487, 1341, 1196, 1055, 920, 792, 674, 565, 467, 379, 302, 236, 179, 132, 94, 63, 39, 21, 9, 2, 0, 0};

// a manual loop-unroller!
#define ADAPDM_REPEAT_LOOP_16(X) X X X X X X X X X X X X X X X X

/**
 * Update our reference to a downstream component.
 * Pass through any connect requests to our output buffer component.
 *
 * @param component The new downstream component for this PDM audio source.
 */
void SAMD21PDM::connect(DataSink& component)
{
    output.connect(component);
}

/**
 * Constructor for an instance of a PDM input (typically microphone),
 *
 * @param sd The pin the PDM data input is connected to.
 * @param sck The pin the PDM clock is conected to.
 * @param dma The DMA controller to use for data transfer.
 * @param sampleRate the rate at which samples are generated in the output buffer (in Hz)
 * @param id The id to use for the message bus when transmitting events.
 */
SAMD21PDM::SAMD21PDM(ZPin &sd, ZPin &sck, int sampleRate, uint16_t id) : output(*this)
{
    dma = DmaFactory::allocate();
    CODAL_ASSERT(dma != NULL);

    this->id = id;
    this->sampleRate = sampleRate;
    this->clockRate = sampleRate*16;
    this->enabled = false;
    this->outputBufferSize = 512;

    this->pdmDataBuffer = NULL;
    this->pdmReceiveBuffer = rawPDM1;

    buffer = ManagedBuffer(outputBufferSize);
    out = (int16_t *) &buffer[0];

    output.setBlocking(false);

    // Configure sd and sck pins as inputs
    sd.getDigitalValue();
    sck.setDigitalValue(0);

    sd._setMux(MUX_G);
    sck._setMux(MUX_G);

    // Enable the I2S bus clock (CLK_I2S_APB)
    turn_on_i2s();
    connect_gclk_to_peripheral(CLK_GEN_48MHZ, I2S_GCLK_ID_0);

    // Configure a DMA channel
    dma->configure(I2S_DMAC_ID_RX_1, DmaBeatSize::BeatWord, &I2S->DATAREG.reg, NULL);
    dma->onTransferComplete(this);

    // Configure for DMA enabled, single channel PDM input.
    int clockDivisor = 1;
    uint32_t cs = 48000000;
    while(cs >= this->clockRate && clockDivisor < 0x1f)
    {
        clockDivisor++;
        cs = 48000000 / clockDivisor;
    }

    // We want to run at least as fast as the requested speed, so scale up if needed.
    if (cs <= this->clockRate)
    {
        clockDivisor--;
        cs = 48000000 / clockDivisor;
    }

    // Record our actual clockRate, as it's useful for calculating sample window sizes etc.
    this->clockRate = cs;

    // Make sure if we change the clock rate we update the sample rate as well
    this->sampleRate = clockRate/16;

    I2S->CTRLA.reg = 0;


    uint32_t clkctrl =
      // I2S_CLKCTRL_MCKOUTINV | // mck out not inverted
      // I2S_CLKCTRL_SCKOUTINV | // sck out not inverted
      // I2S_CLKCTRL_FSOUTINV |  // fs not inverted
      // I2S_CLKCTRL_MCKEN |    // Disable MCK output
      // I2S_CLKCTRL_MCKSEL |   // Disable MCK output
      // I2S_CLKCTRL_SCKSEL |   // SCK source is GCLK
      // I2S_CLKCTRL_FSINV |    // do not invert frame sync
      // I2S_CLKCTRL_FSSEL |    // Configure FS generation from SCK clock.
      // I2S_CLKCTRL_BITDELAY |  // No bit delay (PDM)
      0;

    clkctrl |= I2S_CLKCTRL_MCKOUTDIV(0);
    clkctrl |= I2S_CLKCTRL_MCKDIV(0);
    clkctrl |= I2S_CLKCTRL_NBSLOTS(1);  // STEREO is '1' (subtract one from #)
    clkctrl |= I2S_CLKCTRL_FSWIDTH_SLOT;  // Frame Sync (FS) Pulse is 1 Slot width
    clkctrl |= I2S_CLKCTRL_SLOTSIZE_16;

    // Configure for a 32 bit wide receive, with a SCK clock generated from GCLK_I2S_0.
    I2S->CLKCTRL[0].reg = clkctrl | ((clockDivisor-1) << 19);

#ifdef SAMD21
    // Configure serializer for a 32 bit data word transferred in a single DMA operation, clocked by clock unit 0.
    // set BITREV to give us LSB first data
    I2S->SERCTRL[1].reg = // I2S_SERCTRL_RXLOOP |    // Dont use loopback mode
      // I2S_SERCTRL_DMA    |    // Single DMA channel for all I2S channels
      // I2S_SERCTRL_MONO   |    // Dont use MONO mode
      // I2S_SERCTRL_SLOTDIS7 |  // Dont have any slot disabling
      // I2S_SERCTRL_SLOTDIS6 |
      // I2S_SERCTRL_SLOTDIS5 |
      // I2S_SERCTRL_SLOTDIS4 |
      // I2S_SERCTRL_SLOTDIS3 |
      // I2S_SERCTRL_SLOTDIS2 |
      // I2S_SERCTRL_SLOTDIS1 |
      // I2S_SERCTRL_SLOTDIS0 |
      I2S_SERCTRL_BITREV   |  // Do not transfer LSB first (MSB first!)
      // I2S_SERCTRL_WORDADJ  |  // Data NOT left in word
      I2S_SERCTRL_SLOTADJ     |  // Data is left in slot
      // I2S_SERCTRL_TXSAME   |  // Pad 0 on underrun
      I2S_SERCTRL_SERMODE_PDM2 |
      I2S_SERCTRL_DATASIZE_32 |
      I2S_SERCTRL_TXDEFAULT(0) |
      I2S_SERCTRL_EXTEND(0);
#else
    I2S->RXCTRL.reg =
      I2S_RXCTRL_BITREV   |  // Do not transfer LSB first (MSB first!)
      I2S_RXCTRL_SLOTADJ     |  // Data is left in slot
      I2S_RXCTRL_SERMODE_PDM2 |
      I2S_RXCTRL_DATASIZE_32 |
      //I2S_SERCTRL_TXDEFAULT(0) |
      I2S_RXCTRL_EXTEND(0);
#endif

    // enable all of the things (except SWRST)
    I2S->CTRLA.reg = 0x3E;

    // Create a listener to receive data ready events from our ISR.
    if(EventModel::defaultEventBus)
        EventModel::defaultEventBus->listen(id, SAMD21_PDM_DATA_READY, this, &SAMD21PDM::decimate);
}

/**
 * Provide the next available ManagedBuffer to our downstream caller, if available.
 */
ManagedBuffer SAMD21PDM::pull()
{
	return buffer;
}


void SAMD21PDM::decimate(Event)
{
    // DMESG("DEC!");
    uint32_t *b = (uint32_t *)pdmDataBuffer;

    // Ensure we have a sane buffer
    if (pdmDataBuffer == NULL)
        return;
    // DMESG("D");

    // DMESG("Buf!");

    while(b !=  (uint32_t *)((uint8_t *)pdmDataBuffer + SAMD21_PDM_BUFFER_SIZE)){
        runningSum = 0;
        sincPtr = sincfilter;

        for (uint8_t samplenum=0; samplenum < (SAMD21_PDM_DECIMATION/16) ; samplenum++) {
             uint16_t sample = *b++ & 0xFFFF;    // we read 16 bits at a time, by default the low half

             ADAPDM_REPEAT_LOOP_16(      // manually unroll loop: for (int8_t b=0; b<16; b++)
               {
                 // start at the LSB which is the 'first' bit to come down the line, chronologically
                 // (Note we had to set I2S_SERCTRL_BITREV to get this to work, but saves us time!)
                 if (sample & 0x1) {
                   runningSum += *sincPtr;     // do the convolution
                 }
                 sincPtr++;
                 sample >>= 1;
              }
            )
        }
        *out++ = runningSum - (1<<15);

        // If our output buffer is full, schedule it to flow downstream.
        if (out == (int16_t *) (&buffer[0] + outputBufferSize))
        {
            if (invalid)
            {
                invalid--;
            }
            else
            {
                output.pullRequest();
                buffer = ManagedBuffer(outputBufferSize);
            }

            out = (int16_t *) &buffer[0];
        }
    }

    // Record that we've completed processing.
    pdmDataBuffer = NULL;
}

void SAMD21PDM::dmaTransferComplete(DmaCode c)
{
    if (c == DMA_ERROR)
        while(1) DMESG("POO!!!");
    // If the last puffer has already been processed, start processing this buffer.
    // otherwise, we're running behind for some reason, so drop this buffer.
    if (pdmDataBuffer == NULL)
    {
        pdmDataBuffer = pdmReceiveBuffer;
        Event(id, SAMD21_PDM_DATA_READY);

        pdmReceiveBuffer = pdmReceiveBuffer == rawPDM1 ? rawPDM2 : rawPDM1;
    }

    // start the next DMA transfer, unless we've been asked to stop.
    if (enabled)
        startDMA();
}

/**
 * Enable this component
 */
void SAMD21PDM::enable()
{
    // If we're already running, nothing to do.
    if (enabled)
        return;

    // DMESG("enable");

    // Initiate a DMA transfer.
    enabled = true;
    invalid = SAMD21_START_UP_DELAY;
    startDMA();
}

/**
 * Disable this component
 */
void SAMD21PDM::disable()
{
    // Schedule all DMA transfers to stop after the next DMA transaction completes.
    enabled = false;
}

/**
 * Initiate a DMA transfer into the raw data buffer.
 */
void SAMD21PDM::startDMA()
{
    // DMESG("STRT");
    dma->transfer(NULL, pdmReceiveBuffer, SAMD21_PDM_BUFFER_SIZE);
    // Access the Data buffer once, to ensure we don't miss a DMA trigger...
    I2S->DATAREG.reg = I2S->DATAREG.reg;
}
#endif
