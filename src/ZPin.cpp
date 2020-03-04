/*
The MIT License (MIT)

Copyright (c) 2016 British Broadcasting Corporation.
This software is provided by Lancaster University by arrangement with the BBC.

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

/**
 * Class definition for ZPin.
 *
 * Commonly represents an I/O pin on the edge connector.
 */
#include "ZPin.h"
#include "Button.h"
#include "Timer.h"
#include "codal_target_hal.h"
#include "codal-core/inc/types/Event.h"
#include "pinmap.h"
#include "hal_gpio.h"
#include "hal_adc_sync.h"
#include "sam.h"
#include "CodalDmesg.h"
#include "hpl_gclk_base.h"

#ifdef SAMD21
#include "hpl/pm/hpl_pm_base.h"
#endif

#define LOG1() PORT->Group[0].OUTSET.reg = (1 << 5)
#define LOG0() PORT->Group[0].OUTCLR.reg = (1 << 5)

extern "C"
{
    #include "external_interrupts.h"
    #include "adc.h"
    #include "clocks.h"
}

#define IO_STATUS_CAN_READ                                                                         \
    (IO_STATUS_DIGITAL_IN | IO_STATUS_EVENT_ON_EDGE | IO_STATUS_EVENT_PULSE_ON_EDGE | IO_STATUS_INTERRUPT_ON_EDGE)

#define MUX_B           1

namespace codal
{
#ifdef SAMD21
    static bool eic_enabled = false;
    static ZPin* instances[EIC_CHANNEL_COUNT] = { NULL };
#endif

struct ZEventConfig
{
    CODAL_TIMESTAMP prevPulse;
};

inline gpio_pull_mode map(codal::PullMode pinMode)
{
    switch (pinMode)
    {
    case PullMode::Up:
        return GPIO_PULL_UP;
    case PullMode::Down:
        return GPIO_PULL_DOWN;
    case PullMode::None:
        return GPIO_PULL_OFF;
    }

    return GPIO_PULL_OFF;
}

#define PIN_MASK (1 << (name & 31))
#define PIN_PORT (&PORT->Group[name >> 5])

/**
 * Constructor.
 * Create a ZPin instance, generally used to represent a pin on the edge connector.
 *
 * @param id the unique EventModel id of this component.
 *
 * @param name the mbed PinName for this ZPin instance.
 *
 * @param capability the capabilities this ZPin instance should have.
 *                   (PIN_CAPABILITY_DIGITAL, PIN_CAPABILITY_ANALOG, PIN_CAPABILITY_AD,
 * PIN_CAPABILITY_ALL)
 *
 * @code
 * ZPin P0(DEVICE_ID_IO_P0, DEVICE_PIN_P0, PIN_CAPABILITY_ALL);
 * @endcode
 */
ZPin::ZPin(int id, PinNumber name, PinCapability capability) : codal::Pin(id, name, capability), EICInterface()
{
    this->pullMode = DEVICE_DEFAULT_PULLMODE;

    // Power up in a disconnected, low power state.
    // If we're unused, this is how it will stay...
    this->status = 0x00;

    this->pwmCfg = NULL;
    this->evCfg = NULL;
    this->btn = NULL;
#ifdef SAMD51
    this->chan = NULL;
#endif
    this->pin_obj = NULL;
}

void ZPin::disableEIC()
{
    // for the samd21, we simply disable the isr, memory is retained.
#ifdef SAMD21
    if (!pin_obj)
        pin_obj = samd_peripherals_get_pin(name);
    configure_eic_channel(pin_obj->extint_channel, 0);
#else
    this->chan->disable();
    this->chan = NULL;

    if (this->evCfg) {
        delete this->evCfg;
        this->evCfg = NULL;
    }
#endif
}

void ZPin::disconnect()
{
    target_disable_irq();
    if (this->status & IO_STATUS_ANALOG_OUT)
    {
        if (this->pwmCfg) {
            delete this->pwmCfg;
            pwmout_free(this->pwmCfg);
        }
        this->pwmCfg = NULL;
    }

    if (this->status & (IO_STATUS_EVENT_ON_EDGE | IO_STATUS_EVENT_PULSE_ON_EDGE | IO_STATUS_INTERRUPT_ON_EDGE))
    {
        disableEIC();
    }

    if (this->status & IO_STATUS_TOUCH_IN)
    {
        if (this->btn)
            delete this->btn;
        this->btn = NULL;
    }
    status = 0;
    target_enable_irq();
}

void ZPin::_setMux(int mux, bool isInput)
{
    disconnect();
    gpio_set_pin_direction(name, GPIO_DIRECTION_OUT);
    gpio_set_pin_pull_mode(name, GPIO_PULL_OFF);
    gpio_set_pin_function(name, PINMUX(name, mux));
}

/**
 * Configures this IO pin as a digital output (if necessary) and sets the pin to 'value'.
 *
 * @param value 0 (LO) or 1 (HI)
 *
 * @return DEVICE_OK on success, DEVICE_INVALID_PARAMETER if value is out of range, or
 * DEVICE_NOT_SUPPORTED if the given pin does not have digital capability.
 *
 * @code
 * ZPin P0(DEVICE_ID_IO_P0, DEVICE_PIN_P0, PIN_CAPABILITY_BOTH);
 * P0.setDigitalValue(1); // P0 is now HI
 * @endcode
 */
int ZPin::setDigitalValue(int value)
{
    // Ensure we have a valid value.
    value = ((value > 0) ? 1 : 0);

    // Move into a Digital input state if necessary.
    if (!(status & IO_STATUS_DIGITAL_OUT))
    {
        disconnect();
        gpio_set_pin_function(name, GPIO_PIN_FUNCTION_OFF);
        gpio_set_pin_direction(name, GPIO_DIRECTION_OUT);
        status |= IO_STATUS_DIGITAL_OUT;
    }

    gpio_set_pin_level(name, value);

    return DEVICE_OK;
}

/**
 * Configures this IO pin as a digital input (if necessary) and tests its current value.
 *
 *
 * @return 1 if this input is high, 0 if input is LO, or DEVICE_NOT_SUPPORTED
 *         if the given pin does not have digital capability.
 *
 * @code
 * ZPin P0(DEVICE_ID_IO_P0, DEVICE_PIN_P0, PIN_CAPABILITY_BOTH);
 * P0.getDigitalValue(); // P0 is either 0 or 1;
 * @endcode
 */
int ZPin::getDigitalValue()
{
    // Move into a Digital input state if necessary.
    if (!(status &
          (IO_STATUS_DIGITAL_IN | IO_STATUS_EVENT_ON_EDGE | IO_STATUS_EVENT_PULSE_ON_EDGE | IO_STATUS_INTERRUPT_ON_EDGE)))
    {
        disconnect();

        uint32_t cfg = PIN_PORT->PINCFG[name & 31].reg;
        PIN_PORT->DIRCLR.reg = PIN_MASK;
        uint32_t seven = PORT_PINCFG_PMUXEN | PORT_PINCFG_PULLEN | PORT_PINCFG_INEN;
        if (pullMode == PullMode::None)
             PIN_PORT->PINCFG[name & 31].reg = (cfg & ~seven) | PORT_PINCFG_INEN;
        else
        {
            PIN_PORT->PINCFG[name & 31].reg = (cfg & ~seven) | PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
            if (pullMode == PullMode::Down)
                PIN_PORT->OUTCLR.reg = PIN_MASK;
            else
                PIN_PORT->OUTSET.reg = PIN_MASK;
        }
        status |= IO_STATUS_DIGITAL_IN;
    }

    return PIN_PORT->IN.reg & PIN_MASK ? 1 : 0;
}

/**
 * Configures this IO pin as a digital input with the specified internal pull-up/pull-down
 * configuraiton (if necessary) and tests its current value.
 *
 * @param pull one of the mbed pull configurations: PullUp, PullDown, PullNone
 *
 * @return 1 if this input is high, 0 if input is LO, or DEVICE_NOT_SUPPORTED
 *         if the given pin does not have digital capability.
 *
 * @code
 * ZPin P0(DEVICE_ID_IO_P0, DEVICE_PIN_P0, PIN_CAPABILITY_BOTH);
 * P0.getDigitalValue(PullUp); // P0 is either 0 or 1;
 * @endcode
 */
int ZPin::getDigitalValue(PullMode pull)
{
    setPull(pull);
    return getDigitalValue();
}

int ZPin::setPWM(uint32_t value, uint32_t period)
{
    // sanitise the level value
    if (value > period)
        value = period;

    int r;

    // Move into an analogue output state if necessary
    if (!(status & IO_STATUS_ANALOG_OUT))
    {
        disconnect();
        gpio_set_pin_function(name, GPIO_PIN_FUNCTION_OFF);
        gpio_set_pin_direction(name, GPIO_DIRECTION_OUT);
        this->pwmCfg = new pwmout_t;
        r = pwmout_init(this->pwmCfg, name, value, period);
        status = IO_STATUS_ANALOG_OUT;
    } else {
        r = pwmout_write(this->pwmCfg, value, period);
    }

    CODAL_ASSERT(r == 0, DEVICE_HARDWARE_CONFIGURATION_ERROR);

    return DEVICE_OK;
}

/**
 * Configures this IO pin as an analog/pwm output, and change the output value to the given level.
 *
 * @param value the level to set on the output pin, in the range 0 - 1024
 *
 * @return DEVICE_OK on success, DEVICE_INVALID_PARAMETER if value is out of range, or
 * DEVICE_NOT_SUPPORTED if the given pin does not have analog capability.
 */
int ZPin::setAnalogValue(int value)
{
    // sanitise the level value
    if (value < 0 || value > DEVICE_PIN_MAX_OUTPUT)
        return DEVICE_INVALID_PARAMETER;

    uint32_t period = 20000;
    if (status & IO_STATUS_ANALOG_OUT)
        period = this->pwmCfg->period;

    return setPWM((uint64_t)value * period / DEVICE_PIN_MAX_OUTPUT, period);
}

/**
 * Configures this IO pin as an analog/pwm output (if necessary) and configures the period to be
 * 20ms, with a duty cycle between 500 us and 2500 us.
 *
 * A value of 180 sets the duty cycle to be 2500us, and a value of 0 sets the duty cycle to be 500us
 * by default.
 *
 * This range can be modified to fine tune, and also tolerate different servos.
 *
 * @param value the level to set on the output pin, in the range 0 - 180.
 *
 * @param range which gives the span of possible values the i.e. the lower and upper bounds (center
 * +/- range/2). Defaults to DEVICE_PIN_DEFAULT_SERVO_RANGE.
 *
 * @param center the center point from which to calculate the lower and upper bounds. Defaults to
 * DEVICE_PIN_DEFAULT_SERVO_CENTER
 *
 * @return DEVICE_OK on success, DEVICE_INVALID_PARAMETER if value is out of range, or
 * DEVICE_NOT_SUPPORTED if the given pin does not have analog capability.
 */
int ZPin::setServoValue(int value, int range, int center)
{
    // check if this pin has an analogue mode...
    if (!(PIN_CAPABILITY_ANALOG & capability))
        return DEVICE_NOT_SUPPORTED;

    // sanitise the servo level
    if (value < 0 || range < 1 || center < 1)
        return DEVICE_INVALID_PARAMETER;

    // clip - just in case
    if (value > DEVICE_PIN_MAX_SERVO_RANGE)
        value = DEVICE_PIN_MAX_SERVO_RANGE;

    // calculate the lower bound based on the midpoint
    int lower = (center - (range / 2)) * 1000;

    value = value * 1000;

    // add the percentage of the range based on the value between 0 and 180
    int scaled = lower + (range * (value / DEVICE_PIN_MAX_SERVO_RANGE));

    return setServoPulseUs(scaled / 1000);
}

/**
 * Configures this IO pin as an analogue input (if necessary), and samples the ZPin for its analog
 * value.
 *
 * @return the current analogue level on the pin, in the range 0 - 1024, or
 *         DEVICE_NOT_SUPPORTED if the given pin does not have analog capability.
 *
 * @code
 * ZPin P0(DEVICE_ID_IO_P0, DEVICE_PIN_P0, PIN_CAPABILITY_BOTH);
 * P0.getAnalogValue(); // P0 is a value in the range of 0 - 1024
 * @endcode
 */
int ZPin::getAnalogValue()
{
    // check if this pin has an analogue mode...
    if (!(PIN_CAPABILITY_ANALOG & capability))
        return DEVICE_NOT_SUPPORTED;

    if (!pin_obj)
        pin_obj = samd_peripherals_get_pin(name);
    uint8_t channel = pin_obj->adc_input[0];
#ifdef SAMD21
    Adc *adc = ADC;
#else
    Adc *adc = ADC0;
    if (channel == 0xff) {
        adc = ADC1;
        channel = pin_obj->adc_input[1];
    }
#endif

    if (channel == 0xff)
        return DEVICE_NOT_SUPPORTED;

    uint16_t res = 0;

    // adc function is B
    if (!(status & IO_STATUS_ANALOG_IN))
    {
        disconnect();
        gpio_set_pin_function(name, MUX_B); // mux b is ADC
        status = IO_STATUS_ANALOG_IN;
    }

    // rather than maintain state on many pins, we reconfigure the adc each time it is used.
    static adc_sync_descriptor adc_descriptor;

    memset(&adc_descriptor, 0, sizeof(adc_descriptor));

#ifdef SAMD21
    // the samd_peripherals_adc_setup() uses wrong clock

    // Turn the clocks on.
    _pm_enable_bus_clock(PM_BUS_APBC, ADC);
    _gclk_enable_channel(ADC_GCLK_ID, CLK_GEN_8MHZ);

    adc_sync_init(&adc_descriptor, adc, (void *)NULL);

    // Load the factory calibration
    hri_adc_write_CALIB_BIAS_CAL_bf(ADC, (*((uint32_t*) ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_Msk) >> ADC_FUSES_BIASCAL_Pos);
    // Bits 7:5
    uint16_t linearity = ((*((uint32_t*) ADC_FUSES_LINEARITY_1_ADDR) & ADC_FUSES_LINEARITY_1_Msk) >> ADC_FUSES_LINEARITY_1_Pos) << 5;
    // Bits 4:0
    linearity |= (*((uint32_t*) ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos;
    hri_adc_write_CALIB_LINEARITY_CAL_bf(ADC, linearity);
#else
    samd_peripherals_adc_setup(&adc_descriptor, adc);
#endif

    adc_sync_set_reference(&adc_descriptor, ADC_REFCTRL_REFSEL_INTVCC1_Val);

#ifdef SAMD21
    adc_sync_set_channel_gain(&adc_descriptor, channel, ADC_INPUTCTRL_GAIN_DIV2_Val);
#endif

    adc_sync_set_resolution(&adc_descriptor, ADC_CTRLB_RESSEL_10BIT_Val); // 10 bit conversion
    adc_sync_enable_channel(&adc_descriptor, channel);
    adc_sync_set_inputs(&adc_descriptor, channel, ADC_INPUTCTRL_MUXNEG_GND_Val, channel);

    // first result is always garbaggeeee, according to the datasheet
    adc_sync_read_channel(&adc_descriptor, channel, (uint8_t*) &res, sizeof(res));

    // returns the number of bytes read.
    int ret = adc_sync_read_channel(&adc_descriptor, channel, (uint8_t*)&res, sizeof(res));

    adc_sync_deinit(&adc_descriptor);

    if (ret <= 0)
        return DEVICE_NOT_SUPPORTED;

    return res;
}

/**
 * Determines if this IO pin is currently configured as an input.
 *
 * @return 1 if pin is an analog or digital input, 0 otherwise.
 */
int ZPin::isInput()
{
    return (status & (IO_STATUS_DIGITAL_IN | IO_STATUS_ANALOG_IN)) == 0 ? 0 : 1;
}

/**
 * Determines if this IO pin is currently configured as an output.
 *
 * @return 1 if pin is an analog or digital output, 0 otherwise.
 */
int ZPin::isOutput()
{
    return (PORT->Group[name >> 5].DIR.reg & (1 << (name & 31))) ||
        (status & (IO_STATUS_DIGITAL_OUT | IO_STATUS_ANALOG_OUT)) == 0 ? 0 : 1;
}

/**
 * Determines if this IO pin is currently configured for digital use.
 *
 * @return 1 if pin is digital, 0 otherwise.
 */
int ZPin::isDigital()
{
    return (status & (IO_STATUS_DIGITAL_IN | IO_STATUS_DIGITAL_OUT)) == 0 ? 0 : 1;
}

/**
 * Determines if this IO pin is currently configured for analog use.
 *
 * @return 1 if pin is analog, 0 otherwise.
 */
int ZPin::isAnalog()
{
    return (status & (IO_STATUS_ANALOG_IN | IO_STATUS_ANALOG_OUT)) == 0 ? 0 : 1;
}

/**
 * Configures this IO pin as a "makey makey" style touch sensor (if necessary)
 * and tests its current debounced state.
 *
 * Users can also subscribe to Button events generated from this pin.
 *
 * @return 1 if pin is touched, 0 if not, or DEVICE_NOT_SUPPORTED if this pin does not support touch
 * capability.
 *
 * @code
 * DeviceMessageBus bus;
 *
 * ZPin P0(DEVICE_ID_IO_P0, DEVICE_PIN_P0, PIN_CAPABILITY_ALL);
 * if(P0.isTouched())
 * {
 *     //do something!
 * }
 *
 * // subscribe to events generated by this pin!
 * bus.listen(DEVICE_ID_IO_P0, DEVICE_BUTTON_EVT_CLICK, someFunction);
 * @endcode
 */
int ZPin::isTouched()
{
    // check if this pin has a touch mode...
    if (!(PIN_CAPABILITY_DIGITAL & capability))
        return DEVICE_NOT_SUPPORTED;

    // Move into a touch input state if necessary.
    if (!(status & IO_STATUS_TOUCH_IN))
    {
        disconnect();
        this->btn = new Button(*this, id);
        status |= IO_STATUS_TOUCH_IN;
    }

    return this->btn->isPressed();
}

/**
 * Configures this IO pin as an analog/pwm output if it isn't already, configures the period to be
 * 20ms, and sets the pulse width, based on the value it is given.
 *
 * @param pulseWidth the desired pulse width in microseconds.
 *
 * @return DEVICE_OK on success, DEVICE_INVALID_PARAMETER if value is out of range, or
 * DEVICE_NOT_SUPPORTED if the given pin does not have analog capability.
 */
int ZPin::setServoPulseUs(int pulseWidth)
{
    // check if this pin has an analogue mode...
    if (!(PIN_CAPABILITY_ANALOG & capability))
        return DEVICE_NOT_SUPPORTED;

    // sanitise the pulse width
    if (pulseWidth < 0)
        return DEVICE_INVALID_PARAMETER;

    return setPWM(pulseWidth, DEVICE_DEFAULT_PWM_PERIOD);
}

/**
 * Configures the PWM period of the analog output to the given value.
 *
 * @param period The new period for the analog output in microseconds.
 *
 * @return DEVICE_OK on success.
 */
int ZPin::setAnalogPeriodUs(int period)
{
    if (status & IO_STATUS_ANALOG_OUT)
        // keep the % of duty cycle
        return setPWM((uint64_t)this->pwmCfg->pulse * period / this->pwmCfg->period, period);
    else
        return setPWM(0, period);
}

/**
 * Configures the PWM period of the analog output to the given value.
 *
 * @param period The new period for the analog output in milliseconds.
 *
 * @return DEVICE_OK on success, or DEVICE_NOT_SUPPORTED if the
 *         given pin is not configured as an analog output.
 */
int ZPin::setAnalogPeriod(int period)
{
    return setAnalogPeriodUs(period * 1000);
}

/**
 * Obtains the PWM period of the analog output in microseconds.
 *
 * @return the period on success, or DEVICE_NOT_SUPPORTED if the
 *         given pin is not configured as an analog output.
 */
uint32_t ZPin::getAnalogPeriodUs()
{
    if (!(status & IO_STATUS_ANALOG_OUT))
        return DEVICE_NOT_SUPPORTED;

    return this->pwmCfg->period;
}

/**
 * Obtains the PWM period of the analog output in milliseconds.
 *
 * @return the period on success, or DEVICE_NOT_SUPPORTED if the
 *         given pin is not configured as an analog output.
 */
int ZPin::getAnalogPeriod()
{
    return getAnalogPeriodUs() / 1000;
}

/**
 * Configures the pull of this pin.
 *
 * @param pull one of the mbed pull configurations: PullUp, PullDown, PullNone
 *
 * @return DEVICE_NOT_SUPPORTED if the current pin configuration is anything other
 *         than a digital input, otherwise DEVICE_OK.
 */
int ZPin::setPull(PullMode pull)
{
    if (pullMode == pull)
        return DEVICE_OK;

    pullMode = pull;

    // have to disconnect to flush the change to the hardware
    disconnect();
    getDigitalValue();

    return DEVICE_OK;
}

/**
 * This member function manages the calculation of the timestamp of a pulse detected
 * on a pin whilst in IO_STATUS_EVENT_PULSE_ON_EDGE or IO_STATUS_EVENT_ON_EDGE modes.
 *
 * @param eventValue the event value to distribute onto the message bus.
 */
void ZPin::pulseWidthEvent(int eventValue)
{
    Event evt(id, eventValue, CREATE_ONLY);
    auto now = evt.timestamp;
    auto previous = this->evCfg->prevPulse;

    if (previous != 0)
    {
        evt.timestamp -= previous;
        evt.fire();
    }

    this->evCfg->prevPulse = now;
}

#ifdef SAMD21
extern "C" void EIC_Handler()
{
    uint32_t extint = EIC->INTFLAG.vec.EXTINT;
    EIC->INTFLAG.vec.EXTINT = extint;
    for (uint8_t i = 0; i < 16; i++)
        if ((extint & (1 << i)) && instances[i])
            instances[i]->pinEventDetected();
}
#endif

void ZPin::pinEventDetected()
{
    bool isRise = gpio_get_pin_level(this->name);

    if (status & IO_STATUS_EVENT_PULSE_ON_EDGE)
        pulseWidthEvent(isRise ? DEVICE_PIN_EVT_PULSE_LO : DEVICE_PIN_EVT_PULSE_HI);

    if (status & IO_STATUS_EVENT_ON_EDGE)
        Event(id, isRise ? DEVICE_PIN_EVT_RISE : DEVICE_PIN_EVT_FALL, 0, CREATE_AND_FIRE);

    if (status & IO_STATUS_INTERRUPT_ON_EDGE)
        this->gpio_irq(isRise);
}

/**
 * This member function will construct an TimedInterruptIn instance, and configure
 * interrupts for rise and fall.
 *
 * @param eventType the specific mode used in interrupt context to determine how an
 *                  edge/rise is processed.
 *
 * @return DEVICE_OK on success
 */
int ZPin::enableRiseFallEvents(int eventType)
{
    // if we are in neither of the two modes, configure pin as a TimedInterruptIn.
    if (!(status & (IO_STATUS_EVENT_ON_EDGE | IO_STATUS_EVENT_PULSE_ON_EDGE | IO_STATUS_INTERRUPT_ON_EDGE)))
    {
        if (!(status & IO_STATUS_DIGITAL_IN))
            getDigitalValue();

        if (!pin_obj)
            pin_obj = samd_peripherals_get_pin(name);
        CODAL_ASSERT(pin_obj != NULL, DEVICE_HARDWARE_CONFIGURATION_ERROR);
        CODAL_ASSERT(pin_obj->has_extint, DEVICE_HARDWARE_CONFIGURATION_ERROR);

        if (!evCfg)
            evCfg = new ZEventConfig;

        // this code is optimised for servicing pin interrupts on a slower processor (SAMD21)
#ifdef SAMD21

        if (!eic_enabled)
        {
            NVIC_SetPriority(EIC_IRQn,0);
            eic_enabled = true;

            turn_on_external_interrupt_controller();
            eic_reset();
            eic_set_enable(true);
        }

        instances[pin_obj->extint_channel] = this;
        // last param is ignored as we override EIC_Handlers...
        // 3 is rise fall
        turn_on_eic_channel(pin_obj->extint_channel, 3, EIC_HANDLER_APP);

        // pinmux a is zero (true for both samd21 and 51)
        gpio_set_pin_function(name, PINMUX(name, 0));
#else
        if (!chan)
        {
            EICFactory* factory = EICFactory::getInstance();
            this->chan = factory->getChannel(pin_obj->extint_channel);
            CODAL_ASSERT(chan != NULL, DEVICE_HARDWARE_CONFIGURATION_ERROR);
        }

        // pinmux a is zero (true for both samd21 and 51)
        gpio_set_pin_function(name, PINMUX(name, 0));

        this->chan->setChangeCallback(this);
        this->chan->enable(EICEventsRiseFall);
#endif
    }

    status &= ~(IO_STATUS_EVENT_ON_EDGE | IO_STATUS_EVENT_PULSE_ON_EDGE | IO_STATUS_INTERRUPT_ON_EDGE);

    // set our status bits accordingly.
    if (eventType == DEVICE_PIN_EVENT_ON_EDGE)
        status |= IO_STATUS_EVENT_ON_EDGE;
    else if (eventType == DEVICE_PIN_EVENT_ON_PULSE)
        status |= IO_STATUS_EVENT_PULSE_ON_EDGE;
    else if (eventType == DEVICE_PIN_INTERRUPT_ON_EDGE)
        status |= IO_STATUS_INTERRUPT_ON_EDGE;

    return DEVICE_OK;
}

/**
 * If this pin is in a mode where the pin is generating events, it will destruct
 * the current instance attached to this ZPin instance.
 *
 * @return DEVICE_OK on success.
 */
int ZPin::disableEvents()
{
    target_disable_irq();
    if (status & (IO_STATUS_EVENT_ON_EDGE | IO_STATUS_EVENT_PULSE_ON_EDGE | IO_STATUS_INTERRUPT_ON_EDGE | IO_STATUS_TOUCH_IN))
    {
        // this is 3us on D21
        disableEIC();
        gpio_set_pin_function(name, GPIO_PIN_FUNCTION_OFF);
        // the pin was in EIC mode, which implies input mode
        // avoid unneccessary (and costly!) reconfiguration
        status = IO_STATUS_DIGITAL_IN;
    }
    else if (status & (IO_STATUS_TOUCH_IN))
    {
        // this is 10us on D21
        disconnect();
        getDigitalValue();
    }
    target_enable_irq();

    return DEVICE_OK;
}

/**
 * Configures the events generated by this ZPin instance.
 *
 * DEVICE_PIN_EVENT_ON_EDGE - Configures this pin to a digital input, and generates events whenever
 * a rise/fall is detected on this pin. (DEVICE_PIN_EVT_RISE, DEVICE_PIN_EVT_FALL)
 * DEVICE_PIN_EVENT_ON_PULSE - Configures this pin to a digital input, and generates events where
 * the timestamp is the duration that this pin was either HI or LO. (DEVICE_PIN_EVT_PULSE_HI,
 * DEVICE_PIN_EVT_PULSE_LO) DEVICE_PIN_EVENT_ON_TOUCH - Configures this pin as a makey makey style
 * touch sensor, in the form of a Button. Normal button events will be generated using the ID of
 * this pin. DEVICE_PIN_EVENT_NONE - Disables events for this pin.
 *
 * @param eventType One of: DEVICE_PIN_EVENT_ON_EDGE, DEVICE_PIN_EVENT_ON_PULSE,
 * DEVICE_PIN_EVENT_ON_TOUCH, DEVICE_PIN_EVENT_NONE
 *
 * @code
 * DeviceMessageBus bus;
 *
 * ZPin P0(DEVICE_ID_IO_P0, DEVICE_PIN_P0, PIN_CAPABILITY_BOTH);
 * P0.eventOn(DEVICE_PIN_EVENT_ON_PULSE);
 *
 * void onPulse(Event evt)
 * {
 *     int duration = evt.timestamp;
 * }
 *
 * bus.listen(DEVICE_ID_IO_P0, DEVICE_PIN_EVT_PULSE_HI, onPulse, MESSAGE_BUS_LISTENER_IMMEDIATE)
 * @endcode
 *
 * @return DEVICE_OK on success, or DEVICE_INVALID_PARAMETER if the given eventype does not match
 *
 * @note In the DEVICE_PIN_EVENT_ON_PULSE mode, the smallest pulse that was reliably detected was
 * 85us, around 5khz. If more precision is required, please use the InterruptIn class supplied by
 * ARM mbed.
 */
int ZPin::eventOn(int eventType)
{
    switch (eventType)
    {
    case DEVICE_PIN_EVENT_NONE:
        disableEvents();
        break;

    case DEVICE_PIN_EVENT_ON_EDGE:
    case DEVICE_PIN_EVENT_ON_PULSE:
    case DEVICE_PIN_INTERRUPT_ON_EDGE:
        enableRiseFallEvents(eventType);
        break;

    case DEVICE_PIN_EVENT_ON_TOUCH:
        isTouched();
        break;

    default:
        return DEVICE_INVALID_PARAMETER;
    }

    return DEVICE_OK;
}

__attribute__((noinline))
static void get_and_set(PortGroup *port, uint32_t mask) {
    // 0 -> 1, only set when IN==0
    uint32_t inp = ~port->IN.reg & mask;
    port->DIRSET.reg = inp;
    port->OUTSET.reg = inp;
}

__attribute__((noinline))
static void get_and_clr(PortGroup *port, uint32_t mask) {
    // LOG1();
    // 1 -> 0, only set when IN==1
    uint32_t inp = port->IN.reg & mask;
    port->DIRSET.reg = inp;
    port->OUTCLR.reg = inp;
    // LOG0();
}

int ZPin::getAndSetDigitalValue(int value)
{
    uint32_t mask = PIN_MASK;
    PortGroup *port = PIN_PORT;

    if ((port->DIR.reg & mask) == 0)
    {
        // pin in input mode, do the "atomic" set
        if (value)
            get_and_set(port, mask);
        else
            get_and_clr(port, mask);

        if (port->DIR.reg & mask) {
            disconnect();
            setDigitalValue(value); // make sure 'status' is updated
            return 0;
        } else {
            return DEVICE_BUSY;
        }
    }

    return 0;
}


} // namespace codal
