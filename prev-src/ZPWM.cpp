#include "ZPWM.h"
#include "CodalDmesg.h"
#include "dma.h"

#define LOG DMESG

static ZPWM *instances[4];
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

ZPWM::ZPWM(Pin &pin, DataSource &source, int sampleRate, uint16_t id) : upstream(source)
{
    for (unsigned i = 0; i < ARRAY_SIZE(instances); ++i)
    {
        if (instances[i] == NULL)
        {
            instances[i] = this;
            break;
        }
    }

    buf0 = NULL;
    buf1 = NULL;
    bufCnt = 0;
    outptr = 0;

    pin.setDigitalValue(0);

    pwmout_t pwm;
    pwmout_init(&pwm, pin.name);

    memset(&tim, 0, sizeof(tim));
    tim.Instance = (TIM_TypeDef *)pwm.pwm;

    this->channel = pwm.channel;

    // initialise state
    this->id = id;
    this->dataReady = 0;
    this->active = false;

    // Ensure PWM is currently disabled.
    disable();

    // Configure hardware for requested sample rate.
    setSampleRate(sampleRate);

    dma_init((uint32_t)tim.Instance, this->channel + DMA_TIM_CH1 - 1, &hdma_left, DMA_FLAG_4BYTE);
    __HAL_LINKDMA(&tim, hdma[this->channel], hdma_left);

    // Enable the PWM module
    enable();

    // Register with our upstream component
    upstream.connect(*this);
}

/**
 * Determine the DAC playback sample rate to the given frequency.
 * @return the current sample rate.
 */
int ZPWM::getSampleRate()
{
    return sampleRate;
}

/**
 * Determine the maximum unsigned vlaue that can be loaded into the PWM data values, for the current
 * frequency configuration.
 */
int ZPWM::getSampleRange()
{
    return tim.Init.Period - 1;
}

static const uint32_t channels[] = {
    TIM_CHANNEL_1,
    TIM_CHANNEL_2,
    TIM_CHANNEL_3,
    TIM_CHANNEL_4,
};

/**
 * Change the DAC playback sample rate to the given frequency.
 * @param frequency The new sample playback frequency.
 */
int ZPWM::setSampleRate(int frequency)
{
    int clock_frequency = 2 * HAL_RCC_GetPCLK1Freq();
    int cyclesPerSample = clock_frequency / frequency;

    int prescaler = cyclesPerSample / 256;
    int period_ticks = clock_frequency / (prescaler * frequency);

    CODAL_ASSERT(period_ticks >= 256);
    CODAL_ASSERT(period_ticks <= 512); // in reality it should be 260 or so

    tim.Init.Period = period_ticks;
    if (IS_TIM_ADVANCED_INSTANCE(tim.Instance) && false)
    {
        // this doesn't seem to work - it indeed repeats values, but it seem to trigger
        // DMA for every cycle, so only every 1 of prescaler words in memory is actually used
        tim.Init.RepetitionCounter = prescaler - 1; // TIM1 or TIM8 only
        repCount = 1;
    }
    else
    {
        repCount = prescaler;
    }

    // Update our internal record to reflect an accurate (probably rounded) samplerate.
    sampleRate = (clock_frequency / prescaler) / period_ticks;

    LOG("PWM presc=%d period=%d freq=%d->%d", prescaler, period_ticks, frequency, sampleRate);

    auto res = HAL_TIM_PWM_Init(&tim);
    CODAL_ASSERT(res == HAL_OK);

    TIM_OC_InitTypeDef sConfig;
    memset(&sConfig, 0, sizeof(sConfig));
    sConfig.OCMode = TIM_OCMODE_PWM1;
    res = HAL_TIM_PWM_ConfigChannel(&tim, &sConfig, channels[this->channel - 1]);
    CODAL_ASSERT(res == HAL_OK);

    LOG("PWM inited");

    return DEVICE_OK;
}

extern "C" void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    // TODO use offsetof?
    for (unsigned i = 0; i < ARRAY_SIZE(instances); ++i)
    {
        if (instances[i] && &instances[i]->tim == htim)
        {
            instances[i]->irq();
        }
    }
}

/**
 * Callback provided when data is ready.
 */
int ZPWM::pullRequest()
{
    dataReady++;

    if (!active)
        nextBuffer();

    return DEVICE_OK;
}

void ZPWM::fillBuffer(uint32_t *buf)
{
    auto left = bufCnt;
    auto buflen = buf++;

    while (left)
    {
        auto len = output.length() - outptr;

        if (len <= 0)
        {
            if (!dataReady)
                break;
            dataReady--;
            output = upstream.pull();
            outptr = 0;
            len = output.length();
            if (len == 0)
                break;
        }

        auto src = (uint16_t *)&output[outptr];
        auto num = len >> 1;
        if (num > left)
            num = left;
        outptr += num << 1;

        for (uint32_t i = 0; i < num; ++i)
        {
            uint32_t s = src[i] >> 2;
            auto n = repCount;
            while (n--)
                *buf++ = s;
        }

        left -= num;
    }

    *buflen = bufCnt - left;
}

void ZPWM::nextBuffer()
{
    if (!buf0)
    {
        if (tim.Init.RepetitionCounter)
            bufCnt = 300 / (1 + tim.Init.RepetitionCounter);
        else
            bufCnt = 300 / repCount;
        buf0 = new uint32_t[bufCnt * repCount + 1];
        buf1 = new uint32_t[bufCnt * repCount + 1];
    }

    // do we have something to send?
    // we need to start sending, before we start filling buffers
    // to avoid gaps
    auto buf = buf0;
    if (!*buf)
        buf = buf1;
    if (*buf)
    {
        auto ch = channels[this->channel - 1];
        auto res = HAL_TIM_PWM_Start_DMA(&tim, ch, buf + 1, *buf * repCount);
        // no longer something to send
        *buf = 0;
        CODAL_ASSERT(res == HAL_OK);
        active = true;
    }
    else
    {
        active = false;
        buf = NULL;
    }

    // if buf0 wasn't what we're sending and it's empty, fill it
    if (buf != buf0 && !*buf0)
        fillBuffer(buf0);
    // ditto for buf1
    if (buf != buf1 && !*buf1)
        fillBuffer(buf1);

    // if we didn't start playing and now have some data to play, try again
    if (buf == NULL && *buf0)
        nextBuffer();
}

/**
 * Base implementation of a DMA callback
 */
void ZPWM::irq()
{
    // once the sequence has finished playing, load up the next buffer.
    nextBuffer();
}

/**
 * Enable this component
 */
void ZPWM::enable()
{
    enabled = true;
    //    PWM.ENABLE = 1;
}

/**
 * Disable this component
 */
void ZPWM::disable()
{
    enabled = false;
    __HAL_TIM_DISABLE(&tim);
}
