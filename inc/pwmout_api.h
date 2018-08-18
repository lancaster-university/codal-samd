#pragma once 

#ifdef __cplusplus
extern "C" {
#endif

struct pwmout_s {
    uint32_t pwm;
    uint32_t pin;
    uint32_t prescaler;
    uint32_t period;
    uint32_t pulse;
    uint8_t channel;
    uint8_t inverted;
};

typedef struct pwmout_s pwmout_t;

void pwmout_init(pwmout_t* obj, uint32_t pin);
void pwmout_free(pwmout_t* obj);
void pwmout_write(pwmout_t* obj, uint32_t pulse);
void pwmout_period_us(pwmout_t* obj, int us);

#ifdef __cplusplus
}
#endif
