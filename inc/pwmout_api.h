#pragma once 

#ifdef __cplusplus
extern "C" {
#endif

#include "samd/pins.h"

typedef struct {
    uint32_t period;
    uint32_t pulse;

    const mcu_pin_obj_t *pin;
    const pin_timer_t* timer;
    bool variable_frequency;
} pulseio_pwmout_obj_t;

typedef pulseio_pwmout_obj_t pwmout_t;

void pwmout_init(pwmout_t* obj, uint32_t pin);
void pwmout_free(pwmout_t* obj);
void pwmout_write(pwmout_t* obj, uint32_t pulse);
void pwmout_period_us(pwmout_t* obj, int us);

#ifdef __cplusplus
}
#endif
