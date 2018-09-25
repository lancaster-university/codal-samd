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

int pwmout_init(pwmout_t *obj, uint32_t pin, uint32_t pulse, uint32_t period);
void pwmout_free(pwmout_t *obj);
int pwmout_write(pwmout_t *obj, uint32_t pulse, uint32_t period);

#ifdef __cplusplus
}
#endif
