#ifndef SERVO_UTIL_H
#define SERVO_UTIL_H

#include "servo_cfg.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    uint32_t servo_angle_to_duty(ServoID_t id, float deg);
    float servo_duty_to_angle(ServoID_t id, uint32_t duty);
    float servo_easing_quintic(float tau);
    float servo_easing_lerp(float tau, float from, float to);
    uint32_t servo_get_easing_duration(ServoID_t id);
    void servo_get_pulse_range(ServoID_t id, uint32_t *out_min, uint32_t *out_max);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_UTIL_H */