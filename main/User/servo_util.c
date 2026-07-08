/*=====================================================
 *  servo_util.c
 *
 *  职责: 纯计算工具函数集合。
 *        不操作任何硬件，不调用 LEDC，不读写 GPIO。
 *=====================================================*/

#include "servo_cfg.h"
#include <stdint.h>
#include <math.h>
#include "esp_log.h"

static const char *TAG = "servo_util";

/*------ 查表数组 (编译期初始化，运行时只读) ------*/
static const uint32_t s_duty_min[SERVO_COUNT] = SERVO_DUTY_MIN_TBL;
static const uint32_t s_duty_max[SERVO_COUNT] = SERVO_DUTY_MAX_TBL;
static const uint32_t s_pulse_min[SERVO_COUNT] = SERVO_PULSE_MIN_TBL;
static const uint32_t s_pulse_max[SERVO_COUNT] = SERVO_PULSE_MAX_TBL;
static const uint32_t s_easing_ms[SERVO_COUNT] = SERVO_EASING_MS_TBL;
static const int8_t s_direction[SERVO_COUNT] = SERVO_DIR_TBL;

/*=====================================================
 *  servo_angle_to_duty()
 *
 *  将逻辑角度 [0°, 180°] 映射为 LEDC 占空比。
 *  支持正向和反向映射。
 *  角度越界会自动钳位。
 *=====================================================*/
uint32_t servo_angle_to_duty(ServoID_t id, float deg)
{
    if (id < 0 || id >= SERVO_COUNT)
    {
        ESP_LOGE(TAG, "angle_to_duty: invalid id %d", id);
        return s_duty_min[0];
    }

    /* 钳位: 底座可负角度(左转), 其余关节限 [0,180] */
    if (id == SERVO_ID_BASE) {
        if (deg < -90.0f) deg = -90.0f;
        if (deg > 180.0f) deg = 180.0f;
    } else {
        if (deg < SERVO_ANGLE_MIN_DEG) deg = SERVO_ANGLE_MIN_DEG;
        if (deg > SERVO_ANGLE_MAX_DEG) deg = SERVO_ANGLE_MAX_DEG;
    }

    uint32_t min_d = s_duty_min[id];
    uint32_t max_d = s_duty_max[id];
    uint32_t span  = max_d - min_d;

    float ratio = deg / SERVO_ANGLE_MAX_DEG;
    float foffs = ratio * (float)span;
    int32_t offs = (int32_t)(foffs + (foffs >= 0 ? 0.5f : -0.5f));

    if (s_direction[id] >= 0) {
        /* 正向: duty = min + ratio*span */
        int32_t d = (int32_t)min_d + offs;
        if (d < 0) d = 0;
        if (d > 4095) d = 4095;
        return (uint32_t)d;
    } else {
        /* 反向: duty = max - ratio*span */
        int32_t d = (int32_t)max_d - offs;
        if (d < 0) d = 0;
        if (d > 4095) d = 4095;
        return (uint32_t)d;
    }
}

/*=====================================================
 *  servo_duty_to_angle()
 *
 *  占空比反查 -> 逻辑角度 [0°, 180°]
 *  支持方向，用于调试。
 *=====================================================*/
float servo_duty_to_angle(ServoID_t id, uint32_t duty)
{
    if (id < 0 || id >= SERVO_COUNT)
    {
        ESP_LOGE(TAG, "duty_to_angle: invalid id %d", id);
        return -1.0f;
    }

    uint32_t min_d = s_duty_min[id];
    uint32_t max_d = s_duty_max[id];
    uint32_t span = max_d - min_d;

    if (span == 0)
    {
        /* MIN=MAX 表示固定角度，返回 HOME 角度（或任何值） */
        ESP_LOGW(TAG, "duty_to_angle[%d]: zero span (fixed joint), return 90°", id);
        return 90.0f;
    }

    float deg;
    if (duty <= min_d)
    {
        deg = (s_direction[id] >= 0) ? SERVO_ANGLE_MIN_DEG : SERVO_ANGLE_MAX_DEG;
    }
    else if (duty >= max_d)
    {
        deg = (s_direction[id] >= 0) ? SERVO_ANGLE_MAX_DEG : SERVO_ANGLE_MIN_DEG;
    }
    else
    {
        float ratio = (float)(duty - min_d) / (float)span;
        if (s_direction[id] >= 0)
        {
            deg = ratio * SERVO_ANGLE_MAX_DEG;
        }
        else
        {
            deg = (1.0f - ratio) * SERVO_ANGLE_MAX_DEG;
        }
    }

    /* 钳位 */
    if (deg < SERVO_ANGLE_MIN_DEG)
        deg = SERVO_ANGLE_MIN_DEG;
    if (deg > SERVO_ANGLE_MAX_DEG)
        deg = SERVO_ANGLE_MAX_DEG;
    return deg;
}

/*=====================================================
 *  servo_easing_quintic()
 *
 *  s(τ) = 10τ³ - 15τ⁴ + 6τ⁵
 *=====================================================*/
float servo_easing_quintic(float tau)
{
    if (tau < 0.0f)
        tau = 0.0f;
    if (tau > 1.0f)
        tau = 1.0f;

    float tau2 = tau * tau;
    float tau3 = tau2 * tau;
    float tau4 = tau3 * tau;
    float tau5 = tau4 * tau;

    return 10.0f * tau3 - 15.0f * tau4 + 6.0f * tau5;
}

/*=====================================================
 *  servo_easing_lerp()
 *=====================================================*/
float servo_easing_lerp(float tau, float from, float to)
{
    float factor = servo_easing_quintic(tau);
    return from + factor * (to - from);
}

/*=====================================================
 *  servo_get_easing_duration()
 *=====================================================*/
uint32_t servo_get_easing_duration(ServoID_t id)
{
    if (id < 0 || id >= SERVO_COUNT)
    {
        ESP_LOGW(TAG, "get_easing_duration: invalid id %d, use default", id);
        return SERVO_EASING_DURATION_MS;
    }
    return s_easing_ms[id];
}

/*=====================================================
 *  servo_get_pulse_range()
 *=====================================================*/
void servo_get_pulse_range(ServoID_t id,
                           uint32_t *out_min,
                           uint32_t *out_max)
{
    if (id < 0 || id >= SERVO_COUNT)
    {
        ESP_LOGE(TAG, "get_pulse_range: invalid id %d", id);
        return;
    }
    if (out_min)
        *out_min = s_pulse_min[id];
    if (out_max)
        *out_max = s_pulse_max[id];
}