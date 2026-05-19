/*=====================================================
 *  servo_util.c
 *
 *  职责: 纯计算工具函数集合。
 *        不操作任何硬件，不调用 LEDC，不读写 GPIO。
 *
 *  依赖: servo_cfg.h (配置宏 + TBL 查表宏)
 *        math.h      (fabsf, fminf, fmaxf)
 *
 *  这些函数供 servo_init.c / servo_set.c 调用。
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

/*------ 浮点比较容差 ------*/
#define EPSILON 1e-6f

/*=====================================================
 *  servo_angle_to_duty()
 *
 *  将物理角度 [0°, 180°] 线性映射为 LEDC 占空比。
 *  每个舵机使用独立的 DUTY_MIN / DUTY_MAX。
 *
 *  参数:
 *    id  - ServoID_t 舵机编号
 *    deg - 目标角度，越界自动钳位
 *
 *  返回:
 *    LEDC duty 值 [0, (2^SERVO_PWM_BITS)-1]
 *=====================================================*/
uint32_t servo_angle_to_duty(ServoID_t id, float deg)
{
    if (id < 0 || id >= SERVO_COUNT)
    {
        ESP_LOGE(TAG, "angle_to_duty: invalid id %d", id);
        return s_duty_min[0]; /* 安全回退 */
    }

    /* 钳位到物理极限 */
    if (deg < SERVO_ANGLE_MIN_DEG)
        deg = SERVO_ANGLE_MIN_DEG;
    if (deg > SERVO_ANGLE_MAX_DEG)
        deg = SERVO_ANGLE_MAX_DEG;

    /* 线性映射: duty = min + deg/180 * (max - min) */
    uint32_t min_d = s_duty_min[id];
    uint32_t max_d = s_duty_max[id];
    uint32_t span = max_d - min_d;

    uint32_t duty = min_d + (uint32_t)(deg / SERVO_ANGLE_MAX_DEG * (float)span + 0.5f);
    return duty;
}

/*=====================================================
 *  servo_duty_to_angle()
 *
 *  占空比反查: LEDC duty → 物理角度 [0°, 180°]
 *  用于调试/读取当前舵机软件缓存位置。
 *
 *  参数:
 *    id   - ServoID_t 舵机编号
 *    duty - 当前 LEDC duty 值
 *
 *  返回:
 *    对应角度，钳位到 [0, 180]
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

    /* 防除零: 若 span 为 0 (理论配置错误)，直接返回 90° */
    if (span == 0)
    {
        ESP_LOGW(TAG, "duty_to_angle[%d]: duty span is zero!", id);
        return 90.0f;
    }

    /* 反向映射: deg = (duty - min) / span * 180 */
    float deg;
    if (duty <= min_d)
    {
        deg = SERVO_ANGLE_MIN_DEG;
    }
    else if (duty >= max_d)
    {
        deg = SERVO_ANGLE_MAX_DEG;
    }
    else
    {
        deg = (float)(duty - min_d) / (float)span * SERVO_ANGLE_MAX_DEG;
    }

    /* 最终钳位 */
    if (deg < SERVO_ANGLE_MIN_DEG)
        deg = SERVO_ANGLE_MIN_DEG;
    if (deg > SERVO_ANGLE_MAX_DEG)
        deg = SERVO_ANGLE_MAX_DEG;

    return deg;
}

/*=====================================================
 *  servo_easing_quintic()
 *
 *  五次多项式缓动函数:
 *    s(τ) = 10τ³ − 15τ⁴ + 6τ⁵
 *
 *  数学性质:
 *    s(0) = 0,  s(1) = 1
 *    s'(0) = s'(1) = 0     (起止速度为零)
 *    s''(0) = s''(1) = 0   (起止加速度为零)
 *
 *  参数:
 *    tau - 归一化时间 [0.0, 1.0]，越界自动钳位
 *
 *  返回:
 *    缓动因子 [0.0, 1.0]
 *=====================================================*/
float servo_easing_quintic(float tau)
{
    /* 钳位，防止负数或超过 1 产生怪异值 */
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
 *
 *  带缓动的线性插值:
 *    current = from + servo_easing_quintic(tau) * (to - from)
 *
 *  典型调用方式 (在 servo_set.c 中):
 *    float tau = elapsed_ms / duration_ms;
 *    float cur = servo_easing_lerp(tau, start_angle, target_angle);
 *    servo_write_hardware(id, cur);
 *
 *  参数:
 *    tau       - 归一化时间 [0, 1]
 *    from      - 起始角度
 *    to        - 目标角度
 *
 *  返回:
 *    当前时刻的角度值
 *=====================================================*/
float servo_easing_lerp(float tau, float from, float to)
{
    float factor = servo_easing_quintic(tau); /* 0.0 ~ 1.0 */
    return from + factor * (to - from);
}

/*=====================================================
 *  servo_get_easing_duration()
 *
 *  按 ServoID_t 查表返回该舵机的缓动时长 (毫秒)。
 *  若未定义独立时长，回退到 SERVO_EASING_DURATION_MS。
 *
 *  参数:
 *    id - ServoID_t 舵机编号
 *
 *  返回:
 *    缓动总时长 (ms)
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
 *
 *  按 ServoID_t 查表返回该舵机的脉宽边界 (微秒)。
 *  用于 servo_set.c 在写硬件前做最终脉宽校验/钳位。
 *
 *  参数:
 *    id      - ServoID_t 舵机编号
 *    out_min - 输出: 最小脉宽 (μs)
 *    out_max - 输出: 最大脉宽 (μs)
 *
 *  若 id 非法，out_min / out_max 不被修改。
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