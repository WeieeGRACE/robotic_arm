/**
 * @file  chassis.c
 * @brief 履带底盘差速驱动实现
 *
 * 依赖 motor 层 (M1=右电机, M2=左电机)
 *
 * 差速混控公式:
 *   left  = clamp(linear - angular, -1, +1)
 *   right = clamp(linear + angular, -1, +1)
 *
 * 物理意义:
 *   angular > 0 → 左轮减速 / 右轮加速 → 车体逆时针旋转 (CCW, 左转)
 */

#include "chassis.h"
#include "motor.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "chassis";

/*===============================================================
 *  clampf — 限幅到 [-1, +1]
 *===============================================================*/
static inline float clampf(float v)
{
    if (v > 1.0f)
        return 1.0f;
    if (v < -1.0f)
        return -1.0f;
    return v;
}

/*===============================================================
 *  chassis_init — 初始化底盘
 *===============================================================*/
esp_err_t chassis_init(void)
{
    esp_err_t ret = motor_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Chassis init failed: motor driver error");
        return ret;
    }
    ESP_LOGI(TAG, "Chassis init OK");
    return ESP_OK;
}

/*===============================================================
 *  chassis_move_vector — 核心混控接口
 *===============================================================*/
esp_err_t chassis_move_vector(float linear, float angular)
{
    float left = clampf(linear - angular);
    float right = clampf(linear + angular);

    esp_err_t ret;

    ret = motor_set_speed(MOTOR_M2, left);
    if (ret != ESP_OK)
        return ret;

    ret = motor_set_speed(MOTOR_M1, right);
    if (ret != ESP_OK)
        return ret;

    return ESP_OK;
}

/*===============================================================
 *  chassis_set_speeds — 独立轮速
 *===============================================================*/
esp_err_t chassis_set_speeds(float left, float right)
{
    esp_err_t ret;

    ret = motor_set_speed(MOTOR_M2, clampf(left));
    if (ret != ESP_OK)
        return ret;

    ret = motor_set_speed(MOTOR_M1, clampf(right));
    if (ret != ESP_OK)
        return ret;

    return ESP_OK;
}

/*===============================================================
 *  便捷运动接口
 *===============================================================*/

esp_err_t chassis_forward(float speed)
{
    return chassis_move_vector(clampf(speed), 0.0f);
}

esp_err_t chassis_backward(float speed)
{
    return chassis_move_vector(-clampf(speed), 0.0f);
}

esp_err_t chassis_turn_left(float speed)
{
    /* 右轮全速, 左轮停止 → 绕左履带 pivot 左转 */
    return chassis_set_speeds(0.0f, clampf(speed));
}

esp_err_t chassis_turn_right(float speed)
{
    /* 左轮全速, 右轮停止 → 绕右履带 pivot 右转 */
    return chassis_set_speeds(clampf(speed), 0.0f);
}

esp_err_t chassis_rotate_left(float speed)
{
    /* 原地左转: 左后退 + 右前进 */
    float s = clampf(speed);
    return chassis_set_speeds(-s, s);
}

esp_err_t chassis_rotate_right(float speed)
{
    /* 原地右转: 左前进 + 右后退 */
    float s = clampf(speed);
    return chassis_set_speeds(s, -s);
}

/*===============================================================
 *  chassis_stop — 协调减速 (两电机同步降速)
 *
 *  每次调用以较快电机的速度为准, 两轮同幅度降低,
 *  保证同时到达零速, 不会出现一侧先停一侧还在转.
 *===============================================================*/
esp_err_t chassis_stop(void)
{
    float l = motor_get_actual_speed(MOTOR_M2);
    float r = motor_get_actual_speed(MOTOR_M1);

    float al = fabsf(l);
    float ar = fabsf(r);
    float max_abs = (al > ar) ? al : ar;

    /* 已停止 */
    if (max_abs < 0.001f) {
        motor_set_speed_immediate(MOTOR_M2, 0.0f);
        motor_set_speed_immediate(MOTOR_M1, 0.0f);
        return ESP_OK;
    }

    /* 步进量: 不超过当前最高速度 */
    float step = (MOTOR_STEP_DECEL < max_abs) ? MOTOR_STEP_DECEL : max_abs;
    float ratio = 1.0f - (step / max_abs);

    float new_l = l * ratio;
    float new_r = r * ratio;

    motor_set_speed_immediate(MOTOR_M2, new_l);
    motor_set_speed_immediate(MOTOR_M1, new_r);

    return ESP_OK;
}

/*===============================================================
 *  chassis_brake / chassis_coast — 短接制动与释放
 *===============================================================*/
esp_err_t chassis_brake(void)
{
    motor_brake(MOTOR_M2);
    motor_brake(MOTOR_M1);
    return ESP_OK;
}

esp_err_t chassis_coast(void)
{
    motor_coast(MOTOR_M2);
    motor_coast(MOTOR_M1);
    return ESP_OK;
}
