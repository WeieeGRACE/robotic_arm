/**
 * @file  chassis.h
 * @brief 履带底盘差速驱动抽象层
 *
 * 基于 motor 层提供的独立轮速控制，封装:
 *   - 差速混控 (linear + angular → left/right)
 *   - 视觉伺服可直接调用 chassis_move_vector()
 *   - 便捷运动接口 (forward/backward/turn/rotate/stop)
 *
 * 坐标系约定 (俯视图):
 *   +linear  = 前进
 *   +angular = 逆时针旋转 (CCW, 左转)
 *
 * motor 层自动处理:
 *   - 电压限幅: 最大 ±0.62 (≈7.4V / 12V 电池)
 *   - 斜坡平滑: 每次调速最大步进 0.03, 避免速度突变
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化底盘 (内部调用 motor_init)
 * @return ESP_OK 成功
 */
esp_err_t chassis_init(void);

/*===============================================================
 *  核心接口: 视觉伺服导航层直接调用
 *===============================================================*/

/**
 * @brief 差速混控: 线速度 + 角速度 → 左右轮速
 *
 *   左轮速 = clamp(linear - angular, -1, +1)
 *   右轮速 = clamp(linear + angular, -1, +1)
 *
 * @param linear  线速度  -1.0 (全速后退) ~ +1.0 (全速前进)
 * @param angular 角速度  -1.0 (顺时针)   ~ +1.0 (逆时针)
 *                  angular > 0 → 向左转弯 / 逆时针旋转
 * @return ESP_OK
 */
esp_err_t chassis_move_vector(float linear, float angular);

/**
 * @brief 独立设置左右履带速度
 * @param left  左履带速度 (-1.0 ~ +1.0)
 * @param right 右履带速度 (-1.0 ~ +1.0)
 * @return ESP_OK
 */
esp_err_t chassis_set_speeds(float left, float right);

/*===============================================================
 *  便捷运动接口
 *===============================================================*/

/** @brief 直线前进 */
esp_err_t chassis_forward(float speed);

/** @brief 直线后退 */
esp_err_t chassis_backward(float speed);

/**
 * @brief 差速左转 (右轮全速, 左轮停止 → 绕左履带 pivot)
 * @param speed 转弯速度 (0 ~ 1.0)
 */
esp_err_t chassis_turn_left(float speed);

/**
 * @brief 差速右转 (左轮全速, 右轮停止 → 绕右履带 pivot)
 * @param speed 转弯速度 (0 ~ 1.0)
 */
esp_err_t chassis_turn_right(float speed);

/**
 * @brief 原地左转 (左轮后退, 右轮前进)
 * @param speed 旋转速度 (0 ~ 1.0)
 */
esp_err_t chassis_rotate_left(float speed);

/**
 * @brief 原地右转 (左轮前进, 右轮后退)
 * @param speed 旋转速度 (0 ~ 1.0)
 */
esp_err_t chassis_rotate_right(float speed);

/**
 * @brief 平滑停止 (目标速度=0, 经斜坡减速)
 *
 *   单次调用以 MOTOR_STEP_DECEL 速率协调减速,
 *   视觉控制循环需持续调用直到实际速度为 0.
 *   如需立即急停, 直接调用 motor_stop().
 */
esp_err_t chassis_stop(void);

/**
 * @brief 短接制动 — 两电机同时刹停
 */
esp_err_t chassis_brake(void);

/**
 * @brief 释放制动, 恢复滑行
 */
esp_err_t chassis_coast(void);

#ifdef __cplusplus
}
#endif
