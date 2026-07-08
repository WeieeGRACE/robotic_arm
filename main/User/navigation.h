/**
 * @file  navigation.h
 * @brief 视觉伺服导航层 — 将目标坐标转为底盘运动指令
 *
 * 坐标系 (机器人坐标系, 俯视图):
 *   +X = 机器人正前方
 *   +Y = 机器人右侧
 *   角度: + = 逆时针 (CCW), 单位弧度
 *
 * 控制策略:
 *   航向误差大 → 原地旋转对准
 *   航向误差小 → 直线前进 + 航向微调
 *   进入阈值范围 → 自动停止
 */

#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*------ 导航参数 (均可按需调整) ------*/

/** @brief 目标距离阈值 (米), 进入此范围视为到达 */
#define NAV_ARRIVE_DIST_M     0.10f

/** @brief 先转向再前进的航向误差阈值 (弧度), 约 10° */
#define NAV_TURN_FIRST_RAD    0.175f

/** @brief 航向 P 增益 (角速度 = 增益 × 航向误差) */
#define NAV_KP_HEADING        0.8f

/** @brief 距离 P 增益 (线速度 = 增益 × 距离) */
#define NAV_KP_DISTANCE       0.5f

/** @brief 最大线速度幅值 */
#define NAV_MAX_LINEAR        0.4f

/** @brief 最大角速度幅值 */
#define NAV_MAX_ANGULAR       0.5f

/*===============================================================
 *  API
 *===============================================================*/

/**
 * @brief 初始化导航层 (内部调用 chassis_init)
 */
esp_err_t nav_init(void);

/**
 * @brief 导航主函数 — 每帧调用一次 (视觉循环中)
 *
 * @param target_x_m  目标在机器人前方的距离 (米), >0 = 前方
 * @param target_y_m  目标在机器人右侧的距离 (米), >0 = 右侧
 * @param arrived     [out] true = 已进入到达阈值, 底盘已停止
 * @return ESP_OK 正常
 */
esp_err_t nav_move_to(float target_x_m, float target_y_m, bool *arrived);

/**
 * @brief 停止导航, 平滑停车
 */
esp_err_t nav_stop(void);

#ifdef __cplusplus
}
#endif
