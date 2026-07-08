/**
 * @file  vision_coordinator.h
 * @brief 视觉协调器 — 底盘导航 + 机械臂抓取 统一调度
 *
 * 状态机:
 *   IDLE → NAVIGATING (目标>290mm, 底盘靠近)
 *        → STOPPING    (目标≤290mm, 停车确认)
 *        → PICKING     (可达检查 → 抬臂 → 抓取 → 夹紧 → 抬起)
 *        → HOLDING     (保持5秒)
 *        → RELEASING   (松开 → 回安全位)
 *        → IDLE
 */

#ifndef VISION_COORDINATOR_H
#define VISION_COORDINATOR_H

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 初始化协调器 (内部调用 nav_init) */
esp_err_t vc_init(void);

/**
 * @brief 协调器主循环 — 每帧调用一次 (代替原来 main.c 的手动逻辑)
 *
 *        内部自动读取视觉坐标、状态切换、调用底盘和机械臂。
 *        非阻塞，适合在主循环中周期调用。
 *
 * @param period_ms  调用周期 (ms), 建议 50
 */
void vc_run(int period_ms);

#ifdef __cplusplus
}
#endif

#endif /* VISION_COORDINATOR_H */
