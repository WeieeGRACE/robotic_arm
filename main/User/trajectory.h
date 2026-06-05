/**
 * @file  trajectory.h
 * @brief 关节空间轨迹插补 — 多段梯形速度曲线, 段间速度连续
 */

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "servo_cfg.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 轨迹点: 6轴目标角度 + 本段时间(ms)
 */
typedef struct {
    float angles[SERVO_COUNT];  /* 目标舵机角度 [0,180] */
    float duration_ms;          /* 从上一个点到本点的时间 (ms) */
} TrajPoint_t;

/**
 * @brief 启动多段关节空间轨迹 (非阻塞)
 *
 * @param points       轨迹点数组
 * @param num_points   点数 (至少 2 个)
 * @note  起点 = 当前舵机位置, 所以 points[0] 的 duration_ms 是从当前位置出发的时间
 */
void traj_start(const TrajPoint_t *points, int num_points);

/**
 * @brief 轨迹是否执行完毕
 */
bool traj_is_done(void);

/**
 * @brief 阻塞等待轨迹完成
 */
void traj_wait_done(void);

/**
 * @brief 紧急停止轨迹
 */
void traj_abort(void);

/**
 * @brief 快捷单段轨迹移动 (阻塞)
 * @param target      目标舵机角度 [6]
 * @param duration_ms 移动时长 (ms)
 */
void traj_move_to(const float target[SERVO_COUNT], float duration_ms);

#ifdef __cplusplus
}
#endif

#endif /* TRAJECTORY_H */
