/**
 * @file  collision.h
 * @brief 自碰撞检测 — 履带/地面/云台碰撞检查
 */

#ifndef COLLISION_H
#define COLLISION_H

#include "kinematics.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*=======================================================
 *  车辆履带几何参数 (mm) — 老公实测
 *
 *  履带表面近似为一次线性斜坡:
 *    后方高点: r=150, z=125
 *    前方低点: r=192, z=72
 *    z_track(r) = 125 - 1.262*(r - 150)  (150 ≤ r ≤ 192)
 *
 *  左右对称, 底座旋转中心在车体中轴
 *=======================================================*/
#define TRACK_R_REAR    150.0f  /* 履带后点距底座中心水平距离 */
#define TRACK_Z_REAR    135.0f  /* 履带后点离地高度 + 10mm安全余量 */
#define TRACK_R_FRONT   192.0f  /* 履带前点距底座中心水平距离 */
#define TRACK_Z_FRONT    72.0f  /* 履带前点离地高度 (最低点) */
#define BASE_COLUMN_RADIUS  45.0f  /* 云台立柱半径 */
#define GROUND_MARGIN        5.0f  /* 地面安全余量 */

/**
 * @brief 碰撞检测结果
 */
typedef enum {
    COLLISION_OK = 0,
    COLLISION_GROUND,       /* 末端低于地面 */
    COLLISION_TRACK_LEFT,   /* 末端打到左履带 */
    COLLISION_TRACK_RIGHT,  /* 末端打到右履带 */
    COLLISION_BASE_COLUMN,  /* J2肘部撞到云台立柱 */
    COLLISION_SWEEP,        /* 底座旋转扫过履带区 */
} CollisionResult_t;

/**
 * @brief 检测当前关节位姿是否安全
 * @param geom  几何关节角
 * @return COLLISION_OK 安全, 其他值=碰撞类型
 */
CollisionResult_t collision_check(const JointAngles_t *geom);

/**
 * @brief 检测底座从 from_deg 旋转到 to_deg 是否扫过履带
 * @param from_deg  起始底座角
 * @param to_deg    目标底座角
 * @param geom      关节几何角 (θ1~θ3 不变)
 * @return COLLISION_OK 或 COLLISION_SWEEP
 */
CollisionResult_t collision_check_rotate(float from_deg, float to_deg,
                                         const JointAngles_t *geom);

/**
 * @brief 获取碰撞类型字符串 (用于日志)
 */
const char *collision_str(CollisionResult_t r);

#ifdef __cplusplus
}
#endif

#endif /* COLLISION_H */
