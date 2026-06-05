/**
 * @file  collision.c
 * @brief 自碰撞检测 — 线性斜坡履带模型 + 地面 + 立柱
 */

#include "collision.h"
#include "kinematics.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "collision";

/* 履带斜坡: z_track(r) = a*r + b (150 ≤ r ≤ 192) */
static const float TRACK_SLOPE = (TRACK_Z_FRONT - TRACK_Z_REAR)
                               / (TRACK_R_FRONT - TRACK_R_REAR);
static const float TRACK_INTERCEPT = TRACK_Z_REAR - TRACK_SLOPE * TRACK_R_REAR;

/* 履带角度范围: 底座角在此范围内时 tip 在履带侧方 */
#define TRACK_ANGLE_MIN  30.0f
#define TRACK_ANGLE_MAX 150.0f

/* 履带表面高度 at radius r, 仅当 r∈[R_REAR, R_FRONT] 有意义 */
static inline float track_z_at_r(float r)
{
    return TRACK_SLOPE * r + TRACK_INTERCEPT;
}

/* 底座角是否在履带侧方范围内 */
static inline bool in_track_angle(float theta0_deg)
{
    return (theta0_deg >= TRACK_ANGLE_MIN && theta0_deg <= TRACK_ANGLE_MAX);
}

/* 半径是否在履带范围内 */
static inline bool in_track_radius(float r)
{
    return (r >= TRACK_R_REAR && r <= TRACK_R_FRONT);
}

/* 检测单个臂点 (r,z) 是否撞履带 */
static CollisionResult_t check_point_track(float r, float z, float theta0)
{
    if (!in_track_radius(r) || !in_track_angle(theta0))
        return COLLISION_OK;

    float z_track = track_z_at_r(r);
    if (z < z_track) {
        return (theta0 < 90.0f) ? COLLISION_TRACK_LEFT
                                 : COLLISION_TRACK_RIGHT;
    }
    return COLLISION_OK;
}

CollisionResult_t collision_check(const JointAngles_t *geom)
{
    if (!geom) return COLLISION_OK;

    float t1 = geom->theta1 * M_PI / 180.0f;
    float t2 = geom->theta2 * M_PI / 180.0f;
    float t3 = geom->theta3 * M_PI / 180.0f;
    float d1 = kinematics_get_d1();
    float l3 = kinematics_get_l3();

    /* 臂上 3 个关键点: J2(肘), J3(腕), Tip(末端) */
    float r_j2 = ARM_L1 * cosf(t1);
    float z_j2 = d1 + ARM_L1 * sinf(t1);

    float r_j3 = r_j2 + ARM_L2 * cosf(t1 + t2);
    float z_j3 = z_j2 + ARM_L2 * sinf(t1 + t2);

    float r_tip = r_j3 + l3 * cosf(t1 + t2 + t3);
    float z_tip = z_j3 + l3 * sinf(t1 + t2 + t3);

    /* 1. 地面碰撞 (tip) */
    if (z_tip < GROUND_MARGIN) {
        ESP_LOGW(TAG, "Ground: tip z=%.1f < %.0f", (double)z_tip, (double)GROUND_MARGIN);
        return COLLISION_GROUND;
    }

    /* 2. 履带碰撞 — 沿臂采样: J2, J3, Tip + 连杆中间点 */
    CollisionResult_t cr;

    /* 上臂 (J1→J2) 采样 3 点 */
    for (int k = 1; k <= 3; k++) {
        float frac = k / 4.0f;
        float r_pt = ARM_L1 * cosf(t1) * frac;
        float z_pt = d1 + ARM_L1 * sinf(t1) * frac;
        cr = check_point_track(r_pt, z_pt, geom->theta0);
        if (cr != COLLISION_OK) {
            ESP_LOGW(TAG, "Track[L1/%d]: r=%.0f z=%.0f", k,
                     (double)r_pt, (double)z_pt);
            return cr;
        }
    }

    /* 前臂 (J2→J3) 采样 4 点 */
    for (int k = 1; k <= 4; k++) {
        float frac = k / 5.0f;
        float r_pt = r_j2 + ARM_L2 * cosf(t1 + t2) * frac;
        float z_pt = z_j2 + ARM_L2 * sinf(t1 + t2) * frac;
        cr = check_point_track(r_pt, z_pt, geom->theta0);
        if (cr != COLLISION_OK) {
            ESP_LOGW(TAG, "Track[L2/%d]: r=%.0f z=%.0f", k,
                     (double)r_pt, (double)z_pt);
            return cr;
        }
    }

    /* 末端 (J3→Tip) 采样 3 点 */
    for (int k = 1; k <= 3; k++) {
        float frac = k / 4.0f;
        float r_pt = r_j3 + l3 * cosf(t1 + t2 + t3) * frac;
        float z_pt = z_j3 + l3 * sinf(t1 + t2 + t3) * frac;
        cr = check_point_track(r_pt, z_pt, geom->theta0);
        if (cr != COLLISION_OK) {
            ESP_LOGW(TAG, "Track[L3/%d]: r=%.0f z=%.0f", k,
                     (double)r_pt, (double)z_pt);
            return cr;
        }
    }

    /* 3. 云台立柱: J2 不能太靠近底座 */
    if (r_j2 < BASE_COLUMN_RADIUS && z_j2 < (d1 + 30.0f)) {
        ESP_LOGW(TAG, "Column: J2 r=%.0f < %.0f",
                 (double)r_j2, (double)BASE_COLUMN_RADIUS);
        return COLLISION_BASE_COLUMN;
    }

    return COLLISION_OK;
}

CollisionResult_t collision_check_rotate(float from_deg, float to_deg,
                                         const JointAngles_t *geom)
{
    if (!geom) return COLLISION_OK;

    float t1 = geom->theta1 * M_PI / 180.0f;
    float t2 = geom->theta2 * M_PI / 180.0f;
    float t3 = geom->theta3 * M_PI / 180.0f;
    float d1 = kinematics_get_d1();
    float l3 = kinematics_get_l3();

    float r_j2 = ARM_L1 * cosf(t1);
    float z_j2 = d1 + ARM_L1 * sinf(t1);
    float r_j3 = r_j2 + ARM_L2 * cosf(t1 + t2);
    float z_j3 = z_j2 + ARM_L2 * sinf(t1 + t2);
    float r_tip = r_j3 + l3 * cosf(t1 + t2 + t3);
    float z_tip = z_j3 + l3 * sinf(t1 + t2 + t3);

    float f_min = from_deg < to_deg ? from_deg : to_deg;
    float f_max = from_deg > to_deg ? from_deg : to_deg;

    /* 旋转区间与履带角度区无交集 → 安全 */
    if (!(f_max > TRACK_ANGLE_MIN && f_min < TRACK_ANGLE_MAX))
        return COLLISION_OK;

    /* 沿臂采样检测旋转扫过 */
    /* 上臂 3 点 */
    for (int k = 1; k <= 3; k++) {
        float frac = k / 4.0f;
        float r_pt = ARM_L1 * cosf(t1) * frac;
        float z_pt = d1 + ARM_L1 * sinf(t1) * frac;
        if (in_track_radius(r_pt) && z_pt < track_z_at_r(r_pt)) {
            ESP_LOGW(TAG, "Sweep[L1]: r=%.0f z=%.0f", (double)r_pt, (double)z_pt);
            return COLLISION_SWEEP;
        }
    }
    /* 前臂 4 点 */
    for (int k = 1; k <= 4; k++) {
        float frac = k / 5.0f;
        float r_pt = r_j2 + ARM_L2 * cosf(t1 + t2) * frac;
        float z_pt = z_j2 + ARM_L2 * sinf(t1 + t2) * frac;
        if (in_track_radius(r_pt) && z_pt < track_z_at_r(r_pt)) {
            ESP_LOGW(TAG, "Sweep[L2]: r=%.0f z=%.0f", (double)r_pt, (double)z_pt);
            return COLLISION_SWEEP;
        }
    }
    /* 末端 3 点 */
    for (int k = 1; k <= 3; k++) {
        float frac = k / 4.0f;
        float r_pt = r_j3 + l3 * cosf(t1 + t2 + t3) * frac;
        float z_pt = z_j3 + l3 * sinf(t1 + t2 + t3) * frac;
        if (in_track_radius(r_pt) && z_pt < track_z_at_r(r_pt)) {
            ESP_LOGW(TAG, "Sweep[L3]: r=%.0f z=%.0f", (double)r_pt, (double)z_pt);
            return COLLISION_SWEEP;
        }
    }

    return COLLISION_OK;
}

const char *collision_str(CollisionResult_t r)
{
    switch (r) {
    case COLLISION_OK:           return "OK";
    case COLLISION_GROUND:       return "GROUND";
    case COLLISION_TRACK_LEFT:   return "TRACK_LEFT";
    case COLLISION_TRACK_RIGHT:  return "TRACK_RIGHT";
    case COLLISION_BASE_COLUMN:  return "BASE_COLUMN";
    case COLLISION_SWEEP:        return "SWEEP";
    default:                     return "UNKNOWN";
    }
}
