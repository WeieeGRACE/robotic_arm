/**
 * @file  cartesian.c
 * @brief 笛卡尔直线插补 — 等距IK采样 + 轨迹引擎执行
 */

#include "cartesian.h"
#include "kinematics.h"
#include "arm_control.h"
#include "servo_set.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "cartesian";

#define LINE_STEP_MM    10.0f    /* 采样间距 */
#define MAX_WAYPOINTS    40      /* 最大路径点数 (≤堆栈安全) */

/* 计算两点欧氏距离 */
static float dist3d(float x1, float y1, float z1,
                    float x2, float y2, float z2)
{
    float dx = x2 - x1, dy = y2 - y1, dz = z2 - z1;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

bool cartesian_move_line(float x1, float y1, float z1,
                         float x2, float y2, float z2,
                         float speed_mms)
{
    float total_dist = dist3d(x1, y1, z1, x2, y2, z2);
    if (total_dist < 1.0f) return true; /* 已在目标 */

    int num_segs = (int)(total_dist / LINE_STEP_MM + 0.5f);
    if (num_segs < 1) num_segs = 1;
    if (num_segs > MAX_WAYPOINTS) num_segs = MAX_WAYPOINTS;

    float seg_time_ms = (total_dist / speed_mms) * 1000.0f / (float)num_segs;
    if (seg_time_ms < 50.0f) seg_time_ms = 50.0f; /* 最短 50ms/段 */

    ESP_LOGI(TAG, "Line: (%.0f,%.0f,%.0f)→(%.0f,%.0f,%.0f) "
             "dist=%.0fmm %dsegs %.0fms/seg",
             (double)x1, (double)y1, (double)z1,
             (double)x2, (double)y2, (double)z2,
             (double)total_dist, num_segs, (double)seg_time_ms);

    /* 采样 + IK (静态分配, 避免栈溢出) */
    static TrajPoint_t waypoints[MAX_WAYPOINTS];
    for (int i = 1; i <= num_segs; i++) {
        float t = (float)i / (float)num_segs;
        float x = x1 + (x2 - x1) * t;
        float y = y1 + (y2 - y1) * t;
        float z = z1 + (z2 - z1) * t;

        ArmTipState_t target = { .x = x, .y = y, .z = z, .yaw = 0.0f };
        JointAngles_t geom;
        if (kinematics_inverse(&target, &geom) != IK_OK) {
            ESP_LOGE(TAG, "Line IK failed at waypoint %d: (%.0f,%.0f,%.0f)",
                     i, (double)x, (double)y, (double)z);
            return false;
        }

        ServoAngles_t servo;
        kinematics_geom_to_servo(&geom, &servo);
        for (int j = 0; j < SERVO_COUNT; j++) {
            waypoints[i - 1].angles[j] = ((float *)&servo)[j];
        }
        waypoints[i - 1].duration_ms = seg_time_ms;
    }

    traj_start(waypoints, num_segs);
    traj_wait_done();
    return true;
}

bool cartesian_move_to(float x, float y, float z, float speed_mms)
{
    ArmTipState_t current;
    arm_control_get_current_tip(&current);

    return cartesian_move_line(current.x, current.y, current.z,
                               x, y, z, speed_mms);
}
