/**
 * @file  navigation.c
 * @brief 视觉伺服导航实现
 *
 * 控制逻辑 (每帧执行):
 *   1. 计算目标距离 d 和航向误差 θ
 *   2. d < 阈值 → 到达, 停车
 *   3. |θ| > 转向阈值 → 原地旋转对准 (linear=0)
 *   4. 否则 → 直线前进 + 航向微调 (linear=Kp_d·d, angular=Kp_a·θ)
 */

#include "navigation.h"
#include "chassis.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "nav";

/*===============================================================
 *  nav_init
 *===============================================================*/
esp_err_t nav_init(void)
{
    esp_err_t ret = chassis_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Nav init failed: chassis error");
        return ret;
    }
    ESP_LOGI(TAG, "Nav init OK  [arrive=%.2fm, turn_first=%.0f°, "
             "Kp_h=%.1f, Kp_d=%.1f]",
             (double)NAV_ARRIVE_DIST_M,
             (double)(NAV_TURN_FIRST_RAD * 180.0f / (float)M_PI),
             (double)NAV_KP_HEADING,
             (double)NAV_KP_DISTANCE);
    return ESP_OK;
}

/*===============================================================
 *  clampf_nav — 限幅到 [-limit, +limit]
 *===============================================================*/
static inline float clampf_nav(float v, float limit)
{
    if (v >  limit) return  limit;
    if (v < -limit) return -limit;
    return v;
}

/*===============================================================
 *  nav_move_to — 导航主循环
 *===============================================================*/
esp_err_t nav_move_to(float target_x_m, float target_y_m, bool *arrived)
{
    /* 1. 计算极坐标 */
    float distance = sqrtf(target_x_m * target_x_m + target_y_m * target_y_m);
    float heading  = atan2f(target_y_m, target_x_m);  /* [-π, +π], CCW > 0 */

    /* 2. 到达判定 */
    if (distance < NAV_ARRIVE_DIST_M) {
        chassis_stop();
        if (arrived) *arrived = true;
        return ESP_OK;
    }

    if (arrived) *arrived = false;

    float linear  = 0.0f;
    float angular = 0.0f;

    /* 3. 控制律 */
    if (fabsf(heading) > NAV_TURN_FIRST_RAD) {
        /* 航向偏差大: 原地旋转对准, 不前进 */
        angular = clampf_nav(NAV_KP_HEADING * heading, NAV_MAX_ANGULAR);
        linear  = 0.0f;
    } else {
        /* 航向在容许范围内: 前进 + 微调 */
        linear  = clampf_nav(NAV_KP_DISTANCE * distance, NAV_MAX_LINEAR);
        angular = clampf_nav(NAV_KP_HEADING * heading,    NAV_MAX_ANGULAR);
    }

    ESP_LOGD(TAG, "tgt=(%.2f,%.2f) d=%.2f h=%.1f° → L=%.2f A=%.2f",
             (double)target_x_m, (double)target_y_m,
             (double)distance, (double)(heading * 180.0f / (float)M_PI),
             (double)linear, (double)angular);

    return chassis_move_vector(linear, angular);
}

/*===============================================================
 *  nav_stop
 *===============================================================*/
esp_err_t nav_stop(void)
{
    ESP_LOGI(TAG, "Nav stop requested");
    return chassis_stop();
}
