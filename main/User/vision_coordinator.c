/**
 * @file  vision_coordinator.c
 * @brief 视觉协调器 — 底盘导航 + 机械臂抓取 统一调度
 */

#include "vision_coordinator.h"
#include "vision_uart.h"
#include "chassis.h"
#include "motor.h"
#include "arm_control.h"
#include "kinematics.h"
#include "servo_cfg.h"
#include "servo_set.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "vc";

/*------ 可调参数 ------*/
#define ARM_REACH_MM      310.0f
#define PICK_Z_MM         180.0f
#define HOLD_MS           5000
#define MATCH_COUNT       3
#define ANGLE_CORR_DEG    -2.0f

/*------ 状态 ------*/
typedef enum {
    S_IDLE = 0,
    S_NAVIGATING,
    S_PICKING,
    S_HOLDING,
    S_RELEASING,
} vc_state_t;

static vc_state_t      s_state = S_IDLE;
static vision_target_t s_target;
static vision_target_t s_match_ref;
static int             s_match_cnt = 0;
static TickType_t      s_hold_start = 0;

/*------ 内部函数 ------*/
static void lift_arm(void) {
    servo_set_angle(SERVO_ID_JOINT1, 60.0f);
    servo_set_angle(SERVO_ID_JOINT2, 180.0f);
    servo_set_angle(SERVO_ID_JOINT3, 176.0f);
    servo_wait_all_idle();
}

static bool target_reachable(float x, float y, float z) {
    ArmTipState_t tgt = { .x = x, .y = y, .z = z, .yaw = 0.0f };
    JointAngles_t geom;
    if (kinematics_inverse(&tgt, &geom) != IK_OK) return false;
    return kinematics_check_limits(&geom);
}

static float target_dist_mm(float x, float y) {
    return sqrtf(x * x + y * y);
}

/* 急刹车: 最大减速+短接制动 */
static void smooth_brake(void) {
    /* 快速降速: 每次减0.05, 20ms一次, 0.65→0约13帧(260ms) */
    for (int i = 0; i < 20; i++) {
        float l = motor_get_actual_speed(MOTOR_M2);
        float r = motor_get_actual_speed(MOTOR_M1);
        float al = fabsf(l), ar = fabsf(r);
        float max_abs = (al > ar) ? al : ar;
        if (max_abs < 0.02f) break;
        float step = 0.05f;
        if (step > max_abs) step = max_abs;
        float ratio = 1.0f - step / max_abs;
        motor_set_speed_immediate(MOTOR_M2, l * ratio);
        motor_set_speed_immediate(MOTOR_M1, r * ratio);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    /* 短接制动500ms */
    chassis_brake();
    vTaskDelay(pdMS_TO_TICKS(500));
    chassis_coast();
}

/* 角度修正 + 可达检查 */
static bool check_and_correct(void) {
    float a = ANGLE_CORR_DEG * 3.14159f / 180.0f;
    float ca = cosf(a), sa = sinf(a);
    float cx = (float)s_target.x_mm, cy = (float)s_target.y_mm;
    float rx = cx * ca - cy * sa;
    float ry = cx * sa + cy * ca;
    return target_reachable(rx, ry, PICK_Z_MM);
}

/*===============================================================
 *  vc_init
 *===============================================================*/
esp_err_t vc_init(void) {
    /* chassis_init 已在 main.c 中提前调用, 这里只初始化状态 */
    s_state = S_IDLE;
    ESP_LOGI(TAG, "Coordinator OK [reach=%.0fmm]", ARM_REACH_MM);
    return ESP_OK;
}

/*===============================================================
 *  vc_run — 主循环入口, 50ms 周期
 *===============================================================*/
void vc_run(int period_ms) {
    vision_uart_poll();
    bool fresh = vision_uart_get_target(&s_target);

    switch (s_state) {

    case S_IDLE:
        if (!fresh || !s_target.valid) {
            s_match_cnt = 0;  /* 无目标时清零, 避免旧数据干扰 */
            break;
        }

        if (s_match_cnt == 0 ||
            abs(s_target.x_mm - s_match_ref.x_mm) > 5 ||
            abs(s_target.y_mm - s_match_ref.y_mm) > 5) {
            s_match_ref = s_target;
            s_match_cnt = 1;
            ESP_LOGI(TAG, "Target: X=%+d Y=%+d  d=%.0fmm",
                     s_target.x_mm, s_target.y_mm,
                     (double)target_dist_mm(s_target.x_mm, s_target.y_mm));
        } else {
            s_match_cnt++;
        }

        if (s_match_cnt >= MATCH_COUNT) {
            float d = target_dist_mm(s_target.x_mm, s_target.y_mm);
            if (d <= ARM_REACH_MM && check_and_correct()) {
                ESP_LOGI(TAG, "In range (%.0fmm) → PICKING", (double)d);
                s_state = S_PICKING;
            } else {
                ESP_LOGI(TAG, "Out of range (%.0fmm) → NAVIGATING", (double)d);
                s_match_cnt = 0;
                s_state = S_NAVIGATING;
            }
        }
        break;

    case S_NAVIGATING: {
        int lost_cnt = 0;
        float nav_speed = 0.0f;
        /* 20ms 内循环, 匹配电机斜坡 */
        while (s_state == S_NAVIGATING) {
            vision_uart_poll();
            fresh = vision_uart_get_target(&s_target);

            if (!fresh || !s_target.valid) {
                lost_cnt++;
                if (lost_cnt > 20) {
                    ESP_LOGW(TAG, "Target lost (%d frames) → brake", lost_cnt);
                    smooth_brake();
                    s_match_cnt = 0;
                    s_state = S_IDLE;
                    break;
                }
                /* 短暂丢帧: 保持速度和方向 */
                float h = atan2f((float)s_target.y_mm, (float)s_target.x_mm);
                if (h >  1.047f) h =  1.047f;
                if (h < -1.047f) h = -1.047f;
                float a = -0.5f * h;
                if (a >  0.4f) a =  0.4f;
                if (a < -0.4f) a = -0.4f;
                chassis_move_vector(nav_speed, a);
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
            lost_cnt = 0;

            float d = target_dist_mm(s_target.x_mm, s_target.y_mm);
            if (d <= 320.0f) {
                ESP_LOGI(TAG, "Braking at %.0fmm", (double)d);
                smooth_brake();

                /* 检查目标状态 */
                vision_uart_poll();
                vision_uart_get_target(&s_target);
                if (!s_target.valid) {
                    /* 目标丢失 → 后退1秒再找 */
                    ESP_LOGW(TAG, "Target lost after brake → reversing 1s");
                    for (int j = 0; j < 50; j++) {
                        chassis_backward(0.7f);
                        vTaskDelay(pdMS_TO_TICKS(20));
                    }
                    smooth_brake();
                    /* 再检查一次 */
                    vision_uart_poll();
                    vision_uart_get_target(&s_target);
                    if (!s_target.valid) {
                        ESP_LOGW(TAG, "Still no target → IDLE");
                        s_match_cnt = 0;
                        s_state = S_IDLE;
                        break;
                    }
                }

                if (check_and_correct()) {
                    s_state = S_PICKING;
                } else {
                    ESP_LOGW(TAG, "Unreachable → re-nav");
                    s_state = S_NAVIGATING;
                }
                break;
            }

            /* 连续降速: 800mm=0.7, 320mm=0.59, 线性 */
            if (d > 800.0f) nav_speed = 0.7f;
            else nav_speed = 0.59f + 0.11f * (d - 320.0f) / (800.0f - 320.0f);

            /* 航向修正: ±60°限幅 */
            float heading = atan2f((float)s_target.y_mm, (float)s_target.x_mm);
            if (heading >  1.047f) heading =  1.047f;   /* +60° */
            if (heading < -1.047f) heading = -1.047f;   /* -60° */
            float angular = -0.5f * heading;  /* 负号: 目标在右→右转 */
            if (angular >  0.4f) angular =  0.4f;
            if (angular < -0.4f) angular = -0.4f;

            chassis_move_vector(nav_speed, angular);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        break;
    }

    case S_PICKING: {
        float a = ANGLE_CORR_DEG * 3.14159f / 180.0f;
        float ca = cosf(a), sa = sinf(a);
        float cx = (float)s_target.x_mm, cy = (float)s_target.y_mm;
        float rx = cx * ca - cy * sa;
        float ry = cx * sa + cy * ca;

        ESP_LOGI(TAG, "Lift → move to (%.0f,%.0f,%.0f)", (double)rx, (double)ry, PICK_Z_MM);
        lift_arm();

        if (arm_control_move_to(rx, ry, PICK_Z_MM)) {
            ArmTipState_t tip;
            arm_control_get_current_tip(&tip);
            ESP_LOGI(TAG, "FK: (%.1f,%.1f,%.1f) Δ:(%+.1f,%+.1f,%+.1f)",
                     (double)tip.x, (double)tip.y, (double)tip.z,
                     (double)(tip.x-rx), (double)(tip.y-ry), (double)(tip.z-PICK_Z_MM));

            vTaskDelay(pdMS_TO_TICKS(300));
            arm_control_set_gripper(100.0f);
            lift_arm();
            s_hold_start = xTaskGetTickCount();
            s_state = S_HOLDING;
        } else {
            ESP_LOGE(TAG, "IK FAILED → IDLE");
            s_state = S_IDLE;
            s_match_cnt = 0;
        }
        break;
    }

    case S_HOLDING:
        if ((xTaskGetTickCount() - s_hold_start) >= pdMS_TO_TICKS(HOLD_MS)) {
            arm_control_set_gripper(0.0f);
            s_state = S_RELEASING;
        }
        break;

    case S_RELEASING:
        arm_control_init();
        /* 快速消费积压的视觉数据, 5次批量poll足够追上 */
        for (int i = 0; i < 5; i++) {
            vision_uart_poll();
            vision_uart_get_target(&s_target);
        }
        memset(&s_match_ref, 0, sizeof(s_match_ref));
        ESP_LOGI(TAG, "Ready for next");
        s_match_cnt = 0;
        s_state = S_IDLE;
        break;
    }
}
