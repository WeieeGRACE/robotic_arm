/**
 * @file  motor.h
 * @brief 底盘电机驱动接口 (RZ7899 ×2, MCPWM)
 *
 * M1 = 右电机, 驱动右侧履带
 *      IN1/BI → IO16,  IN2/FI → IO17
 * M2 = 左电机, 驱动左侧履带
 *      IN1/BI → IO18,  IN2/FI → IO19
 *
 * 速度约定:
 *   +1.0 = IN2(FI) PWM, IN1(BI)=0  → 全速前进
 *   -1.0 = IN1(BI) PWM, IN2(FI)=0  → 全速后退
 *    0.0 = 两路均 0                 → 滑行停止
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*------ 电机 ID ------*/
typedef enum {
    MOTOR_M1 = 0,   /* 右电机 (Right) */
    MOTOR_M2 = 1,   /* 左电机 (Left)  */
    MOTOR_COUNT
} motor_id_t;

/*------ 电机引脚 ------*/
#define MOTOR_M1_IN1_GPIO  16   /* M1 BI (Backward Input)  */
#define MOTOR_M1_IN2_GPIO  17   /* M1 FI (Forward Input)   */
#define MOTOR_M2_IN1_GPIO  18   /* M2 BI (Backward Input)  */
#define MOTOR_M2_IN2_GPIO  19   /* M2 FI (Forward Input)   */

/*------ MCPWM 参数 ------*/
#define MOTOR_MCPWM_GROUP      0           /* MCPWM group 0             */
#define MOTOR_MCPWM_FREQ_HZ    1000        /* 1 kHz — 比 10k 扭矩大, 略有蜂鸣可接受 */
#define MOTOR_MCPWM_RESOLUTION 10          /* 10-bit, 0~1023            */
#define MOTOR_MCPWM_PERIOD     ((1U << MOTOR_MCPWM_RESOLUTION) - 1)  /* 1023 */

/*------ 速度约束 ------*/
#define MOTOR_SPEED_LIMIT      0.96f       /* 电压上限: 11.5V / 12V ≈ 96% 占空比 */
#define MOTOR_DEAD_ZONE        0.33f       /* 4V 分界线                         */
#define MOTOR_STEP_FAST        0.036f      /* <4V 加速 (原 0.012 ×3)            */
#define MOTOR_STEP_SLOW        0.009f      /* >4V 加速 (原 0.003 ×3)            */
#define MOTOR_STEP_DECEL       0.006f      /* 减速不变                          */

/**
 * @brief 初始化两路电机 MCPWM 驱动
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t motor_init(void);

/**
 * @brief 设置单个电机目标速度 (带电压限幅 + 斜坡平滑)
 *
 *  内部自动:
 *    1. 限幅到 [-MOTOR_SPEED_LIMIT, +MOTOR_SPEED_LIMIT]
 *    2. 两段式斜坡: <4V 快, >4V 慢, 减速不变
 *
 * @param motor MOTOR_M1(右) 或 MOTOR_M2(左)
 * @param speed 目标速度 -1.0 ~ +1.0 (超范围自动截断)
 * @return ESP_OK 成功
 */
esp_err_t motor_set_speed(motor_id_t motor, float speed);

/**
 * @brief 停止单个电机 (滑行) — 立即清零, 不经过斜坡
 */
esp_err_t motor_stop(motor_id_t motor);

/**
 * @brief 直接设置电机速度 (跳过斜坡, 立即生效)
 */
esp_err_t motor_set_speed_immediate(motor_id_t motor, float speed);

/**
 * @brief 读取当前实际输出速度 (斜坡跟踪值)
 * @return -1.0 ~ +1.0
 */
float motor_get_actual_speed(motor_id_t motor);

/**
 * @brief 短接制动 (IN1=IN2=HIGH) — 电机快速停止
 */
esp_err_t motor_brake(motor_id_t motor);

/**
 * @brief 释放制动, 恢复滑行 (IN1=IN2=LOW)
 */
esp_err_t motor_coast(motor_id_t motor);

#ifdef __cplusplus
}
#endif
