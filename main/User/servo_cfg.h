#ifndef SERVO_CFG_H
#define SERVO_CFG_H

/*=====================================================
 *  文件职责:
 *  舵机驱动唯一配置文件，只放宏/枚举，零函数声明。
 *  其他模块通过 #include "servo_cfg.h" 引用。
 *
 *  修改引脚 / 通道前必须核对 app_config.h 全架构映射。
 *=====================================================*/

/*------ 舵机数量 ------*/
#define SERVO_COUNT 6

/*------ 舵机 ID 枚举 (0-based, 用于数组索引) ------*/
typedef enum
{
    SERVO_ID_BASE = 0,    /* IO15 基座旋转 */
    SERVO_ID_JOINT1 = 1,  /* IO14 肩关节   */
    SERVO_ID_JOINT2 = 2,  /* IO13 肘关节   */
    SERVO_ID_JOINT3 = 3,  /* IO12 腕关节   */
    SERVO_ID_ROTATE = 4,  /* IO11 夹子旋转 */
    SERVO_ID_GRIPPER = 5, /* IO10 夹子开合 (ZP15S) */
} ServoID_t;

/*------ GPIO 引脚映射 (与全局引脚宪法一致) ------*/
#define SERVO_GPIO_BASE 15    /* IO15 */
#define SERVO_GPIO_JOINT1 14  /* IO14 */
#define SERVO_GPIO_JOINT2 13  /* IO13 */
#define SERVO_GPIO_JOINT3 12  /* IO12 */
#define SERVO_GPIO_ROTATE 11  /* IO11 */
#define SERVO_GPIO_GRIPPER 10 /* IO10 */

/*------ LEDC 通道映射 (CH0~CH5) ------*/
#define SERVO_CH_BASE 0
#define SERVO_CH_JOINT1 1
#define SERVO_CH_JOINT2 2
#define SERVO_CH_JOINT3 3
#define SERVO_CH_ROTATE 4
#define SERVO_CH_GRIPPER 5

/*------ LEDC 定时器参数 ------*/
#define SERVO_LEDC_SPEED LEDC_LOW_SPEED_MODE /* ESP32-P4 只有低速模式 */
#define SERVO_LEDC_TIMER LEDC_TIMER_0        /* 共享同一硬件定时器 */
#define SERVO_PWM_FREQ 50                    /* Hz, 舵机标准周期 */
#define SERVO_PWM_BITS 12                    /* 0~4095 占空比精度 */

/*------ Period ------*/
#define SERVO_PERIOD_US (1000000UL / SERVO_PWM_FREQ) /* 20000μs */

/*------ 单舵机独立脉宽范围 (微秒) ------
 *
 * TBS2701 ×5: 典型标称 500~2500μs，上电实测后修正
 * ZP15S 夹爪: 标称 500~2500μs，行程短，上电实测后修正
 *
 * 未校准时先用默认值，校准后只改这一段。
 *----------------------------------------------*/
/* TBS2701 × 5 */
#define SERVO_PULSE_MIN_BASE 500
#define SERVO_PULSE_MAX_BASE 2500
#define SERVO_PULSE_MIN_JOINT1 500
#define SERVO_PULSE_MAX_JOINT1 2500
#define SERVO_PULSE_MIN_JOINT2 500
#define SERVO_PULSE_MAX_JOINT2 2500
#define SERVO_PULSE_MIN_JOINT3 500
#define SERVO_PULSE_MAX_JOINT3 2500
#define SERVO_PULSE_MIN_ROTATE 500
#define SERVO_PULSE_MAX_ROTATE 2500
/* ZP15S 夹爪 */
#define SERVO_PULSE_MIN_GRIPPER 500
#define SERVO_PULSE_MAX_GRIPPER 2500

/*------ 占空比边界 (编译期预计算，每舵机独立) ------
 *  duty = pulse_us / period_us × (2^bits)
 *----------------------------------------------*/
#define SERVO_DUTY_MIN_BASE ((uint32_t)((uint64_t)SERVO_PULSE_MIN_BASE * (1UL << SERVO_PWM_BITS) / SERVO_PERIOD_US))
#define SERVO_DUTY_MAX_BASE ((uint32_t)((uint64_t)SERVO_PULSE_MAX_BASE * (1UL << SERVO_PWM_BITS) / SERVO_PERIOD_US))
#define SERVO_DUTY_MIN_JOINT1 ((uint32_t)((uint64_t)SERVO_PULSE_MIN_JOINT1 * (1UL << SERVO_PWM_BITS) / SERVO_PERIOD_US))
#define SERVO_DUTY_MAX_JOINT1 ((uint32_t)((uint64_t)SERVO_PULSE_MAX_JOINT1 * (1UL << SERVO_PWM_BITS) / SERVO_PERIOD_US))
#define SERVO_DUTY_MIN_JOINT2 ((uint32_t)((uint64_t)SERVO_PULSE_MIN_JOINT2 * (1UL << SERVO_PWM_BITS) / SERVO_PERIOD_US))
#define SERVO_DUTY_MAX_JOINT2 ((uint32_t)((uint64_t)SERVO_PULSE_MAX_JOINT2 * (1UL << SERVO_PWM_BITS) / SERVO_PERIOD_US))
#define SERVO_DUTY_MIN_JOINT3 ((uint32_t)((uint64_t)SERVO_PULSE_MIN_JOINT3 * (1UL << SERVO_PWM_BITS) / SERVO_PERIOD_US))
#define SERVO_DUTY_MAX_JOINT3 ((uint32_t)((uint64_t)SERVO_PULSE_MAX_JOINT3 * (1UL << SERVO_PWM_BITS) / SERVO_PERIOD_US))
#define SERVO_DUTY_MIN_ROTATE ((uint32_t)((uint64_t)SERVO_PULSE_MIN_ROTATE * (1UL << SERVO_PWM_BITS) / SERVO_PERIOD_US))
#define SERVO_DUTY_MAX_ROTATE ((uint32_t)((uint64_t)SERVO_PULSE_MAX_ROTATE * (1UL << SERVO_PWM_BITS) / SERVO_PERIOD_US))
#define SERVO_DUTY_MIN_GRIPPER ((uint32_t)((uint64_t)SERVO_PULSE_MIN_GRIPPER * (1UL << SERVO_PWM_BITS) / SERVO_PERIOD_US))
#define SERVO_DUTY_MAX_GRIPPER ((uint32_t)((uint64_t)SERVO_PULSE_MAX_GRIPPER * (1UL << SERVO_PWM_BITS) / SERVO_PERIOD_US))

/*------ 安全限位 (度) ------
 *  每个关节物理行程不同，上电后必须实测修正。
 *  超出此范围 IK 解算必须拒绝执行。
 *----------------------------------------------*/
#define SERVO_ANGLE_MIN_DEG 0.0f
#define SERVO_ANGLE_MAX_DEG 180.0f

/*------ 上电安全姿态 (度) ------
 *  对应 ServoID_t 顺序。
 *  机械臂收拢 + 夹爪全开，防止上电瞬间甩臂。
 *  根据实际组装姿态调整。
 *----------------------------------------------*/
#define SERVO_HOME_BASE 90.0f
#define SERVO_HOME_JOINT1 45.0f
#define SERVO_HOME_JOINT2 135.0f
#define SERVO_HOME_JOINT3 90.0f
#define SERVO_HOME_ROTATE 90.0f
#define SERVO_HOME_GRIPPER 0.0f /* 0=全开 (ZP15S) */

/*=======================================================
 *  五次多项式缓动 (Quintic Easing) 配置
 *
 *  原理: s(τ) = 10τ³ - 15τ⁴ + 6τ⁵,  τ ∈ [0, 1]
 *  特性: 起止位置、速度、加速度全部连续且为零
 *  用途: 舵机平滑过渡，避免机械冲击
 *
 *  更新机制: esp_timer 周期回调，每 EASING_TICK_MS 毫秒
 *           计算一次目标占空比并写入 LEDC。
 *=======================================================*/

/*------ 更新周期 (毫秒) ------
 *
 *  20ms = 50Hz，对舵机信号响应足够，CPU 占用极低。
 *  ≤10ms 开始有边际收益递减，≥50ms 会感觉卡顿。
 *  不建议改。
 *----------------------------------------------*/
#define SERVO_EASING_TICK_MS 20

/*------ 默认缓动时长 (毫秒) [备用，向后兼容] ------
 *
 *  已被下方 SERVO_EASING_xxx_MS 各舵机独立配置取代。
 *  保留用于未指定独立时长时的兜底值。
 *----------------------------------------------*/
#define SERVO_EASING_DURATION_MS 1000

/*------ 各舵机独立缓动时长 (毫秒) ------
 *
 *  建议范围:
 *    大负载关节(BASE/JOINT1): 1000~1200ms
 *    小负载关节(JOINT2/JOINT3): 800~1000ms
 *    ZP15S 夹爪: 500~700ms
 *----------------------------------------------*/
#define SERVO_EASING_BASE_MS 1200
#define SERVO_EASING_JOINT1_MS 1000
#define SERVO_EASING_JOINT2_MS 1000
#define SERVO_EASING_JOINT3_MS 800
#define SERVO_EASING_ROTATE_MS 600
#define SERVO_EASING_GRIPPER_MS 600

/*=======================================================
 *  查表辅助宏
 *
 *  用法 (在 .c 文件中):
 *    static const uint32_t s_duty_min[SERVO_COUNT] = SERVO_DUTY_MIN_TBL;
 *    uint32_t val = s_duty_min[SERVO_ID_JOINT2];
 *
 *  维护规则: 元素顺序必须严格对应 ServoID_t 枚举顺序。
 *=======================================================*/

#define SERVO_DUTY_MIN_TBL                         \
    {SERVO_DUTY_MIN_BASE, SERVO_DUTY_MIN_JOINT1,   \
     SERVO_DUTY_MIN_JOINT2, SERVO_DUTY_MIN_JOINT3, \
     SERVO_DUTY_MIN_ROTATE, SERVO_DUTY_MIN_GRIPPER}

#define SERVO_DUTY_MAX_TBL                         \
    {SERVO_DUTY_MAX_BASE, SERVO_DUTY_MAX_JOINT1,   \
     SERVO_DUTY_MAX_JOINT2, SERVO_DUTY_MAX_JOINT3, \
     SERVO_DUTY_MAX_ROTATE, SERVO_DUTY_MAX_GRIPPER}

#define SERVO_PULSE_MIN_TBL                          \
    {SERVO_PULSE_MIN_BASE, SERVO_PULSE_MIN_JOINT1,   \
     SERVO_PULSE_MIN_JOINT2, SERVO_PULSE_MIN_JOINT3, \
     SERVO_PULSE_MIN_ROTATE, SERVO_PULSE_MIN_GRIPPER}

#define SERVO_PULSE_MAX_TBL                          \
    {SERVO_PULSE_MAX_BASE, SERVO_PULSE_MAX_JOINT1,   \
     SERVO_PULSE_MAX_JOINT2, SERVO_PULSE_MAX_JOINT3, \
     SERVO_PULSE_MAX_ROTATE, SERVO_PULSE_MAX_GRIPPER}

#define SERVO_EASING_MS_TBL                          \
    {SERVO_EASING_BASE_MS, SERVO_EASING_JOINT1_MS,   \
     SERVO_EASING_JOINT2_MS, SERVO_EASING_JOINT3_MS, \
     SERVO_EASING_ROTATE_MS, SERVO_EASING_GRIPPER_MS}

#endif /* SERVO_CFG_H */