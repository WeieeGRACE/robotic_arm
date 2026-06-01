#ifndef SERVO_CFG_H
#define SERVO_CFG_H

/*=====================================================
 *  文件职责:
 *  舵机驱动唯一配置文件，只放宏/枚举，零函数声明。
 *  其他模块通过 #include "servo_cfg.h" 引用。
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

/*=======================================================
 *  单舵机独立脉宽范围 + 方向（逻辑角度边界）
 *
 *  每个舵机都是 270° 舵机，但你只使用其中一段，并定义了逻辑上的 0°~180°。
 *
 *  方向:  +1 = 脉宽增大时逻辑角度增大
 *         -1 = 脉宽减小逻辑时角度增大（反向）
 *=======================================================*/

/* Base: 逻辑 0° → 2350μs，逻辑 180° → 980μs (反向) */
#define SERVO_PULSE_MIN_BASE 980
#define SERVO_PULSE_MAX_BASE 2350
#define SERVO_DIR_BASE -1

/* Joint1: 逻辑 0° → 900μs，逻辑 180° → 2180μs (正向)
 *         90° 对应于 1540μs（内置一致性） */
#define SERVO_PULSE_MIN_JOINT1 900
#define SERVO_PULSE_MAX_JOINT1 2180
#define SERVO_DIR_JOINT1 1

/* Joint2: 逻辑 0° → 1900μs，逻辑 180° → 800μs (反向) */
#define SERVO_PULSE_MIN_JOINT2 800
#define SERVO_PULSE_MAX_JOINT2 1900
#define SERVO_DIR_JOINT2 -1

/* Joint3: 固定于 1080μs（不参与运动，逻辑角度设为90°但无关紧要） */
#define SERVO_PULSE_MIN_JOINT3 1080
#define SERVO_PULSE_MAX_JOINT3 1080
#define SERVO_DIR_JOINT3 1 /* 方向无实际作用 */

/* Rotate: 固定于 1460μs（不参与运动） */
#define SERVO_PULSE_MIN_ROTATE 1460
#define SERVO_PULSE_MAX_ROTATE 1460
#define SERVO_DIR_ROTATE 1 /* 方向无实际作用 */

/* Gripper: 0°打开 → 1440μs，180°闭合 → 2110μs (正向) */
#define SERVO_PULSE_MIN_GRIPPER 1440
#define SERVO_PULSE_MAX_GRIPPER 2110
#define SERVO_DIR_GRIPPER 1

/*------ 占空比边界 (编译期预计算，每舵机独立) ------
 *  duty = pulse_us / period_us × (2^bits)
 *  注意: SERVO_DUTY_MIN 对应较小的脉冲，SERVO_DUTY_MAX 对应较大的脉宽。
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
 *  每个关节物理行程不同，此处用你定义的逻辑边界。
 *----------------------------------------------*/
#define SERVO_ANGLE_MIN_DEG 0.0f
#define SERVO_ANGLE_MAX_DEG 180.0f

/*------ 上电安全姿态 (度) ------
 *  上电后: Base=0°, Joint1=90°, Joint2=90°,
 *         Joint3/Rotate 固定在各自脉宽，Gripper=0° (打开)
 *----------------------------------------------*/
#define SERVO_HOME_BASE 0.0f
#define SERVO_HOME_JOINT1 90.0f
#define SERVO_HOME_JOINT2 90.0f
#define SERVO_HOME_JOINT3 90.0f /* 固定关节，角度值不影响脉宽 (MIN=MAX) */
#define SERVO_HOME_ROTATE 90.0f /* 固定关节，角度值不影响脉宽 (MIN=MAX) */
#define SERVO_HOME_GRIPPER 0.0f /* 0° 打开 */

/*=======================================================
 *  五次多项式缓动 (Quintic Easing) 配置
 *
 *  原理: s(τ) = 10τ³ - 15τ⁴ + 6τ⁵,  τ ∈ [0, 1]
 *  特性: 起止位置、速度、加速度全部连续且为零
 *=======================================================*/
#define SERVO_EASING_TICK_MS 20
#define SERVO_EASING_DURATION_MS 1000 /* 默认，已被下方覆盖 */
#define SERVO_EASING_BASE_MS 1200
#define SERVO_EASING_JOINT1_MS 1000
#define SERVO_EASING_JOINT2_MS 1000
#define SERVO_EASING_JOINT3_MS 800
#define SERVO_EASING_ROTATE_MS 600
#define SERVO_EASING_GRIPPER_MS 600

/*=======================================================
 *  查表辅助宏
 *
 *  顺序必须严格对应 ServoID_t 枚举: BASE, JOINT1, JOINT2, JOINT3, ROTATE, GRIPPER
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

/* 新增：方向表 */
#define SERVO_DIR_TBL                                    \
    {SERVO_DIR_BASE, SERVO_DIR_JOINT1, SERVO_DIR_JOINT2, \
     SERVO_DIR_JOINT3, SERVO_DIR_ROTATE, SERVO_DIR_GRIPPER}

#endif /* SERVO_CFG_H */