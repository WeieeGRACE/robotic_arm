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

/*------ 脉宽范围 (微秒) ------
 *
 * TBS2701 ×5: 典型 500~2500μs
 * ZP15S 夹爪: 数据手册标称 500~2500μs，
 *             但实际行程短，建议上电后实测：
 *             全开→记录 pulse_open, 全闭→记录 pulse_close，
 *             用 (pulse_open, pulse_close) 替换 (MIN, MAX) 可避免堵转烧毁。
 *
 * 以下为初始化默认值，运行时必须可覆盖。
 *----------------------------------------------*/
#define SERVO_PULSE_MIN_US 500                       /* 对应 0°基准 */
#define SERVO_PULSE_MAX_US 2500                      /* 对应 180°基准 */
#define SERVO_PERIOD_US (1000000UL / SERVO_PWM_FREQ) /* 20000μs */

/*------ 占空比边界 (编译期预计算) ------
 *  duty = pulse_us / period_us × (2^bits)
 *  MIN  = 500/20000 × 4096 = 102
 *  MAX  = 2500/20000 × 4096 = 512
 *----------------------------------------------*/
#define SERVO_DUTY_MIN ((uint32_t)((uint64_t)SERVO_PULSE_MIN_US * (1UL << SERVO_PWM_BITS) / SERVO_PERIOD_US))

#define SERVO_DUTY_MAX ((uint32_t)((uint64_t)SERVO_PULSE_MAX_US * (1UL << SERVO_PWM_BITS) / SERVO_PERIOD_US))

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
/*------ 默认缓动时长 (毫秒) ------
 *
 *  舵机从 0° 到 180° 的完整过渡时间。
 *  实际时长会按角度差等比缩放。
 *  800~1200ms 是机械臂常用范围：
 *    太短 → 惯性冲击，小车会晃
 *    太慢 → 响应迟钝，自动抓取节奏拖沓
 *----------------------------------------------*/
#define SERVO_EASING_DURATION_MS 1000
/*------ 各舵机独立缓动时长 (可选覆盖) ------
 *
 *  若需要某轴单独更慢/更快，在此定义。
 *  只有定义了才生效，否则用上面的默认值。
 *  建议: 夹爪(ZP15S)给 600ms，大关节给 1000~1200ms。
 *----------------------------------------------*/
#define SERVO_EASING_BASE_MS 1200 /* 基座旋转，负载较大，慢一点 */
#define SERVO_EASING_JOINT1_MS 1000
#define SERVO_EASING_JOINT2_MS 1000
#define SERVO_EASING_JOINT3_MS 800  /* 末端关节，载荷小，快一点 */
#define SERVO_EASING_ROTATE_MS 600  /* 夹爪旋转 */
#define SERVO_EASING_GRIPPER_MS 600 /* 夹爪开合，ZP15S 行程短 */

#endif /* SERVO_CFG_H */
