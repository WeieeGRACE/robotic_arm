/*=====================================================
 *  servo_set.c
 *
 *  职责: 舵机运行时控制 — 目标角度设置 + 五次多项式缓动
 *
 *  核心机制:
 *    esp_timer 周期回调 (20ms) 逐一更新 6 路舵机
 *    每路独立维护缓动状态，互不干扰
 *
 *  线程安全说明:
 *    esp_timer 回调在专用 timer task 上下文执行，
 *    与 control_task 并发。32 位对齐 float/uint32
 *    在 RISC-V 双核上读写是原子的，无需加锁。
 *=====================================================*/

#include "servo_cfg.h"
#include "servo_util.h" /* ← 新增 */
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "servo_set";

/*------ 每舵机缓动状态 ------*/
typedef struct
{
    float start_angle;       /* 缓动起始角度 */
    float target_angle;      /* 目标角度 */
    float current_angle;     /* 当前实际角度 (每帧更新) */
    uint32_t elapsed_ticks;  /* 已经过 tick 数 */
    uint32_t total_ticks;    /* 总 tick 需求 */
    volatile bool is_active; /* 是否正在缓动 */
} ServoEasingState_t;

static ServoEasingState_t s_state[SERVO_COUNT];
static esp_timer_handle_t s_timer = NULL;
static bool s_initialized = false;

/*------ 缓动角度触发阈值 (度) ------*/
#define EASING_DELTA 0.1f

/*------ LEDC 通道查表 (与 ServoID_t 顺序对应) ------*/
static const ledc_channel_t s_channel_tbl[SERVO_COUNT] = {
    SERVO_CH_BASE,
    SERVO_CH_JOINT1,
    SERVO_CH_JOINT2,
    SERVO_CH_JOINT3,
    SERVO_CH_ROTATE,
    SERVO_CH_GRIPPER,
};

/*------ 内部工具函数 (与 servo_write_hardware 紧邻，位置不可移动) ------*/
static inline ledc_channel_t get_channel(ServoID_t id)
{
    return s_channel_tbl[id];
}

static inline void servo_write_hardware(ServoID_t id, float deg)
{
    /* deg 已在 servo_angle_to_duty() 内部做钳位，这里直接用 */
    uint32_t duty = servo_angle_to_duty(id, deg);
    ledc_set_duty(SERVO_LEDC_SPEED, get_channel(id), duty);
    ledc_update_duty(SERVO_LEDC_SPEED, get_channel(id));
}

/*=====================================================
 *  定时器回调 — 每 20ms 被 esp_timer 触发一次
 *=====================================================*/
static void timer_callback(void *arg)
{
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        ServoEasingState_t *st = &s_state[i];
        if (!st->is_active)
            continue;

        if (st->elapsed_ticks >= st->total_ticks)
        {
            /* 缓动完成: 确保最终值精确 */
            st->current_angle = st->target_angle;
            servo_write_hardware((ServoID_t)i, st->current_angle);
            st->is_active = false;
            continue;
        }

        /* 归一化时间 tau ∈ [0, 1] */
        float tau = (float)st->elapsed_ticks / (float)st->total_ticks;
        /* 五次多项式插值 */
        st->current_angle = servo_easing_lerp(tau,
                                              st->start_angle,
                                              st->target_angle);
        servo_write_hardware((ServoID_t)i, st->current_angle);
        st->elapsed_ticks++;
    }
}

/*=====================================================
 *  servo_set_init()
 *
 *  初始化舵机状态数组 + 创建/重建 esp_timer
 *  先销毁旧 timer 再创建新的，支持重复调用。
 *  必须在 servo_init() 之后调用
 *=====================================================*/
void servo_set_init(void)
{
    /* HOME 角度数组 (与 ServoID_t 顺序对应) */
    const float home_angles[SERVO_COUNT] = {
        SERVO_HOME_BASE,
        SERVO_HOME_JOINT1,
        SERVO_HOME_JOINT2,
        SERVO_HOME_JOINT3,
        SERVO_HOME_ROTATE,
        SERVO_HOME_GRIPPER,
    };

    for (int i = 0; i < SERVO_COUNT; i++)
    {
        s_state[i].start_angle = home_angles[i];
        s_state[i].target_angle = home_angles[i];
        s_state[i].current_angle = home_angles[i];
        s_state[i].elapsed_ticks = 0;
        s_state[i].total_ticks = 0;
        s_state[i].is_active = false;
    }

    /* 先销毁旧 timer (支持重复调用) */
    if (s_timer != NULL)
    {
        esp_timer_stop(s_timer);
        esp_timer_delete(s_timer);
        s_timer = NULL;
        ESP_LOGW(TAG, "Old easing timer deleted");
    }

    /* 创建新的周期 timer */
    const esp_timer_create_args_t timer_args = {
        .callback = timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "servo_easing",
    };

    esp_err_t ret = esp_timer_create(&timer_args, &s_timer);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_timer_create failed: %s", esp_err_to_name(ret));
        return;
    }

    uint64_t period_us = (uint64_t)SERVO_EASING_TICK_MS * 1000;
    ret = esp_timer_start_periodic(s_timer, period_us);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_timer_start_periodic failed: %s", esp_err_to_name(ret));
        return;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Servo easing timer started [%dms tick, %d servos]",
             SERVO_EASING_TICK_MS, SERVO_COUNT);
}

/*=====================================================
 *  servo_set_angle()
 *
 *  缓动未完成时再次调用 → 覆盖旧目标，从 current_angle 出发
 *  angle 与 current_angle 差值 < epsilon → 无动作
 *  start_angle = current_angle，零跳变
 *=====================================================*/
void servo_set_angle(ServoID_t id, float angle)
{
    if (!s_initialized)
    {
        ESP_LOGE(TAG, "set_angle[%d]: not initialized!", id);
        return;
    }
    if (id < 0 || id >= SERVO_COUNT)
        return;

    /* 钳位到物理极限 */
    if (angle < SERVO_ANGLE_MIN_DEG)
        angle = SERVO_ANGLE_MIN_DEG;
    if (angle > SERVO_ANGLE_MAX_DEG)
        angle = SERVO_ANGLE_MAX_DEG;

    /* 差值小于阈值，无动作 */
    float delta = fabsf(angle - s_state[id].current_angle);
    if (delta < EASING_DELTA)
        return;

    /* 从 current_angle 出发，去跳变 */
    s_state[id].start_angle = s_state[id].current_angle;
    s_state[id].target_angle = angle;

    /* 计算所需 tick 数: duration_ms / tick_ms × (delta/180)
     * 按角度比例缩放, 至少保持 1 个 tick */
    uint32_t duration_ms = servo_get_easing_duration(id);
    float ratio = delta / SERVO_ANGLE_MAX_DEG;
    uint32_t ticks = (uint32_t)((float)duration_ms * ratio / (float)SERVO_EASING_TICK_MS + 0.5f);
    if (ticks < 1)
        ticks = 1;

    s_state[id].elapsed_ticks = 0;
    s_state[id].total_ticks = ticks;
    s_state[id].is_active = true;
}

/*=====================================================
 *  servo_set_joints()
 *
 *  一次性设置全部 6 轴目标 (IK 解算结果直接喂入)
 *  某些轴不在 IK 视野内 (JOINT3, ROTATE, GRIPPER)，
 *  传入当前值或 HOME 值即可，本函数自动识别并跳过无变化轴。
 *=====================================================*/
void servo_set_joints(const float angles[SERVO_COUNT])
{
    if (!s_initialized)
    {
        ESP_LOGE(TAG, "set_joints: not initialized!");
        return;
    }
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        servo_set_angle((ServoID_t)i, angles[i]);
    }
}

/*=====================================================
 *  servo_stop()
 *
 *  停止单个舵机的缓动，current_angle 保持在当前位置。
 *=====================================================*/
void servo_stop(ServoID_t id)
{
    if (id < 0 || id >= SERVO_COUNT)
        return;
    if (!s_initialized)
        return;

    s_state[id].is_active = false;
    ESP_LOGI(TAG, "Servo [%d] stopped at %.1f°", id, s_state[id].current_angle);
}

/*=====================================================
 *  servo_stop_all()
 *
 *  停止全部舵机缓动，用于底盘紧急刹车 / 手柄强接管。
 *=====================================================*/
void servo_stop_all(void)
{
    if (!s_initialized)
        return;

    for (int i = 0; i < SERVO_COUNT; i++)
    {
        s_state[i].is_active = false;
    }
    ESP_LOGI(TAG, "All servos stopped");
}