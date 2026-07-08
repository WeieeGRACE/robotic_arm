/*=====================================================
 *  servo_set.c
 *
 *  职责: 舵机运行时控制 — 目标角度设置 + 五次多项式缓动
 *=====================================================*/

#include "servo_cfg.h"
#include "servo_util.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <math.h>
/* 已在文件头部包含 freertos/FreeRTOS.h 和 freertos/task.h，若没有则添加 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "servo_set";

typedef struct
{
    float start_angle;
    float target_angle;
    float current_angle;
    uint32_t elapsed_ticks;
    uint32_t total_ticks;
    volatile bool is_active;
} ServoEasingState_t;

static ServoEasingState_t s_state[SERVO_COUNT];
static esp_timer_handle_t s_timer = NULL;
static bool s_initialized = false;

#define EASING_DELTA 0.1f

/* 通道查表 (6个) */
static const ledc_channel_t s_channel_tbl[SERVO_COUNT] = {
    SERVO_CH_BASE,
    SERVO_CH_JOINT1,
    SERVO_CH_JOINT2,
    SERVO_CH_JOINT3,
    SERVO_CH_ROTATE,
    SERVO_CH_GRIPPER,
};

static inline ledc_channel_t get_channel(ServoID_t id)
{
    return s_channel_tbl[id];
}

static inline void servo_write_hardware(ServoID_t id, float deg)
{
    uint32_t duty = servo_angle_to_duty(id, deg);
    ledc_set_duty(SERVO_LEDC_SPEED, get_channel(id), duty);
    ledc_update_duty(SERVO_LEDC_SPEED, get_channel(id));
}

static void timer_callback(void *arg)
{
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        ServoEasingState_t *st = &s_state[i];
        if (!st->is_active)
            continue;

        if (st->elapsed_ticks >= st->total_ticks)
        {
            st->current_angle = st->target_angle;
            servo_write_hardware((ServoID_t)i, st->current_angle);
            st->is_active = false;
            continue;
        }

        float tau = (float)st->elapsed_ticks / (float)st->total_ticks;
        st->current_angle = servo_easing_lerp(tau, st->start_angle, st->target_angle);
        servo_write_hardware((ServoID_t)i, st->current_angle);
        st->elapsed_ticks++;
    }
}

void servo_set_init(void)
{
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

    if (s_timer != NULL)
    {
        esp_timer_stop(s_timer);
        esp_timer_delete(s_timer);
        s_timer = NULL;
        ESP_LOGW(TAG, "Old easing timer deleted");
    }

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

void servo_set_angle(ServoID_t id, float angle)
{
    if (!s_initialized)
    {
        ESP_LOGE(TAG, "set_angle[%d]: not initialized!", id);
        return;
    }
    if (id < 0 || id >= SERVO_COUNT)
        return;

    /* 底座允许负角度, 其余关节钳位 [0,180] */
    if (id == SERVO_ID_BASE) {
        if (angle < -90.0f) angle = -90.0f;
        if (angle > 180.0f) angle = 180.0f;
    } else {
        if (angle < SERVO_ANGLE_MIN_DEG) angle = SERVO_ANGLE_MIN_DEG;
        if (angle > SERVO_ANGLE_MAX_DEG) angle = SERVO_ANGLE_MAX_DEG;
    }

    float delta = fabsf(angle - s_state[id].current_angle);
    if (delta < EASING_DELTA)
        return;

    s_state[id].start_angle = s_state[id].current_angle;
    s_state[id].target_angle = angle;

    uint32_t duration_ms = servo_get_easing_duration(id);
    float ratio = delta / SERVO_ANGLE_MAX_DEG;
    uint32_t ticks = (uint32_t)((float)duration_ms * ratio / (float)SERVO_EASING_TICK_MS + 0.5f);
    if (ticks < 1)
        ticks = 1;

    s_state[id].elapsed_ticks = 0;
    s_state[id].total_ticks = ticks;
    s_state[id].is_active = true;
}

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

void servo_set_joints_immediate(const float angles[SERVO_COUNT])
{
    if (!s_initialized) return;
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        float deg = angles[i];
        if (i == SERVO_ID_BASE) {
            if (deg < -90.0f) deg = -90.0f;
            if (deg > 180.0f) deg = 180.0f;
        } else {
            if (deg < SERVO_ANGLE_MIN_DEG) deg = SERVO_ANGLE_MIN_DEG;
            if (deg > SERVO_ANGLE_MAX_DEG) deg = SERVO_ANGLE_MAX_DEG;
        }
        servo_write_hardware((ServoID_t)i, deg);
        s_state[i].current_angle = deg;
        s_state[i].is_active = false; /* 停掉缓动, 轨迹插补接管 */
    }
}

void servo_stop(ServoID_t id)
{
    if (id < 0 || id >= SERVO_COUNT)
        return;
    if (!s_initialized)
        return;
    s_state[id].is_active = false;
    ESP_LOGI(TAG, "Servo [%d] stopped at %.1f°", id, s_state[id].current_angle);
}

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

bool servo_is_easing(ServoID_t id)
{
    if (id < 0 || id >= SERVO_COUNT)
        return false;
    if (!s_initialized)
        return false;
    return s_state[id].is_active;
}

void servo_wait_all_idle(void)
{
    if (!s_initialized)
        return;
    uint32_t timeout_ms = 60000; /* 60秒超时 */
    uint32_t wait_count = 0;
    uint32_t max_count = timeout_ms / 50; /* 50ms 间隔 */
    while (1)
    {
        bool any_active = false;
        for (int i = 0; i < SERVO_COUNT; i++)
        {
            if (s_state[i].is_active)
            {
                any_active = true;
                break;
            }
        }
        if (!any_active)
            break;
        if (wait_count >= max_count)
        {
            ESP_LOGE(TAG, "servo_wait_all_idle TIMEOUT! waited=%lu counts",
                     (unsigned long)wait_count);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50)); /* 50ms 间隔降低CPU占用 */
        wait_count++;
    }
}
