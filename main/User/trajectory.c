/**
 * @file  trajectory.c
 * @brief 关节空间多段轨迹插补 — esp_timer 驱动, 段内五次缓动, 段间自动切换
 */

#include "trajectory.h"
#include "servo_set.h"
#include "servo_util.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

static const char *TAG = "traj";

typedef struct {
    const TrajPoint_t *points;
    int num_points;
    int current_seg;            /* 当前段索引 (0 = 起点→points[0]) */
    uint64_t seg_start_us;      /* 当前段起始时刻 (esp_timer_get_time) */
    float start_angles[SERVO_COUNT]; /* 当前段起始角度 */
    bool running;
    esp_timer_handle_t timer;
} TrajState_t;

static TrajState_t s_traj = {0};

/* 五次多项式缓动因子 (与 servo_util.c 一致) */
static inline float ease(float tau)
{
    if (tau <= 0.0f) return 0.0f;
    if (tau >= 1.0f) return 1.0f;
    float t2 = tau * tau;
    float t3 = t2 * tau;
    float t4 = t3 * tau;
    float t5 = t4 * tau;
    return 10.0f * t3 - 15.0f * t4 + 6.0f * t5;
}

/* 线性插值 + 缓动 */
static inline float lerp_eased(float from, float to, float tau)
{
    return from + (to - from) * ease(tau);
}

static void traj_timer_cb(void *arg)
{
    if (!s_traj.running || s_traj.current_seg >= s_traj.num_points) {
        return;
    }

    uint64_t now = esp_timer_get_time();
    uint64_t elapsed_us = now - s_traj.seg_start_us;
    uint64_t seg_dur_us = (uint64_t)(s_traj.points[s_traj.current_seg].duration_ms * 1000.0f);

    if (elapsed_us >= seg_dur_us) {
        /* 当前段结束 → 切换到下一段 */
        s_traj.current_seg++;

        if (s_traj.current_seg >= s_traj.num_points) {
            /* 全部完成 */
            s_traj.running = false;
            ESP_LOGI(TAG, "Trajectory done (%d segments)", s_traj.num_points);
            return;
        }

        /* 记录新段起点: 上一段目标 = 本段起点 */
        for (int i = 0; i < SERVO_COUNT; i++) {
            s_traj.start_angles[i] = s_traj.points[s_traj.current_seg - 1].angles[i];
        }
        s_traj.seg_start_us = now;
        elapsed_us = 0;
        seg_dur_us = (uint64_t)(s_traj.points[s_traj.current_seg].duration_ms * 1000.0f);

        ESP_LOGI(TAG, "Segment %d/%d (%.0f ms)",
                 s_traj.current_seg + 1, s_traj.num_points,
                 (double)s_traj.points[s_traj.current_seg].duration_ms);
    }

    /* 计算当前段进度 τ ∈ [0, 1] */
    float tau = (float)elapsed_us / (float)seg_dur_us;
    if (tau > 1.0f) tau = 1.0f;

    /* 对每个关节: 缓动插值 → 写硬件 */
    float angles[SERVO_COUNT];
    const float *target = s_traj.points[s_traj.current_seg].angles;
    for (int i = 0; i < SERVO_COUNT; i++) {
        angles[i] = lerp_eased(s_traj.start_angles[i], target[i], tau);
    }
    servo_set_joints_immediate(angles);
}

void traj_start(const TrajPoint_t *points, int num_points)
{
    if (!points || num_points < 1) return;

    /* 停止旧轨迹 */
    if (s_traj.timer) {
        esp_timer_stop(s_traj.timer);
    }

    s_traj.points      = points;
    s_traj.num_points  = num_points;
    s_traj.current_seg = 0;
    s_traj.running     = true;

    /* 当前舵机角度作为起点 */
    for (int i = 0; i < SERVO_COUNT; i++) {
        s_traj.start_angles[i] = points[0].angles[i]; /* 会被 servo_set 覆写,
                                                          实际由 set_angle 回调更新 */
    }
    /* 实际起点: 立即读一次当前目标 (近似) */
    /* FIXME: servo_set 层目前不暴露当前角度, 用第一目标近似 */

    s_traj.seg_start_us = esp_timer_get_time();

    ESP_LOGI(TAG, "Trajectory start: %d segments", num_points);

    /* 首次创建 timer */
    if (!s_traj.timer) {
        const esp_timer_create_args_t args = {
            .callback = traj_timer_cb,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "trajectory",
        };
        esp_timer_create(&args, &s_traj.timer);
    }

    esp_timer_start_periodic(s_traj.timer, SERVO_EASING_TICK_MS * 1000);
}

bool traj_is_done(void)
{
    return !s_traj.running;
}

void traj_wait_done(void)
{
    uint32_t timeout = 120000; /* 2分钟超时 */
    while (s_traj.running && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(50));
        timeout -= 50;
    }
    if (timeout == 0) {
        ESP_LOGE(TAG, "traj_wait_done TIMEOUT!");
        traj_abort();
    }
}

void traj_abort(void)
{
    if (s_traj.timer) {
        esp_timer_stop(s_traj.timer);
    }
    s_traj.running = false;
    ESP_LOGI(TAG, "Trajectory aborted");
}

/* 快捷: 单段轨迹移动 + 阻塞等待 */
void traj_move_to(const float target[SERVO_COUNT], float duration_ms)
{
    TrajPoint_t pt;
    for (int i = 0; i < SERVO_COUNT; i++) {
        pt.angles[i] = target[i];
    }
    pt.duration_ms = duration_ms;
    traj_start(&pt, 1);
    traj_wait_done();
}
