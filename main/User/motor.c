/**
 * @file  motor.c
 * @brief 底盘电机 MCPWM 驱动实现 (RZ7899 ×2)
 *
 * 控制逻辑 (边沿对齐 PWM, UP 计数模式):
 *   前进: IN2(FI)=PWM脉冲  IN1(BI)=cmp=0→LOW
 *   后退: IN1(BI)=PWM脉冲  IN2(FI)=cmp=0→LOW
 *   停止: 两路 cmp=0 → LOW
 *
 * 硬件映射 (每电机 1 Operator, 2 Comparator, 2 Generator):
 *   M1(右): Oper0 → CmpA/GenA=IO16(BI)  CmpB/GenB=IO17(FI)
 *   M2(左): Oper1 → CmpA/GenA=IO18(BI)  CmpB/GenB=IO19(FI)
 */

#include "motor.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "motor";

/*====== MCPWM 句柄 ======*/
static mcpwm_timer_handle_t s_timer = NULL;
static mcpwm_oper_handle_t  s_oper[MOTOR_COUNT]   = {NULL, NULL};
static mcpwm_cmpr_handle_t  s_cmpr_bi[MOTOR_COUNT] = {NULL, NULL};
static mcpwm_cmpr_handle_t  s_cmpr_fi[MOTOR_COUNT] = {NULL, NULL};
static mcpwm_gen_handle_t   s_gen_bi[MOTOR_COUNT]  = {NULL, NULL};
static mcpwm_gen_handle_t   s_gen_fi[MOTOR_COUNT]  = {NULL, NULL};

/*====== 引脚 ======*/
static const int s_gpio_bi[MOTOR_COUNT] = {MOTOR_M1_IN1_GPIO, MOTOR_M2_IN1_GPIO};
static const int s_gpio_fi[MOTOR_COUNT] = {MOTOR_M1_IN2_GPIO, MOTOR_M2_IN2_GPIO};

/*====== 斜坡 ======*/
static float s_actual_speed[MOTOR_COUNT] = {0.0f, 0.0f};

esp_err_t motor_init(void)
{
    esp_err_t ret;

    /*----- 1. MCPWM 定时器 (10 kHz, UP 计数) -----*/
    mcpwm_timer_config_t timer_cfg = {
        .group_id      = MOTOR_MCPWM_GROUP,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = MOTOR_MCPWM_FREQ_HZ * (MOTOR_MCPWM_PERIOD + 1),
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks  = MOTOR_MCPWM_PERIOD,
    };
    ret = mcpwm_new_timer(&timer_cfg, &s_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "timer failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /*----- 2. 每个电机 -----*/
    for (int i = 0; i < MOTOR_COUNT; i++) {

        mcpwm_operator_config_t oper_cfg = { .group_id = MOTOR_MCPWM_GROUP };
        ret = mcpwm_new_operator(&oper_cfg, &s_oper[i]);
        if (ret != ESP_OK) { ESP_LOGE(TAG, "oper[%d]: %s", i, esp_err_to_name(ret)); return ret; }

        ret = mcpwm_operator_connect_timer(s_oper[i], s_timer);
        if (ret != ESP_OK) { ESP_LOGE(TAG, "connect[%d]: %s", i, esp_err_to_name(ret)); return ret; }

        /* Comparator: 同步到 TEZ 更新, 避免半周期毛刺 */
        mcpwm_comparator_config_t cmpr_cfg = {
            .flags = { .update_cmp_on_tez = true },
        };
        ret = mcpwm_new_comparator(s_oper[i], &cmpr_cfg, &s_cmpr_bi[i]);
        if (ret != ESP_OK) { ESP_LOGE(TAG, "bi_cmpr[%d]: %s", i, esp_err_to_name(ret)); return ret; }
        ret = mcpwm_new_comparator(s_oper[i], &cmpr_cfg, &s_cmpr_fi[i]);
        if (ret != ESP_OK) { ESP_LOGE(TAG, "fi_cmpr[%d]: %s", i, esp_err_to_name(ret)); return ret; }

        /* Generator BI */
        mcpwm_generator_config_t gen_bi_cfg = { .gen_gpio_num = s_gpio_bi[i] };
        ret = mcpwm_new_generator(s_oper[i], &gen_bi_cfg, &s_gen_bi[i]);
        if (ret != ESP_OK) { ESP_LOGE(TAG, "bi_gen[%d]: %s", i, esp_err_to_name(ret)); return ret; }

        ret = mcpwm_generator_set_actions_on_timer_event(
                s_gen_bi[i],
                MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                             MCPWM_TIMER_EVENT_EMPTY,
                                             MCPWM_GEN_ACTION_HIGH),
                MCPWM_GEN_TIMER_EVENT_ACTION_END());
        if (ret != ESP_OK) { ESP_LOGE(TAG, "bi_tmr[%d]: %s", i, esp_err_to_name(ret)); return ret; }

        ret = mcpwm_generator_set_actions_on_compare_event(
                s_gen_bi[i],
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                               s_cmpr_bi[i],
                                               MCPWM_GEN_ACTION_LOW),
                MCPWM_GEN_COMPARE_EVENT_ACTION_END());
        if (ret != ESP_OK) { ESP_LOGE(TAG, "bi_cmp[%d]: %s", i, esp_err_to_name(ret)); return ret; }

        /* Generator FI */
        mcpwm_generator_config_t gen_fi_cfg = { .gen_gpio_num = s_gpio_fi[i] };
        ret = mcpwm_new_generator(s_oper[i], &gen_fi_cfg, &s_gen_fi[i]);
        if (ret != ESP_OK) { ESP_LOGE(TAG, "fi_gen[%d]: %s", i, esp_err_to_name(ret)); return ret; }

        ret = mcpwm_generator_set_actions_on_timer_event(
                s_gen_fi[i],
                MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                             MCPWM_TIMER_EVENT_EMPTY,
                                             MCPWM_GEN_ACTION_HIGH),
                MCPWM_GEN_TIMER_EVENT_ACTION_END());
        if (ret != ESP_OK) { ESP_LOGE(TAG, "fi_tmr[%d]: %s", i, esp_err_to_name(ret)); return ret; }

        ret = mcpwm_generator_set_actions_on_compare_event(
                s_gen_fi[i],
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                               s_cmpr_fi[i],
                                               MCPWM_GEN_ACTION_LOW),
                MCPWM_GEN_COMPARE_EVENT_ACTION_END());
        if (ret != ESP_OK) { ESP_LOGE(TAG, "fi_cmp[%d]: %s", i, esp_err_to_name(ret)); return ret; }

        /* 初始 LOW */
        mcpwm_comparator_set_compare_value(s_cmpr_bi[i], 0);
        mcpwm_comparator_set_compare_value(s_cmpr_fi[i], 0);
    }

    /*----- 3. 启动 -----*/
    ret = mcpwm_timer_enable(s_timer);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "enable: %s", esp_err_to_name(ret)); return ret; }

    ret = mcpwm_timer_start_stop(s_timer, MCPWM_TIMER_START_NO_STOP);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "start: %s", esp_err_to_name(ret)); return ret; }

    ESP_LOGI(TAG, "Motor OK [%luHz %lubit]",
             (unsigned long)MOTOR_MCPWM_FREQ_HZ,
             (unsigned long)MOTOR_MCPWM_RESOLUTION);
    return ESP_OK;
}

/*===============================================================
 *  motor_set_speed
 *===============================================================*/
esp_err_t motor_set_speed(motor_id_t motor, float target)
{
    if (motor >= MOTOR_COUNT) return ESP_ERR_INVALID_ARG;

    int i = (int)motor;

    /* M2 左电机物理方向与 M1 相反 (接线/安装差异), 软件取反修正 */
    if (motor == MOTOR_M2) {
        target = -target;
    }

    /* 电压限幅 */
    if (target >  MOTOR_SPEED_LIMIT) target =  MOTOR_SPEED_LIMIT;
    if (target < -MOTOR_SPEED_LIMIT) target = -MOTOR_SPEED_LIMIT;

    /* 斜坡 — 两段式: <4V 快加速, >4V 慢加速, 减速不变 */
    float actual = s_actual_speed[i];
    float diff   = target - actual;
    bool  accel  = (fabsf(target) > fabsf(actual));

    float step;
    if (accel) {
        step = (fabsf(actual) < MOTOR_DEAD_ZONE) ? MOTOR_STEP_FAST : MOTOR_STEP_SLOW;
    } else {
        step = MOTOR_STEP_DECEL;
    }

    if (diff >  step) diff =  step;
    if (diff < -step) diff = -step;

    float speed = actual + diff;
    s_actual_speed[i] = speed;

    /* PWM 输出 */
    uint32_t duty = (uint32_t)(fabsf(speed) * MOTOR_MCPWM_PERIOD);

    if (speed > 0.001f) {
        /* 前进: FI=PWM, BI=LOW */
        mcpwm_comparator_set_compare_value(s_cmpr_fi[i], duty);
        mcpwm_comparator_set_compare_value(s_cmpr_bi[i], 0);
    } else if (speed < -0.001f) {
        /* 后退: BI=PWM, FI=LOW */
        mcpwm_comparator_set_compare_value(s_cmpr_bi[i], duty);
        mcpwm_comparator_set_compare_value(s_cmpr_fi[i], 0);
    } else {
        /* 停止 */
        mcpwm_comparator_set_compare_value(s_cmpr_bi[i], 0);
        mcpwm_comparator_set_compare_value(s_cmpr_fi[i], 0);
    }

    return ESP_OK;
}

esp_err_t motor_stop(motor_id_t motor)
{
    if (motor >= MOTOR_COUNT) return ESP_ERR_INVALID_ARG;

    int i = (int)motor;
    mcpwm_comparator_set_compare_value(s_cmpr_bi[i], 0);
    mcpwm_comparator_set_compare_value(s_cmpr_fi[i], 0);
    s_actual_speed[i] = 0.0f;
    return ESP_OK;
}

esp_err_t motor_set_speed_immediate(motor_id_t motor, float speed)
{
    if (motor >= MOTOR_COUNT) return ESP_ERR_INVALID_ARG;

    int i = (int)motor;

    /* M2 方向修正 */
    if (motor == MOTOR_M2) speed = -speed;

    /* 电压限幅 */
    if (speed >  MOTOR_SPEED_LIMIT) speed =  MOTOR_SPEED_LIMIT;
    if (speed < -MOTOR_SPEED_LIMIT) speed = -MOTOR_SPEED_LIMIT;

    /* 直接生效, 跳过斜坡 */
    s_actual_speed[i] = speed;

    uint32_t duty = (uint32_t)(fabsf(speed) * MOTOR_MCPWM_PERIOD);

    if (speed > 0.001f) {
        mcpwm_comparator_set_compare_value(s_cmpr_fi[i], duty);
        mcpwm_comparator_set_compare_value(s_cmpr_bi[i], 0);
    } else if (speed < -0.001f) {
        mcpwm_comparator_set_compare_value(s_cmpr_bi[i], duty);
        mcpwm_comparator_set_compare_value(s_cmpr_fi[i], 0);
    } else {
        mcpwm_comparator_set_compare_value(s_cmpr_bi[i], 0);
        mcpwm_comparator_set_compare_value(s_cmpr_fi[i], 0);
    }

    return ESP_OK;
}

float motor_get_actual_speed(motor_id_t motor)
{
    if (motor >= MOTOR_COUNT) return 0.0f;
    float speed = s_actual_speed[(int)motor];
    /* 反解 M2 取反, 使返回值与 chassis 层的逻辑方向一致 */
    if (motor == MOTOR_M2) speed = -speed;
    return speed;
}

esp_err_t motor_brake(motor_id_t motor)
{
    if (motor >= MOTOR_COUNT) return ESP_ERR_INVALID_ARG;

    int i = (int)motor;
    /* IN1=HIGH, IN2=HIGH → 短接制动.
       cmp=period 使 EMPTY→HIGH 后永不触发 CMP→LOW, 输出恒定 HIGH */
    uint32_t full = MOTOR_MCPWM_PERIOD;
    mcpwm_comparator_set_compare_value(s_cmpr_bi[i], full);
    mcpwm_comparator_set_compare_value(s_cmpr_fi[i], full);
    s_actual_speed[i] = 0.0f;
    return ESP_OK;
}

esp_err_t motor_coast(motor_id_t motor)
{
    if (motor >= MOTOR_COUNT) return ESP_ERR_INVALID_ARG;

    int i = (int)motor;
    mcpwm_comparator_set_compare_value(s_cmpr_bi[i], 0);
    mcpwm_comparator_set_compare_value(s_cmpr_fi[i], 0);
    s_actual_speed[i] = 0.0f;
    return ESP_OK;
}
