/*=====================================================
 *  servo_init.c
 *
 *  职责: LEDC 硬件初始化 + 上电安全姿态写入
 *  依赖: servo_cfg.h, servo_util.h
 *=====================================================*/

#include "servo_cfg.h"
#include "servo_util.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "servo_init";

/*------ 上电 HOME 角度数组 (6个) ------*/
static const float s_home_angles[SERVO_COUNT] = {
    SERVO_HOME_BASE,
    SERVO_HOME_JOINT1,
    SERVO_HOME_JOINT2,
    SERVO_HOME_JOINT3,
    SERVO_HOME_ROTATE,
    SERVO_HOME_GRIPPER,
};

/*------ 舵机 LEDC 通道查表 (6个) ------*/
static const ledc_channel_t s_servo_chs[SERVO_COUNT] = {
    SERVO_CH_BASE,
    SERVO_CH_JOINT1,
    SERVO_CH_JOINT2,
    SERVO_CH_JOINT3,
    SERVO_CH_ROTATE,
    SERVO_CH_GRIPPER,
};

/*------ 舵机 GPIO 查表 (6个) ------*/
static const int s_servo_gpios[SERVO_COUNT] = {
    SERVO_GPIO_BASE,
    SERVO_GPIO_JOINT1,
    SERVO_GPIO_JOINT2,
    SERVO_GPIO_JOINT3,
    SERVO_GPIO_ROTATE,
    SERVO_GPIO_GRIPPER,
};

esp_err_t servo_init(void)
{
    esp_err_t ret;

    const ledc_timer_config_t timer_cfg = {
        .speed_mode = SERVO_LEDC_SPEED,
        .timer_num = SERVO_LEDC_TIMER,
        .duty_resolution = SERVO_PWM_BITS,
        .freq_hz = SERVO_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    ret = ledc_timer_config(&timer_cfg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "LEDC timer init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    for (int i = 0; i < SERVO_COUNT; i++)
    {
        uint32_t duty = servo_angle_to_duty((ServoID_t)i, s_home_angles[i]);

        const ledc_channel_config_t ch_cfg = {
            .speed_mode = SERVO_LEDC_SPEED,
            .channel = s_servo_chs[i],
            .timer_sel = SERVO_LEDC_TIMER,
            .gpio_num = s_servo_gpios[i],
            .duty = duty,
            .hpoint = 0,
            .intr_type = LEDC_INTR_DISABLE,
        };

        ret = ledc_channel_config(&ch_cfg);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Channel %d (GPIO %d) init failed: %s",
                     i, s_servo_gpios[i], esp_err_to_name(ret));
            return ret;
        }
    }

    ESP_LOGI(TAG, "Servo init OK [%d ch, %dHz, %d-bit]",
             SERVO_COUNT, SERVO_PWM_FREQ, SERVO_PWM_BITS);
    return ESP_OK;
}