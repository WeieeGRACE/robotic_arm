/*=====================================================
 *  servo_init.c
 *
 *  职责: LEDC 硬件初始化 + 上电安全姿态写入
 *  不涉及: 缓动 / esp_timer / 运行时更新
 *
 *  依赖: servo_cfg.h  (配置宏)
 *        servo_util.h (角度→占空比转换)
 *=====================================================*/

#include "servo_cfg.h"
#include "servo_util.h" /* ← 新增: 用外部 servo_angle_to_duty() */
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "servo_init";

/*------ 上电 HOME 角度数组 (与 ServoID_t 顺序对应) ------*/
static const float s_home_angles[SERVO_COUNT] = {
    SERVO_HOME_BASE,
    SERVO_HOME_JOINT1,
    SERVO_HOME_JOINT2,
    SERVO_HOME_JOINT3,
    SERVO_HOME_ROTATE,
    SERVO_HOME_GRIPPER,
    SERVO_HOME_EXTRA,
};

/*------ 舵机 LEDC 通道查表 (仅本文件使用) ------*/
static const ledc_channel_t s_servo_chs[SERVO_COUNT] = {
    SERVO_CH_BASE,
    SERVO_CH_JOINT1,
    SERVO_CH_JOINT2,
    SERVO_CH_JOINT3,
    SERVO_CH_ROTATE,
    SERVO_CH_GRIPPER,
    SERVO_CH_EXTRA,
};

/*------ 舵机 GPIO 查表 (仅本文件使用) ------*/
static const int s_servo_gpios[SERVO_COUNT] = {
    SERVO_GPIO_BASE,
    SERVO_GPIO_JOINT1,
    SERVO_GPIO_JOINT2,
    SERVO_GPIO_JOINT3,
    SERVO_GPIO_ROTATE,
    SERVO_GPIO_GRIPPER,
    SERVO_GPIO_EXTRA,
};

/*=====================================================
 *  servo_init()
 *
 *  1. 配置 LEDC Timer0: 50Hz, 12-bit
 *  2. 配置 7 路 Channel，每路绑定各自 GPIO
 *  3. 立即写入 HOME 安全姿态
 *
 *  返回 ESP_OK 成功，否则失败后打印错误日志。
 *=====================================================*/
esp_err_t servo_init(void)
{
    esp_err_t ret;

    /*--- Step 1: LEDC Timer ---*/
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

    /*--- Step 2: 7 路 Channel ---*/
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        /* 调用 servo_util 的函数，删除了本文件内部的 static angle_to_duty */
        uint32_t duty = servo_angle_to_duty((ServoID_t)i, s_home_angles[i]);

        const ledc_channel_config_t ch_cfg = {
            .speed_mode = SERVO_LEDC_SPEED,
            .channel = s_servo_chs[i],
            .timer_sel = SERVO_LEDC_TIMER,
            .gpio_num = s_servo_gpios[i],
            .duty = duty, /* 直接写 HOME 占空比 */
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

    ESP_LOGI(TAG, "Servo init OK  [%d ch, %dHz, %d-bit]",
             SERVO_COUNT, SERVO_PWM_FREQ, SERVO_PWM_BITS);

    return ESP_OK;
}