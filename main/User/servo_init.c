/*=====================================================
 *  servo_init.c
 *
 *  职责: LEDC 硬件初始化 + 上电安全姿态写入
 *  不涉及: 缓动 / esp_timer / 运行时更新
 *
 *  依赖: servo_cfg.h (配置宏)
 *
 *  angle_to_duty() 目前为 static 临时版本，
 *  等 servo_util.c 写好后统一挪走。
 *=====================================================*/

#include "servo_cfg.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "servo_init";

/*------ static 临时: 角度 → 占空比 ------
 *
 * servo_util.c 写好后删除此处，改为 extern 调用。
 * 整数四舍五入，无浮点开销。
 *
 * 计算:
 *   duty = DUTY_MIN + angle/180 * (DUTY_MAX - DUTY_MIN)
 *        = 102 + angle * 410 / 180
 *
 * 输入钳位防止越界。
 *----------------------------------------------*/
static uint32_t angle_to_duty(float deg)
{
    if (deg < SERVO_ANGLE_MIN_DEG)
        deg = SERVO_ANGLE_MIN_DEG;
    if (deg > SERVO_ANGLE_MAX_DEG)
        deg = SERVO_ANGLE_MAX_DEG;

    uint32_t span = SERVO_DUTY_MAX - SERVO_DUTY_MIN; /* 410 */
    uint32_t duty = SERVO_DUTY_MIN + (uint32_t)(deg * (float)span / SERVO_ANGLE_MAX_DEG + 0.5f);
    return duty;
}

/*------ 上电 HOME 角度数组 (与 ServoID_t 顺序对应) ------*/
static const float s_home_angles[SERVO_COUNT] = {
    SERVO_HOME_BASE,
    SERVO_HOME_JOINT1,
    SERVO_HOME_JOINT2,
    SERVO_HOME_JOINT3,
    SERVO_HOME_ROTATE,
    SERVO_HOME_GRIPPER,
};

/*------ GPIO 映射表 (与 ServoID_t 顺序对应) ------*/
static const gpio_num_t s_servo_gpios[SERVO_COUNT] = {
    SERVO_GPIO_BASE,
    SERVO_GPIO_JOINT1,
    SERVO_GPIO_JOINT2,
    SERVO_GPIO_JOINT3,
    SERVO_GPIO_ROTATE,
    SERVO_GPIO_GRIPPER,
};

/*------ LEDC Channel 映射表 (与 ServoID_t 顺序对应) ------*/
static const ledc_channel_t s_servo_chs[SERVO_COUNT] = {
    SERVO_CH_BASE,
    SERVO_CH_JOINT1,
    SERVO_CH_JOINT2,
    SERVO_CH_JOINT3,
    SERVO_CH_ROTATE,
    SERVO_CH_GRIPPER,
};

/*=====================================================
 *  servo_init()
 *
 *  1. 配置 LEDC Timer0: 50Hz, 12-bit
 *  2. 配置 6 路 Channel，每路绑定各自 GPIO
 *  3. 立即写入 HOME 安全姿态
 *
 *  返回 ESP_OK 成功，否则失败后打印错误日志。
 *=====================================================*/
esp_err_t servo_init(void)
{
    esp_err_t ret;

    /*--- Step 1: LEDC Timer ---*/
    const ledc_timer_config_t timer_cfg = {
        .speed_mode = SERVO_LEDC_SPEED,    /* LOW_SPEED_MODE */
        .timer_num = SERVO_LEDC_TIMER,     /* TIMER_0 */
        .duty_resolution = SERVO_PWM_BITS, /* 12-bit */
        .freq_hz = SERVO_PWM_FREQ,         /* 50Hz */
        .clk_cfg = LEDC_AUTO_CLK,
    };

    ret = ledc_timer_config(&timer_cfg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "LEDC timer init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /*--- Step 2: 6 路 Channel ---*/
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        uint32_t duty = angle_to_duty(s_home_angles[i]);

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