/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "test.h"
#include "servo_cfg.h"

static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

static uint8_t s_led_state = 0;

#ifdef CONFIG_BLINK_LED_STRIP

static led_strip_handle_t led_strip;

static void blink_led(void)
{
    /* If the addressable LED is enabled */
    if (s_led_state)
    {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        led_strip_set_pixel(led_strip, 0, 16, 16, 16);
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
    }
    else
    {
        /* Set all LED off to clear all pixels */
        led_strip_clear(led_strip);
    }
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };
#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
#elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "unsupported LED strip backend"
#endif
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}

#elif CONFIG_BLINK_LED_GPIO

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

#else
#error "unsupported LED type"
#endif

/*=======================================================
 *  pick_and_place() — 夹取→提起→旋转→放下 完整动作序列
 *
 *  使用 move_servo_to (单舵) 和 move_multi_to (多舵并行)
 *  平滑移动各舵机到目标脉宽。
 *
 *  ★ 重要: sweep_pulse 是校准扫描工具，不是"移动到目标位置"函数！
 *    - sweep_pulse 会正向扫到 max_pulse 再反向扫回 min_pulse
 *    - 最终停在 min_pulse，无法用于定点运动
 *    - 执行动作序列必须使用 move_servo_to / move_multi_to
 *
 *  脉宽值说明 (上电实测后修正):
 *    BASE:   2350=取料位, 1110=放置位
 *    JOINT1: 900=低位, 1540=高位(提起)
 *    JOINT2: 1900=低位, 1500=高位(提起)
 *    JOINT3: 1080=取料姿态
 *    ROTATE: 1460=取料姿态
 *    GRIPPER: 1440=张开, 2110=闭合
 *=======================================================*/
static void pick_and_place(void)
{
#define MOVE_STEP_US 10 /* 步进精度: 10μs */
#define MOVE_WAIT_MS 30 /* 每步间隔: 30ms */

    /*--- Phase 1: 多舵机并行移动到取料位 ---*/
    ESP_LOGI(TAG, "\n===== Phase 1: 移动到取料位 =====");
    {
        const ServoID_t ids[] = {
            SERVO_ID_BASE, SERVO_ID_JOINT1, SERVO_ID_JOINT2,
            SERVO_ID_JOINT3, SERVO_ID_ROTATE};
        const uint32_t targets[] = {2350, 900, 1900, 1080, 1460};
        move_multi_to(ids, targets, 5, MOVE_STEP_US, MOVE_WAIT_MS);
    }

    /*--- Phase 2: 爪子闭合 — 夹取物体 (2110μs=闭合) ---*/
    ESP_LOGI(TAG, "\n===== Phase 2: 夹取 (闭合夹爪) =====");
    move_servo_to(SERVO_ID_GRIPPER, 2110, MOVE_STEP_US, MOVE_WAIT_MS);

    /*--- Phase 3: 提起 — 肩关节+肘关节并行抬升 ---*/
    ESP_LOGI(TAG, "\n===== Phase 3: 提起物体 =====");
    {
        const ServoID_t ids[] = {SERVO_ID_JOINT1, SERVO_ID_JOINT2};
        const uint32_t targets[] = {1540, 1500};
        move_multi_to(ids, targets, 2, MOVE_STEP_US, MOVE_WAIT_MS);
    }

    /*--- Phase 4: 旋转基座到放置位 ---*/
    ESP_LOGI(TAG, "\n===== Phase 4: 旋转到放置位 =====");
    move_servo_to(SERVO_ID_BASE, 980, MOVE_STEP_US, MOVE_WAIT_MS);

    /*--- Phase 5: 下降 — 肩关节+肘关节并行下放 ---*/
    ESP_LOGI(TAG, "\n===== Phase 5: 下放物体 =====");
    {
        const ServoID_t ids[] = {SERVO_ID_JOINT1, SERVO_ID_JOINT2};
        const uint32_t targets[] = {900, 1900};
        move_multi_to(ids, targets, 2, MOVE_STEP_US, MOVE_WAIT_MS);
    }

    /*--- Phase 6: 爪子张开 — 释放物体 (1440μs=张开) ---*/
    ESP_LOGI(TAG, "\n===== Phase 6: 释放 (张开夹爪) =====");
    move_servo_to(SERVO_ID_GRIPPER, 1440, MOVE_STEP_US, MOVE_WAIT_MS);

    ESP_LOGI(TAG, "\n===== 夹取→放置 动作序列完成 =====\n");

#undef MOVE_STEP_US
#undef MOVE_WAIT_MS
}

void app_main(void)
{
    /*=== 舵机校准区 (改参数后重新编译烧录即可) ===*/

    /* 初始化舵机硬件 */
    test_init();

    /* 打印所有舵机初始状态 */
    test_print_status();

    /*
     * ★ 测试模式选择 —— 取消注释需要的模式 ★
     */

    /*--- 模式1: 预定义动作序列 (Phase1~5) ---*/
    // action_sequence();

    /*--- 模式2: 夹取→提起→旋转→放下 (当前启用) ---*/
    pick_and_place();

    /* 扫描结束后打印状态 */
    test_print_status();

    /* 每秒打印一次所有舵机状态 */
    while (1)
    {
        test_print_status();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

#if 0
    /*--- LED 测试 (默认关闭) ---*/
    configure_led();
    while (1)
    {
        ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        blink_led();
        s_led_state = !s_led_state;
        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
    }
#endif
}