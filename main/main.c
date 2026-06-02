#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "servo_cfg.h"
#include "servo_init.h"
#include "servo_set.h"

static const char *TAG = "Main";

void app_main(void)
{
    ESP_LOGI(TAG, "================================================");
    ESP_LOGI(TAG, "  ESP32-P4 六轴机械臂控制系统启动");
    ESP_LOGI(TAG, "================================================");

    /* 1. 初始化舵机硬件 — 写入 HOME 安全姿态 */
    servo_init();

    /* 2. 初始化五次多项式缓动系统 */
    servo_set_init();

    /* 3. 运动到目标姿态: Joint1=0°, Joint2=180° */
    ESP_LOGI(TAG, "运动至目标姿态: Joint1=0° Joint2=180°");
    servo_set_angle(SERVO_ID_JOINT1, 0.0f);
    servo_set_angle(SERVO_ID_JOINT2, 90.0f);
    vTaskDelay(pdMS_TO_TICKS(2000)); /* 等待缓动完成 */

    ESP_LOGI(TAG, "系统就绪，保持目标姿态");

    /* 主循环 — 等待后续控制指令（手柄/UART/视觉） */
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
