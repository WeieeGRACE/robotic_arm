/**
 * @file  main.c
 * @brief 视觉引导抓取 — 协调器 (底盘导航 + 机械臂)
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "chassis.h"
#include "servo_init.h"
#include "servo_set.h"
#include "kinematics.h"
#include "arm_control.h"
#include "vision_uart.h"
#include "vision_coordinator.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Vision-Guided Pick & Place");
    ESP_LOGI(TAG, "========================================");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* 底盘最先 */
    ret = chassis_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Chassis init failed!");
        return;
    }

    /* 舵机 + 运动学 */
    kinematics_load_calibration();
    ret = servo_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Servo init failed!");
        return;
    }
    servo_set_init();
    arm_control_init();

    /* 视觉通信 */
    ret = vision_uart_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Vision UART failed!");
        return;
    }
    vision_uart_set_camera_pose(-100, 0, 490, -31.0f);

    /* 协调器 */
    ret = vc_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Coordinator init failed!");
        return;
    }

    vision_uart_send((const uint8_t *)"P4_READY\n", 9);
    ESP_LOGI(TAG, "Running...");

    while (1)
    {
        vc_run(50);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
