/**
 * @file  main.c
 * @brief 机械臂手动 pulse 抓取-旋转-放置测试
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "servo_init.h"
#include "servo_set.h"
#include "arm_control.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  机械臂抓取-旋转-放置测试");
    ESP_LOGI(TAG, "========================================");

    esp_err_t ret = servo_init();
    if (ret != ESP_OK) { ESP_LOGE(TAG, "舵机初始化失败!"); return; }

    servo_set_init();
    arm_control_init();

    vTaskDelay(pdMS_TO_TICKS(2000));

    arm_control_pick_and_place_low();

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  测试完成!");
    ESP_LOGI(TAG, "========================================");

    while (1) { vTaskDelay(pdMS_TO_TICKS(5000)); }
}