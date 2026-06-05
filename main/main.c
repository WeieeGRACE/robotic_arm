/**
 * @file  main.c
 * @brief 机械臂手动 pulse 抓取-旋转-放置测试
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "servo_init.h"
#include "servo_set.h"
#include "kinematics.h"
#include "arm_control.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  机械臂抓取-旋转-放置测试");
    ESP_LOGI(TAG, "========================================");

    /* 初始化 NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* 加载校准参数 (NVS有则用, 无则用默认) */
    kinematics_load_calibration();

    ret = servo_init();
    if (ret != ESP_OK) { ESP_LOGE(TAG, "舵机初始化失败!"); return; }

    servo_set_init();
    arm_control_init();

    vTaskDelay(pdMS_TO_TICKS(2000));

    arm_control_multi_pick_and_place();

    /* 自动保存当前校准参数到 NVS */
    kinematics_save_calibration();

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  测试完成!");
    ESP_LOGI(TAG, "========================================");

    while (1) { vTaskDelay(pdMS_TO_TICKS(5000)); }
}