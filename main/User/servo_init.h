/**
 * @file  servo_init.h
 * @brief 舵机硬件初始化接口
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 初始化所有舵机的 LEDC PWM 通道
     *
     * @return ESP_OK 成功，其他值表示失败
     */
    esp_err_t servo_init(void);

#ifdef __cplusplus
}
#endif