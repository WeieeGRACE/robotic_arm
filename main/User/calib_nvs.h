/**
 * @file  calib_nvs.h
 * @brief 校准参数 NVS 持久化接口
 */

#ifndef CALIB_NVS_H
#define CALIB_NVS_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 校准数据结构体
 */
typedef struct {
    float j2_offset_deg;   /* J2_OFFSET_DEG */
    float j2_scale;        /* J2_SCALE */
    float j3_offset_deg;   /* J3_OFFSET_DEG */
    float j3_scale;        /* J3_SCALE_DEG */
    float arm_d1;          /* ARM_D1 (J1离地高度 mm) */
    float arm_l3;          /* ARM_L3 (末端长度 mm) */
} CalibrationData_t;

/**
 * @brief 从 NVS 加载校准数据
 * @param out  输出结构体, 加载成功时填充
 * @return ESP_OK 成功, ESP_ERR_NOT_FOUND 首次启动无数据, 其他为错误
 */
esp_err_t calib_nvs_load(CalibrationData_t *out);

/**
 * @brief 保存校准数据到 NVS
 * @param data  要保存的数据
 * @return ESP_OK 成功
 */
esp_err_t calib_nvs_save(const CalibrationData_t *data);

#ifdef __cplusplus
}
#endif

#endif /* CALIB_NVS_H */
