/**
 * @file  calib_nvs.c
 * @brief 校准参数 NVS 持久化 — 读写 arm_nvs 分区
 */

#include "calib_nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"

static const char *TAG = "calib_nvs";
static const char *NVS_NAMESPACE = "arm_calib";

/* NVS key 名称 */
static const char *KEY_J2_OFFSET = "j2_off";
static const char *KEY_J2_SCALE  = "j2_scl";
static const char *KEY_J3_OFFSET = "j3_off";
static const char *KEY_J3_SCALE  = "j3_scl";
static const char *KEY_ARM_D1    = "arm_d1";
static const char *KEY_ARM_L3    = "arm_l3";

/* float → int32_t 定点转换 (精度 0.0001) */
static inline int32_t f32_to_i32(float v) { return (int32_t)(v * 10000.0f + 0.5f); }
static inline float i32_to_f32(int32_t v) { return (float)v / 10000.0f; }

esp_err_t calib_nvs_load(CalibrationData_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "NVS open failed (first boot?), using defaults");
        return ret;
    }

    int32_t val;

    ret = nvs_get_i32(handle, KEY_J2_OFFSET, &val);
    if (ret != ESP_OK) { goto close; }
    out->j2_offset_deg = i32_to_f32(val);
    ret = nvs_get_i32(handle, KEY_J2_SCALE,  &val);
    if (ret != ESP_OK) { goto close; }
    out->j2_scale = i32_to_f32(val);
    ret = nvs_get_i32(handle, KEY_J3_OFFSET, &val);
    if (ret != ESP_OK) { goto close; }
    out->j3_offset_deg = i32_to_f32(val);
    ret = nvs_get_i32(handle, KEY_J3_SCALE,  &val);
    if (ret != ESP_OK) { goto close; }
    out->j3_scale = i32_to_f32(val);
    ret = nvs_get_i32(handle, KEY_ARM_D1,    &val);
    if (ret != ESP_OK) { goto close; }
    out->arm_d1 = i32_to_f32(val);
    ret = nvs_get_i32(handle, KEY_ARM_L3,    &val);
    if (ret != ESP_OK) { goto close; }
    out->arm_l3 = i32_to_f32(val);

    ESP_LOGI(TAG, "Loaded from NVS: D1=%.1f L3=%.1f "
             "J2(off=%.1f scl=%.4f) J3(off=%.1f scl=%.4f)",
             (double)out->arm_d1, (double)out->arm_l3,
             (double)out->j2_offset_deg, (double)out->j2_scale,
             (double)out->j3_offset_deg, (double)out->j3_scale);

close:
    nvs_close(handle);
    return ret;
}

esp_err_t calib_nvs_save(const CalibrationData_t *data)
{
    if (!data) return ESP_ERR_INVALID_ARG;

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS open for write failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret  = nvs_set_i32(handle, KEY_J2_OFFSET, f32_to_i32(data->j2_offset_deg));
    ret |= nvs_set_i32(handle, KEY_J2_SCALE,  f32_to_i32(data->j2_scale));
    ret |= nvs_set_i32(handle, KEY_J3_OFFSET, f32_to_i32(data->j3_offset_deg));
    ret |= nvs_set_i32(handle, KEY_J3_SCALE,  f32_to_i32(data->j3_scale));
    ret |= nvs_set_i32(handle, KEY_ARM_D1,    f32_to_i32(data->arm_d1));
    ret |= nvs_set_i32(handle, KEY_ARM_L3,    f32_to_i32(data->arm_l3));

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS write failed");
        nvs_close(handle);
        return ret;
    }

    ret = nvs_commit(handle);
    nvs_close(handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration saved to NVS");
    }
    return ret;
}
