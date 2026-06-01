/**
 * @file  main.c
 * @brief 舵机方向符号验证程序
 *
 * 测试 SERVO_DIR_xxx 是否正确：
 *   +1 → 角度↑ 脉宽↑
 *   -1 → 角度↑ 脉宽↓
 *
 * 逐个舵机执行 0°→180°→HOME 序列，
 * 串口打印期望表现，你观察实际运动判断方向。
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "servo_cfg.h"
#include "servo_init.h"
#include "servo_util.h"
#include "test.h"

static const char *TAG = "dir_check";

/* 舵机名称 */
static const char *s_name[SERVO_COUNT] = {
    "BASE",
    "JOINT1",
    "JOINT2",
    "JOINT3",
    "ROTATE",
    "GRIPPER",
};

/* 每个舵机在 0° 和 180° 时期望的串口输出说明 */
static const char *s_expect_0[SERVO_COUNT] = {
    "BASE     0° → 脉宽≈2350μs → 基座应停在'0°逻辑位置'",
    "JOINT1   0° → 脉宽≈900μs  → 肩关节应停在'0°逻辑位置'",
    "JOINT2   0° → 脉宽≈1900μs → 肘关节应停在'0°逻辑位置'",
    "JOINT3   固定 1080μs，跳过测试",
    "ROTATE   固定 1460μs，跳过测试",
    "GRIPPER  0° → 脉宽≈1440μs → 夹爪应完全打开",
};

static const char *s_expect_180[SERVO_COUNT] = {
    "BASE     180° → 脉宽≈980μs  → 基座应停在'180°逻辑位置'",
    "JOINT1   180° → 脉宽≈2180μs → 肩关节应停在'180°逻辑位置'",
    "JOINT2   180° → 脉宽≈800μs  → 肘关节应停在'180°逻辑位置'",
    "JOINT3   固定，跳过",
    "ROTATE   固定，跳过",
    "GRIPPER  180° → 脉宽≈2110μs → 夹爪应完全闭合",
};

/*-----------------------------------------------------
 *  test_one_servo() — 测试单个舵机的方向
 *
 *  依次设置 0° → 180° → HOME，每步等 2 秒。
 *  跳过固定关节 (MIN==MAX)。
 *-----------------------------------------------------*/
static void test_one_servo(ServoID_t id)
{
    /* 跳过固定关节 */
    uint32_t min_pulse, max_pulse;
    servo_get_pulse_range(id, &min_pulse, &max_pulse);
    if (min_pulse == max_pulse)
    {
        ESP_LOGW(TAG, "[%s] 固定关节 (MIN=MAX=%luμs)，跳过方向测试\n",
                 s_name[id], min_pulse);
        return;
    }

    ESP_LOGI(TAG, "\n========================================");
    ESP_LOGI(TAG, "  测试 [%s]  方向=%d", s_name[id],
             (int)(id == SERVO_ID_BASE || id == SERVO_ID_JOINT2 ? -1 : +1));
    ESP_LOGI(TAG, "========================================");

    /* Step 1: 0° */
    ESP_LOGI(TAG, "\n>>> 设置 %s = 0°", s_name[id]);
    ESP_LOGI(TAG, "    期望: %s", s_expect_0[id]);
    test_angle(id, 0.0f);
    ESP_LOGI(TAG, "    观察舵机是否到达'0°位置'，等待 2 秒...\n");
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* Step 2: 180° */
    ESP_LOGI(TAG, ">>> 设置 %s = 180°", s_name[id]);
    ESP_LOGI(TAG, "    期望: %s", s_expect_180[id]);
    test_angle(id, 180.0f);
    ESP_LOGI(TAG, "    观察舵机是否到达'180°位置'，等待 2 秒...\n");
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* Step 3: 回到 HOME */
    float home;
    switch (id)
    {
    case SERVO_ID_BASE:
        home = SERVO_HOME_BASE;
        break;
    case SERVO_ID_JOINT1:
        home = SERVO_HOME_JOINT1;
        break;
    case SERVO_ID_JOINT2:
        home = SERVO_HOME_JOINT2;
        break;
    case SERVO_ID_GRIPPER:
        home = SERVO_HOME_GRIPPER;
        break;
    default:
        home = 90.0f;
        break;
    }
    ESP_LOGI(TAG, ">>> 回到 HOME (%.1f°)\n", home);
    test_angle(id, home);
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "--- [%s] 测试完成 ---\n", s_name[id]);
}

/*-----------------------------------------------------
 *  判断逻辑 (写在这里方便你对照):
 *
 *  如果 test_angle(0°) 时舵机到了你定义的 0° 位置，
 *  且 test_angle(180°) 时舵机到了 180° 位置，
 *  → 方向符号正确。
 *
 *  如果反了（0° 时舵机跑到 180° 位置），
 *  → 把 SERVO_DIR_xxx 取反 (+1 改 -1, -1 改 +1)。
 *-----------------------------------------------------*/

void app_main(void)
{
    ESP_LOGI(TAG, "\n");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "   舵机方向符号验证程序");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "即将逐个测试 BASE、JOINT1、JOINT2、GRIPPER");
    ESP_LOGI(TAG, "每个舵机: 0°(等2秒) → 180°(等2秒) → HOME");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "观察要点:");
    ESP_LOGI(TAG, "  test_angle(0°)   时舵机是否停在你的'0°逻辑位置'?");
    ESP_LOGI(TAG, "  test_angle(180°) 时舵机是否停在你的'180°逻辑位置'?");
    ESP_LOGI(TAG, "  两次都对 → 方向正确");
    ESP_LOGI(TAG, "  两次都反 → 方向符号取反");
    ESP_LOGI(TAG, "");

    /* 初始化硬件 */
    test_init();
    test_print_status();

    /* 逐个测试 */
    test_one_servo(SERVO_ID_BASE);
    test_one_servo(SERVO_ID_JOINT1);
    test_one_servo(SERVO_ID_JOINT2);
    test_one_servo(SERVO_ID_GRIPPER);

    /* JOINT3 和 ROTATE 是固定关节，自动跳过 */

    ESP_LOGI(TAG, "\n========================================");
    ESP_LOGI(TAG, "   全部测试完成");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "如果某个舵机方向反了，修改 servo_cfg.h 中");
    ESP_LOGI(TAG, "对应的 SERVO_DIR_xxx 符号后重新编译烧录。");

    test_print_status();

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}