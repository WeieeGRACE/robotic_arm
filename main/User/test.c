/**
 * @file  test.c
 * @brief 舵机校准测试工具 (开发阶段专用)
 *
 * 提供脉宽扫描、角度测试、状态查看等函数，
 * 由 main.c 中硬编码参数调用，每次改参数重新编译烧录。
 */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_log.h"

#include "servo_cfg.h"
#include "servo_init.h"
#include "servo_util.h"
#include "test.h"

static const char *TAG = "test_calibrate";

/*------ LEDC 通道查表 (6 个舵机) ------*/
static const ledc_channel_t s_channel_tbl[SERVO_COUNT] = {
    SERVO_CH_BASE,
    SERVO_CH_JOINT1,
    SERVO_CH_JOINT2,
    SERVO_CH_JOINT3,
    SERVO_CH_ROTATE,
    SERVO_CH_GRIPPER,
};

/*------ 舵机名称查表 (6 个) ------*/
static const char *s_servo_names[SERVO_COUNT] = {
    "IO15 基座旋转",
    "IO14 肩关节",
    "IO13 肘关节",
    "IO12 腕关节",
    "IO11 夹子旋转",
    "IO10 夹子开合 (ZP15S)",
};

/*------ 内部工具: 写占空比 ------*/
static void write_duty_raw(ServoID_t id, uint32_t duty)
{
    ledc_set_duty(SERVO_LEDC_SPEED, s_channel_tbl[id], duty);
    ledc_update_duty(SERVO_LEDC_SPEED, s_channel_tbl[id]);
}

/*------ 内部工具: 读取当前占空比 ------*/
static uint32_t read_duty(ServoID_t id)
{
    return ledc_get_duty(SERVO_LEDC_SPEED, s_channel_tbl[id]);
}

/*-----------------------------------------------------
 *  test_init() — 初始化舵机硬件
 *-----------------------------------------------------*/
esp_err_t test_init(void)
{
    esp_err_t ret = servo_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "舵机初始化失败!");
    }
    return ret;
}

/*-----------------------------------------------------
 *  sweep_pulse() — 脉宽扫描 (来回循环)
 *
 *  从 min_pulse 以 step_us 递增到 max_pulse，
 *  再递减回 min_pulse，如此往复 cycles 次。
 *  cycles=0 表示无限循环。
 *
 *  参数:
 *    id        - 舵机编号 (SERVO_ID_BASE ~ SERVO_ID_GRIPPER)
 *    min_pulse - 起始脉宽 (μs)，如 500
 *    max_pulse - 结束脉宽 (μs)，如 2500
 *    step_us   - 步长 (μs)，快速探测 100，精确定位 10
 *    wait_ms   - 每步等待 (ms)，如 500
 *    cycles    - 循环次数 (0=无限循环)
 *-----------------------------------------------------*/
void sweep_pulse(ServoID_t id,
                 uint32_t min_pulse, uint32_t max_pulse,
                 uint32_t step_us, uint32_t wait_ms,
                 uint32_t cycles)
{
    ESP_LOGI(TAG, "=== 舵机 [%s] 脉宽扫描 ===", s_servo_names[id]);
    ESP_LOGI(TAG, "范围: %lu ~ %lu μs, 步长: %lu μs, 延迟: %lu ms, 循环: %s",
             min_pulse, max_pulse, step_us, wait_ms,
             cycles == 0 ? "无限" : "有限");

    if (min_pulse >= max_pulse || step_us == 0)
    {
        ESP_LOGE(TAG, "参数错误! min=%lu, max=%lu, step=%lu", min_pulse, max_pulse, step_us);
        return;
    }

    uint32_t count = 0;
    while (1)
    {
        /* 正向扫描: min → max */
        for (uint32_t pulse = min_pulse; pulse <= max_pulse; pulse += step_us)
        {
            uint32_t duty = (pulse * (1UL << SERVO_PWM_BITS)) / SERVO_PERIOD_US;
            ESP_LOGI(TAG, "[正] 脉宽=%lu μs, 占空比=%lu, 角度≈%.1f°",
                     pulse, duty, servo_duty_to_angle(id, duty));
            write_duty_raw(id, duty);
            vTaskDelay(pdMS_TO_TICKS(wait_ms));
        }

        /* 反向扫描: max → min */
        for (uint32_t pulse = max_pulse; pulse >= min_pulse; pulse -= step_us)
        {
            uint32_t duty = (pulse * (1UL << SERVO_PWM_BITS)) / SERVO_PERIOD_US;
            ESP_LOGI(TAG, "[逆] 脉宽=%lu μs, 占空比=%lu, 角度≈%.1f°",
                     pulse, duty, servo_duty_to_angle(id, duty));
            write_duty_raw(id, duty);
            vTaskDelay(pdMS_TO_TICKS(wait_ms));
        }

        count++;
        ESP_LOGI(TAG, "--- 第 %lu 次循环完成 ---", count);

        if (cycles > 0 && count >= cycles)
        {
            break;
        }
    }

    ESP_LOGI(TAG, "=== 扫描完成 (%lu 次循环) ===\n", count);
}

/*-----------------------------------------------------
 *  test_angle() — 设置舵机到指定角度
 *-----------------------------------------------------*/
void test_angle(ServoID_t id, float angle_deg)
{
    ESP_LOGI(TAG, "设置 [%s] 到 %.1f°", s_servo_names[id], angle_deg);

    uint32_t duty = servo_angle_to_duty(id, angle_deg);
    write_duty_raw(id, duty);

    ESP_LOGI(TAG, "占空比 = %lu, 脉宽 ≈ %lu μs",
             duty, (duty * SERVO_PERIOD_US) / (1UL << SERVO_PWM_BITS));
}

/*-----------------------------------------------------
 *  test_set_duty() — 直接设置占空比
 *-----------------------------------------------------*/
void test_set_duty(ServoID_t id, uint32_t duty)
{
    ESP_LOGI(TAG, "设置 [%s] 占空比 = %lu", s_servo_names[id], duty);
    write_duty_raw(id, duty);
}

/*-----------------------------------------------------
 *  test_read_pulse() — 读取当前脉宽 (μs)
 *-----------------------------------------------------*/
uint32_t test_read_pulse(ServoID_t id)
{
    uint32_t duty = read_duty(id);
    uint32_t pulse = (duty * SERVO_PERIOD_US) / (1UL << SERVO_PWM_BITS);
    ESP_LOGI(TAG, "[%s] 当前: 占空比=%lu, 脉宽=%lu μs",
             s_servo_names[id], duty, pulse);
    return pulse;
}

/*-----------------------------------------------------
 *  move_servo_to() — 平滑移动单个舵机到目标脉宽
 *
 *  从当前脉宽以 step_us 为步长逐步移动到 target_pulse，
 *  每步等待 wait_ms 毫秒。
 *-----------------------------------------------------*/
void move_servo_to(ServoID_t id, uint32_t target_pulse,
                   uint32_t step_us, uint32_t wait_ms)
{
    uint32_t current_duty = read_duty(id);
    uint32_t current_pulse = (current_duty * SERVO_PERIOD_US) / (1UL << SERVO_PWM_BITS);
    uint32_t target_duty = (target_pulse * (1UL << SERVO_PWM_BITS)) / SERVO_PERIOD_US;

    ESP_LOGI(TAG, "[%s] %lu → %lu μs",
             s_servo_names[id], current_pulse, target_pulse);

    if (current_pulse < target_pulse)
    {
        for (uint32_t p = current_pulse; p < target_pulse; p += step_us)
        {
            uint32_t d = (p * (1UL << SERVO_PWM_BITS)) / SERVO_PERIOD_US;
            write_duty_raw(id, d);
            vTaskDelay(pdMS_TO_TICKS(wait_ms));
        }
    }
    else
    {
        for (uint32_t p = current_pulse; p > target_pulse && p >= step_us; p -= step_us)
        {
            uint32_t d = (p * (1UL << SERVO_PWM_BITS)) / SERVO_PERIOD_US;
            write_duty_raw(id, d);
            vTaskDelay(pdMS_TO_TICKS(wait_ms));
        }
    }
    /* 写入最终目标 */
    write_duty_raw(id, target_duty);
}

/*-----------------------------------------------------
 *  move_multi_to() — 多舵机并行运动到各自目标脉宽
 *
 *  所有舵机同时以相同的步长和间隔运动，
 *  以最慢的那个舵机到达目标后停止。
 *
 *  参数:
 *    ids[]         - 舵机ID数组
 *    targets[]     - 目标脉宽数组 (μs)
 *    count         - 舵机数量
 *    step_us       - 步长 (μs)
 *    wait_ms       - 每步等待 (ms)
 *-----------------------------------------------------*/
void move_multi_to(const ServoID_t ids[], const uint32_t targets[],
                   int count, uint32_t step_us, uint32_t wait_ms)
{
    /* 初始化当前位置 */
    uint32_t current[SERVO_COUNT];
    uint32_t target[SERVO_COUNT];
    bool done[SERVO_COUNT];

    for (int i = 0; i < count; i++)
    {
        uint32_t duty = read_duty(ids[i]);
        current[i] = (duty * SERVO_PERIOD_US) / (1UL << SERVO_PWM_BITS);
        target[i] = targets[i];
        done[i] = false;
        ESP_LOGI(TAG, "[%s] %lu → %lu μs",
                 s_servo_names[ids[i]], current[i], target[i]);
    }

    /* 并行步进 */
    bool all_done = false;
    while (!all_done)
    {
        all_done = true;
        for (int i = 0; i < count; i++)
        {
            if (done[i])
                continue;

            if (current[i] < target[i])
            {
                uint32_t next = current[i] + step_us;
                if (next >= target[i])
                {
                    current[i] = target[i];
                    done[i] = true;
                }
                else
                {
                    current[i] = next;
                    all_done = false;
                }
            }
            else if (current[i] > target[i])
            {
                uint32_t next = (current[i] >= step_us) ? current[i] - step_us : 0;
                if (next <= target[i])
                {
                    current[i] = target[i];
                    done[i] = true;
                }
                else
                {
                    current[i] = next;
                    all_done = false;
                }
            }
            else
            {
                done[i] = true;
            }

            uint32_t duty = (current[i] * (1UL << SERVO_PWM_BITS)) / SERVO_PERIOD_US;
            write_duty_raw(ids[i], duty);
        }
        vTaskDelay(pdMS_TO_TICKS(wait_ms));
    }

    ESP_LOGI(TAG, "并行运动完成 (%d 个舵机)", count);
}

/*-----------------------------------------------------
 *  action_sequence() — 预定义动作序列（适配 6 个舵机）
 *
 *  步进值 10μs，间隔 30ms。
 *
 *  阶段:
 *    Phase1: 全部 6 个舵机并行移动到合理测试位置
 *    Phase2: 基座单独旋转 (0° → 180°)
 *    Phase3: 肘、腕、夹爪旋转并行移动
 *    Phase4: 夹爪开合 (打开 → 闭合)
 *    Phase5: 肩、肘并行移动
 *-----------------------------------------------------*/
void action_sequence(void)
{
#define STEP_US 10
#define WAIT_MS 30

    ESP_LOGI(TAG, "\n===== 动作序列开始 =====");

    /* Phase 1: 全部 6 个舵机并行移动 (测试初始姿态) */
    {
        const ServoID_t ids[] = {
            SERVO_ID_BASE,
            SERVO_ID_JOINT1,
            SERVO_ID_JOINT2,
            SERVO_ID_JOINT3,
            SERVO_ID_ROTATE,
            SERVO_ID_GRIPPER,
        };
        const uint32_t targets[] = {
            1100, /* Base   1100μs → 约 170° (反向) 接近极限 */
            1540, /* Joint1 1540μs → 约 90° */
            800,  /* Joint2 800μs → 约 180° (反向) */
            1080, /* Joint3 固定 1080μs */
            1460, /* Rotate 固定 1460μs */
            1440, /* Gripper 打开 */
        };
        ESP_LOGI(TAG, "\n--- Phase 1: 6 舵并行初始姿态 ---");
        move_multi_to(ids, targets, 6, STEP_US, WAIT_MS);
    }

    /* Phase 2: 基座单独旋转 (从 1100 → 2300μs) */
    ESP_LOGI(TAG, "\n--- Phase 2: 基座旋转 1100 → 2300μs ---");
    move_servo_to(SERVO_ID_BASE, 2300, STEP_US, WAIT_MS);

    /* Phase 3: 肘、腕、夹爪旋转并行移动 */
    {
        const ServoID_t ids[] = {SERVO_ID_JOINT2, SERVO_ID_JOINT3, SERVO_ID_ROTATE};
        const uint32_t targets[] = {
            1900, /* Joint2 1900μs → 0° (反向) */
            1080, /* Joint3 固定 */
            1460, /* Rotate 固定 */
        };
        ESP_LOGI(TAG, "\n--- Phase 3: 3 舵并行 ---");
        move_multi_to(ids, targets, 3, STEP_US, WAIT_MS);
    }

    /* Phase 4: 夹爪开合 (打开 → 闭合) */
    ESP_LOGI(TAG, "\n--- Phase 4: 夹爪 1440 → 2110μs (闭合) ---");
    move_servo_to(SERVO_ID_GRIPPER, 2110, STEP_US, WAIT_MS);

    /* Phase 5: 肩、肘并行移动 */
    {
        const ServoID_t ids[] = {SERVO_ID_JOINT1, SERVO_ID_JOINT2};
        const uint32_t targets[] = {
            900, /* Joint1 900μs → 0° */
            800, /* Joint2 800μs → 180° */
        };
        ESP_LOGI(TAG, "\n--- Phase 5: 肩肘并行 ---");
        move_multi_to(ids, targets, 2, STEP_US, WAIT_MS);
    }

    ESP_LOGI(TAG, "\n===== 动作序列完成 =====\n");
}

/*-----------------------------------------------------
 *  test_print_status() — 查看所有舵机状态
 *-----------------------------------------------------*/
void test_print_status(void)
{
    ESP_LOGI(TAG, "\n=== 当前舵机状态 ===");
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        uint32_t duty = read_duty(i);
        uint32_t pulse = (duty * SERVO_PERIOD_US) / (1UL << SERVO_PWM_BITS);
        ESP_LOGI(TAG, "[%d] %s: 占空比=%lu, 脉宽=%lu μs, 角度≈%.1f°",
                 i, s_servo_names[i], duty, pulse,
                 servo_duty_to_angle(i, duty));
    }
    ESP_LOGI(TAG, "====================\n");
}