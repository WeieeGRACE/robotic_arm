/**
 * @file  test.h
 * @brief 舵机校准测试工具接口
 *
 * 开发阶段专用，由 main.c 硬编码参数调用，
 * 每次改参数重新编译烧录即可。
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "servo_cfg.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 初始化舵机硬件 (内部调用 servo_init)
     * @return ESP_OK 成功
     */
    esp_err_t test_init(void);

    /**
     * @brief 脉宽扫描测试
     *
     * 从 min_pulse 以 step_us 递增到 max_pulse，
     * 每步等待 wait_ms 毫秒，到 max 后停止。
     * 日志输出每个脉宽对应的占空比和近似角度。
     *
     * @param id        舵机编号 (SERVO_ID_BASE ~ SERVO_ID_GRIPPER)
     * @param min_pulse 起始脉宽 (μs)，建议 500
     * @param max_pulse 结束脉宽 (μs)，建议 2500
     * @param step_us   步长 (μs)，快速探测建议 100，精确定位建议 10
     * @param wait_ms   每步等待时间 (ms)，建议 500
     * @param cycles    循环次数 (0 = 无限循环，1 = 单次，2 = 来回两次)
     */
    void sweep_pulse(ServoID_t id,
                     uint32_t min_pulse, uint32_t max_pulse,
                     uint32_t step_us, uint32_t wait_ms,
                     uint32_t cycles);

    /**
     * @brief 设置舵机到指定角度
     * @param id        舵机编号
     * @param angle_deg 目标角度 (0.0 ~ 180.0)
     */
    void test_angle(ServoID_t id, float angle_deg);

    /**
     * @brief 直接设置舵机占空比
     * @param id   舵机编号
     * @param duty 占空比值 (0 ~ 2^SERVO_PWM_BITS)
     */
    void test_set_duty(ServoID_t id, uint32_t duty);

    /**
     * @brief 读取指定舵机的当前脉宽 (μs)
     *
     * 读取 LEDC 硬件占空比，换算为脉宽微秒数并打印日志。
     * 可在 sweep_pulse 或 test_angle 之后调用，验证实际输出。
     *
     * @param id  舵机编号
     * @return 当前脉宽 (μs)
     */
    uint32_t test_read_pulse(ServoID_t id);

    /**
     * @brief 平滑移动单个舵机到目标脉宽
     *
     * 从当前脉宽以 step_us 为步长逐步移动到 target_pulse，
     * 每步等待 wait_ms 毫秒。
     *
     * @param id           舵机编号
     * @param target_pulse 目标脉宽 (μs)
     * @param step_us      步长 (μs)
     * @param wait_ms      每步等待 (ms)
     */
    void move_servo_to(ServoID_t id, uint32_t target_pulse,
                       uint32_t step_us, uint32_t wait_ms);

    /**
     * @brief 多舵机并行运动到各自目标脉宽
     *
     * 所有指定舵机同时以相同步长和间隔运动，
     * 以最慢舵机到达目标后停止。
     *
     * @param ids[]     舵机ID数组
     * @param targets[] 目标脉宽数组 (μs)，与 ids 一一对应
     * @param count     舵机数量
     * @param step_us   步长 (μs)
     * @param wait_ms   每步等待 (ms)
     */
    void move_multi_to(const ServoID_t ids[], const uint32_t targets[],
                       int count, uint32_t step_us, uint32_t wait_ms);

    /**
     * @brief 打印所有舵机当前状态 (占空比、脉宽、近似角度)
     */
    void test_print_status(void);

    /**
     * @brief 执行预定义 5 阶段动作序列
     *
     * 步进值 10μs，间隔 30ms。
     * Phase1: 舵机1~6 六路并行到初始目标
     * Phase2: 舵机0 1500→2300 (单舵)
     * Phase3: 舵机3→1450, 4→1200, 5→1500 (并行)
     * Phase4: 舵机6 1150→2250 (单舵)
     * Phase5: 舵机2→1850, 3→1150, 4→2150 (并行)
     */
    void action_sequence(void);

#ifdef __cplusplus
}
#endif
