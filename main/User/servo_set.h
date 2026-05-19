#ifndef SERVO_SET_H
#define SERVO_SET_H

#include "servo_cfg.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 初始化舵机缓动系统 (可重复调用，自动销毁旧 timer)
     * @note  必须在 servo_init() 之后调用
     */
    void servo_set_init(void);

    /**
     * @brief 设置单个舵机目标角度 (五次多项式缓动)
     *        若缓动中再次调用 → 从当前位置平滑过渡到新目标
     * @param id    ServoID_t
     * @param angle 目标角度 [0, 180]
     */
    void servo_set_angle(ServoID_t id, float angle);

    /**
     * @brief 一次性设置全部 6 轴目标
     *        angles 数组索引严格对应 ServoID_t 顺序
     * @param angles[6] 各轴角度 (度)
     */
    void servo_set_joints(const float angles[SERVO_COUNT]);

    /**
     * @brief 停止单个舵机缓动 (保留当前位置)
     */
    void servo_stop(ServoID_t id);

    /**
     * @brief 停止全部舵机缓动 (底盘紧急刹车 / 手柄接管时调用)
     */
    void servo_stop_all(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_SET_H */