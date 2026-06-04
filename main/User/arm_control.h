/**
 * @file  arm_control.h
 * @brief 机械臂高层控制接口 (修正底座映射，限速，增加安全底座移动)
 */

#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#include "kinematics.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /*-----------------------------------------------------
     *  初始化与状态
     *-----------------------------------------------------*/

    /**
     * @brief 初始化机械臂控制
     *        移动到 HOME 姿态: Base 180° (正前方), J1=90°, J2=0° (收拢),
     *        J3=90° (待校准), Gripper 打开
     * @note  调用前必须已初始化 servo_init() 和 servo_set_init()
     */
    void arm_control_init(void);

    /*-----------------------------------------------------
     *  末端移动
     *-----------------------------------------------------*/

    /**
     * @brief 移动末端到指定世界坐标 (阻塞)
     *        内部调用 IK，若可达且满足底座安全限制则驱动关节并等待完成。
     * @param x,y,z 目标位置 (mm)，世界坐标系，原点在 Base 转轴中心投影。
     * @return true 成功
     */
    bool arm_control_move_to(float x, float y, float z);

    /*-----------------------------------------------------
     *  安全底座控制
     *-----------------------------------------------------*/

    /**
     * @brief 直接设置底座舵机角度 (带安全边界钳位)
     * @param base_servo_deg 目标舵机逻辑角 (度)，会被限制在 [45°, 135°]
     * @note  此函数会等待运动完成。
     */
    void arm_control_set_base_safe(float base_servo_deg);

    /**
     * @brief 安全旋转底座 (自动抬臂避免碰撞履带)
     *        1. 记录当前关节
     *        2. 抬起大臂到安全高度 (J1=60°)
     *        3. 抬起小臂与大臂平行 (θ2=0)
     *        4. 执行底座旋转到目标舵机角度
     *        5. 恢复原有关节角度
     * @param base_servo_deg 目标底座舵机逻辑角 (0~180°，会被钳位)
     */
    void arm_control_rotate_base_safe(float base_servo_deg);

    /*-----------------------------------------------------
     *  夹爪控制
     *-----------------------------------------------------*/

    /**
     * @brief 设置夹爪开合百分比
     * @param open_percent 0=全开, 100=全闭
     * @note  函数会等待夹爪运动完成。
     */
    void arm_control_set_gripper(float open_percent);

    /**
     * @brief 设置夹爪旋转角度 (舵机逻辑角 0~180°)
     * @param servo_deg 目标舵机角度
     */
    void arm_control_set_rotate(float servo_deg);

    /*-----------------------------------------------------
     *  预置动作序列
     *-----------------------------------------------------*/

    /**
     * @brief 低点抓取并旋转放置序列 (适配 J3 固定)
     *        流程: 移动到前方 (200,0,140) → 闭合夹爪 →
     *              安全抬起 → 旋转底座到 90° (右侧) →
     *              放置到 (0,200,140) → 打开夹爪 → 回 HOME
     */
    void arm_control_pick_and_place_low(void);

    /*-----------------------------------------------------
     *  状态查询
     *-----------------------------------------------------*/

    /**
     * @brief 获取当前估计末端位姿 (基于目标关节角)
     */
    void arm_control_get_current_tip(ArmTipState_t *tip);

#ifdef __cplusplus
}
#endif

#endif /* ARM_CONTROL_H */