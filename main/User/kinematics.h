/**
 * @file  kinematics.h
 * @brief 机械臂运动学接口 (正运动学、逆运动学、角度映射)
 *
 * 几何模型:
 *   d1 = 13 mm   (Base 转轴 -> J1 转轴 垂直高)
 *   L1 = 104 mm  (上臂 J1->J2)
 *   L2 = 97 mm   (前臂 J2->J3)
 *   L3 = 63 mm   (末端 J3->Rotate 中心)
 *
 * 关节几何角定义:
 *   θ0 : Base 水平旋转 (逆时针为正), 几何角 = servo0
 *   θ1 : J1 绝对俯仰 (0°=水平前伸, 90°=竖直向上), 几何角 = servo1
 *   θ2 : J2 相对角 (伸直=0°, 前臂向下折为负),
 *        几何角 = -60 + (152/180)*servo2
 *   θ3 : J3 固定 60°
 */

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C"
{
#endif

/*-----------------------------------------------------
 *  机械尺寸参数 (单位: mm)
 *-----------------------------------------------------*/
#define ARM_BASE_HEIGHT 128.0f                   /* 实测: 云台离地141 - J1偏移13 = 128 */
#define ARM_J1_OFFSET 13.0f                      /* Base 转轴到 J1 转轴 */
#define ARM_D1 (ARM_BASE_HEIGHT + ARM_J1_OFFSET) /* J1 转轴离地高度 = 148mm */
#define ARM_L1 104.0f                            /* 上臂长 (J1 转轴 -> J2 转轴) */
#define ARM_L2 97.0f                             /* 前臂长 (J2 转轴 -> J3 转轴) */
#define ARM_L3 130.0f                            /* 末端长 (J3 转轴 -> 夹爪抓取中心) — 实测 */

/*-----------------------------------------------------
 *  J2 角度映射参数 (线性关系)
 *   θ2 = J2_OFFSET + J2_SCALE * servo2
 *-----------------------------------------------------*/
#define J2_OFFSET_DEG (-56.0f)   /* 实测: servo2=0° → θ2=-56° */
#define J2_SCALE (151.0f / 180.0f) /* 实测: (95-(-56))/180 ≈ 0.8389 */

    /* J3 不再固定，由 IK 动态求解 */

    /*-----------------------------------------------------
     *  数据结构
     *-----------------------------------------------------*/

    /**
     * @brief 关节几何角度 (单位: 度)
     */
    typedef struct
    {
        float theta0; /* Base 水平旋转 (逆时针为正) */
        float theta1; /* J1 绝对俯仰 (0°=水平前伸) */
        float theta2; /* J2 相对角 (伸直0°) */
        float theta3; /* J3 相对角 (动态, IK 求解) */
        float theta4; /* Rotate (暂未用) */
    } JointAngles_t;

    /**
     * @brief 舵机逻辑角度 (0~180°, 可直接传给 servo_set_angle)
     */
    typedef struct
    {
        float servo0; /* Base */
        float servo1; /* Joint1 */
        float servo2; /* Joint2 */
        float servo3; /* Joint3 (动态) */
        float servo4; /* Rotate (暂未用) */
        float servo5; /* Gripper (暂未用) */
    } ServoAngles_t;

    /**
     * @brief 末端执行器状态 (Rotate 中心)
     */
    typedef struct
    {
        float x;   /* 世界坐标 X (mm) */
        float y;   /* 世界坐标 Y (mm) */
        float z;   /* 世界坐标 Z (mm) */
        float yaw; /* 末端连杆俯仰角 (度, 水平=0, IK 期望输入) */
    } ArmTipState_t;

    /*-----------------------------------------------------
     *  逆运动学返回结果
     *-----------------------------------------------------*/
    typedef enum
    {
        IK_OK = 0,       /* 解算成功 */
        IK_UNREACHABLE,  /* 目标点超出工作空间 */
        IK_INVALID_INPUT /* 输入参数无效 */
    } IKResult_t;

    /*-----------------------------------------------------
     *  函数声明
     *-----------------------------------------------------*/

    /**
     * @brief 几何关节角 -> 舵机逻辑角
     * @param geom   [in]  几何关节角
     * @param servo  [out] 对应的舵机逻辑角度 (0~180°)
     */
    void kinematics_geom_to_servo(const JointAngles_t *geom,
                                  ServoAngles_t *servo);

    /**
     * @brief 舵机逻辑角 -> 几何关节角
     * @param servo  [in]  舵机角度
     * @param geom   [out] 几何关节角
     */
    void kinematics_servo_to_geom(const ServoAngles_t *servo,
                                  JointAngles_t *geom);

    /**
     * @brief 正运动学: 根据几何关节角计算末端位姿
     * @param geom  [in]  几何关节角
     * @param tip   [out] 末端位姿 (Rotate 中心)
     */
    void kinematics_forward(const JointAngles_t *geom,
                            ArmTipState_t *tip);

    /**
     * @brief 逆运动学: 根据目标末端位姿求解几何关节角
     * @param target [in]  目标末端位姿 (只使用 x,y,z，俯仰忽略)
     * @param geom   [out] 解出的几何关节角 (theta0~theta3)
     * @return IK_OK 若成功, 否则 IK_UNREACHABLE / IK_INVALID_INPUT
     */
    IKResult_t kinematics_inverse(const ArmTipState_t *target,
                                  JointAngles_t *geom);

    /**
     * @brief 检查几何关节角是否在舵机允许范围内
     * @param geom 几何关节角
     * @return true 若所有关节对应舵机逻辑角均 ∈ [0,180]
     */
    bool kinematics_check_limits(const JointAngles_t *geom);

    /**
     * @brief 从 NVS 加载校准参数, 失败则使用 servo_cfg.h 默认值
     */
    void kinematics_load_calibration(void);

    /**
     * @brief 将当前运行时校准参数保存到 NVS
     */
    void kinematics_save_calibration(void);

    /** @brief 获取运行时 ARM_D1 值 (mm) */
    float kinematics_get_d1(void);

    /** @brief 获取运行时 ARM_L3 值 (mm) */
    float kinematics_get_l3(void);

#ifdef __cplusplus
}
#endif

#endif /* KINEMATICS_H */