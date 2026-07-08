/**
 * @file  kinematics.c
 * @brief 4-DOF 机械臂运动学 (θ3 动态 IK)
 *
 * 几何模型 (侧视图):
 *   J1 ●──── L1 ────● J2 ──── L2 ────● J3 ─── L3 ───● Rotate中心
 *
 * 坐标系原点 = 底座云台投影到地面的点
 *   X 正前, Y 顺基座逆时针 90°, Z 垂直向上
 *   J1 转轴高度 = d1 = 148 mm (云台135 + 偏移13)
 *
 * 关节几何角定义:
 *   θ0 : Base 水平旋转 (逆时针为正), 几何角 = servo0
 *   θ1 : J1 绝对俯仰 (0°=水平前伸), 几何角 = servo1
 *   θ2 : J2 相对角 (伸直=0°), 几何角 = -60 + (152/180)*servo2
 *   θ3 : J3 相对角 (由 IK 动态求解)
 */

#include "kinematics.h"
#include "servo_cfg.h"
#include "esp_log.h"
#include <math.h>

#define TAG "KIN"

/*-----------------------------------------------------
 *  辅助常量与内联函数 (替代宏，避免预处理器跨平台问题)
 *-----------------------------------------------------*/
static const float PI = 3.14159265358979323846f;
static inline float deg2rad(float d) { return d * PI / 180.0f; }
static inline float rad2deg(float r) { return r * 180.0f / PI; }
#define SQ(x) ((x) * (x))
#define CLAMP(v, lo, hi) ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))

/* 运行时校准参数 — 可从 NVS 加载覆盖宏默认值 */
static float J2_OFFSET_VAL = J2_OFFSET_DEG;
static float J2_SCALE_VAL  = J2_SCALE;
static float INV_J2_SCALE  = 1.0f / J2_SCALE;
static float J3_OFFSET_VAL = J3_OFFSET_DEG;
static float J3_SCALE_VAL  = J3_SCALE_DEG;
static float ARM_D1_VAL     = ARM_D1;
static float ARM_L3_VAL     = ARM_L3;

#include "calib_nvs.h"

void kinematics_load_calibration(void)
{
    CalibrationData_t calib;
    if (calib_nvs_load(&calib) == ESP_OK)
    {
        J2_OFFSET_VAL = calib.j2_offset_deg;
        J2_SCALE_VAL  = calib.j2_scale;
        INV_J2_SCALE  = 1.0f / calib.j2_scale;
        J3_OFFSET_VAL = calib.j3_offset_deg;
        J3_SCALE_VAL  = calib.j3_scale;
        ARM_D1_VAL    = calib.arm_d1;
        ARM_L3_VAL    = calib.arm_l3;
        ESP_LOGI(TAG, "Calibration loaded from NVS");
    }
    else
    {
        ESP_LOGI(TAG, "Using default calibration from servo_cfg.h");
    }
}

void kinematics_save_calibration(void)
{
    CalibrationData_t calib = {
        .j2_offset_deg = J2_OFFSET_VAL,
        .j2_scale      = J2_SCALE_VAL,
        .j3_offset_deg = J3_OFFSET_VAL,
        .j3_scale      = J3_SCALE_VAL,
        .arm_d1        = ARM_D1_VAL,
        .arm_l3        = ARM_L3_VAL,
    };
    calib_nvs_save(&calib);
}

float kinematics_get_d1(void) { return ARM_D1_VAL; }
float kinematics_get_l3(void) { return ARM_L3_VAL; }

/*-----------------------------------------------------
 *  几何角 ↔ 舵机逻辑角 映射
 *-----------------------------------------------------*/
void kinematics_geom_to_servo(const JointAngles_t *geom,
                              ServoAngles_t *servo)
{
    if (!geom || !servo)
        return;

    servo->servo0 = geom->theta0;

    /* θ1 = servo1 */
    servo->servo1 = geom->theta1;

    /* θ2 = -60 + (152/180) * servo2 → servo2 = (θ2 + 60) * 180 / 152 */
    servo->servo2 = (geom->theta2 - J2_OFFSET_VAL) * INV_J2_SCALE;

    /* θ3 → servo3: servo = θ3 * scale + offset (offset=100 时 -100°→0°, 80°→180°) */
    servo->servo3 = geom->theta3 * J3_SCALE_VAL + J3_OFFSET_VAL;

    /* Rotate: servo4 = theta4 (无偏移, 全范围) */
    servo->servo4 = geom->theta4;
    /* Gripper 暂不通过运动学控制 */
    servo->servo5 = 0.0f;
}

void kinematics_servo_to_geom(const ServoAngles_t *servo,
                              JointAngles_t *geom)
{
    if (!servo || !geom)
        return;

    geom->theta0 = servo->servo0;
    geom->theta1 = servo->servo1;
    geom->theta2 = J2_OFFSET_VAL + J2_SCALE_VAL * servo->servo2;
    geom->theta3 = (servo->servo3 - J3_OFFSET_VAL) / J3_SCALE_VAL;
    geom->theta4 = servo->servo4;
}

/*-----------------------------------------------------
 *  正向运动学
 *-----------------------------------------------------*/
void kinematics_forward(const JointAngles_t *geom,
                        ArmTipState_t *tip)
{
    if (!geom || !tip)
        return;

    float t0 = deg2rad(geom->theta0);
    float t1 = deg2rad(geom->theta1);
    float t2 = deg2rad(geom->theta2);
    float t3 = deg2rad(geom->theta3);

    /* 计算侧视平面内的坐标 (y_local = 0) */
    float z_j2 = ARM_D1_VAL + ARM_L1 * sinf(t1);
    float r_j2 = ARM_L1 * cosf(t1);
    float z_j3 = z_j2 + ARM_L2 * sinf(t1 + t2);
    float r_j3 = r_j2 + ARM_L2 * cosf(t1 + t2);
    float z_tip = z_j3 + ARM_L3_VAL * sinf(t1 + t2 + t3);
    float r_plane = r_j3 + ARM_L3_VAL * cosf(t1 + t2 + t3);

    /* 以 base 旋转角 t0 展开到 3D 世界坐标 */
    tip->x = r_plane * cosf(t0);
    tip->y = r_plane * sinf(t0);
    tip->z = z_tip;

    /* 末端连杆俯仰 = t1 + t2 + t3 */
    tip->yaw = geom->theta1 + geom->theta2 + geom->theta3;
}

/*=======================================================
 *  solve_j2_j3_for_j1() — 给定 θ₁ 用余弦定理求解 θ₂/θ₃
 *
 *  原理: 对固定 θ₁，问题退化为 2 连杆 (L2+L3) 到达目标 (rx, rz)。
 *        余弦定理给出至多两个解 (肘向下/肘向上)，不需要 yaw 约束。
 *        yaw 是计算结果而非输入 — 从两个候选中选 yaw 最接近目标者。
 *
 *  输入: j1_rad         θ₁ (弧度)
 *        r, vz          J1 到目标点的向量 (mm)
 *        yaw_target_rad  期望末端俯仰 (弧度, 软约束)
 *  输出: out_j2_deg, out_j3_deg, out_yaw_rad (实际末端俯仰)
 *  返回: 0 = 位置精确可达; INFINITY = 不可达或关节越限
 *=======================================================*/
static float solve_j2_j3_for_j1(float j1_rad, float r, float vz,
                                float yaw_target_rad,
                                float *out_j2_deg, float *out_j3_deg,
                                float *out_yaw_rad)
{
    /* J1 到目标点的向量 */
    float rx = r - ARM_L1 * cosf(j1_rad);
    float rz = vz - ARM_L1 * sinf(j1_rad);
    float d2 = rx * rx + rz * rz;
    float d = sqrtf(d2);

    /* 可达性: |L2 - L3| ≤ d ≤ L2 + L3 */
    float l_min = fabsf(ARM_L2 - ARM_L3_VAL);
    float l_max = ARM_L2 + ARM_L3_VAL;
    if (d < l_min - 0.5f || d > l_max + 0.5f)
        return INFINITY;

    /* 余弦定理: L3² = L2² + d² - 2·L2·d·cos(α)
       → cos(α) = (L2² + d² - L3²) / (2·L2·d)
       α 是 L2 与 d 之间的夹角 */
    float cos_alpha = (ARM_L2 * ARM_L2 + d2 - ARM_L3_VAL * ARM_L3_VAL)
                      / (2.0f * ARM_L2 * d);
    if (cos_alpha > 1.0f) cos_alpha = 1.0f;
    if (cos_alpha < -1.0f) cos_alpha = -1.0f;
    float alpha = acosf(cos_alpha);

    /* 余弦定理: d² = L2² + L3² - 2·L2·L3·cos(π-θ₃)
       → cos(θ₃) = (d² - L2² - L3²) / (2·L2·L3)  */
    float cos_beta = (d2 - ARM_L2 * ARM_L2 - ARM_L3_VAL * ARM_L3_VAL)
                     / (2.0f * ARM_L2 * ARM_L3_VAL);
    if (cos_beta > 1.0f) cos_beta = 1.0f;
    if (cos_beta < -1.0f) cos_beta = -1.0f;
    float beta = acosf(cos_beta); /* β = π - |θ₃| */

    /* d 的基方向 */
    float base_angle = atan2f(rz, rx);

    /* 两个候选解 */
    /* 候选 A (肘向下): θ₂ = base - θ₁ - α,  θ₃ = +β */
    /* 候选 B (肘向上): θ₂ = base - θ₁ + α,  θ₃ = -β */
    float j2_cand[2], j3_cand[2];
    j2_cand[0] = base_angle - j1_rad - alpha;
    j3_cand[0] = beta;
    j2_cand[1] = base_angle - j1_rad + alpha;
    j3_cand[1] = -beta;

    float best_yaw_err = INFINITY;
    float best_j2 = 0.0f, best_j3 = 0.0f, best_yaw = 0.0f;

    for (int i = 0; i < 2; i++)
    {
        float j2_deg = rad2deg(j2_cand[i]);
        float j3_deg = rad2deg(j3_cand[i]);

        /* 关节限位 */
        if (j2_deg < -56.0f || j2_deg > 93.0f) /* 实测极限[-56°,95°] */
            continue;
        if (j3_deg < -140.0f || j3_deg > 150.0f) /* 实测[-146°,156°], 留6°余量 */
            continue;

        float actual_yaw = j1_rad + j2_cand[i] + j3_cand[i];
        float yaw_err = fabsf(actual_yaw - yaw_target_rad);

        /* yaw 误差越小越好; 同等误差下优先肘向下 (i=0) */
        if (yaw_err < best_yaw_err - 1e-6f)
        {
            best_yaw_err = yaw_err;
            best_j2 = j2_deg;
            best_j3 = j3_deg;
            best_yaw = actual_yaw;
        }
    }

    if (best_yaw_err == INFINITY)
        return INFINITY;

    *out_j2_deg = best_j2;
    *out_j3_deg = best_j3;
    *out_yaw_rad = best_yaw;
    return 0.0f; /* 余弦定理解 = 位置精确 */
}

/*=======================================================
 *  ik_solve_full() — 平面 3R IK (余弦定理 + 水平优先)
 *
 *  两遍搜索:
 *    Pass1: 仅接受 |yaw - target| < 10° (优先爪子水平)
 *    Pass2: 若 Pass1 无解, 放宽到任意 yaw (取最佳)
 *
 *  输入: r, vz, yaw_deg (期望末端俯仰, 通常 0=水平)
 *  输出: geom->theta1 / theta2 / theta3
 *=======================================================*/
static IKResult_t ik_solve_full(float r, float vz, float yaw_deg, JointAngles_t *geom)
{
    const float yaw_target_rad = deg2rad(yaw_deg);
    const float J1_STEP = 0.5f;
    const float LEVEL_THRESHOLD_RAD = deg2rad(10.0f); /* ±10° 内算水平 */

    float best_yaw_err = INFINITY;
    float best_j1 = 0.0f, best_j2 = 0.0f, best_j3 = 0.0f;
    bool found_level = false;

    /*=== Pass1: 严格水平 (|Δyaw| < 10°) ===*/
    for (float j1_deg = 0.0f; j1_deg <= 90.0f + 1e-4f; j1_deg += J1_STEP)
    {
        float j1_rad = deg2rad(j1_deg);
        float j2_deg, j3_deg, actual_yaw_rad;
        float err = solve_j2_j3_for_j1(j1_rad, r, vz, yaw_target_rad,
                                       &j2_deg, &j3_deg, &actual_yaw_rad);
        if (err == INFINITY)
            continue;

        float yaw_err = fabsf(actual_yaw_rad - yaw_target_rad);

        if (yaw_err <= LEVEL_THRESHOLD_RAD)
        {
            found_level = true;
            if (yaw_err < best_yaw_err)
            {
                best_yaw_err = yaw_err;
                best_j1 = j1_deg;
                best_j2 = j2_deg;
                best_j3 = j3_deg;
                if (yaw_err < 0.0087f) break; /* < 0.5° */
            }
        }
        else if (!found_level && yaw_err < best_yaw_err)
        {
            /* Pass1 还没找到水平解, 先记下最佳非水平备选 */
            best_yaw_err = yaw_err;
            best_j1 = j1_deg;
            best_j2 = j2_deg;
            best_j3 = j3_deg;
        }
    }

    /*=== Pass2: Pass1 没找到水平解, 放宽重搜 ===*/
    if (!found_level && best_yaw_err != INFINITY)
    {
        for (float j1_deg = 0.0f; j1_deg <= 90.0f + 1e-4f; j1_deg += J1_STEP)
        {
            float j1_rad = deg2rad(j1_deg);
            float j2_deg, j3_deg, actual_yaw_rad;
            float err = solve_j2_j3_for_j1(j1_rad, r, vz, yaw_target_rad,
                                           &j2_deg, &j3_deg, &actual_yaw_rad);
            if (err == INFINITY) continue;
            float yaw_err = fabsf(actual_yaw_rad - yaw_target_rad);
            if (yaw_err < best_yaw_err)
            {
                best_yaw_err = yaw_err;
                best_j1 = j1_deg;
                best_j2 = j2_deg;
                best_j3 = j3_deg;
            }
        }
        ESP_LOGW(TAG, "IK: no level solution, using best yaw (Δ=%.1f°)",
                 (double)rad2deg(best_yaw_err));
    }

    if (best_yaw_err == INFINITY)
    {
        ESP_LOGW(TAG, "IK unreachable: (r=%.1f, vz=%.1f)",
                 (double)r, (double)vz);
        return IK_UNREACHABLE;
    }

    geom->theta1 = best_j1;
    geom->theta2 = best_j2;
    geom->theta3 = best_j3;

    float actual_yaw_deg = best_j1 + best_j2 + best_j3;
    ESP_LOGI(TAG, "IK solved: θ1=%.2f° θ2=%.2f° θ3=%.2f° "
             "(yaw=%.1f°, target=%.1f°, Δ=%.1f°)",
             (double)best_j1, (double)best_j2, (double)best_j3,
             (double)actual_yaw_deg, (double)yaw_deg,
             (double)fabsf(actual_yaw_deg - yaw_deg));
    return IK_OK;
}

/*=======================================================
 *  逆运动学 — 公有接口
 *=======================================================*/
IKResult_t kinematics_inverse(const ArmTipState_t *target, JointAngles_t *geom)
{
    if (!target || !geom)
        return IK_INVALID_INPUT;

    float theta0_deg = rad2deg(atan2f(target->y, target->x));

    float r = sqrtf(target->x * target->x + target->y * target->y);
    float vz = target->z - ARM_D1_VAL;

    IKResult_t ret = ik_solve_full(r, vz, target->yaw, geom);
    if (ret != IK_OK)
        return ret;

    geom->theta0 = theta0_deg;
    return IK_OK;
}

/*-----------------------------------------------------
 *  关节限位检查
 *-----------------------------------------------------*/
bool kinematics_check_limits(const JointAngles_t *geom)
{
    if (!geom)
        return false;

    ServoAngles_t servo;
    kinematics_geom_to_servo(geom, &servo);

    /* 底座安全范围检查 */
    if (servo.servo0 < BASE_SAFE_MIN_DEG || servo.servo0 > BASE_SAFE_MAX_DEG)
    {
        ESP_LOGW("IK_CHECK", "servo0=%.2f out of [%.0f, %.0f]",
                 (double)servo.servo0, (double)BASE_SAFE_MIN_DEG, (double)BASE_SAFE_MAX_DEG);
        return false;
    }
    if (servo.servo1 < SERVO_ANGLE_MIN_DEG || servo.servo1 > SERVO_ANGLE_MAX_DEG)
    {
        ESP_LOGW("IK_CHECK", "servo1=%.2f out of [0, 180]", (double)servo.servo1);
        return false;
    }
    if (servo.servo2 < SERVO_ANGLE_MIN_DEG || servo.servo2 > SERVO_ANGLE_MAX_DEG)
    {
        ESP_LOGW("IK_CHECK", "servo2=%.2f out of [0, 180]", (double)servo.servo2);
        return false;
    }
    if (servo.servo3 < SERVO_ANGLE_MIN_DEG || servo.servo3 > SERVO_ANGLE_MAX_DEG)
    {
        ESP_LOGW("IK_CHECK", "servo3=%.2f out of [0, 180]", (double)servo.servo3);
        return false;
    }
    return true;
}