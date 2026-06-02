/**
 * @file  kinematics.c
 * @brief 机械臂运动学实现 (已加入关节限位与浮点容差)
 */

#include "kinematics.h"
#include <math.h>
#include <stddef.h>

#define DEG2RAD (M_PI / 180.0f)
#define RAD2DEG (180.0f / M_PI)

/* 浮点比较容差 (度) */
#define EPS_ANGLE_DEG 1e-4f

static const float THETA3_RAD = J3_FIXED_GEOM_DEG * DEG2RAD;

static const float C_VALUE = ARM_L2 + ARM_L3 * 0.5f;  /* 128.5 */
static const float D_VALUE = ARM_L3 * 0.86602540378f; /* 54.558 */
static const float C2_D2 = C_VALUE * C_VALUE + D_VALUE * D_VALUE;

static const float J2_OFFSET_VAL = J2_OFFSET_DEG; /* -60 */
static const float J2_SCALE_VAL = J2_SCALE;       /* 152/180 */
static const float INV_J2_SCALE = 1.0f / J2_SCALE;

/*-----------------------------------------------------
 *  几何角 <-> 舵机角
 *-----------------------------------------------------*/
void kinematics_geom_to_servo(const JointAngles_t *geom,
                              ServoAngles_t *servo)
{
    if (!geom || !servo)
        return;
    servo->servo0 = -geom->theta0;
    servo->servo1 = geom->theta1;
    servo->servo2 = (geom->theta2 - J2_OFFSET_VAL) * INV_J2_SCALE;
    servo->servo3 = 90.0f; /* 占位，实际硬件固定 */
    servo->servo4 = 0.0f;
    servo->servo5 = 0.0f;
}

void kinematics_servo_to_geom(const ServoAngles_t *servo,
                              JointAngles_t *geom)
{
    if (!servo || !geom)
        return;
    geom->theta0 = -servo->servo0;
    geom->theta1 = servo->servo1;
    geom->theta2 = J2_OFFSET_VAL + J2_SCALE_VAL * servo->servo2;
    geom->theta3 = J3_FIXED_GEOM_DEG;
    geom->theta4 = 0.0f;
}

/*-----------------------------------------------------
 *  FK
 *-----------------------------------------------------*/
void kinematics_forward(const JointAngles_t *geom,
                        ArmTipState_t *tip)
{
    if (!geom || !tip)
        return;

    float a = geom->theta1 * DEG2RAD;
    float b = (geom->theta1 + geom->theta2) * DEG2RAD;
    float c = b + THETA3_RAD;

    float r = ARM_L1 * cosf(a) + ARM_L2 * cosf(b) + ARM_L3 * cosf(c);
    float z = ARM_D1 + ARM_L1 * sinf(a) + ARM_L2 * sinf(b) + ARM_L3 * sinf(c);

    float th0 = geom->theta0 * DEG2RAD;
    tip->x = r * cosf(th0);
    tip->y = r * sinf(th0);
    tip->z = z;
    tip->yaw = (geom->theta1 + geom->theta2 + geom->theta3);
}

/*-----------------------------------------------------
 *  IK (带约束)
 *-----------------------------------------------------*/
static IKResult_t ik_solve_plane(float r, float vz, JointAngles_t *geom)
{
    float K = (r * r + vz * vz + C2_D2 - ARM_L1 * ARM_L1) * 0.5f;

    float M = r * C_VALUE + vz * D_VALUE;
    float N = vz * C_VALUE - r * D_VALUE;
    float M2_N2 = M * M + N * N;

    if (M2_N2 < 1e-6f)
        return IK_UNREACHABLE;
    if (K * K > M2_N2)
        return IK_UNREACHABLE;

    float sqrt_val = sqrtf(M2_N2 - K * K);
    float phi = atan2f(N, M);
    float cos_acos = K / sqrtf(M2_N2);
    if (cos_acos < -1.0f)
        cos_acos = -1.0f;
    if (cos_acos > 1.0f)
        cos_acos = 1.0f;
    float acos_val = acosf(cos_acos);

    float beta1 = phi + acos_val;
    float beta2 = phi - acos_val;

    float best_alpha = 0, best_theta2_deg = 0;
    int found = 0;

    for (int i = 0; i < 2; i++)
    {
        float beta = (i == 0) ? beta1 : beta2;

        float A_term = C_VALUE * cosf(beta) - D_VALUE * sinf(beta);
        float B_term = C_VALUE * sinf(beta) + D_VALUE * cosf(beta);
        float dx = r - A_term;
        float dz = vz - B_term;
        float alpha = atan2f(dz, dx);
        float theta2_rad = beta - alpha;
        float theta1_deg = alpha * RAD2DEG;
        float theta2_deg = theta2_rad * RAD2DEG;

        /* --- 关键约束 (带浮点容差) --- */
        if (theta1_deg < -EPS_ANGLE_DEG)
            continue; /* J1≥0 */
        if (theta2_deg < -60.0f - EPS_ANGLE_DEG ||
            theta2_deg > 92.0f + EPS_ANGLE_DEG)
            continue;

        /* 优先肘向下 (θ2≤0)，其次绝对值小 */
        if (!found || (theta2_deg <= 0.0f && best_theta2_deg > 0.0f) ||
            (theta2_deg <= 0.0f && fabsf(theta2_deg) < fabsf(best_theta2_deg)))
        {
            best_alpha = alpha;
            best_theta2_deg = theta2_deg;
            found = 1;
        }
    }

    if (!found)
        return IK_UNREACHABLE;

    geom->theta1 = best_alpha * RAD2DEG;
    geom->theta2 = best_theta2_deg;
    geom->theta3 = J3_FIXED_GEOM_DEG;
    return IK_OK;
}

IKResult_t kinematics_inverse(const ArmTipState_t *target,
                              JointAngles_t *geom)
{
    if (!target || !geom)
        return IK_INVALID_INPUT;

    /* Base 角度 */
    float theta0_rad = atan2f(target->y, target->x);
    float theta0_deg = theta0_rad * RAD2DEG;

    /* Base 约束: 只能右转，故 θ0 必须在 [-180, 0] 内。
       使用容差：如果 theta0_deg > EPS 则不可达 (左方)。 */
    if (theta0_deg > EPS_ANGLE_DEG)
    {
        return IK_UNREACHABLE;
    }
    if (theta0_deg < -180.0f - EPS_ANGLE_DEG)
    {
        return IK_UNREACHABLE;
    }

    /* 平面投影 */
    float r = sqrtf(target->x * target->x + target->y * target->y);
    float vz = target->z - ARM_D1;

    IKResult_t ret = ik_solve_plane(r, vz, geom);
    if (ret != IK_OK)
        return ret;

    geom->theta0 = theta0_deg;
    return IK_OK;
}

bool kinematics_check_limits(const JointAngles_t *geom)
{
    if (!geom)
        return false;
    ServoAngles_t servo;
    kinematics_geom_to_servo(geom, &servo);

    if (servo.servo0 < 0.0f || servo.servo0 > 180.0f)
        return false;
    if (servo.servo1 < 0.0f || servo.servo1 > 180.0f)
        return false;
    if (servo.servo2 < 0.0f || servo.servo2 > 180.0f)
        return false;
    return true;
}