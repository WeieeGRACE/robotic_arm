/**
 * @file  arm_control.c
 * @brief 机械臂高层控制实现 (θ3 动态 IK 版本)
 */

#include "arm_control.h"
#include "servo_cfg.h"
#include "servo_set.h"
#include "kinematics.h"
#include "esp_log.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "arm_ctrl";

/* 当前目标几何角 */
static JointAngles_t s_geom_target;

/*-----------------------------------------------------
 *  内部辅助: 等待所有舵机缓动完成
 *-----------------------------------------------------*/
static void wait_all_servos_idle(void)
{
    servo_wait_all_idle();
}

/*-----------------------------------------------------
 *  初始化
 *-----------------------------------------------------*/
void arm_control_init(void)
{
    ESP_LOGI(TAG, "Arm control init -> HOME");

    /* HOME: Base=0° J1=90° J2=-56°(最大折叠) J3=0°(伸直) Gripper打开 */
    s_geom_target.theta0 = 0.0f;
    s_geom_target.theta1 = 90.0f;
    s_geom_target.theta2 = -56.0f; /* 机械极限, servo2=0 */
    s_geom_target.theta3 = 0.0f;
    s_geom_target.theta4 = 0.0f;

    ServoAngles_t servo;
    kinematics_geom_to_servo(&s_geom_target, &servo);

    servo_set_angle(SERVO_ID_BASE, servo.servo0);
    servo_set_angle(SERVO_ID_JOINT1, servo.servo1);
    servo_set_angle(SERVO_ID_JOINT2, servo.servo2);
    servo_set_angle(SERVO_ID_JOINT3, servo.servo3);
    servo_set_angle(SERVO_ID_ROTATE, servo.servo4);
    servo_set_angle(SERVO_ID_GRIPPER, 0.0f); /* 夹爪打开 */

    wait_all_servos_idle();
    ESP_LOGI(TAG, "HOME reached (base=%.1f j1=%.1f j2=%.1f j3=%.1f rot=%.1f)",
             servo.servo0, servo.servo1, servo.servo2, servo.servo3, servo.servo4);
}

/*-----------------------------------------------------
 *  末端移动
 *-----------------------------------------------------*/
bool arm_control_move_to(float x, float y, float z)
{
    ArmTipState_t target = {
        .x = x,
        .y = y,
        .z = z,
        .yaw = 0.0f /* 末端水平 */};

    JointAngles_t geom;
    IKResult_t res = kinematics_inverse(&target, &geom);
    if (res != IK_OK)
    {
        ESP_LOGW(TAG, "IK failed for (%.1f, %.1f, %.1f)", x, y, z);
        return false;
    }
    ESP_LOGI(TAG, "IK solution: θ0=%.2f θ1=%.2f θ2=%.2f θ3=%.2f",
             geom.theta0, geom.theta1, geom.theta2, geom.theta3);

    /* 检查底座安全范围 + 关节限位 */
    if (!kinematics_check_limits(&geom))
    {
        ESP_LOGE(TAG, "Joint limits or base safety exceeded");
        return false;
    }

    /* 转换为舵机角度并执行 */
    ServoAngles_t servo;
    kinematics_geom_to_servo(&geom, &servo);

    servo_set_angle(SERVO_ID_BASE, servo.servo0);
    servo_set_angle(SERVO_ID_JOINT1, servo.servo1);
    servo_set_angle(SERVO_ID_JOINT2, servo.servo2);
    servo_set_angle(SERVO_ID_JOINT3, servo.servo3);
    /* 更新目标几何角 */
    s_geom_target = geom;
    s_geom_target.theta4 = 0.0f; /* Rotate 固定不动 */

    wait_all_servos_idle();
    ESP_LOGI(TAG, "Moved to (%.1f, %.1f, %.1f) servo: base=%.1f j1=%.1f j2=%.1f j3=%.1f",
             x, y, z, servo.servo0, servo.servo1, servo.servo2, servo.servo3);
    return true;
}

/*-----------------------------------------------------
 *  直接设置底座舵机角度 (带安全边界)
 *-----------------------------------------------------*/
void arm_control_set_base_safe(float base_servo_deg)
{
    /* 安全锁: 底座旋转前整条臂必须高于 J1 水平面 */
    ArmTipState_t tip;
    kinematics_forward(&s_geom_target, &tip);
    if (tip.z < (ARM_D1 - 10.0f))
    {
        ESP_LOGE(TAG, "BASE ROTATE BLOCKED: tip z=%.1f < D1=%.0f, "
                 "请先抬臂到水平面以上!", (double)tip.z, (double)ARM_D1);
        return;
    }

    if (base_servo_deg < BASE_SAFE_MIN_DEG)
        base_servo_deg = BASE_SAFE_MIN_DEG;
    if (base_servo_deg > BASE_SAFE_MAX_DEG)
        base_servo_deg = BASE_SAFE_MAX_DEG;

    ESP_LOGI(TAG, "Set base to servo %.1f°", base_servo_deg);

    servo_set_angle(SERVO_ID_BASE, base_servo_deg);
    wait_all_servos_idle();

    s_geom_target.theta0 = base_servo_deg;
}

/*-----------------------------------------------------
 *  安全底座旋转 (自动抬臂)
 *-----------------------------------------------------*/
void arm_control_rotate_base_safe(float base_servo_deg)
{
    /* 钳位目标角度 */
    if (base_servo_deg < BASE_SAFE_MIN_DEG)
        base_servo_deg = BASE_SAFE_MIN_DEG;
    if (base_servo_deg > BASE_SAFE_MAX_DEG)
        base_servo_deg = BASE_SAFE_MAX_DEG;

    ESP_LOGI(TAG, "Safe rotate base to servo %.1f°", base_servo_deg);

    /* 保存当前几何角 (用于恢复) */
    JointAngles_t saved_geom = s_geom_target;

    /* 1. 抬起大臂到安全高度 (J1=60°) 并摆正小臂 (θ2=0°) */
    JointAngles_t lift_geom;
    lift_geom.theta0 = saved_geom.theta0; /* 保持当前底座朝向 */
    lift_geom.theta1 = 60.0f;
    lift_geom.theta2 = 0.0f;
    lift_geom.theta3 = 60.0f; /* J3 保持水平 */

    ServoAngles_t servo;
    kinematics_geom_to_servo(&lift_geom, &servo);
    servo_set_angle(SERVO_ID_JOINT1, servo.servo1);
    servo_set_angle(SERVO_ID_JOINT2, servo.servo2);
    servo_set_angle(SERVO_ID_JOINT3, servo.servo3);
    wait_all_servos_idle();

    /* 2. 执行底座旋转 */
    servo_set_angle(SERVO_ID_BASE, base_servo_deg);
    wait_all_servos_idle();

    /* 3. 恢复原来的关节角度 (底座已更新) */
    ServoAngles_t restore;
    kinematics_geom_to_servo(&saved_geom, &restore);
    servo_set_angle(SERVO_ID_JOINT1, restore.servo1);
    servo_set_angle(SERVO_ID_JOINT2, restore.servo2);
    servo_set_angle(SERVO_ID_JOINT3, restore.servo3);
    wait_all_servos_idle();

    /* 更新内部状态 (底座已变，其余恢复) */
    s_geom_target.theta0 = base_servo_deg;
    s_geom_target.theta1 = saved_geom.theta1;
    s_geom_target.theta2 = saved_geom.theta2;
    s_geom_target.theta3 = saved_geom.theta3;

    ESP_LOGI(TAG, "Safe rotate complete");
}

/*-----------------------------------------------------
 *  夹爪控制
 *-----------------------------------------------------*/
/*-----------------------------------------------------
 *  夹爪旋转控制
 *-----------------------------------------------------*/
void arm_control_set_rotate(float servo_deg)
{
    if (servo_deg < SERVO_ANGLE_MIN_DEG)
        servo_deg = SERVO_ANGLE_MIN_DEG;
    if (servo_deg > SERVO_ANGLE_MAX_DEG)
        servo_deg = SERVO_ANGLE_MAX_DEG;

    servo_set_angle(SERVO_ID_ROTATE, servo_deg);
    wait_all_servos_idle();
    s_geom_target.theta4 = servo_deg;
    ESP_LOGI(TAG, "Rotate set to %.1f°", servo_deg);
}

/*-----------------------------------------------------
 *  夹爪控制
 *-----------------------------------------------------*/
void arm_control_set_gripper(float open_percent)
{
    if (open_percent < 0.0f)
        open_percent = 0.0f;
    if (open_percent > 100.0f)
        open_percent = 100.0f;

    /* 0°=全开, 180°=全闭 */
    float angle = (open_percent / 100.0f) * 180.0f;
    servo_set_angle(SERVO_ID_GRIPPER, angle);
    wait_all_servos_idle();
    ESP_LOGI(TAG, "Gripper %.0f%% open (angle %.1f°)", open_percent, angle);
}

/*-----------------------------------------------------
 *  获取当前末端位姿 (基于目标几何角)
 *-----------------------------------------------------*/
void arm_control_get_current_tip(ArmTipState_t *tip)
{
    kinematics_forward(&s_geom_target, tip);
}

/*-----------------------------------------------------
 *  IK 驱动的抓取-旋转-放置序列
 *
 *  Pick: (287, 0, 60)  Base=0° 前方低点
 *  Place: (-287, 0, 60) Base=180° 后方同高度
 *  安全: 底座旋转前必须抬臂到水平面以上
 *-----------------------------------------------------*/
void arm_control_pick_and_place_low(void)
{
    ESP_LOGI(TAG, "=== IK pick-and-place start ===");

    /* Step 1: 直接舵机角抓取 (θ1=0 θ2=-56 θ3=56 → yaw=0 爪子水平)
       servo: base=0 j1=0 j2=0 j3=60, FK≈(288,0,61) */
    ESP_LOGI(TAG, "Step1: Direct servo pick (yaw=0, gripper level)");
    servo_set_angle(SERVO_ID_BASE,    0.0f);
    servo_set_angle(SERVO_ID_JOINT1,  0.0f);
    servo_set_angle(SERVO_ID_JOINT2,  0.0f);   /* θ2=-56° 最大折叠 */
    servo_set_angle(SERVO_ID_JOINT3,  60.0f);  /* θ3=56° 回正水平 */
    servo_set_angle(SERVO_ID_GRIPPER, 0.0f);   /* 夹爪打开 */
    wait_all_servos_idle();
    /* 更新几何状态 */
    s_geom_target.theta0 = 0.0f;
    s_geom_target.theta1 = 0.0f;
    s_geom_target.theta2 = -56.0f;
    s_geom_target.theta3 = 56.0f;

    /* Step 2: 夹取 */
    ESP_LOGI(TAG, "Step2: Close gripper");
    arm_control_set_gripper(100.0f);

    /* Step 3: 抬臂到安全高度 (底座旋转前必须高于水平面) */
    ESP_LOGI(TAG, "Step3: Lift to safe height (200, 0, 350)");
    if (!arm_control_move_to(200.0f, 0.0f, 350.0f))
    {
        ESP_LOGE(TAG, "Lift point unreachable! Releasing gripper.");
        arm_control_set_gripper(0.0f);
        return;
    }

    /* Step 4: 旋转底座 180° (安全锁自动检查) */
    ESP_LOGI(TAG, "Step4: Rotate base to 180°");
    arm_control_set_base_safe(180.0f);

    /* Step 5: 直接舵机角放置 (Base=180°, 其余同Step1, yaw=0) */
    ESP_LOGI(TAG, "Step5: Direct servo place (yaw=0, gripper level)");
    servo_set_angle(SERVO_ID_JOINT1,  0.0f);
    servo_set_angle(SERVO_ID_JOINT2,  0.0f);
    servo_set_angle(SERVO_ID_JOINT3,  60.0f);
    wait_all_servos_idle();
    /* 更新几何状态 */
    s_geom_target.theta0 = 180.0f;
    s_geom_target.theta1 = 0.0f;
    s_geom_target.theta2 = -56.0f;
    s_geom_target.theta3 = 56.0f;

    /* Step 6: 释放 */
    ESP_LOGI(TAG, "Step6: Release gripper");
    arm_control_set_gripper(0.0f);

    /* Step 7: 先抬臂再回 HOME (底座旋转安全) */
    ESP_LOGI(TAG, "Step7: Lift then return HOME");
    arm_control_move_to(150.0f, 0.0f, 350.0f); /* 抬到安全高度 */
    arm_control_init();

    ESP_LOGI(TAG, "=== IK pick-and-place done ===");
}