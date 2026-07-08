/**
 * @file  arm_control.c
 * @brief 机械臂高层控制实现 (θ3 动态 IK 版本)
 */

#include "arm_control.h"
#include "servo_cfg.h"
#include "servo_set.h"
#include "kinematics.h"
#include "trajectory.h"
#include "collision.h"
#include "cartesian.h"
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
 *  初始化 → 安全位 (正前方, 末端贴近地面)
 *-----------------------------------------------------*/
void arm_control_init(void)
{
    ESP_LOGI(TAG, "Arm control init → SAFE position");

    /* 尝试通过 IK 走到安全位 (正前方, 离地 40mm) */
    ArmTipState_t safe_target = {
        .x   = 250.0f,
        .y   = 0.0f,
        .z   = 40.0f,
        .yaw = 0.0f,
    };

    JointAngles_t geom;
    IKResult_t ik_res = kinematics_inverse(&safe_target, &geom);

    if (ik_res == IK_OK && kinematics_check_limits(&geom)) {
        /* IK 成功 → 走安全位 */
        ServoAngles_t servo;
        kinematics_geom_to_servo(&geom, &servo);

        servo_set_angle(SERVO_ID_BASE,    servo.servo0);
        servo_set_angle(SERVO_ID_JOINT1,  servo.servo1);
        servo_set_angle(SERVO_ID_JOINT2,  servo.servo2);
        servo_set_angle(SERVO_ID_JOINT3,  servo.servo3);
        servo_set_angle(SERVO_ID_ROTATE,  servo.servo4);
        servo_set_angle(SERVO_ID_GRIPPER, 0.0f);

        wait_all_servos_idle();

        s_geom_target = geom;
        s_geom_target.theta4 = 0.0f;

        /* FK 验证 */
        ArmTipState_t tip;
        kinematics_forward(&s_geom_target, &tip);
        ESP_LOGI(TAG, "SAFE reached  tip=(%.1f,%.1f,%.1f)  "
                 "servo=(%.1f,%.1f,%.1f,%.1f)",
                 (double)tip.x, (double)tip.y, (double)tip.z,
                 (double)servo.servo0, (double)servo.servo1,
                 (double)servo.servo2, (double)servo.servo3);
    } else {
        /* IK 失败 → 回退用近似舵机角 */
        ESP_LOGW(TAG, "IK for (250,0,40) failed, using fallback joint pose");

        s_geom_target.theta0 = 0.0f;
        s_geom_target.theta1 = 10.0f;
        s_geom_target.theta2 = -30.0f;
        s_geom_target.theta3 = 20.0f;
        s_geom_target.theta4 = 0.0f;

        ServoAngles_t servo;
        kinematics_geom_to_servo(&s_geom_target, &servo);

        servo_set_angle(SERVO_ID_BASE,    servo.servo0);
        servo_set_angle(SERVO_ID_JOINT1,  servo.servo1);
        servo_set_angle(SERVO_ID_JOINT2,  servo.servo2);
        servo_set_angle(SERVO_ID_JOINT3,  servo.servo3);
        servo_set_angle(SERVO_ID_ROTATE,  servo.servo4);
        servo_set_angle(SERVO_ID_GRIPPER, 0.0f);

        wait_all_servos_idle();
        ESP_LOGI(TAG, "Fallback SAFE reached (servo: %.1f,%.1f,%.1f,%.1f)",
                 (double)servo.servo0, (double)servo.servo1,
                 (double)servo.servo2, (double)servo.servo3);
    }
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

    /* 碰撞预检 */
    CollisionResult_t cr = collision_check(&geom);
    if (cr != COLLISION_OK)
    {
        ESP_LOGE(TAG, "Collision detected: %s — move rejected!", collision_str(cr));
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
    float d1 = kinematics_get_d1();
    if (tip.z < (d1 - 10.0f))
    {
        ESP_LOGE(TAG, "BASE ROTATE BLOCKED: tip z=%.1f < D1=%.0f, "
                 "请先抬臂到水平面以上!", (double)tip.z, (double)d1);
        return;
    }

    /* 碰撞检测: 底座旋转是否会扫过履带 */
    CollisionResult_t cr = collision_check_rotate(
        s_geom_target.theta0, base_servo_deg, &s_geom_target);
    if (cr != COLLISION_OK)
    {
        ESP_LOGE(TAG, "BASE ROTATE BLOCKED: %s — 请抬臂后再旋转!",
                 collision_str(cr));
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

/*-----------------------------------------------------
 *  通用抓取-放置: 直接舵机角抓取+放置, IK抬臂过渡
 *-----------------------------------------------------*/
static void pick_and_place_at(float j1, float j2, float j3,
                              float base_place, const char *label)
{
    ESP_LOGI(TAG, "=== Pick&Place [%s] start ===", label);

    /* Step 1: 在 base=0 (正前安全区) 摆出抓取姿态 */
    ESP_LOGI(TAG, "[%s] Step1: Pose at base=0 (j1=%.0f j2=%.0f j3=%.0f)",
             label, (double)j1, (double)j2, (double)j3);
    servo_set_angle(SERVO_ID_JOINT1,  j1);
    servo_set_angle(SERVO_ID_JOINT2,  j2);
    servo_set_angle(SERVO_ID_JOINT3,  j3);
    servo_set_angle(SERVO_ID_GRIPPER, 0.0f);
    wait_all_servos_idle();
    s_geom_target.theta0 = 0.0f;
    s_geom_target.theta1 = j1;
    s_geom_target.theta2 = J2_OFFSET_DEG + J2_SCALE * j2;
    s_geom_target.theta3 = (j3 - J3_OFFSET_DEG) / J3_SCALE_DEG;

    /* Step 2: 夹取 */
    ESP_LOGI(TAG, "[%s] Step2: Grab", label);
    arm_control_set_gripper(100.0f);

    /* Step 3: IK 抬臂到安全高度 (避开履带) */
    ESP_LOGI(TAG, "[%s] Step3: Lift", label);
    arm_control_move_to(200.0f, 0.0f, 350.0f);

    /* Step 4: 旋转底座到放置角 (抬升后安全) */
    ESP_LOGI(TAG, "[%s] Step4: Rotate base to %.0f°", label, (double)base_place);
    arm_control_set_base_safe(base_place);

    /* Step 5: 摆出放置姿态 — 若碰撞则自动抬肩 */
    {
        float safe_j1 = j1;
        /* 逐步抬升 θ1 直到碰撞检测通过 */
        for (int tries = 0; tries < 10; tries++) {
            JointAngles_t test_geom = {
                .theta0 = base_place,
                .theta1 = safe_j1,
                .theta2 = J2_OFFSET_DEG + J2_SCALE * j2,
                .theta3 = (j3 - J3_OFFSET_DEG) / J3_SCALE_DEG,
            };
            if (collision_check(&test_geom) == COLLISION_OK) break;
            safe_j1 += 5.0f; /* 每次抬 5° */
            ESP_LOGW(TAG, "[%s] Place pose unsafe, lifting J1 to %.0f°",
                     label, (double)safe_j1);
        }
        ESP_LOGI(TAG, "[%s] Step5: Place pose (j1=%.0f j2=%.0f j3=%.0f)",
                 label, (double)safe_j1, (double)j2, (double)j3);
        servo_set_angle(SERVO_ID_JOINT1,  safe_j1);
        servo_set_angle(SERVO_ID_JOINT2,  j2);
        servo_set_angle(SERVO_ID_JOINT3,  j3);
        wait_all_servos_idle();
    }

    /* Step 6: 释放 */
    ESP_LOGI(TAG, "[%s] Step6: Release", label);
    arm_control_set_gripper(0.0f);

    ESP_LOGI(TAG, "=== Pick&Place [%s] done ===", label);
}

/*-----------------------------------------------------
 *  多点抓取测试 — 5 个不同高度/距离, 全部 yaw=0
 *-----------------------------------------------------*/
void arm_control_multi_pick_and_place(void)
{
    /* 点1: 低远 — (288, 0, 61) */
    pick_and_place_at(0.0f, 0.0f, 60.0f, 90.0f, "P1-low");

    arm_control_move_to(150.0f, 0.0f, 350.0f);

    pick_and_place_at(0.0f, 40.0f, 80.0f, 90.0f, "P2-midlow");
    arm_control_move_to(150.0f, 0.0f, 350.0f);

    pick_and_place_at(0.0f, 60.0f, 90.0f, 90.0f, "P3-mid");
    arm_control_move_to(150.0f, 0.0f, 350.0f);

    pick_and_place_at(0.0f, 100.0f, 110.0f, 90.0f, "P4-midhigh");
    arm_control_move_to(150.0f, 0.0f, 350.0f);

    pick_and_place_at(40.0f, 160.0f, 163.0f, 90.0f, "P5-high");

    /* 回 HOME */
    arm_control_move_to(150.0f, 0.0f, 350.0f);
    arm_control_init();

    /*=== 轨迹插补演示: 3段平滑抓→抬→放 ===*/
    ESP_LOGI(TAG, "=== Trajectory demo (3-segment smooth pick&place) ===");

    /* 先到抓取位 */
    servo_set_angle(SERVO_ID_BASE,    0.0f);   /* 正前安全区抓取 */
    servo_set_angle(SERVO_ID_JOINT1,  0.0f);
    servo_set_angle(SERVO_ID_JOINT2,  0.0f);
    servo_set_angle(SERVO_ID_JOINT3,  60.0f);
    servo_set_angle(SERVO_ID_GRIPPER, 0.0f);
    wait_all_servos_idle();
    arm_control_set_gripper(100.0f);

    /* 3段轨迹: 抬升 → 旋转到90° → 放下 */
    {
        TrajPoint_t traj[3] = {
            { .angles = {0.0f, 69.0f, 67.0f, 129.0f, 0.0f, 0.0f}, .duration_ms = 1500 },
            { .angles = {90.0f, 69.0f, 67.0f, 129.0f, 0.0f, 0.0f}, .duration_ms = 2000 },
            { .angles = {90.0f, 0.0f, 0.0f, 60.0f, 0.0f, 0.0f}, .duration_ms = 1500 },
        };
        traj_start(traj, 3);
        traj_wait_done();
    }

    arm_control_set_gripper(0.0f);
    ESP_LOGI(TAG, "=== Trajectory demo done ===");

    /*=== 笛卡尔直线演示: 末端从高到低走直线 ===*/
    ESP_LOGI(TAG, "=== Cartesian line demo ===");
    arm_control_move_to(200.0f, 0.0f, 350.0f);
    /* 从 (200,0,350) 直线走到 (300,0,120), 速度 80mm/s */
    cartesian_move_to(300.0f, 0.0f, 120.0f, 80.0f);
    /* 直线回到上方 */
    cartesian_move_to(200.0f, 0.0f, 350.0f, 80.0f);
    ESP_LOGI(TAG, "=== Cartesian line demo done ===");

    /* 回 HOME */
    arm_control_move_to(150.0f, 0.0f, 350.0f);
    arm_control_init();
    ESP_LOGI(TAG, "=== All tests done ===");
}