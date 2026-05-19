## **一、servo_cfg.h上电后必须实测才能填最终值。下面是两个校准的详细步骤：**

------

## 校准一：ZP15S 脉宽范围确认

### 为什么要校准？

ZP15S 的真实机械行程跟手册标称值一定有误差。直接用 `500~2500μs` 如果超出实际行程，夹爪会堵转 → 电流飙升 → 舵机发烫烧毁。

### 详细步骤

**前期准备：**

- 夹爪单独上电（5V 独立供电，先不接底盘）
- 夹爪上不装任何负载

**第一步：写一个临时测试程序**

```
C
// test_servo_sweep.c  —— 校准完就删

#include "servo_cfg.h"

#include "servo_init.c"



// 手动发送指定脉宽 (μs)

static void test_pulse(gpio_num_t gpio, uint32_t pulse_us)

{

    uint32_t duty = (uint32_t)((uint64_t)pulse_us 

                     * (1UL << SERVO_PWM_BITS) / SERVO_PERIOD_US);

    ledc_set_duty(SERVO_LEDC_SPEED, SERVO_CH_GRIPPER, duty);

    ledc_update_duty(SERVO_LEDC_SPEED, SERVO_CH_GRIPPER);

}



void app_main(void)

{

    hal_servo_init();  // 先初始化 LEDC



    // 从 800μs 开始，每次加 50μs，直到夹爪动到极限

    for (uint32_t p = 800; p <= 2000; p += 50) {

        test_pulse(SERVO_GPIO_GRIPPER, p);

        printf("pulse = %lu μs\n", p);

        vTaskDelay(pdMS_TO_TICKS(1000));  // 每档等 1 秒观察

    }

}
```

**第二步：观察并记录**

| 观察现象                     | 记录值            | 说明       |
| ---------------------------- | ----------------- | ---------- |
| 夹爪**刚好停止再开口**的脉宽 | `pulse_open_min`  | 再小会堵转 |
| 夹爪**完全闭合**的脉宽       | `pulse_close_max` | 再大会堵转 |
| 中间任何位置听到持续"嗡"声   | 标记该区间避用    | 堵转区间   |

**第三步：回填到 servo_cfg.h**

```
C
// 用实测值替换默认值

#define SERVO_PULSE_OPEN_US   850   // 你实测的 pulse_open_min

#define SERVO_PULSE_CLOSE_US  1600  // 你实测的 pulse_close_max
```

施加负载后**再次校准一次**——负载会让堵转点偏移。

------

## 校准二：SERVO_HOME 安全姿态

### 为什么要校准？

上电瞬间舵机不受控乱甩，`SERVO_HOME_*` 决定初始化后机械臂停在哪个位置。填错了要么撞到自己，要么甩出去。

### 详细步骤

**第一步：全部舵机断电，手动摆到收拢位**

- 基座朝正前方
- 三个关节尽量并拢（上臂贴躯干）
- 夹爪全开

**第二步：逐个量角度**

| 关节               | 量角器/目测  | 记录值     |
| ------------------ | ------------ | ---------- |
| SERVO_HOME_BASE    | 基座旋转角度 | 90°        |
| SERVO_HOME_JOINT1  | 肩关节角度   | 45°        |
| SERVO_HOME_JOINT2  | 肘关节角度   | 135°       |
| SERVO_HOME_JOINT3  | 腕关节角度   | 90°        |
| SERVO_HOME_ROTATE  | 夹爪旋转角度 | 90°        |
| SERVO_HOME_GRIPPER | 夹爪开口角度 | 0°（全开） |

**第三步：回填到 servo_cfg.h**

```
C
#define SERVO_HOME_BASE       90.0f   // 你实测值

#define SERVO_HOME_JOINT1     45.0f

#define SERVO_HOME_JOINT2    135.0f

#define SERVO_HOME_JOINT3     90.0f

#define SERVO_HOME_ROTATE     90.0f

#define SERVO_HOME_GRIPPER     0.0f
```

**第四步：上电验证**

程序里 `app_main()` 初始化后立刻写入 HOME 角度，观察机械臂是否平稳停在预期姿态。



**补充说明**

| 配置项                     | 你需要做的                                     |
| -------------------------- | ---------------------------------------------- |
| `SERVO_EASING_TICK_MS`     | 一般不需要改                                   |
| `SERVO_EASING_DURATION_MS` | 上电后实际感受速度，觉得太慢就减小、太快就加大 |
| 各舵机独立时长             | 组装完后逐个微调，夹爪动作应该比关节快         |

上电实测后如果觉得某个舵机速度不合适，只改这一组数字，其他文件不动。

## **二、servo_init.c**

| 项                   | 说明                                                         |
| -------------------- | ------------------------------------------------------------ |
| 调用方式             | `esp_err_t ret = servo_init();` 检查返回值                   |
| HOME 写入时机        | 在 `ledc_channel_config()` 的 `.duty` 字段里直接写，**不额外调用 ledc_set_duty**，减少一次硬件操作 |
| `angle_to_duty` 删除 | 等 `servo_util.c` 搬好后，把 `static` 版本删掉，改为 `extern` 声明 |
| ESP32-P4 注意        | 只用 `LEDC_LOW_SPEED_MODE`，`LEDC_HIGH_SPEED_MODE` 不存在    |