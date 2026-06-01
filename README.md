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

# 舵机驱动模块文件功能详解

------

## 一、module 整体架构

```
Text
用户调用 (control_task)

        │

        ▼

   servo_set.c          ← 控制入口: 设置目标角度

        │

        ├──servo_util.c  ← 纯计算: 角度转占空比、缓动查表

        │

        └──servo_init.c  ← 硬件: LEDC 定时器 + 通道初始化

             │

             ▼

         LEDC 硬件       → 舵机转动
```

------

## 二、各文件职责表格

| 文件               | 一句话职责 | 包含什么                            | 不该包含什么         |
| ------------------ | ---------- | ----------------------------------- | -------------------- |
| **servo_cfg.h**    | 配置宪法   | 宏定义、枚举、查表 TBL 宏           | 任何函数声明或实现   |
| **servo_init.c**   | 上电初始化 | LEDC 硬件配置、HOME 位置写入        | 运行时逻辑、缓动控制 |
| **servo_util.c/h** | 纯计算工具 | 角度↔占空比换算、缓动公式、查表函数 | 硬件操作、GPIO、LEDC |
| **servo_set.c/h**  | 运行时控制 | esp_timer、缓动状态机、目标角度管理 | 硬件初始化、底层计算 |

------

## 三、逐文件详解

### 1. `servo_cfg.h` — 配置宪法

**作用**：系统中所有舵机相关的"设定值"都集中在这里，改任何一个参数（引脚、脉宽、缓动时间）都只改这一个文件，其他文件自动生效。

**包含内容：**

```
C
SERVO_COUNT = 6                          // 舵机总数

ServoID_t  枚举                          // 舵机编号 (0~5)

SERVO_GPIO_xxx / SERVO_CH_xxx           // 硬件映射

SERVO_LEDC_SPEED / TIMER / FREQ / BITS  // LEDC 参数

SERVO_PULSE_MIN/MAX_xxx                 // 各舵机独立脉宽范围

SERVO_DUTY_MIN/MAX_xxx                  // 各舵机占空比边界

SERVO_HOME_xxx                          // 上电安全姿态

SERVO_EASING_xxx_MS                     // 各舵机缓动时长

SERVO_xxx_TBL                           // 查表 TBL 宏
```

**设计原则：**

- **零函数**：只有宏定义和枚举，编译后不产生任何代码段
- **零 include 依赖**：除标准类型外不依赖其他 .h
- **单一修改点**：参数调整只改这里，其他文件自动同步

**你需要上电后修改什么：**

- `SERVO_PULSE_MIN/MAX_xxx`：实测脉宽后填入真实值
- `SERVO_HOME_xxx`：实测安全姿态后填入真实角度

------

### 2. `servo_init.c` — 硬件初始化

**作用**：系统上电后只运行一次，完成 LEDC 硬件配置并写入 HOME 安全位置。

**包含内容：**

```
C
servo_init() 函数

  ├── ledc_timer_config()    // 配置 Timer: 50Hz, 12-bit

  ├── ledc_channel_config()  // 配置 6 路 Channel，每路绑定各自 GPIO

  └── servo_angle_to_duty()  // 调用 servo_util 的函数写入 HOME 占空比
```

**初始化流程：**

```
Text
servo_init() 被调用

        │

        ├── 1. LEDC Timer 配置

        │      - 频率: 50Hz

        │      - 分辨率: 12-bit (0~4095)

        │      - 速度模式: LOW_SPEED_MODE (ESP32-P4 唯一选项)

        │

        ├── 2. 6 路 Channel 配置

        │      - 每路绑定各自 GPIO 和 LEDC 通道

        │      - 通过 s_servo_chs[] 和 s_servo_gpios[] 查表

        │

        └── 3. 立即写入 HOME 位置

               - 调用 servo_angle_to_duty() 计算占空比

               - 通过 ledc_channel_config() 的 .duty 字段写入

               - 舵机瞬间到达 HOME，不经过缓动
```

**设计原则：**- **无状态**：函数执行完后不维护任何变量，底层硬件由 LEDC 驱动接管

- **无错误恢复**：返回 `esp_err_t`，调用者检查返回值决定后续
- **可重复调用**：理论上可以多次调用，但会导致舵机瞬间跳变到 HOME

**文件中保留的两个数组：**

- `s_servo_chs[6]`：LEDC 通道映射，仅本文件使用
- `s_servo_gpios[6]`：GPIO 映射，仅本文件使用

这两个数组在 `servo_set.c` 中有 **另一份独立副本** `s_channel_tbl[6]`，两处独立维护，避免相互依赖。

------

### 3. `servo_util.c / .h` — 纯计算工具

**作用**：提供所有"脑力活"的计算函数，**零硬件操作**，方便单元测试和复用。

**包含 6 个函数：**

| 函数                          | 输入                  | 输出     | 功能                                |
| ----------------------------- | --------------------- | -------- | ----------------------------------- |
| `servo_angle_to_duty()`       | ServoID_t, float      | uint32_t | 角度 → 占空比                       |
| `servo_duty_to_angle()`       | ServoID_t, uint32_t   | float    | 占空比 → 角度                       |
| `servo_easing_quintic()`      | float tau             | float    | 五次多项式 s(τ) = 10τ³ - 15τ⁴ + 6τ⁵ |
| `servo_easing_lerp()`         | float tau, from, to   | float    | 带缓动的线性插值                    |
| `servo_get_easing_duration()` | ServoID_t             | uint32_t | 查表返回该舵机动时长 (ms)           |
| `servo_get_pulse_range()`     | ServoID_t, *min, *max | void     | 查表返回该舵机脉宽范围 (μs)         |

**核心函数详解：**

#### `servo_angle_to_duty(id, deg)`

```
Text
输入: deg = 90.0f

       ↓

  查表: s_duty_min[id] = 102

        s_duty_max[id] = 512

        span = 410

       ↓

  计算: duty = 102 + (90/180) × 410 + 0.5f = 307

       ↓

输出: 307 (12-bit duty)
```

- 自动钳位：`deg < 0` → 0°，`deg > 180` → 180°
- 查表机制：每舵机独立 MIN/MAX，ZP15S 和 TBS2701 互不干扰

#### `servo_easing_quintic(tau)`

```
Text
输入: tau = 0.5  (缓动过程一半)

       ↓

  计算: τ² = 0.25, τ³ = 0.125, τ⁴ = 0.0625, τ⁵ = 0.03125

        s = 10×0.125 - 15×0.0625 + 6×0.03125 = 0.5

       ↓

输出: 0.5  (恰好在行程中点)
```

**数学性质：**

- s(0) = 0, s(1) = 1 （两端到位）
- s'(0) = s'(1) = 0 （起止速度为零，无冲击）
- s''(0) = s''(1) = 0 （起止加速度为零，无抖动）

#### `servo_easing_lerp(tau, from, to)`

```
Text
输入: tau=0.5, from=0°, to=180°

       ↓

  factor = servo_easing_quintic(0.5) = 0.5

  result = 0 + 0.5 × (180 - 0) = 90°

       ↓

输出: 90°
```

**设计原则：**- **无静态变量**：所有函数是纯函数，相同输入永远相同输出

- **无硬件依赖**：只是数学计算，可以在 PC 上单元测试
- **无副作用**：不修改全局状态

------

### 4. `servo_set.c / .h` — 运行时控制

**作用**：管理 6 路舵机的缓动状态，驱动 esp_timer 周期更新，是用户（control_task）的直接调用入口。

**内部数据结构：**

```
C
ServoEasingState_t s_state[6] = {

    [0] = {

        .start_angle   = 90.0f,   // 本段缓动起始

        .target_angle  = 45.0f,   // 目标角度

        .current_angle = 90.0f,   // 当前实际位置 (每帧更新)

        .elapsed_ticks = 0,       // 已经过 tick

        .total_ticks   = 50,      // 总 tick 需求

        .is_active     = true,    // 是否正在缓动

    },

    [1] = { ... },

    ...

};
```

**包含 5 个公开函数：**

| 函数                 | 调用者         | 功能                            |
| -------------------- | -------------- | ------------------------------- |
| `servo_set_init()`   | `app_main()`   | 初始化状态数组 + 创建 esp_timer |
| `servo_set_angle()`  | `control_task` | 设置单个舵机目标，启动缓动      |
| `servo_set_joints()` | `control_task` | 设置全部 6 轴目标 (IK 结果喂入) |
| `servo_stop()`       | `control_task` | 停止单个舵机                    |
| `servo_stop_all()`   | `control_task` | 停止全部 (刹车/接管)            |

**核心流程：**

```
Text
control_task 调用 servo_set_angle(SERVO_ID_BASE, 45.0f)

           │

           ├── 1. 计算差值 delta = |45 - 90| = 45°

           │      若 delta < 0.1° → 忽略

           │

           ├── 2. 记录起始: start = current = 90°

           │

           ├── 3. 计算所需 tick:

           │      duration = 1200ms (BASE)

           │      ratio = 45/180 = 0.25

           │      ticks = 1200 × 0.25 / 20 = 15 tick

           │

           └── 4. is_active = true, elapsed = 0



esp_timer 回调每 20ms 触发:

           │

           ├── 查找 is_active=true 的舵机

           │

           ├── tau = elapsed / total = 0/15, 1/15, 2/15 ...

           │

           ├── current = servo_easing_lerp(tau, 90, 45)

           │      = 90 + quintic(tau) × (45 - 90)

           │

           ├── servo_write_hardware():

           │      duty = servo_angle_to_duty(BASE, current)

           │      ledc_set_duty(duty)

           │      ledc_update_duty()

           │

           └── elapsed++，直到 elapsed >= total → is_active=false
```

**设计原则：**- **状态机驱动**：每舵机独立状态，互不干扰

- **覆盖安全**：缓动中再次调用 → 从 current_angle 出发，无跳变
- **线程安全**：32位对齐数据在 RISC-V 双核上原子读写，无需锁

**文件中的两个内部工具函数：**

```
C
static inline ledc_channel_t get_channel(ServoID_t id)

    → 查表 s_channel_tbl[] 返回 LEDC 通道号



static inline void servo_write_hardware(ServoID_t id, float deg)

    → 调用 servo_angle_to_duty() 计算占空比

    → 调用 ledc_set_duty() + ledc_update_duty() 写硬件
```

------

## 四、文件依赖关系图

```
Text
┌─────────────────────────────────────────────────────┐

│                    servo_cfg.h                       │

│  (配置宏, 无依赖)                                    │

└──────────┬────────────────────┬──────────────────────┘

           │                    │

           ▼                    ▼

    ┌──────────────┐     ┌──────────────┐

    │ servo_util.h │     │ servo_util.c │

    │  (函数声明)  │     │  (纯计算)    │

    └──────┬───────┘     └──────┬───────┘

           │                     │

           ▼                     ▼

    ┌──────────────┐     ┌──────────────┐

    │ servo_set.h  │     │ servo_set.c  │

    │  (控制接口)  │     │  (缓动逻辑) │

    └──────┬───────┘     └──────┬───────┘

           │                     │

           ▼                     ▼

    ┌──────────────┐     ┌──────────────┐

    │ servo_init.c │     │   LEDC HAL   │

    │  (硬件初始化)│────→│  (ESP-IDF)   │

    └──────────────┘     └──────────────┘
```

------

## 五、用户视角：使用场景

**场景 1：修改某个舵机的脉宽范围**

- 只改 `servo_cfg.h` 的 `SERVO_PULSE_MIN/MAX_xxx`
- 重新编译，其他文件自动同步

**场景 2：修改缓动时长**

- 只改 `servo_cfg.h` 的 `SERVO_EASING_xxx_MS`
- 重新编译生效

**场景 3：查看当前舵机角度**

```
C
float deg = servo_duty_to_angle(SERVO_ID_BASE, current_duty);

printf("Base angle: %.1f°\n", deg);
```

**场景 4：紧急停止**

```
C
servo_stop_all();  // 停止所有缓动，保留当前位置
```

**场景 5：手动微调夹爪（不缓动，瞬间到达）**

- 目前没有 `servo_set_angle_immediate()` 函数
- 可以通过 `servo_set_angle()` + `servo_stop()` 组合实现
- 或者直接调用 `servo_write_hardware()` (但需要暴露这个内部函数)

------

这样拆分的好处是：**改参数只动 cfg，改计算只动 util，改控制逻辑只动 set，互不干扰。**

**调用链路示例（control_task 里）：**

**/* 1. IK 解算完成后得到 angles[5] + 夹爪百分比 */**

**float angles[6] = {ik.joint_deg[0], ik.joint_deg[1], ik.joint_deg[2],**

​                   0, 0, gripper_pct / 100.0f * 180.0f};

**/* 2. 一次调用，6 轴同时缓动 */**

**servo_set_joints(angles);**

**/* 3. 停止所有舵机 (比如底盘急停) */**

**servo_stop_all();**