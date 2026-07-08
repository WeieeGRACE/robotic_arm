# 视觉引导机械臂抓取系统

基于 ESP32-P4 的履带底盘 + 4-DOF 机械臂协同抓取系统。

## 硬件

- **主控**: ESP32-P4
- **机械臂**: 4-DOF (TBS2701 ×5 + ZP15S 夹爪)
- **底盘**: 履带差速驱动 (RZ7899 ×2, MCPWM)
- **视觉**: 上位机 PC 处理, 通过蓝牙 UART (115200 8N1) 回传 3D 坐标

## 软件架构

```
PC 视觉上位机
    │  BT UART (9字节协议, 100Hz)
    ▼
vision_uart     UART DMA 接收 + 状态机解码
    │
vision_coordinator  状态机调度
    ├─ 目标 ≤310mm 且在 IK 范围内 → 机械臂抓取
    └─ 目标超出范围 → 底盘导航靠近 → 停车 → 抓取
```

## 目录结构

```
main/
├── User/
│   ├── servo_cfg.h          # 舵机配置
│   ├── servo_init.c/h       # LEDC 硬件初始化
│   ├── servo_set.c/h        # 运行时缓动控制
│   ├── servo_util.c/h       # 角度↔占空比计算
│   ├── kinematics.c/h       # 正/逆运动学
│   ├── arm_control.c/h      # 机械臂高层控制
│   ├── collision.c/h        # 碰撞检测
│   ├── trajectory.c/h       # 关节轨迹插补
│   ├── cartesian.c/h        # 笛卡尔空间插补
│   ├── calib_nvs.c/h        # NVS 校准存储
│   ├── motor.c/h            # 底盘电机 MCPWM 驱动
│   ├── chassis.c/h          # 底盘差速控制
│   ├── navigation.c/h       # 视觉伺服导航
│   ├── vision_uart.c/h      # 上位机通信 (DMA + 协议解码)
│   ├── vision_coordinator.c/h # 视觉协调器 (状态机)
│   └── main.c               # 入口
├── CMakeLists.txt
└── partitions.csv
```

## 通信协议

9 字节二进制帧, 100Hz:

```
[0xAA] [0xBB] [X_LE] [X_BE] [Y_LE] [Y_BE] [Z_LE] [Z_BE] [CS]
```

- X/Y/Z: int16 LE, 单位 mm, 相机坐标系
- CS: XOR(byte[0]..byte[7])
- (-1, -1, -1): 无目标

## 底盘参数

| 参数 | 值 |
|------|-----|
| PWM 频率 | 1 kHz |
| 电压上限 | 11.5V (96% 占空比) |
| 加速 (<4V) | 0.036/帧 |
| 加速 (>4V) | 0.009/帧 |
| 减速 | 0.006/帧 |
| 刹车 | 急减速 + 500ms 短接制动 |

## 编译

```bash
idf.py build flash monitor
```

ESP-IDF v5.4, 目标芯片 `esp32p4`.
