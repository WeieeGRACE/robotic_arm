#!/usr/bin/env python3
"""
机械臂工作空间扫描器
- 使用与 kinematics.c 完全相同的余弦定理 IK 算法
- 扫描 XZ 平面 (Y=0)，输出可达区域边界
- 用于: (1) 验证IK正确性 (2) 确定抓取点合理范围 (3) 辅助手眼标定选点
"""

import math
import sys

# ============================================================
# 机械参数 (与 kinematics.h 完全一致)
# ============================================================
ARM_D1  = 141.0   # J1 转轴离地高度 (mm) — 实测
ARM_L1  = 104.0   # 上臂长
ARM_L2  = 97.0    # 前臂长
ARM_L3  = 130.0   # 末端长 — 实测

# ============================================================
# 关节限位 (与 solve_j2_j3_for_j1 完全一致)
# ============================================================
J1_MIN, J1_MAX = 0.0, 90.0       # θ1 几何角 (°)
J2_MIN, J2_MAX = -54.0, 93.0     # θ2 实测[-56,95] 留2°余量
J3_MIN, J3_MAX = -140.0, 150.0   # θ3 实测[-146,156] 留6°余量

# 枚举步长
J1_STEP = 0.5                     # 与 ik_solve_full 一致

# ============================================================
# 余弦定理 IK (C 代码的逐行翻译)
# ============================================================

def solve_j2_j3(j1_deg, r, vz):
    """
    给定 θ1，用余弦定理求解 θ2, θ3。
    返回: [(j2_deg, j3_deg, yaw_deg, candidate_label), ...]
          空列表 = 该 J1 无有效解
    """
    j1 = math.radians(j1_deg)

    # J1 到目标点的向量
    rx = r - ARM_L1 * math.cos(j1)
    rz = vz - ARM_L1 * math.sin(j1)
    d2 = rx * rx + rz * rz
    d = math.sqrt(d2)

    # 可达性检查: |L2 - L3| ≤ d ≤ L2 + L3
    l_min = abs(ARM_L2 - ARM_L3)
    l_max = ARM_L2 + ARM_L3
    if d < l_min - 0.5 or d > l_max + 0.5:
        return []

    # 余弦定理: cos(α) = (L2² + d² - L3²) / (2·L2·d)
    cos_alpha = (ARM_L2**2 + d2 - ARM_L3**2) / (2.0 * ARM_L2 * d)
    cos_alpha = max(-1.0, min(1.0, cos_alpha))
    alpha = math.acos(cos_alpha)

    # 余弦定理: cos(θ3) = (d² - L2² - L3²) / (2·L2·L3)
    cos_beta = (d2 - ARM_L2**2 - ARM_L3**2) / (2.0 * ARM_L2 * ARM_L3)
    cos_beta = max(-1.0, min(1.0, cos_beta))
    beta = math.acos(cos_beta)  # β = π - |θ3|

    base_angle = math.atan2(rz, rx)

    # 候选 A (肘向下): θ2 = base - θ1 - α,  θ3 = +β
    # 候选 B (肘向上): θ2 = base - θ1 + α,  θ3 = -β
    candidates = [
        (base_angle - j1 - alpha,  beta,   "elbow_down"),
        (base_angle - j1 + alpha, -beta,   "elbow_up"),
    ]

    results = []
    for j2_rad, j3_rad, label in candidates:
        j2_deg = math.degrees(j2_rad)
        j3_deg = math.degrees(j3_rad)

        # 关节限位
        if j2_deg < J2_MIN or j2_deg > J2_MAX:
            continue
        if j3_deg < J3_MIN or j3_deg > J3_MAX:
            continue

        yaw = math.degrees(j1 + j2_rad + j3_rad)
        results.append((j2_deg, j3_deg, yaw, label))

    return results


def ik_solve(r, vz, yaw_target=0.0):
    """
    完整 IK: 枚举 θ1，选择 yaw 最接近目标的解。
    返回: (j1, j2, j3, actual_yaw, label) 或 None
    """
    best = None
    best_yaw_err = float('inf')

    j1 = J1_MIN
    while j1 <= J1_MAX + 1e-9:
        candidates = solve_j2_j3(j1, r, vz)
        for j2, j3, yaw, label in candidates:
            yaw_err = abs(yaw - yaw_target)
            if yaw_err < best_yaw_err - 1e-6:
                best_yaw_err = yaw_err
                best = (j1, j2, j3, yaw, label)
        j1 += J1_STEP

    return best


# ============================================================
# 工作空间扫描
# ============================================================

def scan_workspace(x_range, z_range, step=5.0, yaw_target=0.0):
    """
    扫描 XZ 平面 (Y=0)。
    返回: [(x, z, j1, j2, j3, yaw), ...] — 可达点的 IK 解
    """
    reachable = []
    unreachable = 0
    total = 0

    x_vals = [x_range[0] + i * step for i in range(int((x_range[1] - x_range[0]) / step) + 1)]
    z_vals = [z_range[0] + i * step for i in range(int((z_range[1] - z_range[0]) / step) + 1)]

    for z in z_vals:
        for x in x_vals:
            total += 1
            r = x  # Y=0 时 r = |x|
            vz = z - ARM_D1
            result = ik_solve(r, vz, yaw_target)
            if result:
                j1, j2, j3, yaw, label = result
                reachable.append((x, z, j1, j2, j3, yaw))
            else:
                unreachable += 1

    return reachable, unreachable, total


def print_boundary(reachable):
    """打印每个 Z 高度下 X 的边界"""
    by_z = {}
    for x, z, j1, j2, j3, yaw in reachable:
        by_z.setdefault(z, []).append(x)

    print(f"\n{'='*70}")
    print(f"  工作空间边界扫描 (Y=0, yaw 软约束)")
    print(f"{'='*70}")
    print(f"  {'Z(mm)':>7s}  {'X_min':>8s}  {'X_max':>8s}  {'范围':>8s}  {'典型 θ1':>8s}  {'典型 θ2':>8s}  {'典型 θ3':>8s}  {'实际yaw':>8s}")
    print(f"  {'-'*7}  {'-'*8}  {'-'*8}  {'-'*8}  {'-'*8}  {'-'*8}  {'-'*8}  {'-'*8}")

    for z in sorted(by_z.keys(), reverse=True):
        xs = sorted(by_z[z])
        x_min, x_max = xs[0], xs[-1]
        # 找该高度下中点的 IK 解
        mid_x = (x_min + x_max) / 2
        r = mid_x
        vz_calc = z - ARM_D1
        sol = ik_solve(r, vz_calc)
        if sol:
            j1, j2, j3, yaw, _ = sol
            print(f"  {z:7.0f}  {x_min:8.0f}  {x_max:8.0f}  {x_max-x_min:8.0f}  {j1:8.1f}  {j2:8.1f}  {j3:8.1f}  {yaw:8.1f}")
        else:
            print(f"  {z:7.0f}  {x_min:8.0f}  {x_max:8.0f}  {x_max-x_min:8.0f}  {'-':>8s}  {'-':>8s}  {'-':>8s}  {'-':>8s}")


def test_specific_point(x, y, z, yaw_target=0.0):
    """测试特定点，打印详细 IK 信息"""
    r = math.sqrt(x*x + y*y)
    vz = z - ARM_D1
    result = ik_solve(r, vz, yaw_target)
    if result:
        j1, j2, j3, yaw, label = result
        # 计算底座角
        theta0 = abs(math.degrees(math.atan2(y, x)))
        # FK 验证
        t0, t1, t2, t3 = map(math.radians, [theta0, j1, j2, j3])
        fk_x = (ARM_L3 * math.cos(t1+t2+t3) + ARM_L2 * math.cos(t1+t2) + ARM_L1 * math.cos(t1)) * math.cos(t0)
        fk_y = (ARM_L3 * math.cos(t1+t2+t3) + ARM_L2 * math.cos(t1+t2) + ARM_L1 * math.cos(t1)) * math.sin(t0)
        fk_z = ARM_D1 + ARM_L3 * math.sin(t1+t2+t3) + ARM_L2 * math.sin(t1+t2) + ARM_L1 * math.sin(t1)

        print(f"\n点 ({x}, {y}, {z}) IK 结果:")
        print(f"  θ0={theta0:.2f}°  θ1={j1:.2f}°  θ2={j2:.2f}°  θ3={j3:.2f}°")
        print(f"  实际 yaw={yaw:.2f}° (目标={yaw_target}°) 臂型={label}")
        print(f"  FK 验证: ({fk_x:.1f}, {fk_y:.1f}, {fk_z:.1f}) — Δ=({fk_x-x:.2f}, {fk_y-y:.2f}, {fk_z-z:.2f}) mm")
        return True
    else:
        print(f"\n点 ({x}, {y}, {z}): IK_UNREACHABLE")
        return False


# ============================================================
# main
# ============================================================

if __name__ == "__main__":
    print("机械臂 IK 工作空间扫描器")
    print(f"连杆: L1={ARM_L1} L2={ARM_L2} L3={ARM_L3} D1={ARM_D1}")
    print(f"J2 限位: [{J2_MIN}°, {J2_MAX}°]")
    print(f"J3 限位: [{J3_MIN}°, {J3_MAX}°]")
    print(f"J1 枚举步长: {J1_STEP}°")

    # --- 1. 测试当前失败点 ---
    print("\n" + "="*70)
    print("  关键点测试")
    print("="*70)

    test_specific_point(200, 0, 80)   # pick_and_place_low 的抓取点
    test_specific_point(0, 200, 80)   # pick_and_place_low 的放置点
    test_specific_point(150, 0, 100)  # 中间试探点
    test_specific_point(100, 0, 60)   # 更低点
    test_specific_point(250, 0, 150)  # 更高更远点

    # --- 2. 扫描工作空间 ---
    print("\n" + "="*70)
    print("  工作空间扫描 (这可能需要几秒)...")
    print("="*70)

    reachable, unreachable, total = scan_workspace(
        x_range=(0, 350),   # X: 0~350mm
        z_range=(0, 400),   # Z: 0~400mm (地面到云台上方)
        step=10.0,           # 10mm 步长快速扫描
    )
    print(f"\n共扫描 {total} 个点, 可达 {len(reachable)}, 不可达 {unreachable}")

    print_boundary(reachable)

    # --- 3. 打印边界数据供后续参考 ---
    print(f"\n{'='*70}")
    print(f"  可达区域摘要")
    print(f"{'='*70}")
    if reachable:
        xs = [p[0] for p in reachable]
        zs = [p[1] for p in reachable]
        print(f"  X 范围: [{min(xs):.0f}, {max(xs):.0f}] mm")
        print(f"  Z 范围: [{min(zs):.0f}, {max(zs):.0f}] mm")
        print(f"  总计 {len(reachable)} 个可达网格点 (10mm 分辨率)")
