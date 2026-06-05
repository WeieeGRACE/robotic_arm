#!/usr/bin/env python3
"""
碰撞边界扫描 — 履带斜坡模型 + 全臂链路检测
输出: 安全抓取点 vs 碰撞点, 用于代码限位
"""

import math
from types import SimpleNamespace

# 机械参数 (与 kinematics.h 同步)
ARM_D1  = 141.0
ARM_L1  = 104.0
ARM_L2  = 97.0
ARM_L3  = 130.0
J2_OFFSET = -56.0
J2_SCALE  = 151.0 / 180.0
J3_OFFSET = 93.0
J3_SCALE  = -0.595

# ============================================================
# 履带斜坡模型 (collision.h 实测参数)
# ============================================================
TRACK_R_REAR  = 150.0   # 履带后点距中心 (mm)
TRACK_Z_REAR  = 125.0   # 履带后点高度 (最高)
TRACK_R_FRONT = 192.0   # 履带前点距中心 (mm)
TRACK_Z_FRONT = 72.0    # 履带前点高度 (最低)
TRACK_ANGLE_MIN = 30.0  # 履带角度范围 (deg)
TRACK_ANGLE_MAX = 150.0

TRACK_SLOPE = (TRACK_Z_FRONT - TRACK_Z_REAR) / (TRACK_R_FRONT - TRACK_R_REAR)
TRACK_INTERCEPT = TRACK_Z_REAR - TRACK_SLOPE * TRACK_R_REAR

def track_z_at_r(r):
    """履带表面高度"""
    return TRACK_SLOPE * r + TRACK_INTERCEPT

def in_track_zone(r, theta0_deg):
    """点 (r, theta0) 是否在履带水平范围内"""
    return (TRACK_R_REAR <= r <= TRACK_R_FRONT and
            TRACK_ANGLE_MIN <= theta0_deg <= TRACK_ANGLE_MAX)

def check_arm_collision(geom):
    """检测整条臂 (J2, J3, Tip) 是否撞履带"""
    t1 = math.radians(geom.theta1)
    t2 = math.radians(geom.theta2)
    t3 = math.radians(geom.theta3)
    
    r_j2 = ARM_L1 * math.cos(t1)
    z_j2 = ARM_D1 + ARM_L1 * math.sin(t1)
    r_j3 = r_j2 + ARM_L2 * math.cos(t1 + t2)
    z_j3 = z_j2 + ARM_L2 * math.sin(t1 + t2)
    r_tip = r_j3 + ARM_L3 * math.cos(t1 + t2 + t3)
    z_tip = z_j3 + ARM_L3 * math.sin(t1 + t2 + t3)
    
    pts = [("J2", r_j2, z_j2), ("J3", r_j3, z_j3), ("Tip", r_tip, z_tip)]
    
    for name, r, z in pts:
        if in_track_zone(r, geom.theta0):
            z_trk = track_z_at_r(r)
            if z < z_trk:
                return (True, name, r, z, z_trk)
    return (False, None, 0, 0, 0)

# ============================================================
# 扫描: 遍历 yaw=0 可达点, 对多个 base 角度检查碰撞
# ============================================================
print("="*70)
print("  碰撞边界扫描 — 履带斜坡模型")
print(f"  履带: r=[{TRACK_R_REAR},{TRACK_R_FRONT}] z=[{TRACK_Z_FRONT},{TRACK_Z_REAR}]")
print(f"  角度: [{TRACK_ANGLE_MIN}°,{TRACK_ANGLE_MAX}°]")
print("="*70)

# 收集所有 yaw=0 可达点
points = []
for t1_deg in range(0, 91, 2):
    for s2 in range(0, 181, 5):
        t2_deg = J2_OFFSET + J2_SCALE * s2
        t3_deg = -t1_deg - t2_deg  # yaw=0
        if t3_deg < -140 or t3_deg > 150: continue
        s3 = t3_deg * J3_SCALE + J3_OFFSET
        if s3 < 0 or s3 > 180: continue
        if t2_deg < -56 or t2_deg > 93: continue
        
        t1 = math.radians(t1_deg); t2 = math.radians(t2_deg); t3 = math.radians(t3_deg)
        r = ARM_L1*math.cos(t1)+ARM_L2*math.cos(t1+t2)+ARM_L3*math.cos(t1+t2+t3)
        z = ARM_D1+ARM_L1*math.sin(t1)+ARM_L2*math.sin(t1+t2)+ARM_L3*math.sin(t1+t2+t3)
        
        if z < 0 or z > 500: continue
        if r < 50 or r > 400: continue
        
        # 对 3 个典型底座角检查碰撞
        safe_at = []
        for base_deg in [0, 45, 90, 135, 180]:
            geom = SimpleNamespace(theta0=base_deg, theta1=t1_deg,
                                   theta2=t2_deg, theta3=t3_deg)
            hit, part, hr, hz, hzt = check_arm_collision(geom)
            if hit:
                safe_at.append((base_deg, False, part))
            else:
                safe_at.append((base_deg, True, None))
        
        points.append((r, z, t1_deg, s2, s3, safe_at))

# 按 Z 排序输出
points.sort(key=lambda p: p[1])

print(f"\n{'r':>6s} {'z':>6s} {'j1':>4s} {'j2':>4s} {'j3':>4s}  base=0  base=45 base=90 base=135 base=180")
print("-"*72)

safe_any = 0
safe_all = 0
for r, z, t1, s2, s3, safe_at in points:
    cols = []
    for base, ok, part in safe_at:
        if ok:
            cols.append("  OK  ")
        else:
            cols.append(f"{part:>4s}XX")
    
    all_ok = all(ok for ok,_,_ in safe_at)
    any_ok = any(ok for ok,_,_ in safe_at)
    
    if any_ok:
        safe_any += 1
        if all_ok: safe_all += 1
        
        # 只打印有碰撞的点
        if not all_ok:
            print(f"{r:6.0f} {z:6.0f} {t1:4.0f} {s2:4.0f} {s3:4.0f}  {cols[0]} {cols[1]} {cols[2]} {cols[3]} {cols[4]}")

print(f"\n总 yaw=0 点: {len(points)}")
print(f"至少一个角度安全: {safe_any}")
print(f"全角度安全: {safe_all}")
print(f"部分碰撞: {safe_any - safe_all}")

# ============================================================
# 安全边界: r-z 平面里 base=135° 的安全区域
# ============================================================
print(f"\n{'='*70}")
print(f"  base=135° 安全 Z 下限 (r vs min_safe_z)")
print(f"{'='*70}")

# 对每个 r, 找最低安全 z
r_bins = {}
for r, z, t1, s2, s3, safe_at in points:
    ok135 = safe_at[3][1]  # base=135
    if ok135:
        r_key = int(r / 10) * 10
        if r_key not in r_bins or z < r_bins[r_key]:
            r_bins[r_key] = z

for r_key in sorted(r_bins.keys()):
    print(f"  r={r_key:3d}mm → min_safe_z = {r_bins[r_key]:.0f}mm")
