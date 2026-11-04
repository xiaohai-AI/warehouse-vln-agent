#!/usr/bin/env python3
"""新视觉里程计（旋转补偿 + 联合解算）的**离线自检**。

为什么必须先做这一步：S5 定版的 ATE 0.234 m 卡在 0.214 m 线上方 9%，分链实测已经定位到
病灶 —— 里程计链 `odom→base` 自己偏 2.837 m，**航向误差仅 −0.13°、累计路径只差 +0.4%**，
也就是"距离对、位置错"：每帧约 **3 mm 的横向假位移**。

机理假设：`estimate_forward` 把 dx（径向反解）与 dy（行坐标）当成两件互不相关的事来算，
且假设帧间**纯平移**。机器人边走边转（全向车在横通道里换向、转弯腿）时，旋转在图像里
是**大范围水平偏移**，会被 dy 通道整个吞掉 → 假侧移。

这个脚本用**几何完全已知**的合成地面验证修法：
  1. 渲染器支持相机位姿 (x, y, yaw)，投影用精确式（含偏航旋转）；
  2. 同一批 KLT 特征、同一段图像，分别跑
       old = `estimate_forward`（不补偿旋转）
       new = `solve_motion`（已知 ψ → 先去掉旋转，再联合解 dx/dy，MAD 剔外点）
       new+1° = 故意把 ψ 给错 1°，看对 IMU 精度的敏感度；
  3. 判据：`new` 在所有工况下都必须 ≤ max(5 mm, 15%)；纯旋转工况必须 ≤ 5 mm。

六种工况都是这台车真的会遇到的：
  前进 / 侧移 / 对角（前进+侧移）/ 前进+转 2°/帧 / 侧移+转 2°/帧 / 纯转 3°/帧

用法：.venv/bin/python scripts/s4_vo_selftest.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from depot_vln.floorflow import band_mask, estimate_forward, solve_motion, track_points  # noqa: E402

W_IMG, H_IMG = 320, 240
H_CAM = 0.426
HFOV = 90.0
F = (W_IMG / 2.0) / np.tan(np.radians(HFOV) / 2.0)
CX, CY = W_IMG / 2.0, H_IMG / 2.0

# 合成地面：2 mm/px。横向放大到 ±3.5 m —— 转弯+侧移时画面底部会扫到更远的侧向区域，
# 纹理不够宽会在边界采到 0（黑边），KLT 会把这些边界当特征。
RES = 0.002
X0, X1 = 0.05, 10.0
Y0, Y1 = -3.5, 3.5
GW, GH = int((Y1 - Y0) / RES), int((X1 - X0) / RES)
rng = np.random.default_rng(20261101)
world = cv2.GaussianBlur(rng.normal(128, 45, (GH, GW)).astype(np.float32), (0, 0), 1.3)
for _ in range(11000):
    r = rng.uniform(1.0, 4.0)
    px, py = rng.uniform(0, GW), rng.uniform(0, GH)
    cv2.circle(world, (int(px), int(py)), int(r), float(rng.uniform(40, 230)), -1)
world = np.clip(world, 0, 255)


def render(cam_x: float, cam_y: float = 0.0, cam_yaw: float = 0.0) -> np.ndarray:
    """精确投影渲染：相机在 (cam_x, cam_y)，偏航 cam_yaw（左转为正）。

    像素 (u,v) → 相机系地面点：Xl = f·h/(v−cy)，Yl = (cx−u)·Xl/f（y 向左为正）
    再按偏航转到世界系：Xw = cam_x + Xl·cosψ − Yl·sinψ，Yw = cam_y + Xl·sinψ + Yl·cosψ
    """
    us, vs = np.meshgrid(np.arange(W_IMG, dtype=np.float32),
                         np.arange(H_IMG, dtype=np.float32))
    with np.errstate(divide="ignore", invalid="ignore"):
        Xl = F * H_CAM / (vs - CY)
        Yl = (CX - us) * Xl / F
        c, s = math.cos(cam_yaw), math.sin(cam_yaw)
        map_x = (cam_x + Xl * c - Yl * s - X0) / RES
        map_y = (cam_y + Xl * s + Yl * c - Y0) / RES
    map_x = np.nan_to_num(map_x, nan=-1, posinf=-1, neginf=-1).astype(np.float32)
    map_y = np.nan_to_num(map_y, nan=-1, posinf=-1, neginf=-1).astype(np.float32)
    img = cv2.remap(world, map_y, map_x, interpolation=cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    return img.astype(np.uint8)


def noisy(img: np.ndarray, sigma: float = 3.0) -> np.ndarray:
    return np.clip(img.astype(np.float32) + rng.normal(0, sigma, img.shape), 0, 255).astype(np.uint8)


CASES = [
    ("前进 3.5 cm",            0.035, 0.000, 0.0),
    ("侧移 3.5 cm",            0.000, 0.035, 0.0),
    ("对角 2.5/2.5 cm",        0.025, 0.025, 0.0),
    ("前进 3.5 + 转 2°/帧",    0.035, 0.000, 2.0),
    ("侧移 3.5 + 转 2°/帧",    0.000, 0.035, 2.0),
    ("纯转 3°/帧",             0.000, 0.000, 3.0),
]

mask = band_mask((H_IMG, W_IMG), 0.40)
print(f"合成地面 {X0}–{X1} m × {Y0}–{Y1} m @ {RES*1000:.0f} mm/px（{GW}×{GH}）  "
      f"h={H_CAM} m  f={F:.0f} px  FOV={HFOV}°")
print("同一批 KLT 特征、同一段图像；old 不补偿旋转，new 用已知 ψ 补偿\n")
print(f"{'工况':<22}{'真值 (dx,dy)':>18}{'old (dx,dy)':>18}{'new (dx,dy)':>18}"
      f"{'new+1°误差':>14}{'内点':>6}")
rows = []
for name, tx, ty, dyaw_deg in CASES:
    psi = math.radians(dyaw_deg)
    a = noisy(render(0.0, 0.0, 0.0))
    b = noisy(render(tx, ty, psi))
    tr = track_points(a, b, band=0.40, mask=mask, max_corners=260)
    if tr is None:
        print(f"{name:<22}{'—':>18}{'追踪失败':>18}")
        continue
    p0, p1 = tr
    old = estimate_forward(a, b, h=H_CAM, f=F, cy=CY, band=0.40, mask=mask, cx=CX)
    new = solve_motion(p0, p1, h=H_CAM, f=F, cx=CX, cy=CY, dyaw=psi)
    bad = solve_motion(p0, p1, h=H_CAM, f=F, cx=CX, cy=CY, dyaw=psi + math.radians(1.0))
    if old is None or new is None or bad is None:
        print(f"{name:<22}{f'({tx:.3f},{ty:.3f})':>18}{'估计失败':>18}")
        continue
    print(f"{name:<22}({tx:+.3f},{ty:+.3f})".ljust(40)
          + f"({old['dx']:+.3f},{old['dy'] or 0:+.3f})".rjust(18)
          + f"({new['dx']:+.3f},{new['dy']:+.3f})".rjust(18)
          + f"({bad['dx']:+.3f},{bad['dy']:+.3f})".rjust(14)
          + f"{new['n_in']:6d}")
    rows.append((name, tx, ty, old, new, bad))

# ---- 旋转中心偏移补偿：相机在车体前方 0.26 m、左侧 0.06 m（URDF stereo_left_joint）
# 车绕 base 原点转时，相机点额外走圆弧 (R(Δψ)−I)·p。solve_motion 量到的是**相机**位移，
# 里程计必须减掉这一项才是车体位移。验证方式：让车沿圆弧逐帧前进（每帧 Δψ≈2.3°，
# 与真实 10 Hz / 0.4 rad/s 一致），逐帧比对"补偿后"与真值，并看累积误差。
# 注意：**不能用"单帧转 57°"来测** —— 那是 246 px 的位移，KLT 追不到，测的是追踪失败
# 而不是补偿公式（第一版就这么写错了，量出 614 mm 的假失败）。
CAM_X, CAM_Y = 0.26, 0.06
V_ARC, W_ARC, DT_ARC, N_ARC = 0.25, 0.40, 0.1, 12


def body_pose(t: float):
    """车沿自身朝向以 V 前进、朝向以 W 旋转 → 世界系圆弧（圆心在左侧 R=V/W）。"""
    psi = W_ARC * t
    rad = V_ARC / W_ARC
    return rad * math.sin(psi), rad * (1.0 - math.cos(psi)), psi


def cam_pose(t: float):
    x, y, psi = body_pose(t)
    c, s = math.cos(psi), math.sin(psi)
    return x + c * CAM_X - s * CAM_Y, y + s * CAM_X + c * CAM_Y, psi


print()
print(f"旋转中心偏移补偿：车沿圆弧前进 {N_ARC} 帧（V={V_ARC} W={W_ARC} dt={DT_ARC}，"
      f"每帧 Δψ={math.degrees(W_ARC * DT_ARC):.1f}°）")
print(f"{'帧':>4}{'车体真值(本帧系)':>22}{'solve 的相机位移':>20}{'补偿后':>20}{'误差':>10}")
offs = []
cum_true = [0.0, 0.0]
cum_fix = [0.0, 0.0]
for k in range(N_ARC):
    t0, t1 = k * DT_ARC, (k + 1) * DT_ARC
    cx0, cy0, psi0 = cam_pose(t0)
    cx1, cy1, psi1 = cam_pose(t1)
    px0, py0, _ = body_pose(t0)
    px1, py1, _ = body_pose(t1)
    dpsi = psi1 - psi0
    c0, s0 = math.cos(psi0), math.sin(psi0)
    tdx = c0 * (px1 - px0) + s0 * (py1 - py0)
    tdy = -s0 * (px1 - px0) + c0 * (py1 - py0)
    a = noisy(render(cx0, cy0, psi0))
    b = noisy(render(cx1, cy1, psi1))
    tr = track_points(a, b, band=0.40, mask=mask, max_corners=260)
    r = None if tr is None else solve_motion(tr[0], tr[1], h=H_CAM, f=F, cx=CX, cy=CY,
                                             dyaw=dpsi)
    if r is None:
        print(f"{k:>4}{'追踪/解算失败':>22}")
        continue
    cc, ss = math.cos(dpsi), math.sin(dpsi)
    ex = (cc - 1.0) * CAM_X - ss * CAM_Y
    ey = ss * CAM_X + (cc - 1.0) * CAM_Y
    fdx, fdy = r["dx"] - ex, r["dy"] - ey
    err = math.hypot(fdx - tdx, fdy - tdy)
    print(f"{k:>4}{'({:+.3f},{:+.3f})'.format(tdx, tdy):>22}"
          f"{'({:+.3f},{:+.3f})'.format(r['dx'], r['dy']):>20}"
          f"{'({:+.3f},{:+.3f})'.format(fdx, fdy):>20}{err * 1000:9.1f} mm")
    offs.append(err)
    cum_true[0] += tdx
    cum_true[1] += tdy
    cum_fix[0] += fdx
    cum_fix[1] += fdy
if offs:
    tot_true = math.hypot(*cum_true)
    tot_err = math.hypot(cum_fix[0] - cum_true[0], cum_fix[1] - cum_true[1])
    print(f"  逐帧最大 {max(offs) * 1000:.1f} mm；累计走 {tot_true:.2f} m、"
          f"累计误差 {tot_err * 1000:.1f} mm（{tot_err / max(tot_true, 1e-6) * 100:.1f}%）")

print()
worst = 0.0
fails = []
for name, tx, ty, old, new, bad in rows:
    for comp, truth, val in (("dx", tx, new["dx"]), ("dy", ty, new["dy"])):
        tol = max(0.005, 0.15 * abs(truth))
        err = abs(val - truth)
        worst = max(worst, err / tol)
        if err > tol:
            fails.append(f"{name} {comp}: {val:+.4f} vs {truth:+.4f}（超限 {err*1000:.1f} mm）")
    old_lat = abs((old["dy"] or 0.0) - ty)
    if abs(tx) + abs(ty) > 1e-9 and old_lat > max(0.005, 0.15 * abs(ty)):
        print(f"  [对照] {name}：old 的 dy 偏 {old_lat*1000:5.1f} mm"
              f"（真值 dy {ty:+.3f}）← 这就是每帧假侧移的来源")

if offs:
    worst_off = max(offs)
    print(f"  偏移补偿最大误差 {worst_off * 1000:.1f} mm（判据 ≤ 30 mm）")
    if worst_off > 0.030:
        fails.append(f"旋转中心偏移补偿误差 {worst_off * 1000:.1f} mm > 30 mm")

print()
if fails:
    print("自检结论：FAIL")
    for f_ in fails:
        print("  -", f_)
    sys.exit(1)
print(f"自检结论：PASS —— 六种工况全部在 max(5 mm, 15%) 内；最差用例余量 {worst*100:.0f}% 用掉")
print("          （new+1° 列是对 IMU 偏航精度的敏感度对照：1° 误差时仍可用）")
sys.exit(0)
