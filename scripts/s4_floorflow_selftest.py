#!/usr/bin/env python3
"""S4 探针的**离线自检**：用合成地面图验证"地面带光流 → 沿轴位移"这套数学。

为什么先做这个：探针一旦上仿真就要 10 分钟 + GPU；如果公式或常数写错了，
那 10 分钟会被浪费掉，而且失败时分不清是"物理不可测"还是"代码写错"。
所以先在一个**几何已知**的世界里闭环：已知相机高 h、焦距 f、真实前进 Δx，
渲染两帧地面图像，让同一个估算函数去反推 —— 它必须把 Δx 还原出来。

渲染用的是精确投影（不是线性近似）：
    相机在 x_c、高 h、pitch 0 时，像素 (u,v) 对应的地面点为
        d = f·h / (v − cy)          X_w = x_c + d          Y_w = (u − cx)·d / f
    相机前进 Δx 就是让它看到同一个像素时对应的 X_w 增大 Δx。

用法：.venv/bin/python scripts/s4_floorflow_selftest.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from depot_vln.floorflow import band_mask, estimate_forward, fit_scale, floor_fraction  # noqa: E402

W_IMG, H_IMG = 320, 240
H_CAM = 0.426          # 与探针默认一致（0.35 相对 base_footprint + 0.076 轮轴高）
HFOV = 90.0
F = (W_IMG / 2.0) / np.tan(np.radians(HFOV) / 2.0)
CX, CY = W_IMG / 2.0, H_IMG / 2.0

# ---- 合成地面纹理：2 mm/px 的世界图，覆盖相机前方 0.10–12 m、横向 ±3 m
RES = 0.002
X0, X1 = 0.05, 12.0
Y0, Y1 = -3.0, 3.0
GW, GH = int((Y1 - Y0) / RES), int((X1 - X0) / RES)
rng = np.random.default_rng(20261031)
world = cv2.GaussianBlur(rng.normal(128, 45, (GH, GW)).astype(np.float32), (0, 0), 1.3)
# 多尺度斑点：模拟程序化地面纹理里的碎石/补丁（保证角点足够且不周期）
for _ in range(9000):
    r = rng.uniform(1.0, 4.0)
    cx_, cy_ = rng.uniform(0, GW), rng.uniform(0, GH)
    v = rng.uniform(40, 230)
    cv2.circle(world, (int(cx_), int(cy_)), int(r), float(v), -1)
world = np.clip(world, 0, 255)


def render(cam_x: float, cam_y: float = 0.0) -> np.ndarray:
    """按精确投影把地面渲染成图像。cam_y 用 **ROS 车体约定：向左为正**。

    相机 y 轴向左 → 世界左坐标 Y = cam_y + (cx − u)·X/f，所以 map_y 用 (CX − u)。
    """
    us, vs = np.meshgrid(np.arange(W_IMG, dtype=np.float32),
                         np.arange(H_IMG, dtype=np.float32))
    with np.errstate(divide="ignore", invalid="ignore"):
        d = F * H_CAM / (vs - CY)          # 每行对应的地面距离（行越靠下越近）
        map_x = (cam_x + d - X0) / RES
        map_y = (cam_y + (CX - us) * d / F - Y0) / RES
    map_x = np.nan_to_num(map_x, nan=-1, posinf=-1, neginf=-1).astype(np.float32)
    map_y = np.nan_to_num(map_y, nan=-1, posinf=-1, neginf=-1).astype(np.float32)
    img = cv2.remap(world, map_y, map_x, interpolation=cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    return img.astype(np.uint8)


def noisy(img: np.ndarray, sigma: float = 3.0) -> np.ndarray:
    return np.clip(img.astype(np.float32) + rng.normal(0, sigma, img.shape), 0, 255).astype(np.uint8)


print(f"合成地面 {X0}–{X1} m × {Y0}–{Y1} m @ {RES * 1000:.0f} mm/px  "
      f"（{GW}×{GH} px）  相机 h={H_CAM} m  f={F:.0f} px  FOV={HFOV}°")
print(f"地面带 = 底部 40%（行 {int(H_IMG * 0.6)}–{H_IMG}）\n")
print(f"{'真值 Δx':>9}{'估计 Δx':>10}{'误差':>9}{'特征数':>8}{'光流 px/帧':>12}{'全图落地面比':>13}")

mask = band_mask((H_IMG, W_IMG), 0.40)
ests, truths = [], []
for step in (0.010, 0.020, 0.035, 0.070, 0.120):
    a = noisy(render(0.0))
    b = noisy(render(step))
    r = estimate_forward(a, b, h=H_CAM, f=F, cy=CY, band=0.40, mask=mask)
    ff = floor_fraction(a, 0.40)
    if r is None:
        print(f"{step:9.3f}{'失败':>10}{'—':>9}{'—':>8}{'—':>12}{ff * 100:12.1f}%")
        continue
    err = (r["dx"] - step) / step * 100
    print(f"{step:9.3f}{r['dx']:10.4f}{err:8.1f}%{r['n_feat']:8d}{r['med_dy_px']:12.2f}{ff * 100:12.1f}%")
    # 只有"每帧位移"在真实范围内（≤8 cm/帧 ≈ 0.8 m/s@10 Hz）的样本进拟合：
    # 12 cm 那档是 KLT 大位移退化的测试台伪影，照常打印但不参与尺度解算。
    if step <= 0.08:
        ests.append(r["dx"])
        truths.append(step)

# ---- 侧移（麦克纳姆横移）：ΔY_left = −ΔY_right
print()
print(f"{'真值 Δy(左+, 相机左移)':>24}{'估计 dy(左+)':>14}{'误差':>9}{'du px/帧':>10}")
lat_ests, lat_truths = [], []
mask2 = band_mask((H_IMG, W_IMG), 0.40)
for step in (0.010, 0.020, 0.035, 0.070):
    a = noisy(render(0.0, 0.0))
    b = noisy(render(0.0, step))
    r = estimate_forward(a, b, h=H_CAM, f=F, cy=CY, band=0.40, mask=mask2, cx=CX)
    if r is None or r.get("dy") is None:
        print(f"{step:22.3f}{'失败':>14}")
        continue
    want = step                       # 相机左移 step → dy(左+) 应为 +step
    err = (r["dy"] - want) / max(abs(want), 1e-9) * 100
    print(f"{step:22.3f}{r['dy']:14.4f}{err:8.1f}%{r['med_du_px']:10.2f}")
    lat_ests.append(r["dy"])
    lat_truths.append(want)
if len(lat_ests) >= 3:
    sl = fit_scale(lat_ests, lat_truths)
    print(f"侧移尺度 s = {sl:.3f}（1.000 = 方向与幅值都对）")

if len(ests) >= 3:
    s = fit_scale(ests, truths)
    print(f"\n最优尺度 s = {s:.3f}（1.000 = 几何常数完全正确；"
          f"等效相机高 {s * H_CAM:.3f} m vs 标称 {H_CAM} m）")
    ok = abs(s - 1.0) < 0.20
    print("自检结论：", "PASS —— 公式与常数正确，探针可以上仿真"
          if ok else "FAIL —— 尺度偏差 >20%，先修公式/常数再上仿真")
    sys.exit(0 if ok else 1)
print("\n自检结论：FAIL —— 有效样本不足（合成纹理角点不够），先改纹理再上仿真")
sys.exit(1)
