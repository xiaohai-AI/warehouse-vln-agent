#!/usr/bin/env python3
"""干涉自检：轮子不许碰车体、载荷箱不许进激光扇面。

为什么单独写
  这两种错都**不会让任何判据 FAIL**，只会在 GUI 里看着别扭：
    · 裙边 y±0.22 / 保险杠 y±0.21 直接插进轮子（轮子视觉占 y 0.185~0.235）
    · 车体底 0.150 比轮顶 0.152 还低 → 轮子上半截埋进车体
    · 载荷箱顶 0.498 m，而激光最下一环(−25°)在箱体远端只有 0.408 m → 挡扫描
  （箱体是纯 visual，激光其实打不到它；但**真车会挡**，所以按真车口径约束。）

断言
  A. 每个轮子的视觉包围盒与 裙边/车体/保险杠 的包围盒**不相交**（容差 1 mm）
  B. 载荷箱顶面 < 激光 −25° 环在其足迹最远点的高度（留 5 mm 余量）

用法
  python3 scripts/check_robot_clearance.py            # 断言，失败 exit 1
  python3 scripts/check_robot_clearance.py --verbose
"""
from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SDF = ROOT / "robot" / "depot_bot.sdf"

LIDAR = {"x": -0.100, "z": 0.550}      # lidar3d_link 位姿（不许动，是判据口径）
LIDAR_MIN_DEG = -25.0                   # 垂直视场下界
LIDAR_MAX_DEG = 15.0
R_WHEEL = 0.076                         # 轮半径（= 辊子外沿包络，见 check_wheels.py）
PAYLOAD = "crate"                       # 载荷箱视觉名关键字
BODY_PARTS = ("skirt", "body_main", "bumper")   # 不许和轮子相交的车体件
WHEELS = ("fl", "fr", "rl", "rr")
EPS = 0.001


def aabbs(sdf: str) -> dict[str, tuple[float, float, float, float, float, float]]:
    """每个 visual 的世界系 AABB：(x0,x1,y0,y1,z0,z1)。旋转件按 |R| 保守放大。"""
    out = {}
    for m in re.finditer(r"<visual name='([^']+)'>(.*?)</visual>", sdf, re.S):
        name, body = m.group(1), m.group(2)
        pm = re.search(r"<pose>([^<]+)</pose>", body)
        if not pm:
            continue
        px, py, pz, rr, rp, ry = (float(v) for v in pm.group(1).split()[:6])
        sz = re.search(r"<size>([^<]+)</size>", body)
        cy = re.search(r"<cylinder>\s*<length>([\d.]+)</length>\s*<radius>([\d.]+)</radius>", body)
        if sz:
            sx, sy, szz = (float(v) for v in sz.group(1).split()[:3])
        elif cy:
            ln, rad = float(cy.group(1)), float(cy.group(2))
            sx, sy, szz = 2 * rad, 2 * rad, ln
        else:
            continue
        # 绕 y 的旋转把 x/z 混合；绕 x 把 y/z 混合（保守取绝对值合成）
        ax = (sx * abs(math.cos(rp)) + szz * abs(math.sin(rp))) / 2
        az = (sx * abs(math.sin(rp)) + szz * abs(math.cos(rp))) / 2
        ay = (sy * abs(math.cos(rr)) + szz * abs(math.sin(rr))) / 2
        out[name] = (px - ax, px + ax, py - ay, py + ay, pz - az, pz + az)
    return out


def overlap(a, b, eps: float = EPS) -> tuple[float, float, float] | None:
    """返回三轴重叠量；任一轴不重叠则返回 None。"""
    ox = min(a[1], b[1]) - max(a[0], b[0])
    oy = min(a[3], b[3]) - max(a[2], b[2])
    oz = min(a[5], b[5]) - max(a[4], b[4])
    if ox > eps and oy > eps and oz > eps:
        return ox, oy, oz
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    if not SDF.exists():
        print(f"✗ 找不到 {SDF}（先 make robot）")
        return 2
    sdf_txt = SDF.read_text(encoding="utf-8")
    boxes = aabbs(sdf_txt)
    # 轮子链的视觉都在 wheel_*_link 坐标系里，要加上该 link 相对 base_footprint 的偏置
    offs = {}
    for w in WHEELS:
        jm = re.search(rf"<joint name='wheel_{w}_joint'[^>]*>(.*?)</joint>", sdf_txt, re.S)
        if not jm:
            continue
        pm = re.search(r"<pose[^>]*>([^<]+)</pose>", jm.group(1))
        if pm:
            ox, oy, oz = (float(v) for v in pm.group(1).split()[:3])
            offs[w] = (ox, oy, oz)
    for name in list(boxes):
        for w in WHEELS:
            if f"wheel_{w}_link" in name and w in offs:
                ox, oy, oz = offs[w]
                x0, x1, y0, y1, z0, z1 = boxes[name]
                boxes[name] = (x0 + ox, x1 + ox, y0 + oy, y1 + oy, z0 + oz, z1 + oz)
    problems: list[str] = []

    # ---- A. 轮子 vs 车体
    wheel_names = {w: [n for n in boxes if f"wheel_{w}_link" in n] for w in WHEELS}
    body_names = [n for n in boxes if any(k in n.split("__")[-1] and k in n for k in BODY_PARTS)]
    body_names = [n for n in body_names if not any(f"wheel_{w}_link" in n for w in WHEELS)]
    for w, names in wheel_names.items():
        if not names:
            problems.append(f"wheel_{w}: 找不到视觉件")
            continue
        wb = [boxes[n] for n in names]
        # **不能用逐件 AABB 并集**：不同辊子的极值来自不同角度，并集会把包络放大到 0.083
        # （真包络 = 轮半径 0.076，已由 check_wheels.py 断言）。这里用解析包络：
        #   径向 = R_wheel（辊子外沿），轴向 = 各视觉件 y 方向的真实最大范围（含倾斜带来的 0.039）
        cx, cy, cz = offs[w]
        half_y = max(abs(b[2] - cy) for b in wb) if wb else 0.0
        wbox = (cx - R_WHEEL, cx + R_WHEEL, cy - half_y, cy + half_y, cz - R_WHEEL, cz + R_WHEEL)
        if args.verbose:
            print(f"  wheel_{w} 包络: x {wbox[0]:+.3f}~{wbox[1]:+.3f}  y {wbox[2]:+.3f}~{wbox[3]:+.3f}"
                  f"  z {wbox[4]:+.3f}~{wbox[5]:+.3f}")
        for bn in body_names:
            ov = overlap(wbox, boxes[bn])
            if ov:
                short = bn.split("__")[-1].replace("_visual", "").rsplit("_", 1)[0]
                problems.append(f"wheel_{w} 与 {short} 相交 "
                                f"Δx={ov[0] * 1000:.0f} Δy={ov[1] * 1000:.0f} Δz={ov[2] * 1000:.0f} mm")

    # ---- B. 载荷箱 vs 激光扇面
    pay = {n: b for n, b in boxes.items() if PAYLOAD in n}
    top = z_ray = 0.0
    if pay:
        x0 = min(b[0] for b in pay.values())
        x1 = max(b[1] for b in pay.values())
        y0 = min(b[2] for b in pay.values())
        y1 = max(b[3] for b in pay.values())
        top = max(b[5] for b in pay.values())
        far = max(math.hypot(px - LIDAR["x"], py) for px in (x0, x1) for py in (y0, y1))
        z_ray = LIDAR["z"] - math.tan(math.radians(-LIDAR_MIN_DEG)) * far
        if args.verbose:
            print(f"  载荷箱: x {x0:+.3f}~{x1:+.3f} y {y0:+.3f}~{y1:+.3f} 顶 {top:.3f}")
            print(f"  激光最下一环({LIDAR_MIN_DEG:.0f}°) 在远端 {far:.3f} m 处高度 {z_ray:.3f}")
        if top > z_ray - 0.005:
            problems.append(f"载荷箱顶 {top:.3f} 挡到激光最下一环（该处环高 {z_ray:.3f}，需 ≤ {z_ray - 0.005:.3f}）")

    print()
    if problems:
        print("✗ 干涉自检失败：")
        for p in problems:
            print("   -", p)
        return 1
    top_msg = f"，载荷箱顶 {top:.3f} < 激光环 {z_ray:.3f}" if pay else ""
    print(f"✓ 干涉自检通过：轮子与车体无接触（4 轮 × {len(body_names)} 个车体件）{top_msg}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
