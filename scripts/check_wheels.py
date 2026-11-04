#!/usr/bin/env python3
"""麦克纳姆轮几何自检：断言"看起来像真轮子"的那几条。

为什么需要（踩过）
  上一版轮子：外面套了一圈 **实心轮胎筒**（把辊子全盖住）、辊子中心半径 0.070 + 自身 0.018
  → 外沿 **0.088 > 碰撞半径 0.076**（视觉比物理大 12 mm，像陷进地里）、
  而且辊子轴线**没有轴向分量**（只在轮平面内偏转）→ 根本不是麦克纳姆轮。
  这类错不会让任何判据 FAIL，只会在镜头里看出来，所以单独断言。

断言
  1. 每个轮子有 9 根辊子
  2. 没有半径 ≥ 0.07 的"轮胎筒"把辊子包住（真轮子没有胎筒）
  3. 辊子外沿包络 = 轮半径 0.076（容差 1.5 mm）
  4. 辊子轴线与轮轴(ŷ)夹角 45° → |axis·ŷ| = 0.707（容差 0.06）
  5. 左右轮镜像：左侧与右侧同一编号辊子的切向分量符号相反

用法
  python3 scripts/check_wheels.py            # 断言，失败 exit 1
  python3 scripts/check_wheels.py --verbose  # 打印每个轮子的实测值
"""
from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SDF = ROOT / "robot" / "depot_bot.sdf"
R_WHEEL = 0.076
N_ROLLERS = 9
TOL_R = 0.0015
TOL_AXIS = 0.06


def axis_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float]:
    """圆柱默认轴 +z，经 Rz·Ry·Rx 旋转后的方向。"""
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    X, Y, Z = sp * cr, -sr, cp * cr
    return (cy * X - sy * Y, sy * X + cy * Y, Z)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    if not SDF.exists():
        print(f"✗ 找不到 {SDF}（先 make robot）")
        return 2
    sdf = SDF.read_text(encoding="utf-8")

    problems: list[str] = []
    summary: dict[str, dict] = {}

    for wheel in ("fl", "fr", "rl", "rr"):
        blk = re.search(rf"<link name='wheel_{wheel}_link'>(.*?)</link>", sdf, re.S)
        if not blk:
            problems.append(f"找不到 wheel_{wheel}_link")
            continue
        body = blk.group(1)
        rollers, tire_like = [], []
        for vm in re.finditer(r"<visual name='([^']+)'>(.*?)</visual>", body, re.S):
            name, vb = vm.group(1), vm.group(2)
            pm = re.search(r"<pose>([^<]+)</pose>", vb)
            # gz sdf -p 输出的是子元素形式：<cylinder><length>..</length><radius>..</radius></cylinder>
            rm = re.search(r"<cylinder>\s*<length>([\d.]+)</length>\s*<radius>([\d.]+)</radius>", vb)
            if rm:
                ln, rad = float(rm.group(1)), float(rm.group(2))
            else:   # 兼容属性形式
                ra = re.search(r"<cylinder[^>]*radius='([\d.]+)'[^>]*length='([\d.]+)'", vb)
                if not (pm and ra):
                    continue
                rad, ln = float(ra.group(1)), float(ra.group(2))
            if not pm:
                continue
            px, py, pz, roll, pitch, yaw = (float(v) for v in pm.group(1).split()[:6])
            if "roller" in name:
                ax = axis_from_rpy(roll, pitch, yaw)
                radial = math.hypot(px, pz)
                im = re.search(r"(\d+)", name.rsplit("_visual", 1)[0].split("roller")[-1])
                rollers.append({"i": int(im.group(1)) if im else -1,
                                "radial": radial, "r": rad, "outer": radial + rad,
                                "ay": abs(ax[1]), "tangential": ax[2] if abs(ax[2]) > 1e-9 else ax[0]})
            elif rad >= 0.070:
                tire_like.append((name, rad))

        if len(rollers) != N_ROLLERS:
            problems.append(f"wheel_{wheel}: 辊子 {len(rollers)} 根（应 {N_ROLLERS}）")
        if tire_like:
            problems.append(f"wheel_{wheel}: 还有轮胎筒把辊子包住 {tire_like}")
        for r in rollers:
            if abs(r["outer"] - R_WHEEL) > TOL_R:
                problems.append(f"wheel_{wheel} roller{r['i']}: 外沿 {r['outer']:.4f} ≠ 轮半径 {R_WHEEL}")
            if abs(r["ay"] - math.cos(math.radians(45))) > TOL_AXIS:
                problems.append(f"wheel_{wheel} roller{r['i']}: 轴线 |·ŷ|={r['ay']:.3f}，不是 45°")
        summary[wheel] = {"rollers": sorted(rollers, key=lambda d: d["i"])}

    # 左右镜像
    for lw, rw in (("fl", "fr"), ("rl", "rr")):
        if lw in summary and rw in summary:
            for a, b in zip(summary[lw]["rollers"], summary[rw]["rollers"]):
                if a["tangential"] * b["tangential"] > 0:
                    problems.append(f"{lw} 与 {rw} 的 roller{a['i']} 没有镜像（切向分量同号）")
                    break

    if args.verbose:
        for w, d in summary.items():
            outs = [r["outer"] for r in d["rollers"]]
            ays = [r["ay"] for r in d["rollers"]]
            print(f"  wheel_{w}: {len(d['rollers'])} 辊  外沿 {min(outs):.4f}~{max(outs):.4f}"
                  f"  |轴·ŷ| {min(ays):.3f}~{max(ays):.3f}")
        print()

    if problems:
        print("✗ 轮子几何自检失败：")
        for p in problems:
            print("   -", p)
        return 1
    print(f"✓ 轮子几何自检通过：每轮 {N_ROLLERS} 根辊子、无胎筒、外沿 = {R_WHEEL} m、"
          f"轴线 45°、左右镜像")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
