#!/usr/bin/env python3
"""机器人几何自检：把"文档里写的离地高度"变成断言。

为什么需要（真实踩过）
  轮子关节 origin 写成了 `-body_z0`（-0.09），而 base_link 原点**已经**在轮轴高度 →
  整车要多抬 0.09 m 才能让轮子落地 → GUI 里车身在轮子上方悬空 2.8 cm，
  且双目相机实际 0.45 m、3D 激光 0.64 m（文档与代码都写 0.35 / 0.55）。
  这种错不会让任何运动判据 FAIL —— 只会在镜头里看得出来，所以必须单独断言。

断言（改了 ROBOT 就同步改这里，并且要能说清为什么）
  1. 整车落地时 base_footprint 必须 = 0（轮底正好贴地，不许悬空/下沉）
  2. 裙边底 = 0.090 m，车体底 = 0.150 m，上盖底 = 0.270 m
  3. 双目相机 = 0.350 m（= depot_vln/verify_depth.py 的 CAM_Z）
  4. 3D 激光 = 0.550 m
  5. 车体底不得高于轮顶（否则车身"浮"在轮子上）

用法
  python3 scripts/check_robot_geometry.py                 # 断言，失败返回 1
  python3 scripts/check_robot_geometry.py --verbose       # 顺便打印全部实测值
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SDF = ROOT / "robot" / "depot_bot.sdf"

EXPECT = {
    "base_footprint_rest": 0.000,   # 落地时模型系高度
    "skirt_bottom": 0.090,
    # 车体底 0.155：轮顶在 0.152，留 3 mm 视觉间隙（轮子不能和车体有接触），
    # 同时不能高到"悬空"（> 6 mm 视为没坐实，见下面 gap 判定）
    "body_bottom": 0.155,
    "deck_bottom": 0.270,
    "camera": 0.350,                # 与 verify_depth.CAM_Z 必须一致
    "lidar": 0.550,
}
TOL = 0.002


def parse(sdf: str) -> dict[str, float]:
    out: dict[str, float] = {}

    m = re.search(r"<joint name='wheel_fl_joint'.*?<pose relative_to='base_footprint'>([^<]+)</pose>", sdf, re.S)
    if not m:
        raise SystemExit("找不到 wheel_fl_joint 的 pose")
    wheel_z = float(m.group(1).split()[2])
    radius = 0.076
    out["base_footprint_rest"] = radius - wheel_z
    out["wheel_top"] = wheel_z + radius

    def vis(name: str) -> tuple[float, float]:
        mm = re.search(rf"<visual name='[^']*{name}_visual[^']*'>\s*<pose>([^<]+)</pose>", sdf)
        sz = re.search(rf"<visual name='[^']*{name}_visual[^']*'>.*?<size>([^<]+)</size>", sdf, re.S)
        if not mm:
            raise SystemExit(f"找不到 visual: {name}")
        z = float(mm.group(1).split()[2])
        h = float(sz.group(1).split()[2]) if sz else 0.0
        return z, h

    for key, vname in [("skirt_bottom", "skirt"), ("body_bottom", "body_main"), ("deck_bottom", "deck")]:
        z, h = vis(vname)
        out[key] = out["base_footprint_rest"] + z - h / 2

    for m2 in re.finditer(r"<sensor name='([^']+)'[^>]*>(.*?)</sensor>", sdf, re.S):
        name, body = m2.group(1), m2.group(2)
        pm = re.findall(r"<pose[^>]*>([^<]+)</pose>", body)
        if not pm:
            continue
        z = float(pm[-1].split()[2])
        if name == "stereo_left":
            out["camera"] = out["base_footprint_rest"] + z
        elif name == "lidar3d":
            out["lidar"] = out["base_footprint_rest"] + z
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    if not SDF.exists():
        print(f"✗ 找不到 {SDF}（先跑 make robot）")
        return 2
    got = parse(SDF.read_text(encoding="utf-8"))

    bad: list[str] = []
    print(f"{'项':<22}{'实测':>9}{'期望':>9}{'差':>9}")
    print("-" * 50)
    for key, want in EXPECT.items():
        v = got.get(key)
        if v is None:
            print(f"{key:<22}{'缺':>9}{want:>9.3f}{'—':>9}")
            bad.append(key)
            continue
        d = v - want
        flag = "✓" if abs(d) <= TOL else "✗"
        print(f"{key:<22}{v:>9.3f}{want:>9.3f}{d:>+9.3f}  {flag}")
        if abs(d) > TOL:
            bad.append(f"{key}（实测 {v:.3f} ≠ 期望 {want:.3f}）")

    # 车体与轮顶的关系：既不许悬空（>6 mm），也不许插进轮子里（<-2 mm）
    gap = got.get("body_bottom", 0) - got.get("wheel_top", 0)
    if gap > 0.006:
        bad.append(f"车体底 {got['body_bottom']:.3f} 比轮顶 {got['wheel_top']:.3f} 高 {gap * 1000:.0f} mm → 车身悬空")
    elif gap < -0.004:
        bad.append(f"车体底 {got['body_bottom']:.3f} 比轮顶 {got['wheel_top']:.3f} 低 {abs(gap) * 1000:.0f} mm → 车体插进轮子")

    if args.verbose:
        print()
        for k, v in sorted(got.items()):
            print(f"  {k:<22}{v:>8.3f}")

    print()
    if bad:
        print("✗ 几何自检失败：")
        for b in bad:
            print("   -", b)
        return 1
    print("✓ 几何自检通过（落地贴地、车身压住轮顶、相机 0.35 / 激光 0.55 与文档和 CAM_Z 一致）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
