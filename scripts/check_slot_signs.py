#!/usr/bin/env python3
"""货位编号牌自检：牌在不在、字够不够大、会不会污染 verify 窗口。

为什么要有（这类错不会让任何运动判据 FAIL）
  * 牌漏发 -> 相机读不到地址 -> L1 又变回"不可观测"
  * 牌太小 -> OCR 读不出来，等于没贴（**必须按实测像素判断，不能只看配置里的 0.26 m**）
  * 牌压进 verify 观测窗口 -> 空位被自己的牌子判成"有货"（假阳性，L8 同类）
  * 牌伸进巷道太多 -> 与"巷道净宽 3.20 ± 0.05"的量测口径打架

断言（对 576 个货位逐个）
  A. SDF 里有 `sign_slot_<地址>_vis` 牌面，且贴图存在
  B. 实测字高（量 PNG 墨迹）在观测距离处的像素 ≥ MIN_CAP_PX(20)
  C. 牌底 ≥ verify 窗口上沿 + 0.10（窗口 = deck_z + 0.70）
  D. 牌面在所属 bay 的 x 范围内（不跨格）
  E. 牌面离货架面 ≤ 8 mm（不改变净宽口径）
  F. level 1 的牌顶在标准观测位的视野内（相机 0.35 m、俯仰 0、2.15 m）
  G. 牌子全在一个**无碰撞** link 里（不产生物理实体、不进静态图）

用法
  .venv/bin/python scripts/check_slot_signs.py
  .venv/bin/python scripts/check_slot_signs.py --verbose
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from depot_vln import slot_signs as S  # noqa: E402

SDF = ROOT / "world" / "warehouse.sdf"
GRID = ROOT / "world" / "slot_grid.json"
LBL = ROOT / "world" / "textures" / "slot_labels"

CAM_Z = 0.35          # 与 verify_depth.CAM_Z 一致
WINDOW_UP = 0.70      # verify 观测窗口上沿 = deck_z + 0.70
MARGIN_Z = 0.10       # 牌底到窗口上沿至少这么多
MAX_OUT = 0.008       # 牌面外凸上限


def load_png_gray(path: Path):
    import cv2  # type: ignore
    import numpy as np  # type: ignore

    img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    return img


def ink_cap_px(path: Path) -> int:
    """量贴图里字的像素高度（去掉边框后取深色墨迹的竖直范围）。"""
    import numpy as np  # type: ignore

    img = load_png_gray(path)
    if img is None:
        return 0
    inner = img[8:-8, 8:-8]
    mask = inner < 128
    ys, _ = np.nonzero(mask)
    if len(ys) == 0:
        return 0
    return int(ys.max() - ys.min() + 1)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    if not SDF.exists() or not GRID.exists():
        print("✗ 缺 world/warehouse.sdf 或 world/slot_grid.json（先 make world）")
        return 2

    slots = json.loads(GRID.read_text(encoding="utf-8"))["slots"]
    sdf = SDF.read_text(encoding="utf-8")

    # --- G: 牌子所在的 link 不许有碰撞
    m_link = re.search(r'<link name="slot_signs_link">(.*?)</link>', sdf, re.S)
    if not m_link:
        print("✗ SDF 里没有 slot_signs_link —— 编号牌没生成（检查 build_sdf 是否接了 _slot_signs）")
        return 1
    link_body = m_link.group(1)
    n_col = link_body.count("<collision")
    n_vis = link_body.count("<visual")

    # --- 解析所有牌面
    plates: dict[str, tuple[list[float], list[float]]] = {}
    for m in re.finditer(r'<visual name="(sign_slot_[^"]+_vis)">(.*?)</visual>', link_body, re.S):
        name, body = m.group(1), m.group(2)
        addr = S.addr_from_visual(name)
        pm = re.search(r"<pose>([^<]+)</pose>", body)
        sm = re.search(r"<size>([^<]+)</size>", body)
        if not (addr and pm and sm):
            continue
        pose = [float(v) for v in pm.group(1).split()[:6]]
        size = [float(v) for v in sm.group(1).split()[:3]]
        plates[addr] = (pose[:3], size)

    problems: list[str] = []
    warn: list[str] = []
    caps: list[tuple[str, int, float]] = []
    if n_col:
        problems.append(f"slot_signs_link 里有 {n_col} 个 collision —— 牌必须是纯视觉")

    vfov_half = math.atan(math.tan(S.CAM_HFOV / 2) * S.CAM_H / S.CAM_W)
    n_lvl2_out = 0

    for addr, rec in sorted(slots.items()):
        lvl = int(rec["level"])
        deck_z = float(rec["deck_z"])
        bay_x = float(rec["pose"]["x"])
        depth = float(rec.get("slot_depth") or 0.55)
        dist = float(rec.get("obs_distance") or S.DEFAULT_OBS_DIST)

        if addr not in plates:
            problems.append(f"{addr}: 没有牌面 visual")
            continue
        (px, py, pz), (sx, sy, sz) = plates[addr]
        z0, z1 = pz - sz / 2, pz + sz / 2

        # B: 字高（实测）
        p = LBL / f"{addr}.png"
        if not p.exists():
            problems.append(f"{addr}: 缺贴图 {p.name}")
        else:
            cap = ink_cap_px(p)
            cap_m = cap / S.TEX_H * S.PLATE_H
            capv = S.cap_px(dist, cap_m)
            caps.append((addr, cap, capv))
            if capv < S.MIN_CAP_PX:
                problems.append(f"{addr}: 字高 {cap_m:.3f} m → {dist:.2f} m 处 {capv:.1f} px < {S.MIN_CAP_PX}")

        # C: 不压 verify 窗口
        if z0 < deck_z + WINDOW_UP + MARGIN_Z:
            problems.append(f"{addr}: 牌底 {z0:.3f} 压到观测窗口（需 ≥ {deck_z + WINDOW_UP + MARGIN_Z:.3f}）")

        # D: 不跨格（bay 宽 bay_pitch 2.4，取 ±1.0 保守）
        if abs(px - bay_x) > 0.01 or sx > 2.0:
            problems.append(f"{addr}: 牌中心 x={px:+.3f} 偏出格口中心 {bay_x:+.3f}，或牌宽 {sx:.2f} 过大")

        # E: 外凸量
        obs_y = float(rec["obs_pose"]["y"])
        y_slot = float(rec["pose"]["y"])              # 半深货位的中心（不是牌面 y！）
        sgn = 1.0 if obs_y > y_slot else -1.0
        y_face = y_slot + sgn * depth / 2             # 货架面
        out = abs(sgn * (py - y_face)) + sy / 2       # 牌外表面离货架面
        if out > MAX_OUT:
            problems.append(f"{addr}: 牌面外凸 {out * 1000:.1f} mm > {MAX_OUT * 1000:.0f} mm")

        # F: level 1 必须在视野内
        z_top_view = CAM_Z + dist * math.tan(vfov_half)
        if lvl == 1 and z1 > z_top_view - 0.02:
            problems.append(f"{addr}: 牌顶 {z1:.3f} 超出观测位视野上沿 {z_top_view:.3f}")
        if lvl != 1 and z1 > z_top_view:
            n_lvl2_out += 1

    if args.verbose:
        print(f"  slot_signs_link: {n_vis} 个 visual（牌面 {len(plates)} + 吊带 {n_vis - len(plates)}）"
              f"，collision {n_col}")
        if caps:
            mn = min(caps, key=lambda c: c[2])
            print(f"  字高实测：最小 {mn[0]} {mn[1]} px → {mn[2]:.1f} px @ {S.DEFAULT_OBS_DIST} m")
        print(f"  视野上沿 @2.15 m = {CAM_Z + S.DEFAULT_OBS_DIST * math.tan(vfov_half):.3f} m")

    if n_lvl2_out:
        warn.append(f"level 2 有 {n_lvl2_out} 块牌不在 level-1 观测位的视野内"
                    f"（2 层本身需要更高/后撤的观测位，属已知限制）")

    print()
    for w in warn:
        print(f"⚠️  {w}")
    if problems:
        print(f"✗ 编号牌自检失败（{len(problems)} 条）：")
        for p in problems[:15]:
            print("   -", p)
        if len(problems) > 15:
            print(f"   ... 另有 {len(problems) - 15} 条")
        return 1
    print(f"✓ 编号牌自检通过：{len(plates)} 块牌 / {n_vis} 个 visual / 0 collision / "
          f"字高 ≥ {S.MIN_CAP_PX:.0f} px / 无牌压窗口")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
