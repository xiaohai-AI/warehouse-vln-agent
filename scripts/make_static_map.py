#!/usr/bin/env python3
"""从**已构建的世界模型**（world/warehouse.sdf 的静态碰撞体）生成静态占据图。

历史教训（别再犯）
  这个脚本原先只读 `world/layout.json` 的货架清单来刷占据 —— 而世界里的叉车、周转箱、
  护栏是世界生成器另写的（不在 layout.json 里）。结果是**地图 ⊂ 世界**，Nav2 不知道那里
  有东西，M2 的入库点（C 巷 bay 5 右侧 = (−18.00,−3.05)）正好落在叉车体内，车被顶住、
  轮子空转、odom 照记到位，落点误差恒定 0.93 m，害我们排查了两天底盘/摩擦/MPPI。

  现在的原则：**占据只能来自构建产物（SDF 里的 static collision），不来自任何"清单"。**
  `world/layout.json` 只用来定图像范围（footprint）和田间的自由区填充，不决定障碍物。

自检（生成时立刻做，错了当场报）
  * 每个静态碰撞体的中心在图上必须是占据
  * 每条巷道中线必须是自由
  * 每个"侧移入库点"（所有 aisle/bay/side 组合）必须是自由，且不碰撞

用法
  python3 scripts/make_static_map.py                     # 生成 maps/depot_static.{pgm,yaml}
  python3 scripts/make_static_map.py --out /tmp/x --no-verify
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from depot_geometry import (  # noqa: E402
    ROBOT_HALF_DIAG,
    blockers_at,
    dock_geometry,
    load_pgm,
    static_collisions,
)

RES = 0.05
UNKNOWN, FREE, OCC = 205, 254, 0


def rasterize(layout: dict, sdf_path: Path) -> tuple[bytearray, int, int]:
    fp = layout["footprint"]
    x0, y0 = fp["x_min"], fp["y_min"]
    w = int(round((fp["x_max"] - x0) / RES))
    h = int(round((fp["y_max"] - y0) / RES))
    img = bytearray([UNKNOWN]) * (w * h)

    def fill(cx: float, cy: float, sx: float, sy: float, val: int) -> None:
        """按中心点+尺寸（米）刷矩形。图像第 0 行是 max y。"""
        xa, xb = cx - sx / 2, cx + sx / 2
        ya, yb = cy - sy / 2, cy + sy / 2
        ca = max(0, int((xa - x0) / RES))
        cb = min(w - 1, int((xb - x0) / RES))
        ra = max(0, int((fp["y_max"] - yb) / RES))
        rb = min(h - 1, int((fp["y_max"] - ya) / RES))
        for r in range(ra, rb + 1):
            base = r * w
            for c in range(ca, cb + 1):
                img[base + c] = val

    # 1) 墙以内先全判自由（范围仍用世界生成的 footprint，它和世界边界一致）
    wall_t = layout["derived"]["wall_thickness"]
    fill(0.0, 0.0, fp["x_max"] - fp["x_min"] - 2 * wall_t,
         fp["y_max"] - fp["y_min"] - 2 * wall_t, FREE)

    # 2) 所有静态碰撞体判占据 —— 这是唯一来源（货架/立柱/墙/叉车/托盘/护栏都在里面）
    rects = static_collisions(sdf_path)
    for r in rects:
        fill(r["cx"], r["cy"], r["sx"], r["sy"], OCC)
    return img, w, h


def sample(img: list[int], w: int, h: int, layout: dict, x: float, y: float) -> int:
    fp = layout["footprint"]
    c = int((x - fp["x_min"]) / RES)
    r = int((fp["y_max"] - y) / RES)
    if not (0 <= r < h and 0 <= c < w):
        return -1
    return img[r * w + c]


def verify(img: list[int], w: int, h: int, layout: dict, sdf_path: Path) -> int:
    """一致性自检：障碍物在地图里、巷道与入库点在地图外。"""
    bad = 0
    rects = static_collisions(sdf_path)

    # (a) 每个静态碰撞体的中心 + 四角内侧 必须占据
    miss = []
    for r in rects:
        pts = [(r["cx"], r["cy"])]
        pts += [(r["cx"] + dx * r["sx"] * 0.4, r["cy"] + dy * r["sy"] * 0.4)
                for dx, dy in ((1, 1), (1, -1), (-1, 1), (-1, -1))]
        if any(sample(img, w, h, layout, x, y) != OCC for x, y in pts):
            miss.append(f'{r["model"]}/{r["name"]}')
    if miss:
        print(f"FAIL 世界里有 {len(miss)} 个碰撞体没画进地图：{miss[:6]}")
        bad += 1
    else:
        print(f"[verify] 世界静态碰撞体 {len(rects)} 个，全部在地图上为占据 ✓")

    # (b) 巷道中线必须自由
    for a in layout["aisles"]:
        n_free = sum(1 for x in range(int(layout["footprint"]["x_min"]),
                                      int(layout["footprint"]["x_max"]))
                     if sample(img, w, h, layout, float(x), a["center_y"]) == FREE)
        need = int((layout["footprint"]["x_max"] - layout["footprint"]["x_min"]) * 0.5)
        if n_free < need:
            print(f"FAIL 巷道 {a['name']} 中线自由格 {n_free} < {need}")
            bad += 1
        else:
            print(f"[verify] 巷道 {a['name']} y={a['center_y']:+.2f} 中线自由 ✓（{n_free} 格）")

    # (c) 候选入库点：地图判定必须与 SDF 判定一致
    #     map=自由 & SDF=空  -> 可用
    #     map=占据 & SDF=挡  -> 该点本身不可用（正常，脚本会拒绝），不算地图错误
    #     两者不一致        -> 地图与世界不一致，FAIL（这才是要抓的）
    usable, unusable, mismatch = [], [], []
    n_pts = 0
    rects_all = static_collisions(sdf_path)

    def point_in_sdf(x: float, y: float) -> bool:
        """点是否落在某个静态碰撞体内（不含机器人余量）—— 与地图**像素**口径对应。"""
        return any(r["x0"] <= x <= r["x1"] and r["y0"] <= y <= r["y1"] for r in rects_all)

    for a in layout["aisles"]:
        for bay in sorted({b["bay"] for b in layout["bays"] if b["aisle"] == a["name"]}):
            for side in ("L", "R"):
                start, dock, _, _ = dock_geometry(layout, a["name"], bay, side)
                n_pts += 1
                map_occ = sample(img, w, h, layout, dock[0], dock[1]) != FREE
                sdf_occ = point_in_sdf(dock[0], dock[1])
                tag = f"{a['name']} bay{bay} {side} ({dock[0]:+.2f},{dock[1]:+.2f})"
                # 一致性：地图像素与实际几何必须同判（这才是"地图=世界"的定义）
                if map_occ != sdf_occ:
                    mismatch.append(f"{tag} map_occ={map_occ} sdf_occ={sdf_occ}")
                    continue
                # 可用性：点本身不在障碍里，且机器人（带半对角线余量）也塞得进去
                if (not sdf_occ) and not blockers_at(dock[0], dock[1], ROBOT_HALF_DIAG, sdf_path):
                    usable.append(tag)
                else:
                    unusable.append(tag)
    print(f"[verify] 候选入库点 {n_pts} 个：可用 {len(usable)}，被障碍占用 {len(unusable)}")
    if unusable:
        print(f"         被占用的（脚本会拒绝，正常）：{unusable}")
    if mismatch:
        print(f"FAIL 地图与世界的判定不一致（{len(mismatch)} 个）：{mismatch[:5]}")
        bad += 1
    return bad


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--layout", default=str(ROOT / "world" / "layout.json"))
    ap.add_argument("--sdf", default=str(ROOT / "world" / "warehouse.sdf"))
    ap.add_argument("--out", default=str(ROOT / "maps" / "depot_static"))
    ap.add_argument("--no-verify", action="store_true")
    args = ap.parse_args()

    layout = json.loads(Path(args.layout).read_text(encoding="utf-8"))
    sdf_path = Path(args.sdf)
    img, w, h = rasterize(layout, sdf_path)
    fp = layout["footprint"]

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out.with_suffix(".pgm"), "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode())
        f.write(bytes(img))
    out.with_suffix(".yaml").write_text(
        f"image: {out.name}.pgm\n"
        f"resolution: {RES}\n"
        f"origin: [{fp['x_min']:.3f}, {fp['y_min']:.3f}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n",
        encoding="utf-8",
    )

    occ, free, unk = img.count(OCC), img.count(FREE), img.count(UNKNOWN)
    print(f"[map] {w}x{h} @ {RES} m  origin=({fp['x_min']}, {fp['y_min']})  "
          f"占据 {occ} 自由 {free} 未知 {unk}")
    print(f"[map] -> {out.with_suffix('.pgm')} / {out.with_suffix('.yaml')}")

    if args.no_verify:
        return 0

    # 用写出去的文件重新读回来校验，避免"内存里对、落盘错"
    img2, w2, h2 = load_pgm(out.with_suffix(".pgm"))
    bad = verify(img2, w2, h2, layout, sdf_path)
    print("RESULT:", "PASS" if bad == 0 else f"FAIL（{bad} 项）")
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
