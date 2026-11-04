#!/usr/bin/env python3
"""一致性闸门：**地图 == 世界**，且验收点位可用。

为什么必须独立成一条命令（而不是只在生成时顺手看一眼）
  我们踩过的坑：世界里有叉车，静态图里没有。Nav2 于是不知道那里有东西，M2 的入库点正好
  在叉车体内 → 车被顶住、轮子空转、odom 照记 → 落点误差恒定 0.93 m，排查了两天。
  文档当时只有"地图与真值的占据一致性"作为 SLAM 的**事后指标**，没有任何地方把
  "地图必须覆盖世界里的静态碰撞体"当成**强制闸门**。这个脚本就是那道闸门。

检查项（一条命令，任何一项 FAIL 都是硬错误）
  A. 世界里每个静态碰撞体（机器人高度带内）在地图上都必须是占据
  B. 该自由的地方必须自由：巷道中线、以及 M2 默认入库点
  C. 一致性：地图像素判定 与 SDF 几何判定 必须同判（防"地图缺东西"或"地图多东西"）
  D. 可用性汇总：所有候选入库点里哪些不能用、为什么（这是数据，不是错误）

用法
  python3 scripts/map_check.py                     # 用 maps/depot_static.{pgm,yaml}
  python3 scripts/map_check.py --map maps/xxx      # 检查别的图
退出码：0 = PASS，1 = FAIL
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

FREE, OCC = 254, 0


def read_yaml(path: Path) -> dict:
    out: dict = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if ":" not in line or line.strip().startswith("#"):
            continue
        k, v = line.split(":", 1)
        out[k.strip()] = v.strip()
    origin = out.get("origin", "[0, 0, 0]").strip("[]").split(",")
    return {
        "image": path.parent / out.get("image", "").strip(),
        "resolution": float(out.get("resolution", 0.05)),
        "origin_x": float(origin[0]),
        "origin_y": float(origin[1]),
    }


class Map:
    def __init__(self, yaml_path: Path) -> None:
        meta = read_yaml(yaml_path)
        self.img, self.w, self.h = load_pgm(meta["image"])
        self.res = meta["resolution"]
        self.x0 = meta["origin_x"]
        self.y0 = meta["origin_y"]

    def at(self, x: float, y: float) -> int:
        """返回像素值；越界返回 -1。图像第 0 行是 y0 + h*res（y 轴反向）。"""
        c = int((x - self.x0) / self.res)
        r = int((self.y0 + self.h * self.res - y) / self.res)
        if not (0 <= r < self.h and 0 <= c < self.w):
            return -1
        return self.img[r * self.w + c]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default=str(ROOT / "maps" / "depot_static.yaml"))
    ap.add_argument("--sdf", default=str(ROOT / "world" / "warehouse.sdf"))
    ap.add_argument("--layout", default=str(ROOT / "world" / "layout.json"))
    args = ap.parse_args()

    m = Map(Path(args.map))
    layout = json.loads(Path(args.layout).read_text(encoding="utf-8"))
    rects = static_collisions(args.sdf)
    print(f"[map-check] 图 {m.w}x{m.h} @ {m.res} m, origin=({m.x0:.2f},{m.y0:.2f})  "
          f"世界静态碰撞体 {len(rects)} 个")

    bad = 0

    # A. 世界里的静态碰撞体必须在图上为占据（中心 + 四角内侧采样）
    miss = []
    for r in rects:
        pts = [(r["cx"], r["cy"])] + [
            (r["cx"] + dx * r["sx"] * 0.4, r["cy"] + dy * r["sy"] * 0.4)
            for dx, dy in ((1, 1), (1, -1), (-1, 1), (-1, -1))
        ]
        if any(m.at(x, y) != OCC for x, y in pts):
            miss.append(f'{r["model"]}/{r["name"]}')
    if miss:
        print(f"FAIL A: {len(miss)} 个静态碰撞体没进地图（前 6 个）：{miss[:6]}")
        print("        成因几乎总是：地图从某个‘清单’派生，而清单里没有这些模型。")
        bad += 1
    else:
        print(f"PASS A: {len(rects)} 个静态碰撞体全部在地图上为占据")

    # B. 该自由的地方自由：巷道中线 + M2 默认入库点
    free_bad = []
    for a in layout["aisles"]:
        xs = [m.x0 + i * m.res for i in range(0, m.w, 20)]
        n_free = sum(1 for x in xs if m.at(x, a["center_y"]) == FREE)
        if n_free < len(xs) * 0.5:
            free_bad.append(f"巷道 {a['name']} 中线")
    mid_bay = 4                                    # m2_side_dock.py 的默认 bay
    start, dock, _, _ = dock_geometry(layout, "C", mid_bay, "R")
    for label, pt in (("M2 起点", start), ("M2 入库点", dock)):
        if m.at(pt[0], pt[1]) != FREE:
            free_bad.append(f"{label} ({pt[0]:+.2f},{pt[1]:+.2f}) 在图上不是自由")
    if free_bad:
        print(f"FAIL B: 以下位置在图上不是自由：{free_bad}")
        bad += 1
    else:
        print(f"PASS B: 6 条巷道中线 + M2 默认点 "
              f"({start[0]:+.1f},{start[1]:+.1f})→({dock[0]:+.1f},{dock[1]:+.1f}) 均为自由")

    # C. 一致性：像素判定 == 几何判定（对全部候选入库点）
    def in_sdf(x: float, y: float) -> bool:
        return any(r["x0"] <= x <= r["x1"] and r["y0"] <= y <= r["y1"] for r in rects)

    mismatch, usable, unusable = [], 0, []
    for a in layout["aisles"]:
        for bay in sorted({b["bay"] for b in layout["bays"] if b["aisle"] == a["name"]}):
            for side in ("L", "R"):
                _, ddock, _, _ = dock_geometry(layout, a["name"], bay, side)
                map_occ = m.at(ddock[0], ddock[1]) != FREE
                sdf_occ = in_sdf(ddock[0], ddock[1])
                if map_occ != sdf_occ:
                    mismatch.append(f"{a['name']} bay{bay} {side}: "
                                    f"map_occ={map_occ} sdf_occ={sdf_occ}")
                    continue
                hits = blockers_at(ddock[0], ddock[1], ROBOT_HALF_DIAG, args.sdf)
                if (not sdf_occ) and not hits:
                    usable += 1
                else:
                    unusable.append(f"{a['name']} bay{bay} {side} <- {(hits or ['点本身在障碍内'])[0]}")
    if mismatch:
        print(f"FAIL C: 地图与世界判定不一致 {len(mismatch)} 处：{mismatch[:5]}")
        bad += 1
    else:
        print(f"PASS C: 288 个候选入库点的地图判定与几何判定完全一致")
    print(f"[map-check] 入库点可用 {usable}，不可用 {len(unusable)}"
          + (f"（例：{unusable[:3]}）" if unusable else ""))

    # E. layout.json 的几何字段不得与世界漂移
    #    layout 是"同一次生成的摘要"：它声称的货架块/墙/柱，在世界与地图上必须真的占据。
    #    它现在已经不参与"障碍物"判定（障碍物只从 SDF 派生），但仍是 b 巷/货位的几何来源；
    #    一旦世界改了而摘要没重建（或反之），这里会立刻暴露。
    def rect_occ_ratio(cx: float, cy: float, sx: float, sy: float) -> float:
        n = ok = 0
        step = max(0.1, m.res * 2)
        x = cx - sx / 2 + step / 2
        while x < cx + sx / 2:
            y = cy - sy / 2 + step / 2
            while y < cy + sy / 2:
                n += 1
                ok += 1 if m.at(x, y) == OCC else 0
                y += step
            x += step
        return ok / n if n else 0.0

    drift = []
    claim = []
    for b in layout.get("blocks", []):
        claim.append((f"货架块 {b['name']}",
                      (b["x0"] + b["x1"]) / 2, (b["y0"] + b["y1"]) / 2,
                      b["x1"] - b["x0"], b["y1"] - b["y0"]))
    for w in layout.get("walls", []):
        claim.append((f"墙 {w['name']}", w["center"][0], w["center"][1], w["size"][0], w["size"][1]))
    for c in layout.get("columns", []):
        claim.append((f"立柱 {c['name']}", c["center"][0], c["center"][1], c["size"][0], c["size"][1]))
    for label, cx, cy, sx, sy in claim:
        ratio = rect_occ_ratio(cx, cy, sx, sy)
        if ratio < 0.85:
            drift.append(f"{label} 占据率 {ratio:.0%}")
    if drift:
        print(f"FAIL E: layout.json 的几何与世界/地图不一致（{len(drift)} 处）：{drift[:5]}")
        print("        说明摘要与世界不同步 —— 重跑 make world 让两者一起重建。")
        bad += 1
    else:
        print(f"PASS E: layout.json 的 {len(claim)} 个几何声明（货架块/墙/立柱）"
              f"在世界与地图上都成立（占据率 ≥85%）")

    # D. M2 默认点必须可用（否则判据根本跑不通）
    hits = blockers_at(dock[0], dock[1], ROBOT_HALF_DIAG, args.sdf)
    if hits:
        print(f"FAIL D: M2 默认入库点 ({dock[0]:+.2f},{dock[1]:+.2f}) 被占用：{hits[:3]}")
        bad += 1
    else:
        print(f"PASS D: M2 默认入库点 ({dock[0]:+.2f},{dock[1]:+.2f}) 机器人可进")

    print("RESULT:", "PASS" if bad == 0 else f"FAIL（{bad} 项）")
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
