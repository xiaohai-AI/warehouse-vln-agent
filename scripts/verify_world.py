#!/usr/bin/env python3
"""世界自检 —— 判据 P1 的执行体。全部数字从 world/warehouse.sdf 反推，不信 config。

检查项
  A  结构   SDF 可解析 / 世界名 / 静态实体数量
  B  边界   所有实体在 footprint 内
  C  重叠   实体两两 AABB 不重叠（允许的接触用 eps 处理）
  D  巷道净宽 每条巷道 × 每个 bay 站位，从 SDF 几何量：3.20 ± 0.05，且左右对称
  E  横通道  两端横通道净宽 4.50 ± 0.10
  F  货位   slot_grid 与 SDF 一致（bay 数、slot 数、地址唯一、位姿在多边形内）
  G  真值   ground_truth/slots.json 与 slot_grid 一一对应；占用统计可复现
  H  语法   `ign sdf -k`（若装了 ign）

退出码 0 = 全部通过；非 0 = 有 FAIL（`make world` 会跟着失败）。
"""
from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from depot_vln.layout import compute_layout, load_config  # noqa: E402

TOL_AISLE = 0.05     # 判据：3.20 ± 0.05
TOL_SYMM = 0.02
TOL_CROSS = 0.10
EPS = 1e-4

SOLID_PREFIXES = ("wall_", "rack_", "column_", "barrier_", "obstacle_")
VISUAL_ONLY_PREFIXES = ("line_", "hazard_", "bc_", "cargo_")


class Checker:
    def __init__(self) -> None:
        self.fails: list[str] = []
        self.rows: list[tuple[str, bool, str]] = []

    def check(self, name: str, ok: bool, detail: str = "") -> bool:
        self.rows.append((name, ok, detail))
        if not ok:
            self.fails.append(f"{name}: {detail}")
        return ok

    def report(self) -> None:
        width = max(len(r[0]) for r in self.rows) + 2
        for name, ok, detail in self.rows:
            tag = "PASS" if ok else "FAIL"
            print(f"  [{tag}] {name:<{width}} {detail}")
        print()
        if self.fails:
            print(f"自检失败 {len(self.fails)} 项：")
            for f in self.fails:
                print(f"  - {f}")
        else:
            print(f"自检全部通过（{len(self.rows)} 项）")


# ---------------------------------------------------------------- SDF 解析
class Solid:
    __slots__ = ("name", "kind", "x0", "x1", "y0", "y1", "z0", "z1")

    def __init__(self, name: str, kind: str, cx, cy, cz, sx, sy, sz):
        self.name = name
        self.kind = kind
        self.x0, self.x1 = cx - sx / 2, cx + sx / 2
        self.y0, self.y1 = cy - sy / 2, cy + sy / 2
        self.z0, self.z1 = cz - sz / 2, cz + sz / 2

    def overlaps(self, o: "Solid", eps: float = EPS) -> bool:
        return (
            self.x0 < o.x1 - eps
            and o.x0 < self.x1 - eps
            and self.y0 < o.y1 - eps
            and o.y0 < self.y1 - eps
            and self.z0 < o.z1 - eps
            and o.z0 < self.z1 - eps
        )

    def __repr__(self) -> str:
        return f"<{self.name} x[{self.x0:.2f},{self.x1:.2f}] y[{self.y0:.2f},{self.y1:.2f}] z[{self.z0:.2f},{self.z1:.2f}]>"


def parse_sdf(path: Path) -> tuple[ET.Element, list[Solid], list[dict[str, Any]]]:
    tree = ET.parse(path)
    root = tree.getroot()
    world = root.find("world")
    solids: list[Solid] = []
    lights: list[dict[str, Any]] = []
    for model in world.findall("model"):
        mname = model.get("name", "?")
        for link in model.findall("link"):
            lname = link.get("name", "?")
            pose_el = link.find("pose")
            px = py = pz = 0.0
            if pose_el is not None and pose_el.text:
                vals = [float(v) for v in pose_el.text.split()]
                px, py, pz = vals[0], vals[1], vals[2]
            col = link.find("collision")
            if col is None:
                continue  # visual-only
            geom = col.find("geometry")
            if geom is None:
                continue
            box = geom.find("box")
            cyl = geom.find("cylinder")
            if box is not None:
                sx, sy, sz = [float(v) for v in box.find("size").text.split()]
            elif cyl is not None:
                r = float(cyl.find("radius").text)
                ln = float(cyl.find("length").text)
                sx = sy = 2 * r
                sz = ln
            else:
                continue
            kind = next((p for p in SOLID_PREFIXES if lname.startswith(p)), "other")
            solids.append(Solid(lname, kind, px, py, pz, sx, sy, sz))
    for light in world.findall("light"):
        lights.append({"name": light.get("name"), "type": light.get("type")})
    return root, solids, lights


# ---------------------------------------------------------------- 几何量测
def clearance_at(solids: list[Solid], x: float, y_c: float, zmin=0.05, zmax=2.2) -> tuple[float, float, str, str]:
    """在 x 处、以 y_c 为中心量左右净距（只算在该 x 处真实存在的实体）。"""
    down, up = math.inf, math.inf
    dname = uname = ""
    for s in solids:
        if s.kind not in ("rack_", "wall_", "column_", "barrier_"):
            continue
        if not (s.x0 - EPS <= x <= s.x1 + EPS):
            continue
        if s.z1 < zmin or s.z0 > zmax:
            continue
        if s.y1 <= y_c + EPS:  # 在中心下方
            g = y_c - s.y1
            if g < down:
                down, dname = g, s.name
        elif s.y0 >= y_c - EPS:  # 在中心上方
            g = s.y0 - y_c
            if g < up:
                up, uname = g, s.name
    return down, up, dname, uname


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(ROOT / "config" / "world.yaml"))
    ap.add_argument("--world", default=str(ROOT / "world" / "warehouse.sdf"))
    ap.add_argument("--slot-grid", default=str(ROOT / "world" / "slot_grid.json"))
    ap.add_argument("--gt", default=str(ROOT / "ground_truth" / "slots.json"))
    ap.add_argument("--no-ign", action="store_true", help="跳过 ign sdf -k 语法检查")
    args = ap.parse_args()

    cfg = load_config(args.config)
    layout = compute_layout(cfg)
    der = layout["derived"]
    ck = Checker()

    print(f"== 世界自检 {Path(args.world).name} ==")

    # --- A 结构
    world_el, solids, lights = parse_sdf(Path(args.world))
    n_models = len(world_el.findall("world/model")) if world_el.tag == "sdf" else len(world_el.findall("model"))
    kinds: dict[str, int] = {}
    for s in solids:
        kinds[s.kind] = kinds.get(s.kind, 0) + 1
    expected_decks = len(layout["blocks"]) * der["bays_per_aisle"] * len(cfg["racks"]["levels"])
    expected_walls = 4
    ck.check("A1 SDF 解析", True, f"{n_models} 个 model，{len(solids)} 个碰撞体")
    ck.check(
        "A2 外墙数",
        kinds.get("wall_", 0) == expected_walls,
        f"实测 {kinds.get('wall_', 0)} / 期望 {expected_walls}",
    )
    ck.check(
        "A3 层板数",
        kinds.get("rack_", 0) >= expected_decks,
        f"rack 实体 {kinds.get('rack_', 0)} ≥ 层板 {expected_decks}（含立柱）",
    )
    ck.check(
        "A4 照明",
        len(lights) >= der["n_aisles"],
        f"{len(lights)} 盏（directional {sum(1 for l in lights if l['type'] == 'directional')}，"
        f"spot {sum(1 for l in lights if l['type'] == 'spot')}）",
    )
    # A4b 封堵路障的高度必须"挡车不挡雷达"（2026-10-29 的教训）
    # 原来 1.14 m 高：雷达装在 0.474 m，被封堵段后面的货架一起看不见，
    # "车进不去的段"顺带成了"感知不到的段"，SEG-1 永远少 7 个 bay。
    # 判据：顶面 ≥0.30 m（否则 /scan 的 z 0.10–0.60 切带里没有它，Nav2 会穿过去）
    #       且 ≤0.42 m（低于雷达 0.474 m 才看得过去）。
    LIDAR_Z, SCAN_TOP = 0.474, 0.60
    barr = [s for s in solids if s.kind.startswith("barrier_")]
    if barr:
        # 按"每一道护栏"（同一 x）取最高碰撞体：底座 0.10 m 那个不算数，
        # /scan 靠的是主体（0.02–0.32 m，落在 z 0.10–0.60 切带里）。
        per_station: dict[float, float] = {}
        for s in barr:
            key = round(0.5 * (s.x0 + s.x1), 2)
            per_station[key] = max(per_station.get(key, 0.0), s.z1)
        tops = sorted(per_station.values())
        ck.check(
            "A4b 路障高度：挡车不挡雷达",
            all(0.30 <= t <= 0.42 for t in tops),
            f"{len(per_station)} 道护栏（{len(barr)} 个碰撞体），各自最高顶面 "
            f"{tops} m（需 ≥0.30 才能被 /scan 的 z≤{SCAN_TOP} 切带看到，"
            f"需 ≤0.42 才低于雷达 {LIDAR_Z} m、让后面货架可观测）",
        )
    # A5 静态性：SDF 里 static 是**元素**不是属性。写成 static="true" 属性时 gz 只 warning、
    # 然后把这些 model 当动态刚体——货架会倒、墙会被撞开，而画面一切正常。踩过，故设为硬检查。
    world_node = world_el.find("world") if world_el.tag == "sdf" else world_el
    models_el = world_node.findall("model") if world_node is not None else []
    non_static = [
        m.get("name") for m in models_el
        if (m.findtext("static") or "").strip().lower() not in ("true", "1")
    ]
    ck.check(
        "A5 全部实体 static",
        not non_static,
        f"{len(models_el) - len(non_static)}/{len(models_el)} 静态"
        + ("" if not non_static else f"，非静态：{non_static[:5]}"),
    )

    # --- B 边界
    fp = layout["footprint"]
    oob = [
        s
        for s in solids
        if s.x0 < fp["x_min"] - EPS
        or s.x1 > fp["x_max"] + EPS
        or s.y0 < fp["y_min"] - EPS
        or s.y1 > fp["y_max"] + EPS
        or s.z1 > fp["z_max"] + EPS
    ]
    ck.check(
        "B1 越界实体",
        not oob,
        f"{len(oob)} 个越界" + ("" if not oob else f"，首个 {oob[0]}"),
    )

    # --- C 重叠（结构与结构）
    struct = [s for s in solids if s.kind in ("wall_", "rack_", "column_", "barrier_")]
    overlaps: list[str] = []
    for i in range(len(struct)):
        for j in range(i + 1, len(struct)):
            a, b = struct[i], struct[j]
            # 真实货架里横梁是螺栓固定在立柱上的，两者几何上必然交叠 -> 这一对豁免
            if {a.kind, b.kind} == {"rack_"} and "beam" in a.name + b.name and "post" in a.name + b.name:
                continue
            if a.overlaps(b):
                overlaps.append(f"{a.name} × {b.name}")
    ck.check(
        "C1 结构重叠",
        not overlaps,
        f"{len(overlaps)} 对重叠" + ("" if not overlaps else f"，例：{overlaps[:3]}"),
    )

    # --- D 巷道净宽（逐巷 × 逐 bay 量测）
    widths: list[float] = []
    sym_err: float = 0.0
    worst = None
    for aisle in layout["aisles"]:
        for st in [b for b in layout["bays"] if b["aisle"] == aisle["name"]]:
            down, up, dn, un = clearance_at(solids, st["center_x"], aisle["center_y"])
            if math.isinf(down) or math.isinf(up):
                widths.append(math.inf)
                continue
            w = down + up
            widths.append(w)
            e = abs(down - up)
            if e > sym_err:
                sym_err = e
            if worst is None or abs(w - der["aisle_clear_width"]) > abs(worst[0] - der["aisle_clear_width"]):
                worst = (w, aisle["name"], st["bay"], dn, un)
    finite = [w for w in widths if math.isfinite(w)]
    wmin, wmax = (min(finite), max(finite)) if finite else (math.nan, math.nan)
    wmean = sum(finite) / len(finite) if finite else math.nan
    ck.check(
        "D1 巷道净宽",
        bool(finite) and abs(wmean - der["aisle_clear_width"]) <= TOL_AISLE and (wmax - wmin) <= TOL_AISLE,
        f"目标 {der['aisle_clear_width']:.2f}±{TOL_AISLE}  min {wmin:.3f} / mean {wmean:.3f} / max {wmax:.3f} m"
        f"（{len(finite)} 个站位；最差 {worst[1]}-{worst[2]:02d} = {worst[0]:.3f}）"
        if finite
        else "有站位量不到净距",
    )
    ck.check(
        "D2 左右对称",
        sym_err <= TOL_SYMM,
        f"max |左-右| = {sym_err * 1000:.1f} mm（容差 {TOL_SYMM * 1000:.0f} mm）",
    )

    # --- E 横通道净宽
    # 端部横通道 = 外墙内表面 到 货架最外端面。端面位于 block 的 y 带上（不在巷道中线上），
    # 所以量测时取"与该巷道相邻的两个 block 的 y 带"里所有 rack 实体的最外 X。
    cross_ws = []
    rack_d = der["rack_depth"]
    for aisle in layout["aisles"]:
        band0 = aisle["y0"] - rack_d
        band1 = aisle["y1"] + rack_d
        for sign in (-1, 1):
            wall_x = None
            rack_x = None
            for s in solids:
                if s.z1 < 0.05 or s.z0 > 2.2:
                    continue
                if s.kind == "wall_":
                    # 只取端墙（东西墙）在巷道中线高度上的内表面
                    if not (s.y0 - EPS <= aisle["center_y"] <= s.y1 + EPS):
                        continue
                    if sign < 0 and s.x0 < 0:
                        wall_x = s.x1
                    elif sign > 0 and s.x1 > 0:
                        wall_x = s.x0
                elif s.kind == "rack_":
                    # 端面在"与巷道相邻的两个 block"的 y 带上，取该侧最外的 X
                    if s.y1 < band0 - EPS or s.y0 > band1 + EPS:
                        continue
                    v = s.x0 if sign < 0 else s.x1
                    rack_x = v if rack_x is None else (min(rack_x, v) if sign < 0 else max(rack_x, v))
            if wall_x is None or rack_x is None:
                cross_ws.append(math.inf)
            else:
                cross_ws.append(abs(wall_x - rack_x))
    cw_finite = [w for w in cross_ws if math.isfinite(w)]
    ck.check(
        "E1 横通道净宽",
        bool(cw_finite) and all(abs(w - float(cfg["cross_aisles"]["width"])) <= TOL_CROSS for w in cw_finite),
        f"目标 {float(cfg['cross_aisles']['width']):.2f}±{TOL_CROSS}  "
        + " / ".join(f"{w:.3f}" for w in cw_finite),
    )

    # --- F slot_grid 一致性
    grid = json.loads(Path(args.slot_grid).read_text(encoding="utf-8"))
    slots = grid["slots"]
    n_expected_slots = der["n_aisles"] * der["bays_per_aisle"] * 2 * len(cfg["racks"]["levels"])
    ck.check(
        "F1 slot 数",
        len(slots) == n_expected_slots,
        f"实测 {len(slots)} / 期望 {n_expected_slots}"
        f"（{der['n_aisles']} 巷 × {der['bays_per_aisle']} bay × 2 侧 × {len(cfg['racks']['levels'])} 层）",
    )
    def _inside(x: float, y: float, poly: list) -> bool:
        xs = [p[0] for p in poly]
        ys = [p[1] for p in poly]
        return min(xs) - EPS <= x <= max(xs) + EPS and min(ys) - EPS <= y <= max(ys) + EPS

    bad_pose = [a for a, s in slots.items() if not _inside(s["pose"]["x"], s["pose"]["y"], s["polygon"])]
    ck.check(
        "F2 位姿在多边形内",
        not bad_pose,
        f"{len(bad_pose)} 个 slot 位姿越出 bay 多边形" + ("" if not bad_pose else f"，例 {bad_pose[:3]}"),
    )
    bad_addr = []
    for a in slots:
        try:
            aisle, side, bay, level = a.split("-")
            assert side in ("L", "R") and 1 <= int(bay) <= der["bays_per_aisle"] and int(level) >= 1
        except Exception:
            bad_addr.append(a)
    ck.check("F3 地址格式", not bad_addr, f"{len(bad_addr)} 个非法地址" + ("" if not bad_addr else f"，例 {bad_addr[:3]}"))
    n_bays_grid = len(grid["bays"])
    ck.check(
        "F4 bay 数",
        n_bays_grid == der["n_bays"],
        f"实测 {n_bays_grid} / 期望 {der['n_bays']}（{der['n_aisles']} 巷 × {der['bays_per_aisle']}）",
    )
    # 每个 bay 正好 2 侧 × 层数 个 slot
    per_bay = {}
    for a, s in slots.items():
        per_bay[(s["aisle"], s["bay"])] = per_bay.get((s["aisle"], s["bay"]), 0) + 1
    bad_bay = {k: v for k, v in per_bay.items() if v != 2 * len(cfg["racks"]["levels"])}
    ck.check(
        "F5 每 bay 槽位数",
        not bad_bay,
        f"{len(bad_bay)} 个 bay 槽位数异常" + ("" if not bad_bay else f"，例 {list(bad_bay.items())[:3]}"),
    )

    # --- G 真值一致性
    gt = json.loads(Path(args.gt).read_text(encoding="utf-8"))
    gt_slots = gt["slots"]

    # --- G0 货位几何不得重合（2026-10-29 的教训：两巷货位共用整块多边形）
    # 原来两个相邻巷道的货位都取货架块中心、多边形取整块 -> 576 个逻辑货位只对应 168 个物理位置，
    # 118 组里同一块空间有两个货位同时"占用"：从 A 巷能看见 B 巷摆在块中间的货，
    # 感知判据的答案键本身自相矛盾（SEG-2 因此无法评分）。
    groups: dict[tuple, list] = {}
    for a, s in slots.items():
        groups.setdefault(tuple(tuple(p) for p in s["polygon"]), []).append(a)
    big = {k: v for k, v in groups.items() if len(v) > len(cfg["racks"]["levels"])}
    ck.check(
        "G0 货位物理空间唯一",
        not big,
        f"{len(slots)} 个货位 -> {len(groups)} 个物理多边形"
        f"（每 Polygon 最多 {len(cfg['racks']['levels'])} 个 = 上下层共用 footprint）"
        + ("" if not big else f"，{len(big)} 组超员，例 {[v for _, v in list(big.items())[:2]]}"),
    )
    conf = []
    for k, addrs in groups.items():
        by_lvl: dict[int, list] = {}
        for a in addrs:
            by_lvl.setdefault(slots[a]["level"], []).append(a)
        for lvl, same in by_lvl.items():
            occ = [a for a in same if gt_slots.get(a, {}).get("occupied")]
            if len(occ) > 1:
                conf.append((lvl, occ))
    ck.check(
        "G0b 同层同空间不重复占用",
        not conf,
        f"{len(conf)} 组自相矛盾" + ("" if not conf else f"，例 {conf[:2]}"),
    )
    ck.check(
        "G1 真值覆盖面",
        set(gt_slots) == set(slots),
        f"真值 {len(gt_slots)} / 网格 {len(slots)}"
        f"（差集 {len(set(gt_slots) ^ set(slots))}）",
    )
    occ = sum(1 for s in gt_slots.values() if s["occupied"])
    ck.check(
        "G2 占用率可复现",
        gt["stats"]["occupied"] == occ,
        f"occupied {occ} / {len(gt_slots)} = {occ / max(1, len(gt_slots)):.1%}"
        f"（期望 p={cfg['semantics']['occupancy_prob']}）",
    )
    cls_set = set(cfg["semantics"]["classes"])
    unseen = {s["class"] for s in gt_slots.values() if s["occupied"]} - cls_set
    ck.check("G3 类别闭集", not unseen, f"越界类别 {sorted(unseen) if unseen else '无'}")

    # --- H SDF 语法
    ign = shutil.which("ign") or shutil.which("gz")
    if args.no_ign or not ign:
        ck.check("H1 SDF 语法", True, "跳过（--no-ign 或未安装 ign/gz）")
    else:
        proc = subprocess.run([ign, "sdf", "-k", str(args.world)], capture_output=True, text=True)
        detail = (proc.stderr or proc.stdout).strip().splitlines()
        ck.check(
            "H1 SDF 语法",
            proc.returncode == 0,
            f"`{Path(ign).name} sdf -k` rc={proc.returncode}"
            + (f" {detail[-1][:120]}" if detail and proc.returncode != 0 else " 无错误"),
        )

    # --- 汇总数字（报告用）
    print()
    print("关键数字：")
    print(f"  footprint            {fp['x_max'] - fp['x_min']:.2f} × {fp['y_max'] - fp['y_min']:.2f} m")
    print(f"  巷道净宽 mean        {wmean:.3f} m   (min {wmin:.3f} / max {wmax:.3f})")
    print(f"  左右不对称 max       {sym_err * 1000:.1f} mm")
    print(f"  横通道净宽           {' / '.join(f'{w:.3f}' for w in cw_finite)} m")
    print(f"  bay / slot           {n_bays_grid} / {len(slots)}")
    print(f"  占用 slot            {occ} ({occ / max(1, len(gt_slots)):.1%})")
    print(f"  静态碰撞体           {len(solids)}（rack {kinds.get('rack_', 0)} / wall {kinds.get('wall_', 0)}"
          f" / column {kinds.get('column_', 0)} / barrier {kinds.get('barrier_', 0)}"
          f" / obstacle {kinds.get('obstacle_', 0)}）")
    print()
    ck.report()
    print()
    print("RESULT:", "FAIL" if ck.fails else "PASS")
    return 1 if ck.fails else 0


if __name__ == "__main__":
    raise SystemExit(main())
