"""depot-vln 布局几何（唯一来源）。

纯 Python，不依赖 ROS。生成器 (scripts/build_world.py) 与自检 (scripts/verify_world.py)
都调用这里，所以"SDF 里的数字"和"slot_grid 里的数字"不可能对不上。

坐标约定
  +X  = 巷道前进方向（A–F 六条巷道都朝 +X）
  侧面 = 面朝 +X 时的左/右；左 = +Y，右 = -Y
  bay 沿 X 每 bay_pitch 一个；货架 block 沿 Y 排布（6 条巷道 => 7 个 block）
"""
from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]

LEVEL_SIDES = ("L", "R")


# ---------------------------------------------------------------- config
def load_config(path: str | Path | None = None) -> dict[str, Any]:
    path = Path(path) if path else ROOT / "config" / "world.yaml"
    with open(path, "r", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    cfg["_path"] = str(path)
    return cfg


# ---------------------------------------------------------------- helpers
def address(aisle: str, side: str, bay: int, level: int) -> str:
    return f"{aisle}-{side}-{bay:02d}-{level}"


def parse_address(addr: str) -> tuple[str, str, int, int]:
    aisle, side, bay, level = addr.split("-")
    return aisle, side.upper(), int(bay), int(level)


def slot_yaw(side: str) -> float:
    """货位朝向：朝向巷道内部（L 在 +Y 侧 -> 面朝 -Y）。"""
    return -math.pi / 2 if side == "L" else math.pi / 2


def obs_yaw(side: str) -> float:
    """观测位朝向：站在巷道中线看向该侧货架。"""
    return math.pi / 2 if side == "L" else -math.pi / 2


# ---------------------------------------------------------------- layout
def compute_layout(cfg: dict[str, Any]) -> dict[str, Any]:
    racks = cfg["racks"]
    walls_cfg = cfg["walls"]
    xa = cfg["cross_aisles"]
    mk = cfg["markings"]
    lt = cfg["lights"]

    aisles_names: list[str] = list(racks["aisle_names"])
    n_aisles = len(aisles_names)
    aisle_w = float(racks["aisle_clear_width"])
    rack_d = float(racks["rack_depth"])
    bays_n = int(racks["bays_per_aisle"])
    pitch = float(racks["bay_pitch"])
    wt = float(walls_cfg["thickness"])
    wall_h = float(walls_cfg["height"])
    cross_w = float(xa["width"])

    # --- X 方向：货架运行段 + 两端横通道 + 两道墙
    rack_run = bays_n * pitch
    footprint_x = rack_run + 2 * cross_w + 2 * wt
    footprint_y = float(cfg["footprint_y"]) if "footprint_y" in cfg else 40.0
    x_rack0, x_rack1 = -rack_run / 2, rack_run / 2

    # --- Y 方向：块(货架) / 巷道 交替
    rack_total_y = n_aisles * aisle_w + (n_aisles + 1) * rack_d
    if rack_total_y >= footprint_y - 2 * wt:
        raise ValueError(
            f"货架区 {rack_total_y:.2f} m 放不进 {footprint_y:.2f} m 的 Y 方向占地"
        )
    block_pitch = rack_d + aisle_w
    y_block0 = -rack_total_y / 2 + rack_d / 2

    blocks: list[dict[str, Any]] = []
    for k in range(n_aisles + 1):
        cy = y_block0 + k * block_pitch
        blocks.append(
            {
                "name": f"rack_block_{k}",
                "index": k,
                "center_y": round(cy, 6),
                "y0": round(cy - rack_d / 2, 6),
                "y1": round(cy + rack_d / 2, 6),
                "x0": round(x_rack0, 6),
                "x1": round(x_rack1, 6),
            }
        )

    aisles: list[dict[str, Any]] = []
    for i, name in enumerate(aisles_names):
        cy = y_block0 + rack_d / 2 + aisle_w / 2 + i * block_pitch
        aisles.append(
            {
                "name": name,
                "index": i,
                "center_y": round(cy, 6),
                "y0": round(cy - aisle_w / 2, 6),
                "y1": round(cy + aisle_w / 2, 6),
                "x0": round(x_rack0, 6),
                "x1": round(x_rack1, 6),
                "width": aisle_w,
            }
        )

    # --- bays 与 slots
    bays: list[dict[str, Any]] = []
    slots: list[dict[str, Any]] = []
    for aisle in aisles:
        for b in range(1, bays_n + 1):
            bx0 = x_rack0 + (b - 1) * pitch
            bx1 = bx0 + pitch
            bxc = (bx0 + bx1) / 2
            bay_rec = {
                "aisle": aisle["name"],
                "bay": b,
                "x0": round(bx0, 6),
                "x1": round(bx1, 6),
                "center_x": round(bxc, 6),
                "aisle_center_y": aisle["center_y"],
                "slots": [],
            }
            for side in LEVEL_SIDES:
                sy_block = aisle["center_y"] + (aisle_w / 2 + rack_d / 2) * (1 if side == "L" else -1)
                # 半深货位：一个货架块只有 1.1 m 厚，两侧巷道共用的其实只有块中心那条线。
                # 原来两侧货位都取块中心当 pose.y、多边形都取整块 -> 两巷货位几何完全重合
                # （实测 576 个逻辑货位只对应 168 个物理位置，118 组真值自相矛盾：
                #  同一块空间里两个货位同时"占用"，从 A 巷能看见 B 巷的货）。
                # 现在每侧只占**朝自己巷道的那一半**：面 -> 块中心。
                inward = -1.0 if aisle["center_y"] < sy_block else 1.0   # 从块中心指向本侧巷道的方向
                sy = sy_block + inward * rack_d / 4                       # 半深中心
                y_face = sy_block + inward * rack_d / 2                   # 本侧货架面
                y_mid = sy_block
                for lvl in racks["levels"]:
                    lvl_no = int(lvl["level"])
                    addr = address(aisle["name"], side, b, lvl_no)
                    center_z = float(lvl["deck_z"]) + 0.30
                    rec = {
                        "address": addr,
                        "aisle": aisle["name"],
                        "side": side,
                        "bay": b,
                        "level": lvl_no,
                        "pose": {
                            "x": round(bxc, 6),
                            "y": round(sy, 6),
                            "z": round(center_z, 6),
                            "yaw": round(slot_yaw(side), 6),
                        },
                        "deck_z": float(lvl["deck_z"]),
                        "clear_height": float(lvl["clear_height"]),
                        "slot_depth": round(rack_d / 2, 6),
                        "polygon": [
                            [round(bx0, 6), round(min(y_face, y_mid), 6)],
                            [round(bx1, 6), round(min(y_face, y_mid), 6)],
                            [round(bx1, 6), round(max(y_face, y_mid), 6)],
                            [round(bx0, 6), round(max(y_face, y_mid), 6)],
                        ],
                        "obs_pose": {
                            "x": round(bxc, 6),
                            "y": aisle["center_y"],
                            "z": 0.0,
                            "yaw": round(obs_yaw(side), 6),
                        },
                        # 观测位到货位面的距离：语义错格率的几何上限依据
                        "obs_distance": round(aisle_w / 2 + rack_d / 2, 6),
                    }
                    slots.append(rec)
                    bay_rec["slots"].append(addr)
            bays.append(bay_rec)

    # --- 外墙（内表面正好落在 ±(footprint/2 - wt)）
    wx = footprint_x / 2 - wt / 2
    wy = footprint_y / 2 - wt / 2
    walls = [
        {"name": "wall_south", "center": [0.0, -wy, wall_h / 2], "size": [footprint_x, wt, wall_h]},
        {"name": "wall_north", "center": [0.0, wy, wall_h / 2], "size": [footprint_x, wt, wall_h]},
        # 东西墙夹在南北墙之间，避免角部 AABB 重叠（自检 C1 会查）
        {"name": "wall_west", "center": [-wx, 0.0, wall_h / 2], "size": [wt, footprint_y - 2 * wt, wall_h]},
        {"name": "wall_east", "center": [wx, 0.0, wall_h / 2], "size": [wt, footprint_y - 2 * wt, wall_h]},
    ]

    # --- 承重柱（只沿两侧外墙，绝不进入巷道）
    col = cfg["columns"]
    cols: list[dict[str, Any]] = []
    cs = float(col["size"])
    y_col = rack_total_y / 2 + float(col["offset_from_wall"])
    half = footprint_x / 2 - wt
    n = int(half * 2 // float(col["spacing"])) + 1
    xs = [round(-((n - 1) / 2) * float(col["spacing"]) + i * float(col["spacing"]), 6) for i in range(n)]
    for sx in (-1, 1):
        for x in xs:
            if abs(x) > half - cs:
                continue
            cols.append(
                {
                    "name": f"column_{'N' if sx > 0 else 'S'}_{len(cols)}",
                    "center": [x, sx * y_col, wall_h / 2],
                    "size": [cs, cs, wall_h],
                }
            )

    # --- 地面标线（视觉纹理，无碰撞）
    marks: list[dict[str, Any]] = []
    clw = float(mk["center_line_width"])
    for aisle in aisles:
        marks.append(
            {
                "name": f"line_{aisle['name']}",
                "center": [0.0, aisle["center_y"], 0.004],
                "size": [rack_run, clw, 0.008],
                "color": [0.86, 0.72, 0.10],
            }
        )
        for sx in (-1, 1):
            marks.append(
                {
                    "name": f"hazard_{aisle['name']}_{'E' if sx > 0 else 'W'}",
                    "center": [sx * (rack_run / 2 + float(mk["rack_end_zone_width"]) / 2), aisle["center_y"], 0.004],
                    "size": [float(mk["rack_end_zone_width"]), aisle_w + 0.4, 0.008],
                    "color": [0.85, 0.70, 0.08],
                }
            )
    if mk.get("bay_barcode", True):
        stripe = float(mk["barcode_stripe"])
        gap = float(mk["barcode_gap"])
        for bay in bays:
            aisle_y = bay["aisle_center_y"]
            nbits = max(5, int(math.ceil(math.log2(bays_n + 1))))
            span = nbits * (stripe + gap)
            x_start = bay["center_x"] - span / 2
            for bit in range(nbits):
                if not (bay["bay"] >> bit) & 1:
                    continue  # 0 位不画 -> 条纹模式编码 bay 号，给视觉提供唯一纹理
                marks.append(
                    {
                        "name": f"bc_{bay['aisle']}_{bay['bay']}_{bit}",
                        "center": [x_start + bit * (stripe + gap) + stripe / 2, aisle_y, 0.004],
                        "size": [stripe, 0.90, 0.008],
                        "color": [0.10, 0.10, 0.12],
                    }
                )

    # --- 照明：均匀照度（视觉 SLAM 需要稳定光照与纹理）
    lights: list[dict[str, Any]] = []
    lamp = lt["aisle_lamp"]
    # 灯数由配置决定（默认每巷 1 盏）。灯在 ogre2 里是实打实的渲染成本：
    # 每盏投影 spot 灯各要一张阴影贴图，堆到 18 盏会让 Sensors 渲染线程初始化卡死。
    n_lamp = max(0, int(lamp.get("per_aisle", 1)))  # 0 = 只靠环境光 + 方向光
    for aisle in aisles:
        for i in range(n_lamp):
            x = -rack_run / 2 + (i + 0.5) * (rack_run / n_lamp)
            lights.append(
                {
                    "name": f"lamp_{aisle['name']}_{i}",
                    "pose": [x, aisle["center_y"], float(lamp["z"])],
                    "intensity": float(lamp["intensity"]),
                    "range": float(lamp["range"]),
                    "direction": [0.0, 0.0, -1.0],
                }
            )

    return {
        "aisles": aisles,
        "blocks": blocks,
        "bays": bays,
        "slots": slots,
        "walls": walls,
        "columns": cols,
        "markings": marks,
        "lights": lights,
        "footprint": {
            "x_min": -footprint_x / 2,
            "x_max": footprint_x / 2,
            "y_min": -footprint_y / 2,
            "y_max": footprint_y / 2,
            "z_max": wall_h,
        },
        "derived": {
            "rack_run": rack_run,
            "rack_total_y": rack_total_y,
            "footprint_x": footprint_x,
            "footprint_y": footprint_y,
            "aisle_clear_width": aisle_w,
            "rack_depth": rack_d,
            "cross_aisle_width": cross_w,
            "wall_thickness": wt,
            "block_pitch": block_pitch,
            "n_aisles": n_aisles,
            "bays_per_aisle": bays_n,
            "n_bays": len(bays),
            "n_slots": len(slots),
        },
        "seed": int(cfg.get("seed", 0)),
    }


if __name__ == "__main__":  # 快速自检
    lay = compute_layout(load_config())
    d = lay["derived"]
    print(f"footprint {d['footprint_x']:.2f} x {d['footprint_y']:.2f} m")
    print(f"bays {d['n_bays']}  slots {d['n_slots']}")
    print(f"rack_total_y {d['rack_total_y']:.2f} m")
