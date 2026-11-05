#!/usr/bin/env python3
"""世界几何的唯一解析入口（地图生成、点位校验、验收脚本共用）。

2026-11-05 恢复：本文件在重构提交 56ea3cd 的"清理一次性脚本"里被误删，但
`scripts/make_static_map.py` / `map_check.py` / `slam_metrics.py` 仍在 import 它
（`make world` / `make map-check` 因此直接 ModuleNotFoundError）。内容与删除前一致。

为什么要有这个模块
  踩过的坑：世界由 `config/world.yaml` 生成成 `world/warehouse.sdf`（含货架、立柱、墙、
  叉车、周转箱、护栏），而静态图当时只从 `world/layout.json` 里拿了货架 —— 于是
  "地图 ⊂ 世界"却没有任何地方会发现，M2 的入库点正好落在叉车体内，白白排查了两天。

  根治办法不是"记得把叉车也画上"，而是：
    * 地图必须从**已构建的世界模型**（SDF 里的静态碰撞体）派生，而不是从任何"清单"派生；
    * 验收点位必须用同一份几何做碰撞检查；
    * 两边共用这一个模块，保证口径一致。

坐标与单位：SDF 世界系，米。矩形用 AABB（旋转的 box 取旋转后的包围盒）。
"""
from __future__ import annotations

import math
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# 机器人外壳：0.52 × 0.46（SPEC §135），半对角线 ≈ 0.347 -> 取 0.35 作为"点位不能贴障碍"的余量
ROBOT_HALF_DIAG = 0.35
# 只关心落在机器人身体高度带内的碰撞体（地面机器人的可通行性）
ROBOT_BAND_Z = (0.02, 0.60)

_NAME = r"""name=['"]([^'"]+)['"]"""


def _floats(s: str) -> list[float]:
    return [float(v) for v in s.replace(",", " ").split()]


def _rpy_matrix(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _pose(block: str) -> tuple[float, float, float, float, float, float]:
    m = re.search(r"<pose[^>]*>([^<]+)</pose>", block)
    if not m:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    v = _floats(m.group(1))
    while len(v) < 6:
        v.append(0.0)
    return (v[0], v[1], v[2], v[3], v[4], v[5])


def _aabb_of_box(cx, cy, cz, sx, sy, sz, rpy) -> tuple[float, float, float, float, float, float]:
    """把可能旋转的长方体转成世界系 AABB。"""
    r = _rpy_matrix(*rpy)
    ex = ey = ez = 0.0
    for dx in (-sx / 2, sx / 2):
        for dy in (-sy / 2, sy / 2):
            for dz in (-sz / 2, sz / 2):
                wx = r[0][0] * dx + r[0][1] * dy + r[0][2] * dz
                wy = r[1][0] * dx + r[1][1] * dy + r[1][2] * dz
                wz = r[2][0] * dx + r[2][1] * dy + r[2][2] * dz
                ex, ey, ez = max(ex, abs(wx)), max(ey, abs(wy)), max(ez, abs(wz))
    return (cx - ex, cy - ey, cz - ez, cx + ex, cy + ey, cz + ez)


def static_collisions(sdf_path: Path | str | None = None,
                      z_band: tuple[float, float] = ROBOT_BAND_Z) -> list[dict]:
    """解析世界 SDF 里所有 static 模型的碰撞体，返回落在机器人高度带内的 AABB 列表。

    每项：{model, link, name, x0, y0, x1, y1, z0, z1, cx, cy, sx, sy}
    """
    path = Path(sdf_path) if sdf_path else ROOT / "world" / "warehouse.sdf"
    txt = path.read_text(encoding="utf-8")
    out: list[dict] = []
    for mm in re.finditer(r"<model\s+%s>" % _NAME, txt):
        model_name = mm.group(1)
        start = mm.end()
        end = txt.find("</model>", start)
        block = txt[start:end]
        if "<static>true</static>" not in block:
            continue
        mpose = _pose(block[: block.find("<link")] if "<link" in block else block)
        for lm in re.finditer(r"<link\s+%s>(.*?)</link>" % _NAME, block, re.S):
            link_name, lbody = lm.group(1), lm.group(2)
            lpose = _pose(lbody)
            for cm in re.finditer(r"<collision\s+%s>(.*?)</collision>" % _NAME, lbody, re.S):
                cname, cbody = cm.group(1), cm.group(2)
                cpose = _pose(cbody)
                mb = re.search(r"<box><size>([^<]+)</size></box>", cbody)
                mc = re.search(r"<cylinder><radius>([^<]+)</radius><length>([^<]+)</length>", cbody)
                if mb:
                    v = _floats(mb.group(1))
                    sx, sy, sz = (v + [0.0, 0.0, 0.0])[:3]
                elif mc:
                    r_, l_ = float(mc.group(1)), float(mc.group(2))
                    sx, sy, sz = 2 * r_, 2 * r_, l_
                else:
                    continue
                # 位置：模型位姿 + link 位姿 + collision 位姿（逐级累加平移，姿态直接取后者，够用）
                cx = mpose[0] + lpose[0] + cpose[0]
                cy = mpose[1] + lpose[1] + cpose[1]
                cz = mpose[2] + lpose[2] + cpose[2]
                rpy = (cpose[3] or lpose[3], cpose[4] or lpose[4], cpose[5] or lpose[5])
                x0, y0, z0, x1, y1, z1 = _aabb_of_box(cx, cy, cz, sx, sy, sz, rpy)
                if z1 < z_band[0] or z0 > z_band[1]:
                    continue
                out.append(dict(model=model_name, link=link_name, name=cname,
                                x0=x0, y0=y0, x1=x1, y1=y1, z0=z0, z1=z1,
                                cx=(x0 + x1) / 2, cy=(y0 + y1) / 2,
                                sx=x1 - x0, sy=y1 - y0))
    return out


def blockers_at(x: float, y: float, margin: float = ROBOT_HALF_DIAG,
                sdf_path: Path | str | None = None) -> list[str]:
    """返回把这个点（含机器人半径余量）挡住的静态碰撞体名字（"model/link/name"）。"""
    hits = []
    for r in static_collisions(sdf_path):
        if r["x1"] < x - margin or r["x0"] > x + margin:
            continue
        if r["y1"] < y - margin or r["y0"] > y + margin:
            continue
        hits.append(f'{r["model"]}/{r["link"]}/{r["name"]}')
    return hits


# ---------------------------------------------------------------- 点位（与 m2_side_dock 同口径）
def dock_geometry(layout: dict, aisle_name: str, bay: int, side: str):
    """巷道中线上的起点 + 侧移入库点。返回 (start, dock, lateral, dock_yaw)。"""
    aisle = next(a for a in layout["aisles"] if a["name"] == aisle_name)
    bay_obj = next(b for b in layout["bays"]
                   if b["aisle"] == aisle_name and b["bay"] == bay)
    aisle_w = float(layout["derived"]["aisle_clear_width"])
    dock_off = aisle_w / 2 - 0.70          # 距货架面 0.7 m 的观测位
    sign = 1.0 if side == "L" else -1.0
    start = (bay_obj["center_x"], aisle["center_y"], 0.0)
    dock = (bay_obj["center_x"], aisle["center_y"] + sign * dock_off, sign * math.pi / 2)
    return start, dock, abs(dock_off), dock[2]


# ---------------------------------------------------------------- 地图读取（供校验用）
def load_pgm(path: Path | str) -> tuple[list[int], int, int]:
    """读 P5 二进制 pgm，返回 (像素行优先 0..255, w, h)。"""
    data = Path(path).read_bytes()
    parts, idx = [], 0
    while len(parts) < 4:
        # 跳过注释行
        while data[idx:idx + 1] in (b"#",):
            idx = data.find(b"\n", idx) + 1
        nxt = idx
        while data[nxt:nxt + 1] not in (b" ", b"\n", b"\t"):
            nxt += 1
        parts.append(data[idx:nxt])
        idx = nxt + 1
    w, h = int(parts[1]), int(parts[2])
    return list(data[idx: idx + w * h]), w, h
