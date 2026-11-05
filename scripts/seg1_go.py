#!/usr/bin/env python3
"""世界系安全转场：把车开到 (tx,ty)，路径**只走横通道 + 目标巷道**，绝不斜穿货架。

为什么必须单独做路由：`drive_to_point` 只会走直线。从"东侧横通道 y=-6.45"直接去
"西侧横通道 y=-10.75"，直线会横切 A/B 两组货架。之前能跑通只是因为起点恰好对，
换个起点就撞；而"换个起点"在分批采集里是**常态**（上一批收尾在哪、中途有没有失败）。

路由规则（每条腿都是一维移动，天然安全）：
  1. 车在巷道里（|x|<30）→ 先沿本巷开到同侧横通道（只动 x）；
  2. 换到目标 y（在横通道里只动 y）；
  3. 横向开到目标点（此时 y 已是目标巷道的中线，这一腿沿着巷道走）。

坐标常量全部来自 `depot_vln/catalog.py` 与 `config/nav.yaml`：
  巷道中线 = 巷道中心 + 车道偏移（capture_lane_offsets / lane_offsets）
  横通道 X = |出生点 X|
  护栏位置  = blocked 区间派生的 barrier_xs

用法（CLI）
  .venv/bin/python scripts/seg1_go.py --x -28.5 --y -6.45
  .venv/bin/python scripts/seg1_go.py --x 28.5 --y 10.75 --speed 0.6 --dry-run

用法（import）
  from seg1_go import safe_route
  safe_route(poller, -28.5, -10.75, speed=0.4)
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from drive_to import drive_to_point  # noqa: E402
from slam_route import TruthPoller  # noqa: E402
from depot_vln.catalog import get_catalog  # noqa: E402

_CAT = get_catalog()


def _offsets(kind: str) -> dict[str, float]:
    import yaml

    cfg = yaml.safe_load((ROOT / "config" / "nav.yaml").read_text(encoding="utf-8")) or {}
    return {str(k).upper(): float(v) for k, v in (cfg.get(kind) or {}).items()}


CAPTURE_OFFSETS = _offsets("capture_lane_offsets")
NAV_OFFSETS = _offsets("lane_offsets")


def aisle_lines(offsets: dict[str, float] | None = None) -> list[float]:
    """可行驶的巷道中线 y（巷道中心 + 偏移）。采集用 CAPTURE_OFFSETS，导航用 NAV_OFFSETS。"""
    off = CAPTURE_OFFSETS if offsets is None else offsets
    return [round(float(a["center_y"]) + off.get(str(a["name"]).upper(), 0.0), 3)
            for a in _CAT.layout["aisles"]]


def barrier_x(aisle: str) -> tuple[float, float]:
    """某条巷两道护栏的 X（没有封堵就返回一个不会触发的区间）。"""
    seg = next((s for s in _CAT.blocked_segments()
                if s["aisle"] == aisle and s.get("barrier_xs")), None)
    if not seg:
        return (1e9, 1e9)
    xs = sorted(float(v) for v in seg["barrier_xs"])
    return (xs[0], xs[-1])


AISLE_LINES = aisle_lines()
CROSS_X = abs(float(_CAT.spawn["x"]))          # 横通道中线 = 出生点 X
BARRIER_X = barrier_x("F")                     # 兼容旧调用点


def _pick_side(px: float, py: float, lines: list[float]) -> float:
    """选走哪一侧的横通道。

    **不能只看 x 的正负**：F 巷被两道横跨全宽的护栏切断，车停在护栏以西（x=10.6）时
    按"x>0 往东"会一路顶在护栏上（实测 420 s 超时）。
    """
    for aisle in _CAT.aisles:
        seg = next((s for s in _CAT.blocked_segments()
                    if s["aisle"] == aisle and s.get("barrier_xs")), None)
        if seg is None:
            continue
        line = float(next(a["center_y"] for a in _CAT.layout["aisles"] if a["name"] == aisle))
        if abs(float(py) - line) > 1.6:
            continue
        lo, hi = sorted(float(v) for v in seg["barrier_xs"])
        if px < lo:
            return -CROSS_X
        east = float(_CAT.layout["derived"]["rack_run"]) / 2 + 1.0
        if px > east:
            return CROSS_X
        # 中段（两道护栏之间，正常开不进去）：去更近的一侧
        return -CROSS_X if (px - lo) < (east - px) else CROSS_X
    return -CROSS_X if px <= 0 else CROSS_X


def safe_route(poller, tx: float, ty: float, *, speed: float = 0.6, tol: float = 0.25,
               max_seconds: float = 420.0, dry_run: bool = False,
               lines: list[float] | None = None) -> bool:
    """把车开到 (tx, ty)，只走横通道 + 目标巷道。返回是否到位。"""
    lines = AISLE_LINES if lines is None else lines
    p = poller.get()
    if p is None:
        print("[go] 读不到真值位姿 —— 仿真没起？")
        return False
    x, y = p[0], p[1]
    print(f"[go] 起点 ({x:+.2f},{y:+.2f})  目标 ({tx:+.2f},{ty:+.2f})")

    near = min(abs(ty - v) for v in lines)
    if near > 0.6:
        print(f"[go] ⚠️ 目标 y={ty:+.2f} 不在任何巷道中线上（最近差 {near:.2f} m）："
              f"最后一腿可能斜穿货架，请确认这是横通道内的点")

    side = _pick_side(x, y, lines)
    # 已经在目标巷道同一条中线上：直接沿巷走，不必先绕到横通道再折回来。
    # （例：车停在 B 巷 x=+14.3，目标是 B 巷西口 —— 直接向西 43 m，绕东横通道要 76 m。）
    if abs(y - ty) < 0.60 and abs(x - tx) > 0.20:
        print(f"[go]   同巷直行: -> ({tx:+.2f},{ty:+.2f})  {abs(tx - x):.1f} m")
        if dry_run:
            return True
        ok = drive_to_point(poller, tx, ty, speed, tol, max_seconds)
        if not ok:
            print(f"[go] 未到 ({tx:+.2f},{ty:+.2f})，中止")
        return ok
    legs: list[tuple[float, float, str]] = []
    # 第一腿一律是"把 x 拉到横通道中线"的纯一维移动。
    # 原判据是 `abs(x) < 30`，于是车停在 x=-33.06（横通道里、但比中线更靠墙）时
    # 第一腿被跳过 -> 直接出一条**斜线**，转向时车尾顶住西墙，实测卡了 18 分钟才蹭出来。
    if abs(x - side) > 0.30:
        legs.append((side, y, "回到横通道中线（只动 x）"))
    if abs(y - ty) > 0.30:                  # 换巷：在横通道里纯纵向
        legs.append((side, ty, "横通道内纵向换巷"))
    legs.append((tx, ty, "沿目标巷道到位"))
    legs = [lg for lg in legs if math.hypot(lg[0] - x, lg[1] - y) > 0.20]

    for lx, ly, why in legs:
        print(f"[go]   {why}: -> ({lx:+.2f},{ly:+.2f})  {math.hypot(lx - x, ly - y):.1f} m")
        if dry_run:
            continue
        if not drive_to_point(poller, lx, ly, speed, tol, max_seconds):
            print(f"[go] 未到 ({lx:+.2f},{ly:+.2f})，中止")
            return False
        pp = poller.get()
        if pp is not None:
            x, y = pp[0], pp[1]
    if not dry_run:
        print(f"[go] 到位 ({x:+.2f},{y:+.2f})")
    return True


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, required=True)
    ap.add_argument("--y", type=float, required=True)
    ap.add_argument("--speed", type=float, default=0.6)
    ap.add_argument("--tol", type=float, default=0.25)
    ap.add_argument("--max-seconds", type=float, default=420.0)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    poller = TruthPoller()
    t0 = time.monotonic()
    while poller.get(10.0) is None and time.monotonic() - t0 < 30:
        time.sleep(0.3)
    try:
        ok = safe_route(poller, args.x, args.y, speed=args.speed, tol=args.tol,
                        max_seconds=args.max_seconds, dry_run=args.dry_run)
    finally:
        poller.stop()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
