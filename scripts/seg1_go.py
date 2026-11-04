#!/usr/bin/env python3
"""世界系安全转场：把车开到 (tx,ty)，路径**只走横通道 + 目标巷道**，绝不斜穿货架。

为什么必须单独做路由：`drive_to_point` 只会走直线。从"东侧横通道 y=-6.45"直接去
"西侧横通道 y=-10.75"，直线会横切 A/B 两组货架。之前能跑通只是因为起点恰好对，
换个起点就撞；而"换个起点"在分批采集里是**常态**（上一批收尾在哪、中途有没有失败）。

路由规则（每条腿都是一维移动，天然安全）：
  1. 车在巷道里（|x|<30）→ 先沿本巷开到同侧横通道（只动 x）；
  2. 换到目标 y（在横通道里只动 y）；
  3. 横向开到目标点（此时 y 已是目标巷道的中线，这一腿沿着巷道走）。

用法
  .venv/bin/python scripts/seg1_go.py --x -28.5 --y -6.45
  .venv/bin/python scripts/seg1_go.py --x 31.5 --y 10.75 --speed 0.6 --dry-run
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from drive_to import drive_to_point  # noqa: E402
from slam_route import TruthPoller  # noqa: E402

AISLE_LINES = [-10.75, -6.45, -1.85, 2.15, 6.45, 10.75]   # C 巷走 -1.85 绕叉车
CROSS_X = 31.5
BARRIER_X = (11.0, 12.3)          # F 巷护栏：x<11 只能往西、x>12 只能往东


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
    p = poller.get()
    if p is None:
        print("[go] 读不到真值位姿 —— 仿真没起？")
        return 2
    x, y = p[0], p[1]
    print(f"[go] 起点 ({x:+.2f},{y:+.2f})  目标 ({args.x:+.2f},{args.y:+.2f})")

    # 目标 y 必须在某条巷道中线上，否则"沿 y 横穿"这一腿会穿货架
    near = min(abs(args.y - v) for v in AISLE_LINES)
    if near > 0.6:
        print(f"[go] ⚠️ 目标 y={args.y:+.2f} 不在任何巷道中线上（最近差 {near:.2f} m）："
              f"最后一腿可能斜穿货架，请确认这是横通道内的点")

    # 选哪一侧的横通道：**不能只看 x 的正负**。
    # F 巷在 x 11.39–11.91 与 28.19–28.71 各有一道横跨全宽的护栏，
    # 车停在 x=10.57（护栏以西）时按"x>0 往东"会一路顶在护栏上（实测 420 s 超时）。
    def pick_side(px: float, py: float) -> float:
        if abs(py - 10.75) < 1.2:                      # F 巷：按护栏分段
            if px < BARRIER_X[0]:
                return -CROSS_X
            if px > 28.8:
                return CROSS_X
            # 中段（两道护栏之间，车正常开不进去）：去更近的一侧
            return -CROSS_X if (px - BARRIER_X[0]) < (28.8 - px) else CROSS_X
        return -CROSS_X if px <= 0 else CROSS_X

    west_side = pick_side(x, y)
    legs: list[tuple[float, float, str]] = []
    if abs(x) < 30.0:                       # 在巷道里：先沿本巷回横通道
        legs.append((west_side, y, "沿当前巷道回横通道"))
    if abs(y - args.y) > 0.30:              # 换巷：在横通道里纯纵向
        legs.append((west_side, args.y, "横通道内纵向换巷"))
    legs.append((args.x, args.y, "沿目标巷道到位"))

    # 去掉与起点重合的腿
    legs = [lg for lg in legs if math.hypot(lg[0] - x, lg[1] - y) > 0.20]
    for lx, ly, why in legs:
        print(f"[go]   {why}: -> ({lx:+.2f},{ly:+.2f})  {math.hypot(lx - x, ly - y):.1f} m")
        if args.dry_run:
            continue
        if not drive_to_point(poller, lx, ly, args.speed, args.tol, args.max_seconds):
            print(f"[go] 未到 ({lx:+.2f},{ly:+.2f})，中止")
            poller.stop()
            return 1
        pp = poller.get()
        if pp is not None:
            x, y = pp[0], pp[1]
    poller.stop()
    print(f"[go] 到位 ({x:+.2f},{y:+.2f})" if not args.dry_run else "[go] dry-run 结束")
    return 0


if __name__ == "__main__":
    sys.exit(main())
