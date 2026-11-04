#!/usr/bin/env python3
"""采样 Nav2 的全局规划 `/plan`，报告"路径在某个 x 处的 y"。

用途：判断一次"路径偏出"到底是**避让障碍**还是**静态地图本来的绕行**（对照实验）。
只看 y 的 min/max 会被斜线骗（踩过两次假阳性），唯一可靠的量是路径在**指定的 x**上的 y。

  .venv/bin/python scripts/plan_probe.py --x -26.0 --seconds 45
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))

from move_obstacle import plan_y_at, sample_plan_points   # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, default=-26.0, help="在哪个 x 处取路径的 y")
    ap.add_argument("--seconds", type=float, default=45.0)
    ap.add_argument("--interval", type=float, default=2.0)
    ap.add_argument("--straight-y", type=float, default=None,
                    help="参考：起点→目标直线在该 x 处的 y（给出就直接打印差值）")
    args = ap.parse_args()

    ys: list[float] = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.seconds:
        pts = sample_plan_points(timeout=6.0)
        y = plan_y_at(pts, args.x) if pts else None
        if y is not None:
            ys.append(y)
            ref = (f"  直线应为 {args.straight_y:+.2f} → 偏出 {y - args.straight_y:+.2f} m"
                   if args.straight_y is not None else "")
            print(f"  [plan] 采样 {len(ys):2d}: 路径在 x={args.x:+.2f} 处 y={y:+.2f}{ref}")
        time.sleep(args.interval)

    if not ys:
        print("[plan] 一个采样都没取到（/plan 没发布？Nav2 没活动目标？）")
        return 1
    lo, hi, mean = min(ys), max(ys), sum(ys) / len(ys)
    print(f"\n[plan] 共 {len(ys)} 个采样：y 在 x={args.x:+.2f} 处 {lo:+.2f} ~ {hi:+.2f}，均值 {mean:+.2f}")
    if args.straight_y is not None:
        print(f"[plan] 相对直线（{args.straight_y:+.2f}）偏出 均值 {mean - args.straight_y:+.2f} m"
              f"（最大 {max(abs(v - args.straight_y) for v in ys):.2f} m）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
