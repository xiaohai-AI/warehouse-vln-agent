#!/usr/bin/env python3
"""把车归位到"安全的起步位置"：最近一侧的横通道，且 y 落在某条巷道的中线上。

为什么需要：`seg1_capture_all.sh` 每一批的第一步都是 `go (-31.5|+31.5, 某巷道 y)`。
如果车当时停在别的地方（上一批失败、或中途 Ctrl-C），这条 `go` 会**斜穿货架**。
归一化只做两种运动，两种都天然安全：
  * 车在横通道里（|x| > 30）→ 只做**纵向**移动，把 y 对到最近的巷道中线；
  * 车在巷道里（|x| ≤ 30）→ 只做**横向**移动，开到最近一侧的横通道。
特例：F 巷在 x 11.3–12.0 有一道横跨全宽的护栏，所以"最近一侧"按护栏分：
x < 11 往西、x > 12 往东（否则会顶在护栏上）。

用法
  .venv/bin/python scripts/seg1_norm.py            # 归位
  .venv/bin/python scripts/seg1_norm.py --dry-run  # 只打印打算去哪
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
# 让 `import depot_vln` 在任何 cwd 下都能用（脚本以 scripts/ 为 sys.path[0] 运行）
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

sys.path.insert(0, str(ROOT / "scripts"))

from drive_to import drive_to_point  # noqa: E402
from slam_route import TruthPoller  # noqa: E402

from depot_vln.catalog import get_catalog  # noqa: E402
from depot_vln.semantic_graph import SemanticGraph  # noqa: E402

_CAT = get_catalog()
_LAYOUT = _CAT.layout


def _capture_offsets() -> dict[str, float]:
    import yaml

    p = _CAT.world_path
    _ = p  # 路径只为可读性；偏移来自 config/nav.yaml
    cfg = yaml.safe_load((__import__("pathlib").Path(__file__).resolve().parents[1]
                          / "config" / "nav.yaml").read_text(encoding="utf-8")) or {}
    return {str(k).upper(): float(v) for k, v in (cfg.get("capture_lane_offsets") or {}).items()}


_OFFS = _capture_offsets()
# 巷道行驶中线 = 巷道中心 + 该巷的采集车道偏移（config/nav.yaml:capture_lane_offsets）。
# 原来写死 [-10.75, -6.45, -1.85, ...]，-1.85 的来历只存在于注释里。
AISLE_LINES = [float(a["center_y"]) + _OFFS.get(str(a["name"]).upper(), 0.0)
               for a in _LAYOUT["aisles"]]
CROSS_X = abs(float(_CAT.spawn["x"]))          # 横通道中线 = 出生点 X（几何派生）
_BARRIER = [s for s in _CAT.blocked_segments() if s.get("barrier_xs")]
BARRIER_X = (_BARRIER[0]["barrier_xs"][0], _BARRIER[0]["barrier_xs"][1]) if _BARRIER else (0.0, 0.0)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--speed", type=float, default=0.6)
    args = ap.parse_args()

    poller = TruthPoller()
    t0 = time.monotonic()
    while poller.get(10.0) is None and time.monotonic() - t0 < 30:
        time.sleep(0.3)
    p = poller.get()
    if p is None:
        print("[norm] 读不到真值位姿 —— 仿真没起？")
        return 2
    x, y, _ = p
    print(f"[norm] 当前位置 ({x:+.2f},{y:+.2f})")

    line = min(AISLE_LINES, key=lambda v: abs(v - y))
    if abs(x) > 30.0:
        # 在横通道里：只动 y（纵向本就是空的）
        tx, ty = x, line
        why = "横通道内纵向对齐到巷道中线"
    else:
        # 在巷道里：只动 x（沿巷走到最近一侧横通道）
        west = x < BARRIER_X[0]
        tx = -CROSS_X if west else CROSS_X
        ty = y
        why = f"沿当前巷道开到{'西' if west else '东'}侧横通道"
    print(f"[norm] {why} -> ({tx:+.2f},{ty:+.2f})  距离 {abs(tx - x) + abs(ty - y):.1f} m")
    if args.dry_run:
        poller.stop()
        return 0
    ok = drive_to_point(poller, tx, ty, args.speed, tol=0.30, max_seconds=420)
    poller.stop()
    p = None
    print(f"[norm] {'到位' if ok else '未到位（后续 go 会自己纠正，但请留意）'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
