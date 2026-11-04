#!/usr/bin/env python3
"""路径守卫：开车前扫掠整条路线，撞货架 / 撞叉车 / 撞任何静态障碍都拒绝。

# 为什么要有（真实踩过三次，一次比一次贵）

1. `drive_to.py` 的 docstring 早就写"斜穿会撞货架端面"，但**代码不检查**；
2. 出生点 (-31,0) 的 y=0 正好落在 rack_block_3 中线上 → 直线跑 `X=-22 Y=-1.60`，
   实测压住货架 2.74 m，GUI 里就是"机器人拐进货架"；
3. **2026-11-03：只查货架块，把车直接开进了叉车。**
   叉车碰撞体 y∈[-3.01,-2.39]，而路线取 C 巷中线 y=-2.15、扫掠带 y∈[-2.43,-1.87]
   → 重叠 4 cm。守卫当时打印"✓ 全程不压货架"，因为它**只认识 layout.json 的货架块**，
   根本不知道世界里有叉车（`config/world.yaml: obstacles.forklift {at_aisle: C, bay: 5, side: R}`）。

> 教训：**只认识一类障碍物的守卫，比没有守卫更糟** —— 它给出假的安心。
> 所以现在扫的是 `world/warehouse.sdf` 里**全部静态碰撞体**（与 `make world` 的碰撞几何、
> 也与静态占据图同源），不是任何"清单"。

# 判定口径

* 机器人按**扫掠圆**处理，半径 = 车体半宽 0.23 + 轮外沿 0.235 + 2 cm → 0.28 m
* 高度带 **0.02 ~ 1.45 m**（轮子接地 → 载荷箱顶），高于此的（顶横梁、灯）不算挡路
* 地面（z1 < 0.02）跳过
* drive_to 过程中不转向（wz=0），所以直线扫掠足够；加转向后要改成多边形扫掠

# 被挡时会算"哪一侧能过"

对每个挡路体，把它两侧的净宽和机器人半宽比，算出可行的 y 带，并给出
"在它之前横移、越过之后回位"的两个建议路点。

用法
  python3 scripts/path_guard.py --start=-31,0 --x -10.8 --y -2.15 --wp=-30.5,-2.15
  # 退出码：0 = 全程干净；3 = 会撞（并打印挡路体、可行 y 带、建议路点）
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LAYOUT = ROOT / "world" / "layout.json"
SDF = ROOT / "world" / "warehouse.sdf"
ROBOT_HALF = 0.28      # 车体半宽 0.23 + 轮外沿 0.235，再留 2 cm
SAMPLES = 600          # 每段采样数（1 m 约 60 点，足够发现擦边）
CLEAR_MARGIN = 0.10    # 判"能过"时额外留的余量
Z_LO, Z_HI = 0.02, 1.45   # 机器人高度带

_solids_cache: list[tuple[str, float, float, float, float, float, float]] | None = None


# ---------------------------------------------------------------- 数据
def load_layout() -> tuple[list, list]:
    """返回 (货架块, 巷道)。layout.json 由 make world 生成。"""
    if not LAYOUT.exists():
        print(f"[guard] 找不到 {LAYOUT}（先 make world）", file=sys.stderr)
        return [], []
    d = json.loads(LAYOUT.read_text(encoding="utf-8"))
    return d.get("blocks") or [], d.get("aisles") or []


def load_solids() -> list[tuple[str, float, float, float, float, float, float]]:
    """世界 SDF 里全部静态碰撞体：(名字, x0, x1, y0, y1, z0, z1)。

    **复用 verify_world.parse_sdf**，避免"守卫一套口径、自检另一套口径"再次漂移。
    """
    global _solids_cache
    if _solids_cache is not None:
        return _solids_cache
    if not SDF.exists():
        print(f"[guard] ⚠️ 找不到 {SDF}：只能查货架块（先 make world）", file=sys.stderr)
        _solids_cache = []
        return _solids_cache
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from verify_world import parse_sdf  # 同目录，纯 XML 解析，无 ROS 依赖

    _root, solids, _lights = parse_sdf(SDF)
    _solids_cache = [
        (s.name, s.x0, s.x1, s.y0, s.y1, s.z0, s.z1)
        for s in solids
        if s.z1 > Z_LO and s.z0 < Z_HI
    ]
    return _solids_cache


# ---------------------------------------------------------------- 扫掠
def _sweep(p0: tuple[float, float], p1: tuple[float, float], n: int = SAMPLES):
    for i in range(n + 1):
        t = i / n
        yield (p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t)


def check_leg(p0, p1, blocks, half: float = ROBOT_HALF, n: int = SAMPLES):
    """货架块（layout 口径）：返回 [(块名, 压住起点 x, 压住终点 x)]。"""
    hits: dict[str, list[float]] = {}
    for x, y in _sweep(p0, p1, n):
        for b in blocks:
            if ((b["x0"] - half) <= x <= (b["x1"] + half)
                    and (b["y0"] - half) <= y <= (b["y1"] + half)):
                r = hits.setdefault(b["name"], [x, x])
                r[0], r[1] = min(r[0], x), max(r[1], x)
    return [(k, v[0], v[1]) for k, v in sorted(hits.items())]


def check_leg_solids(p0, p1, solids, half: float = ROBOT_HALF, n: int = SAMPLES):
    """世界静态碰撞体：返回 [(名字, 压住起点 x, 压住终点 x, y0, y1)]。"""
    hits: dict[str, list] = {}
    for x, y in _sweep(p0, p1, n):
        for name, x0, x1, y0, y1, _z0, _z1 in solids:
            if (x0 - half) <= x <= (x1 + half) and (y0 - half) <= y <= (y1 + half):
                r = hits.setdefault(name, [x, x, y0, y1])
                r[0], r[1] = min(r[0], x), max(r[1], x)
                r[2], r[3] = min(r[2], y0), max(r[3], y1)
    return [(k, v[0], v[1], v[2], v[3]) for k, v in sorted(hits.items())]


# ---------------------------------------------------------------- 建议
def aisle_of(y: float, aisles: list) -> dict | None:
    for a in aisles:
        if a["y0"] - 0.4 <= y <= a["y1"] + 0.4:
            return a
    return None


def free_bands(y0: float, y1: float, aisles: list, half: float) -> list[tuple[float, float]]:
    """挡路体 (y0,y1) 两侧能过的 y 带（已扣掉 half + CLEAR_MARGIN）。"""
    a = aisle_of((y0 + y1) / 2, aisles)
    if a is None:
        return []
    out = []
    for lo, hi in ((a["y0"], y0), (y1, a["y1"])):
        if hi - lo >= 2 * (half + CLEAR_MARGIN):
            out.append((lo + half + CLEAR_MARGIN, hi - half - CLEAR_MARGIN))
    return out


def suggest_detour(start, goal, aisles, bad_solids, half=ROBOT_HALF) -> str:
    """给"从哪一侧绕、什么时候横移/回位"的具体路点。"""
    detours = []
    for name, xa, xb, y0, y1 in bad_solids:
        bands = free_bands(y0, y1, aisles, half)
        if not bands:
            detours.append(f"{name}: 该巷净空不足（两侧都过不去）")
            continue
        best = min(bands, key=lambda b: abs((b[0] + b[1]) / 2 - goal[1]))
        y_pass = round((best[0] + best[1]) / 2, 2)
        x_in = round(min(xa, xb) - 1.5, 1)
        x_out = round(max(xa, xb) + 1.5, 1)
        detours.append(
            f"{name}: 挡 y[{y0:+.2f},{y1:+.2f}]，可过 y[{best[0]:+.2f},{best[1]:+.2f}]"
            f" → 走 y={y_pass:+.2f}\n"
            f"         --wp={x_in:+.1f},{y_pass:+.2f} --wp={x_out:+.1f},{y_pass:+.2f}"
            f"   （越过之后回到 y={goal[1]:+.2f}）"
        )
    return "\n   ".join(detours)


# ---------------------------------------------------------------- 主判定
def check_route(start, waypoints, blocks, aisles, half: float = ROBOT_HALF,
                verbose: bool = True, solids: list | None = None) -> tuple[bool, list]:
    """返回 (是否安全, 违规明细)。明细 = (腿起点, 腿终点, 名字, xa, xb, y0, y1)。"""
    sol = load_solids() if solids is None else solids
    bad: list[tuple] = []
    cur = start
    for wp in waypoints:
        for name, xa, xb in check_leg(cur, wp, blocks, half):
            bad.append((cur, wp, name, xa, xb, float("nan"), float("nan")))
        for name, xa, xb, y0, y1 in check_leg_solids(cur, wp, sol, half):
            bad.append((cur, wp, name, xa, xb, y0, y1))
        cur = wp

    if verbose and bad:
        print(f"[guard] ✗ 这条路线会撞到东西（扫的是世界 SDF 里 {len(sol)} 个静态碰撞体）：")
        for a, b, name, xa, xb, y0, y1 in bad:
            extra = "" if y0 != y0 else f"，y {y0:+.2f}~{y1:+.2f}"
            print(f"    {name:<26} x {xa:+.2f} → {xb:+.2f}{extra}"
                  f"   腿 ({a[0]:+.2f},{a[1]:+.2f}) → ({b[0]:+.2f},{b[1]:+.2f})")
        # 明细是 7 元组；suggest_detour 只要 (名字, xa, xb, y0, y1)
        sol_bad = [(n, xa, xb, y0, y1) for (_a, _b, n, xa, xb, y0, y1) in bad
                   if y0 == y0]
        if sol_bad:
            print("[guard] 建议绕行：\n   "
                  f"{suggest_detour(start, waypoints[-1], aisles, sol_bad, half)}")
    return (not bad), bad


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", required=True, help='"x,y" 起点')
    ap.add_argument("--x", type=float, required=True)
    ap.add_argument("--y", type=float, required=True)
    ap.add_argument("--wp", action="append", default=[], help='中间路点 "x,y"，可重复')
    ap.add_argument("--half", type=float, default=ROBOT_HALF, help="扫掠半径（默认 0.28）")
    args = ap.parse_args()

    blocks, aisles = load_layout()
    if not blocks:
        print("[guard] 没有货架块数据，无法检查", file=sys.stderr)
        return 2
    solids = load_solids()

    s = tuple(float(v) for v in args.start.split(","))
    wps = [tuple(float(v) for v in w.split(",")) for w in args.wp] + [(args.x, args.y)]
    ok, bad = check_route(s, wps, blocks, aisles, half=args.half, solids=solids)

    print(f"[guard] 起点 ({s[0]:+.2f},{s[1]:+.2f})  终点 ({args.x:+.2f},{args.y:+.2f})  "
          f"{len(wps)} 段  扫掠半径 {args.half:.2f} m  碰撞体 {len(solids)} 个")
    if ok:
        print("[guard] ✓ 全程不压货架、不撞静态障碍")
        return 0
    print(f"[guard] ✗ {len(bad)} 处干涉 —— 拒绝执行（exit 3）")
    return 3


if __name__ == "__main__":
    raise SystemExit(main())
