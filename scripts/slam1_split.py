#!/usr/bin/env python3
"""把 SLAM 轨迹拆成「里程计自己的误差」和「后端修正」两条链。

为什么需要：S5 的 ATE 0.258 m 里 dx 小、dy 全程 ≈ −0.25 m、yaw 误差 <0.3°、单帧最大跳变
0.074 m —— 形状像"横向系统偏置"，不像漂移。只看 `map->base_footprint` 分不清这偏差是
**里程计自己带的**，还是 **slam_toolbox 的 map->odom 修正引入的**。

`slam_route.py` 现在同时记三路：
    t_sim, wp, tx, ty, tyaw   ← Gazebo 真值
    ex, ey, eyaw              ← map -> base_footprint（SLAM 输出）
    ox, oy, oyaw              ← odom -> base_footprint（**不含** SLAM 修正）

判读：
    里程计链 |odom − truth| 大  ->  问题在前端（里程计/标定/切面几何），调后端没用
    里程计链干净、修正量大     ->  问题在后端（slam_toolbox 参数/回环）
    两条链的偏差同向同量       ->  是"世界/答案键"层面的几何偏置，先核对参照系

用法：
    .venv/bin/python scripts/slam1_split.py results/slam1_s5_traj.csv
"""
from __future__ import annotations

import csv
import math
import sys
from pathlib import Path


def load(path: Path) -> tuple[list[dict], dict]:
    rows: list[dict] = []
    with open(path, encoding="utf-8") as f:
        rd = csv.DictReader(f)
        cols = rd.fieldnames or []
        has_odom = "ox" in cols
        for r in rd:
            if r["ex"] in ("", "None"):
                continue
            row = dict(t=float(r["t_sim"]), wp=int(r["wp"]),
                       tx=float(r["tx"]), ty=float(r["ty"]), tyaw=float(r["tyaw"]),
                       ex=float(r["ex"]), ey=float(r["ey"]), eyaw=float(r["eyaw"]),
                       dist=float(r["dist"]))
            if has_odom and r["ox"] not in ("", "None"):
                row["ox"], row["oy"], row["oyaw"] = (float(r["ox"]), float(r["oy"]),
                                                     float(r["oyaw"]))
            rows.append(row)
    return rows, dict(has_odom=has_odom)


def med(v):
    v = sorted(v)
    return v[len(v) // 2] if v else 0.0


def main() -> int:
    if len(sys.argv) < 2:
        print("用法: slam1_split.py <traj.csv>")
        return 2
    path = Path(sys.argv[1])
    if not path.is_absolute():
        path = Path(__file__).resolve().parents[1] / path
    rows, meta = load(path)
    if not rows:
        print("没有有效估计")
        return 2
    n0 = min(30, len(rows))
    # 仅平移对齐（与 slam_metrics.py 的 t0 口径一致），分别对两条链
    ax, ay = med([r["ex"] - r["tx"] for r in rows[:n0]]), med([r["ey"] - r["ty"] for r in rows[:n0]])
    has_odom = meta["has_odom"]
    bx = by = 0.0
    if has_odom:
        bx, by = med([r["ox"] - r["tx"] for r in rows[:n0]]), med([r["oy"] - r["ty"] for r in rows[:n0]])

    def err_slam(r):
        return r["ex"] - ax - r["tx"], r["ey"] - ay - r["ty"]

    def err_odom(r):
        return r["ox"] - bx - r["tx"], r["oy"] - by - r["ty"]

    def rmse(f):
        return math.sqrt(sum(x * x + y * y for x, y in map(f, rows)) / len(rows))

    print(f"轨迹 {path.name}：{len(rows)} 帧")
    print(f"SLAM 链（map->base）  对齐 ({ax:+.3f},{ay:+.3f})  RMSE {rmse(err_slam):.3f} m")
    if has_odom:
        print(f"里程计链（odom->base）对齐 ({bx:+.3f},{by:+.3f})  RMSE {rmse(err_odom):.3f} m")
        corr = [math.hypot(*(a - b for a, b in zip(err_slam(r), err_odom(r)))) for r in rows]
        print(f"后端修正量 |map->odom|     中位 {med(corr):.3f} m  最大 {max(corr):.3f} m")

    print("\n分段（每段末尾的沿轴/横向偏差；横向 = 相对该段走向的垂直分量）")
    print(f"{'段':>4}{'真值(m)':>9}{'SLAM偏差':>12}{'里程计偏差':>14}{'修正量':>10}")
    wps = sorted({r["wp"] for r in rows})
    for i, wp in enumerate(wps):
        seg = [r for r in rows if r["wp"] == wp]
        if not seg:
            continue
        # 段方向：用段内真值首末点
        r0, r1 = seg[0], seg[-1]
        # 该段末尾的偏差（相对段起点做差分，去掉段间累积）
        s0, s1 = err_slam(r0), err_slam(r1)
        dsx, dsy = s1[0] - s0[0], s1[1] - s0[1]
        line = f"{wp:>4}{(r1['dist'] - r0['dist']):9.1f}{math.hypot(*s1):11.3f} m"
        if has_odom and "ox" in r0 and "ox" in r1:
            o0, o1 = err_odom(r0), err_odom(r1)
            line += f"{math.hypot(*o1):12.3f} m"
            line += f"{math.hypot(dsx - (o1[0] - o0[0]), dsy - (o1[1] - o0[1])):9.3f}"
        print(line)

    # 横向偏置：按段统计 dy 的中位数（判断是不是"整段整体偏移"）
    print("\n各段横向偏置中位数（SLAM 链 / 里程计链）")
    for wp in wps:
        seg = [r for r in rows if r["wp"] == wp]
        if len(seg) < 5:
            continue
        m_s = med([err_slam(r)[1] for r in seg])
        s = f"  wp{wp:<3} dy_slam {m_s:+.3f} m"
        if has_odom:
            s += f"   dy_odom {med([err_odom(r)[1] for r in seg]):+.3f} m"
        print(s)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
