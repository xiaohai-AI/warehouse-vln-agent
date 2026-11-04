#!/usr/bin/env python3
"""判据 GRAPH-1：语义-拓扑图的路径长度 vs 真值最短路（SPEC §7.2）。

做法（两条独立路径，避免自我循环）
  真值：直接用 world/warehouse.sdf 的碰撞体建 0.20 m 栅格，机器人按 0.26 m 半径膨胀，
        8 邻接 Dijkstra 求到每个观测位的最短路 —— 完全不看 agent 的图。
  被测：depot_vln.semantic_graph.SemanticGraph（1D 走廊图 + 横通道）的 Dijkstra。

指标：随机取 N 个货位，比较两者路径长度；中位相对误差 < 15% 为达标，并报连通分量数。

  python3.10 scripts/check_graph.py --n 60
"""
from __future__ import annotations

import argparse
import heapq
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from depot_vln.semantic_graph import SemanticGraph  # noqa: E402
from verify_world import parse_sdf  # noqa: E402

RES = 0.20
ROBOT_R = 0.26
from depot_vln.catalog import get_catalog

_SPAWN = get_catalog().spawn
START = (float(_SPAWN["x"]), float(_SPAWN["y"]))   # 出生点来自 catalog，不再手抄

DIRS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)]


def build_grid(solids, x_min, y_min, nx, ny, zmin=0.05, zmax=2.0):
    """True = 被占据（含机器人半径膨胀）。"""
    occ = bytearray(nx * ny)
    for s in solids:
        if s.kind not in ("wall_", "rack_", "column_", "barrier_", "obstacle_"):
            continue
        if s.z1 < zmin or s.z0 > zmax:
            continue
        i0 = max(0, int(math.floor((s.x0 - ROBOT_R - x_min) / RES)))
        i1 = min(nx - 1, int(math.ceil((s.x1 + ROBOT_R - x_min) / RES)))
        j0 = max(0, int(math.floor((s.y0 - ROBOT_R - y_min) / RES)))
        j1 = min(ny - 1, int(math.ceil((s.y1 + ROBOT_R - y_min) / RES)))
        for i in range(i0, i1 + 1):
            base = i * ny
            for j in range(j0, j1 + 1):
                occ[base + j] = 1
    return occ


def dijkstra_grid(occ, nx, ny, x_min, y_min, start_ij):
    dist = [math.inf] * (nx * ny)
    si, sj = start_ij
    s = si * ny + sj
    if occ[s]:
        # 出生点被膨胀吃掉时找最近的自由格
        best, bd = None, 1e9
        for i in range(nx):
            for j in range(ny):
                if not occ[i * ny + j]:
                    d = (i - si) ** 2 + (j - sj) ** 2
                    if d < bd:
                        bd, best = d, (i, j)
        if best is None:
            return dist
        si, sj = best
        s = si * ny + sj
    dist[s] = 0.0
    pq = [(0.0, si, sj)]
    while pq:
        d, i, j = heapq.heappop(pq)
        if d > dist[i * ny + j] + 1e-12:
            continue
        for di, dj, w in DIRS:
            ni, nj = i + di, j + dj
            if not (0 <= ni < nx and 0 <= nj < ny):
                continue
            if occ[ni * ny + nj]:
                continue
            nd = d + w * RES
            k = ni * ny + nj
            if nd < dist[k] - 1e-12:
                dist[k] = nd
                heapq.heappush(pq, (nd, ni, nj))
    return dist


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=60)
    ap.add_argument("--world", default=str(ROOT / "world" / "warehouse.sdf"))
    ap.add_argument("--slot-grid", default=str(ROOT / "world" / "slot_grid.json"))
    ap.add_argument("--out", default=str(ROOT / "results" / "graph1.json"))
    args = ap.parse_args()

    grid_json = json.loads(Path(args.slot_grid).read_text(encoding="utf-8"))
    aisles_truth = json.loads((ROOT / "ground_truth" / "aisles.json").read_text(encoding="utf-8"))
    blocked = aisles_truth.get("blocked") or []

    root, solids, _lights = parse_sdf(Path(args.world))
    fp = grid_json["footprint"]
    nx = int((fp["x_max"] - fp["x_min"]) / RES) + 1
    ny = int((fp["y_max"] - fp["y_min"]) / RES) + 1
    print(f"[GRAPH-1] 栅格 {nx}×{ny} @ {RES} m，膨胀半径 {ROBOT_R} m")
    occ = build_grid(solids, fp["x_min"], fp["y_min"], nx, ny)
    si = int((START[0] - fp["x_min"]) / RES)
    sj = int((START[1] - fp["y_min"]) / RES)
    dist = dijkstra_grid(occ, nx, ny, fp["x_min"], fp["y_min"], (si, sj))

    g = SemanticGraph(grid_json, blocked)
    st = g.stats()
    print(f"[GRAPH-1] 被测图：节点 {st['nodes']} / 边 {st['edges']} / 连通分量 {st['components']}")

    slots = grid_json["slots"]
    addrs = sorted(slots)
    step = max(1, len(addrs) // args.n)
    errs = []
    rows = []
    for a in addrs[::step][: args.n]:
        slot = slots[a]
        obs = slot["obs_pose"]
        node = g.node_for_slot(a)
        g_len = g.path_length(START, node)
        oi = int((obs["x"] - fp["x_min"]) / RES)
        oj = int((obs["y"] - fp["y_min"]) / RES)
        t_len = dist[oi * ny + oj] if (0 <= oi < nx and 0 <= oj < ny) else math.inf
        # 观测位在巷道中线：grid 的距离直接可比
        if math.isinf(t_len) and math.isinf(g_len):
            continue
        if math.isinf(t_len) or math.isinf(g_len):
            rows.append({"address": a, "graph": g_len, "truth": t_len, "rel_err": None,
                         "note": "一侧不可达（真值图 vs 被测图不一致）"})
            continue
        rel = abs(g_len - t_len) / t_len
        errs.append(rel)
        rows.append({"address": a, "graph": round(g_len, 3), "truth": round(t_len, 3),
                     "rel_err": round(rel, 4)})

    errs.sort()
    med = errs[len(errs) // 2] if errs else float("nan")
    p90 = errs[int(0.9 * (len(errs) - 1))] if errs else float("nan")
    mismatched = [r for r in rows if r["rel_err"] is None]
    # 被封堵段在物理上就是孤立分量（两端都有路障），所以"连通分量 = 1 + 已封堵段数"
    expected_components = 1 + len(blocked)
    ok = bool(errs) and med < 0.15 and st["components"] == expected_components and not mismatched

    print()
    print(f"  比对了 {len(errs)} 个货位（不一致 {len(mismatched)} 个）")
    print(f"  路径长度相对误差：中位 {med:.1%}，90 分位 {p90:.1%}（判据：中位 < 15%）")
    print(f"  连通分量 {st['components']}（期望 {expected_components} = 1 个主网 + {len(blocked)} 个封堵段）")
    if mismatched:
        for r in mismatched[:5]:
            print(f"    ! {r['address']} graph={r['graph']} truth={r['truth']}")

    payload = {
        "criterion": "GRAPH-1",
        "grid_res_m": RES,
        "robot_radius_m": ROBOT_R,
        "start": START,
        "nodes": st["nodes"],
        "edges": st["edges"],
        "components": st["components"],
        "expected_components": 1 + len(blocked),
        "blocked_segments": [seg.get("aisle") for seg in blocked],
        "n_compared": len(errs),
        "median_rel_err": round(med, 4) if errs else None,
        "p90_rel_err": round(p90, 4) if errs else None,
        "mismatched": len(mismatched),
        "pass": ok,
        "samples": rows[:40],
    }
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print()
    print(f"RESULT: {'PASS' if ok else 'FAIL'}  -> {out.relative_to(ROOT)}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
