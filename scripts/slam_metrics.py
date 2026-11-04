#!/usr/bin/env python3
"""SLAM-1 指标：从 slam_route.py 的 CSV 算 ATE / 终点漂移，并把建图结果与世界几何对账。

指标定义（都在世界系，真值只来自 gz pose/info）
  ATE       ：以第一帧为基准把估计轨迹平移对齐（map 帧原点不等于世界原点），
              之后逐帧算 ||est' - truth||，报 RMSE / 最大 / 终点漂移。
  dx, dy    ：终点漂移在世界轴上的分量（沿巷道方向 vs 横向；长直巷道通常 dx 更差）。
  地图一致性：把 SLAM 图的占据点按最优平移投到世界静态图上，报命中率（precision）与
              最优偏移量；偏移量应与 ATE 的终点漂移同量级 —— 两者不一致说明地图与轨迹自相矛盾。

  SPEC 没有给 SLAM-1 的数值阈值（只要求 200 m 固定路线 + 真值轨迹），这里用工程阈值并在
  输出里明确标注：ATE RMSE < 0.50 m、终点漂移 < 1.00 m。

用法
  .venv/bin/python scripts/slam_metrics.py --traj results/slam1_traj.csv \
      --map maps/depot_slam1 --out results/slam1.json
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from depot_geometry import load_pgm  # noqa: E402

ATE_RMSE_MAX = 0.50
FINAL_DRIFT_MAX = 1.00


def read_yaml_meta(p: Path) -> tuple[float, float, float]:
    meta = {}
    for line in p.read_text(encoding="utf-8").splitlines():
        if ":" in line and not line.strip().startswith("#"):
            k, v = line.split(":", 1)
            meta[k.strip()] = v.strip()
    org = [float(v) for v in meta["origin"].strip("[]").split(",")[:2]]
    return float(meta["resolution"]), org[0], org[1]


def occupied_points(pgm: Path, yaml: Path) -> list[tuple[float, float]]:
    res, x0, y0 = read_yaml_meta(yaml)
    img, w, h = load_pgm(pgm)
    pts = []
    for r in range(h):
        base = r * w
        for c in range(w):
            if img[base + c] < 100:
                pts.append((x0 + (c + 0.5) * res, y0 + (h - r - 0.5) * res))
    return pts


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--traj", default=str(ROOT / "results" / "slam1_traj.csv"))
    ap.add_argument("--map", default=str(ROOT / "maps" / "depot_slam1"), help="SLAM 图前缀（不含扩展名）")
    ap.add_argument("--static", default=str(ROOT / "maps" / "depot_static"), help="世界静态图前缀")
    ap.add_argument("--out", default=str(ROOT / "results" / "slam1.json"))
    ap.add_argument("--label", default="S1 纯激光（3D 点云 -> p2l 切片 -> slam_toolbox）",
                    help="写进 JSON 的配置名（S2 等要区分，否则产物分不清是哪一配置）")
    args = ap.parse_args()

    # ---------- 轨迹 ----------
    rows = []
    with open(args.traj, encoding="utf-8") as f:
        for row in csv.DictReader(f):
            rows.append(row)
    if not rows:
        print("CSV 是空的")
        return 2

    valid = [r for r in rows if r["ex"] not in ("", None)]
    if len(valid) < 10:
        print(f"估计轨迹只有 {len(valid)} 帧（TF map->base_footprint 没拿到）—— 检查 slam 是否在跑")
        return 3

    # 对齐偏移取前 30 帧的中位数：slam_toolbox 的 map 帧原点跟 odom（出生点偏移），
    # 用单帧对齐会被一次"跳 bay"的坏匹配带偏（实测偏差可达 2 m 量级）。
    import statistics
    n0 = min(30, len(valid))
    offx = statistics.median(float(valid[i]["tx"]) - float(valid[i]["ex"]) for i in range(n0))
    offy = statistics.median(float(valid[i]["ty"]) - float(valid[i]["ey"]) for i in range(n0))
    print(f"[SLAM-1] 对齐偏移（前 {n0} 帧中位数）({offx:+.2f},{offy:+.2f})")

    # **测量链自检**：map 帧原点与世界原点应当是**常量偏移**（出生点 −31 m），所以对齐偏移
    # 的 y 分量应该接近 0。实测（2026-11-01，关掉视觉侧移通道那次）前 30 帧估计一直停在
    # (0.03,−0.02) 而真值已南移 9.6 m → y 偏移被算成 −2.58 m，ATE 虚高到 4.95 m。
    # 也就是说：**对齐偏移本身超出常量偏移范围时，ATE 不可信**，先查起点段估计是否跟得上。
    if abs(offy) > 1.0:
        print(f"[SLAM-1] ⚠️ 对齐偏移 y={offy:+.2f} m（应接近 0）：起点段估计没跟上真值，"
              f"ATE 可能失真 —— 先看前 30 帧的 估计 vs 真值，别拿这个数下结论")

    errs = []
    for r in valid:
        ex = float(r["ex"]) + offx
        ey = float(r["ey"]) + offy
        errs.append(math.hypot(ex - float(r["tx"]), ey - float(r["ty"])))
    rmse = math.sqrt(sum(e * e for e in errs) / len(errs))
    last = valid[-1]
    fdx = (float(last["ex"]) + offx) - float(last["tx"])
    fdy = (float(last["ey"]) + offy) - float(last["ty"])
    final_drift = math.hypot(fdx, fdy)
    path_len = float(rows[-1]["dist"])

    print(f"[SLAM-1] 轨迹帧 {len(rows)}（有效估计 {len(valid)}）  真值里程 {path_len:.1f} m")
    print(f"[SLAM-1] ATE(t0 对齐) RMSE {rmse:.3f} m  最大 {max(errs):.3f} m")
    print(f"[SLAM-1] 终点漂移 {final_drift:.3f} m  (dx {fdx:+.3f} / dy {fdy:+.3f})")

    # ---------- 地图一致性 ----------
    map_info: dict = {}
    mp = Path(args.map)
    sp = Path(args.static)
    if mp.with_suffix(".pgm").exists() and sp.with_suffix(".pgm").exists():
        slam_pts = occupied_points(mp.with_suffix(".pgm"), mp.with_suffix(".yaml"))
        # 稀疏图直接判不可信：踩过"路线崩了 5 秒、图只有 29 个占据点，却算出 69% 命中"。
        if len(slam_pts) < 1000:
            print(f"[SLAM-1] 警告：SLAM 图只有 {len(slam_pts)} 个占据点，明显不完整 -> 地图指标不可信")
        static_pts = occupied_points(sp.with_suffix(".pgm"), sp.with_suffix(".yaml"))
        # 世界静态占据点按 0.1 m 粒度建集合（含 3x3 邻域，容忍 5 cm 栅格差）
        GRID = 0.10
        world_set = set()
        for x, y in static_pts:
            i, j = round(x / GRID), round(y / GRID)
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    world_set.add((i + di, j + dj))

        # 初值用对齐偏移，±6 m 粗搜（0.2 m 步长）再 0.05 m 精修 —— 长直巷道会整 bay 滑，
        # 搜索窗给窄了会把"最优点"丢在窗外，得出偏低的命中率。
        sample = slam_pts[::max(1, len(slam_pts) // 700)]        # 最多 700 点，够统计
        best = (-1.0, 0.0, 0.0)
        coarse = [(oi * 0.2, oj * 0.2) for oi in range(-30, 31) for oj in range(-15, 16)]
        for dox, doy in coarse:
            ox, oy = offx + dox, offy + doy
            hit = sum(1 for x, y in sample
                      if (round((x + ox) / GRID), round((y + oy) / GRID)) in world_set)
            if hit / len(sample) > best[0]:
                best = (hit / len(sample), ox, oy)
        bx, by = best[1], best[2]
        for oi in range(-8, 9):
            for oj in range(-8, 9):
                ox, oy = bx + oi * 0.05, by + oj * 0.05
                hit = sum(1 for x, y in sample
                          if (round((x + ox) / GRID), round((y + oy) / GRID)) in world_set)
                if hit / len(sample) > best[0]:
                    best = (hit / len(sample), ox, oy)
        ratio, ox, oy = best
        print(f"[SLAM-1] 地图一致性：SLAM 占据点 {len(slam_pts)} 个，"
              f"最优平移 ({ox:+.2f},{oy:+.2f}) 后命中世界几何 {ratio * 100:.1f}%")
        map_info = {"cells": len(slam_pts), "match_ratio": round(ratio, 4),
                    "offset": [round(ox, 3), round(oy, 3)],
                    "offset_vs_ate": [round(ox - offx, 3), round(oy - offy, 3)]}
    else:
        print("[SLAM-1] 没有地图文件，跳过地图一致性")

    # S1 是"基线"，它的意义是给出数字、而不是过某个绝对阈值（SPEC 未规定 SLAM-1 数值阈值）。
    # 真正要比的是 S2（+IMU）/S3（双目惯导）相对这份基线有没有改善。
    ok = rmse < ATE_RMSE_MAX and final_drift < FINAL_DRIFT_MAX
    result = "PASS" if ok else "BASELINE"          # 基线不判 FAIL，只记录
    out = {
        "config": args.label,
        "frames": len(rows), "frames_with_estimate": len(valid),
        "path_length_m": round(path_len, 2),
        "ate_rmse_m": round(rmse, 4), "ate_max_m": round(max(errs), 4),
        "final_drift_m": round(final_drift, 4), "final_drift_dx": round(fdx, 4),
        "final_drift_dy": round(fdy, 4),
        "map": map_info,
        "thresholds": {"ate_rmse_m": ATE_RMSE_MAX, "final_drift_m": FINAL_DRIFT_MAX,
                       "note": "SPEC 未规定 SLAM-1 数值阈值，此处为工程阈值"},
        "result": result,
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"[SLAM-1] RESULT: {out['result']}"
          f"（参考阈值 ATE<{ATE_RMSE_MAX} / 漂移<{FINAL_DRIFT_MAX}，SPEC 未规定）-> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
