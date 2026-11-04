"""离线诊断：S1 双目深度 vs RGB-D 标尺，把"几何口径 / 尺度 / 离群点"三类原因分开。

输入 results/s1_frame.npz（由 scripts/s1_depth_eval.py 落盘）。
只读、纯离线，不连 ROS。
"""
from __future__ import annotations

import json
import math

from pathlib import Path

import cv2
import numpy as np

# 仓库根 = 本脚本的上一级；不要写死绝对路径（别人 clone 下来就跑不了）
P = Path(__file__).resolve().parents[1] / "results" / "s1_frame.npz"
z = np.load(P)
Lg, Rg = z["left"], z["right"]
D, Z, disp = z["depth_ref"], z["depth_stereo"], z["disp"]
W = int(z["width"])
HFOV = float(z["hfov"])
B = float(z["baseline"])
f = (W / 2.0) / math.tan(HFOV / 2.0)

print(f"W={W}  HFOV={HFOV:.4f} rad  B={B:.3f} m  ->  f={f:.2f} px")
print(f"左图 {Lg.shape}  标尺深度 {D.shape}  视差 {disp.shape}")
print(f"标尺深度 {D[D > 0].min():.2f}..{D[D > 0].max():.2f} m (中位 {np.median(D[D > 0]):.2f})")
print(f"实测视差 {disp[disp > 0.5].min():.2f}..{disp[disp > 0.5].max():.2f} px "
      f"(中位 {np.median(disp[disp > 0.5]):.2f})")

# ---------- ① 尺度：实测视差 vs 由标尺深度反推的理论视差 ----------
mask = (disp > 0.5) & (D > 0.05)
d_meas = disp[mask]
d_theo = f * B / D[mask]
k = float(np.median(d_meas / d_theo))
slope = float(np.polyfit(d_theo, d_meas, 1)[0])
print("\n① 尺度（同像素配对，仅看量级）")
print(f"   比值 med(实测/理论) = {k:.3f}    拟合斜率 = {slope:.3f}   (1.0 = 尺度正确)")
print(f"   若斜率偏离 1，等价基线应为 B*{slope:.3f} = {B * slope:.4f} m")
print(f"   实测视差饱和点数：{int((disp >= 63.9).sum())} / {disp.size}（numDisparities=64 上限）")

# ---------- ② 对齐口径：三种 u 方向 × 若干位移 ----------
def err_at(shift_fn, name, lo=1.0, hi=6.0):
    ys, xs = np.nonzero(Z > 0)
    d = f * B / Z[ys, xs]
    us = shift_fn(xs, d).astype(int)
    ok = (us >= 0) & (us < W)
    ys, xs, us = ys[ok], xs[ok], us[ok]
    ref = D[ys, us]
    m = (ref >= lo) & (ref <= hi)
    if m.sum() < 200:
        return None
    e = np.abs(Z[ys[m], xs[m]] - ref[m]) / ref[m]
    return {"name": name, "n": int(m.sum()), "median": round(float(np.median(e)), 4),
            "p95": round(float(np.percentile(e, 95)), 4),
            "within5": round(float((e < 0.05).mean()), 3),
            "within12": round(float((e < 0.12).mean()), 3)}


variants = [
    (lambda u, d: u, "u（同像素）"),
    (lambda u, d: u + d / 2, "u + d/2"),
    (lambda u, d: u - d / 2, "u - d/2"),
    (lambda u, d: u + d, "u + d"),
    (lambda u, d: u - d, "u - d"),
    (lambda u, d: (W - 1) - u, "镜像 u"),
]
print("\n② 对齐口径（1–6 m）")
rows = [r for r in (err_at(fn, nm) for fn, nm in variants) if r]
for r in rows:
    print(f"   {r['name']:<10} n={r['n']:6d}  中位 {r['median'] * 100:6.2f}%  "
          f"p95 {r['p95'] * 100:8.1f}%  ≤5% {r['within5'] * 100:5.1f}%  ≤12% {r['within12'] * 100:5.1f}%")
best = min(rows, key=lambda r: r["p95"])
print(f"   -> p95 最优口径：{best['name']}")

# ---------- ③ 误差的空间分布（用最优口径）----------
best_fn = dict((nm, fn) for fn, nm in variants)[best["name"]]
ys, xs = np.nonzero(Z > 0)
dd = f * B / Z[ys, xs]
us = best_fn(xs, dd).astype(int)
ok = (us >= 0) & (us < W)
ys, xs, us = ys[ok], xs[ok], us[ok]
ref = D[ys, us]
m = (ref >= 1.0) & (ref <= 6.0)
ys, xs, ref = ys[m], xs[m], ref[m]
Zs = Z[ys, xs]
e = np.abs(Zs - ref) / ref
bad = e > 0.5
print(f"\n③ 误差空间分布（口径 {best['name']}，1–6 m，n={e.size}）")
print(f"   误差 >50% 的像素占比 {bad.mean() * 100:.1f}%")
if bad.any():
    h, w = D.shape
    print(f"   这些像素的行 range {ys[bad].min()}..{ys[bad].max()}（整图 0..{h - 1}）"
          f"  中位行 {int(np.median(ys[bad]))}")
    print(f"   这些像素的列 range {xs[bad].min()}..{xs[bad].max()}（整图 0..{w - 1}）"
          f"  中位列 {int(np.median(xs[bad]))}")
    rows_hist = np.bincount(ys[bad] // (h // 6 + 1), minlength=7)
    print(f"   按行均分 6 带的 >50% 计数 {rows_hist[:6].tolist()}")
    # 方向性偏差：是系统性偏近还是偏远
    ratio = Zs[bad] / ref[bad]
    print(f"   坏点的 Z_stereo/Z_ref：中位 {np.median(ratio):.2f}  "
          f"（<1 说明双目偏近，>1 偏远）")
    print(f"   坏点里 双目=无穷远(0) 的数量 {int((Zs[bad] <= 0).sum())}")

# ---------- ④ 反解基线/FOV（用"正确口径"下的视差-深度关系）----------
good = e < 0.05
if good.sum() > 500:
    d_good = f * B / Zs[good]
    d_theo_good = f * B / ref[good]
    ratio_good = float(np.median(Zs[good] / ref[good]))
    print("\n④ 用'误差<5% 的像素'反解尺度")
    print(f"   med(Z_stereo/Z_ref) = {ratio_good:.4f}（1.0 表示 f、B 都对）")
    print(f"   反解基线 B' = B / {ratio_good:.4f} = {B / ratio_good:.4f} m")

out = {"f_px": round(f, 3), "scale_ratio_median": round(k, 4), "variants": rows,
       "best_variant": best["name"], "bad_frac_gt50pct": round(float(bad.mean()), 4)}
print("\n" + json.dumps(out["variants"], ensure_ascii=False, indent=1)[:1200])
