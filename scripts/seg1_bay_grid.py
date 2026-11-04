#!/usr/bin/env python3
"""SEG-1（T1 货架结构发现）：从 3D 点云纯几何地建立 bay 网格，并与真值对账。

SPEC §7.1 判据：bay 网格与 `world/slot_grid.json` 匹配 ——
  **中心点误差中位 < 0.15 m，bay 数量误差 < 5%，朝向误差 < 3°**。

设计纪律
  * **不许读真值来做发现**。真值只在最后一步打分；`discover()` 里没有任何 truth 输入。
  * 结构来自三件事：①货架面 = y 方向的平面（密集 y 峰）；②立杆 = 面上沿 x 的周期性密集柱；
    ③层板 = 面上的 z 峰。bay 网格 = 立杆栅格的**单元**。
  * 输入是**累积**点云（`scripts/capture_cloud_path.py` 边开边存）：16 线单帧一面只有 300–400 点，
    立杆基本打不到，物理上恢复不出 0.15 m。

用法
  .venv/bin/python scripts/seg1_bay_grid.py                       # 读 results/seg1_path_A.npz
  .venv/bin/python scripts/seg1_bay_grid.py --cloud a.npz b.npz --out results/seg1.json
  .venv/bin/python scripts/seg1_bay_grid.py --plot                # 画结构图 results/seg1_grid.png
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

# 世界生成器里的规格：bay 宽 2.4 m、巷道净宽 3.2 m、层板顶面 0.15 / 1.75 m。
# 这些是"发现时的先验搜索范围"，不是答案：搜索区间给得宽，最终值由点云决定。
BAY_PITCH_HINT = 2.4
AISLE_WIDTH_HINT = 3.2


# ---------------------------------------------------------------- 基础工具
def load_clouds(paths: list[Path]) -> tuple[np.ndarray, np.ndarray | None]:
    """返回 (世界系点云, 位姿序列)。位姿可能是单点（单帧云）或整条路径（累积云）。

    口径统一成**世界系**。踩过的坑：单帧云原本只存雷达系，分析端把它当世界系用，
    结果把"巷宽 ±1.6"误读成"绝对位置 y=±1.58"，还让自击点过滤（按世界路径半径）完全失效。
    """
    from depot_vln.extrinsics import to_world     # 同一个外参，避免两处口径漂移

    pts, poses = [], []
    for p in paths:
        d = np.load(p)
        if "points_world" in d:
            P = d["points_world"].astype(np.float64)
        elif "poses" in d:
            P = d["points"].astype(np.float64)          # capture_cloud_path 存的已是世界系
        else:
            P = np.asarray(to_world(d["points"].astype(np.float64),
                                    tuple(float(v) for v in d["pose"])), np.float64)
        if "poses" in d:
            poses.append(d["poses"].astype(np.float64))
        elif "pose" in d:
            poses.append(d["pose"].astype(np.float64)[None, :])
        pts.append(P)
    return np.concatenate(pts, axis=0), (np.concatenate(poses, axis=0) if poses else None)


def filter_self_hits(P: np.ndarray, poses: np.ndarray | None, radius: float = 0.70) -> np.ndarray:
    """剔掉机器人自击点（点云里最大的伪结构）。

    雷达能看到自己的底盘与轮子（在它下方 0.3–0.6 m），实测是云里最大的单个 z 簇，
    既污染 z 标定，又把"巷道中间必须是空的"这条判别器破坏掉。
    新抓的云在抓取时就按传感器半径滤过；这里按**路径**再滤一次，兼容旧云与多帧拼接。
    """
    if poses is None or len(poses) == 0:
        return P
    try:
        from scipy.spatial import cKDTree
    except Exception:
        return P
    tree = cKDTree(np.asarray(poses)[:, :2])
    d, _ = tree.query(P[:, :2], k=1)
    return P[d > radius]


def calibrate_floor_z(P: np.ndarray) -> float:
    """地面在点云里是最大的水平面。返回需要扣掉的 z 偏移。

    ⚠️ 踩过的坑：雷达外参里 `lidar_z=0.55`、`zr=0.076`，但实测地面点落在世界 z≈−0.17，
    说明模型原点不在我假设的高度。z 只用来筛点，**不参与 SEG-1 的三条判据**，
    所以这里用点云自身标定，而不是相信外参。
    """
    h, e = np.histogram(P[:, 2], bins=np.arange(P[:, 2].min(), min(P[:, 2].max(), 1.0), 0.02))
    if h.size == 0:
        return 0.0
    return float(e[int(np.argmax(h))] + 0.01)


def smooth(v: np.ndarray, k: int = 3) -> np.ndarray:
    if k <= 1:
        return v
    ker = np.ones(k) / k
    return np.convolve(v, ker, mode="same")


# ---------------------------------------------------------------- ① 货架面
def detect_faces(P: np.ndarray, dz: float, min_support: int = 350,
                 min_tall: float = 1.10, min_long: float = 8.0) -> list[dict]:
    """y 方向的密集峰会对应货架侧面（竖直、长、高的平面）。

    三道闸门（都是在稀疏/含货箱数据上调出来的）：
      密集（支撑点够多）→ 高（z 跨度 ≥1.1 m，货箱只有 0.3–0.5 m）→ 长（x 跨度 ≥8 m）。
    """
    band = P
    if band.shape[0] < min_support:
        return []
    h, e = np.histogram(band[:, 1], bins=np.arange(band[:, 1].min(), band[:, 1].max(), 0.02))
    hs = smooth(h, 5)
    faces: list[dict] = []
    for i in range(2, len(hs) - 2):
        if hs[i] < min_support / 50.0 or hs[i] < hs[i - 1] or hs[i] < hs[i + 1]:
            continue
        yf = float(e[i] + 0.01)
        if faces and abs(yf - faces[-1]["y"]) < 0.25:      # 同一面合并（面有厚度/噪声）
            if hs[i] > faces[-1]["support"]:
                faces[-1].update(y=yf, support=float(hs[i]))
            continue
        faces.append({"y": yf, "support": float(hs[i])})
    kept: list[dict] = []
    for f in faces:
        sel = band[np.abs(band[:, 1] - f["y"]) < 0.10]
        if sel.shape[0] < min_support:
            continue
        zspan = float(sel[:, 2].max() - sel[:, 2].min())
        xspan = float(sel[:, 0].max() - sel[:, 0].min())
        if zspan < min_tall or xspan < min_long:
            continue
        f["n"] = int(sel.shape[0])
        f["z_lo"], f["z_hi"] = float(np.percentile(sel[:, 2], 1)) - dz, \
            float(np.percentile(sel[:, 2], 99)) - dz
        f["x_lo"], f["x_hi"] = float(sel[:, 0].min()), float(sel[:, 0].max())
        # 朝向：面内点做 PCA，(x,y) 主方向与 x 轴的夹角就是面的偏航
        xy = sel[:, :2] - sel[:, :2].mean(axis=0)
        if xy.shape[0] > 10:
            w, v = np.linalg.eigh(xy.T @ xy)
            d = v[:, np.argmax(w)]
            f["yaw_deg"] = float(math.degrees(math.atan2(d[1], d[0])))
            f["yaw_deg"] = ((f["yaw_deg"] + 90.0) % 180.0) - 90.0     # 归到 ±90°
        else:
            f["yaw_deg"] = 0.0
        kept.append(f)
    kept.sort(key=lambda d: d["y"])
    return kept


def detect_aisles(P: np.ndarray, dz: float, faces: list[dict]) -> list[dict]:
    """一对互相面对、**中间是空的**货架面 = 一条巷道。

    判据是"内部空"，但要**按 x 分箱取中位数**，不能拿总点数比：
      * 假巷道（两面货箱夹出来的空隙）内部点沿 x 铺满 → 中位分箱就很满，被拒；
      * 真巷道里停着障碍物（本世界 C 巷有叉车、F 巷有横跨全宽的护栏）只占几米 →
        中位分箱仍然接近空，照样通过。
    早期用"总点数比"时，C/F 会因为障碍物把整体占比抬到 0.15–0.45 而被整条误杀；
    后来改用"3 m 滑窗集中度"又太松（假巷道末端也有密集点），会凭空多出假巷道 ——
    中位分箱是这两者的正解。
    """
    cands = []
    Zall = P[:, 2] - dz          # 提到循环外：合并 7 个云后 P 有 240 万点
    band = (Zall > 0.25) & (Zall < 2.00)
    Pb = P[band]
    Zb = Zall[band]
    for i in range(len(faces)):
        for j in range(i + 1, len(faces)):
            gap = faces[j]["y"] - faces[i]["y"]
            if not (2.9 <= gap <= 3.6):
                continue
            ylo, yhi = faces[i]["y"], faces[j]["y"]
            lo, hi = ylo + 0.35, yhi - 0.35
            if hi <= lo:
                continue
            m_in = (Pb[:, 1] > lo) & (Pb[:, 1] < hi)
            m_f = (np.abs(Pb[:, 1] - ylo) < 0.12) | (np.abs(Pb[:, 1] - yhi) < 0.12)
            if m_f.sum() < 300:
                continue
            xs = Pb[m_f, 0]
            x0, x1 = float(xs.min()), float(xs.max())
            bins = np.arange(x0, x1 + 0.5, 0.5)
            if bins.size < 4:
                continue
            h_in = np.histogram(Pb[m_in, 0], bins)[0]
            h_f = np.histogram(xs, bins)[0]
            med_in = float(np.median(h_in))
            med_f = max(float(np.median(h_f)), 1.0)
            med_ratio = med_in / med_f
            # 障碍物 = 内部点明显高于中位的分箱（只用于报告，不参与判据）
            obst = [round(float(bins[k] + 0.25), 1) for k in np.flatnonzero(h_in > max(3 * med_in, 60))]
            cands.append({"lo": faces[i], "hi": faces[j], "gap": gap,
                          "n_inner": int(h_in.sum()), "inner_ratio": med_ratio,
                          "median_inner": med_in, "median_face": med_f,
                          "obstacle_x": obst[:8],
                          "center_y": 0.5 * (lo + hi), "support": min(faces[i]["n"], faces[j]["n"])})
    good = [c for c in cands if c["inner_ratio"] < 0.15]
    good.sort(key=lambda c: (c["inner_ratio"], -c["support"]))
    picked: list[dict] = []
    for c in good:                       # 去重叠：同一对货架面区间只保留最佳一条
        if any(not (c["hi"]["y"] < p["lo"]["y"] or c["lo"]["y"] > p["hi"]["y"]) for p in picked):
            continue
        picked.append(c)
    picked.sort(key=lambda c: c["center_y"])
    if not picked:
        print("[SEG-1] ⚠️ 没有配对成功的巷道。候选（中位分箱占比，闸门 <0.15）：")
        for c in sorted(cands, key=lambda d: d["inner_ratio"])[:6]:
            print(f"         面 y={c['lo']['y']:+.2f} / {c['hi']['y']:+.2f}  间距 {c['gap']:.2f}  "
                  f"中位分箱 内部 {c['median_inner']:.0f} / 面 {c['median_face']:.0f}  "
                  f"占比 {c['inner_ratio']:.3f}")
        if not cands:
            print("         一个间距在 2.9–3.6 m 的面配对都没有 —— 面检测本身没找到货架面")
    return picked


def inner_concentration(P: np.ndarray, dz: float, c: dict,
                        win: float = 3.0) -> tuple[float, float]:
    """巷道内部点在 x 上的集中度：返回 (3 m 滑窗内的最大占比, 该窗中心 x)。

    真巷道里停着障碍物（叉车/护栏）时内部点会抬到 0.15–0.45，但它们只占几米；
    假巷道（两面货箱夹出来的空隙）内部点虽然也多，却沿 x 铺满 —— 集中度把两者分开。
    """
    lo, hi = c["lo"]["y"] + 0.35, c["hi"]["y"] - 0.35
    Z = P[:, 2] - dz
    sel = P[(P[:, 1] > lo) & (P[:, 1] < hi) & (Z > 0.25) & (Z < 2.00)]
    if sel.shape[0] < 200:
        return 0.0, float("nan")
    xs = np.sort(sel[:, 0])
    n = xs.size
    best, best_x = 0.0, float("nan")
    j = 0
    for i in range(n):
        while j < n and xs[j] <= xs[i] + win:
            j += 1
        if (j - i) > best:
            best, best_x = j - i, float(xs[i] + win / 2)
    return best / n, best_x


def rack_extent(pr: dict, frac: float = 0.25, win: float = 2.0) -> tuple[float, float]:
    """一段剖面里"货架真正延伸的范围"：滑动窗内有点的 bin 占比 > frac。

    为什么需要：巷道尽头有**端墙**，它在每个货架面的 y 上都留下几十个点，
    于是剖面在墙处冒出一个孤峰，格点会外推出假 bay（实测多出 bay x=30.0）。
    试过"检测 x 方向的墙"，判据不成立 —— 栈结构在 x 上的投影本来就处处密集，
    会把 170 个位置都判成墙。滑动占用率是稳的：墙只占 0.2–0.3 m，2 m 窗占比 ~0.13 < 0.25。
    """
    h = pr["h"]
    bw = 0.05
    w = max(1, int(win / bw))
    occ = (h > 0).astype(np.float64)
    r = np.convolve(occ, np.ones(w) / w, mode="same")
    idx = np.nonzero(r > frac)[0]
    if idx.size == 0:
        return pr["x0"], pr["x0"]
    return pr["x0"] + idx[0] * bw, pr["x0"] + (idx[-1] + 1) * bw


# ---------------------------------------------------------------- ② 立杆栅格
def face_profile(P: np.ndarray, dz: float, yf: float, lo: float, hi: float,
                 bw: float = 0.05) -> dict:
    """一面货架在 x 方向的点密度剖面（只在立杆会出现的 z 带里取）。

    返回 {"x0": 第一个 bin 的左边界, "h": 密度}。
    用对象而不是裸数组，是为了把"索引 ↔ 世界 x"的换算固定在一处
    （踩过：索引原点写在两处，跨度一改就错位）。
    """
    Z = P[:, 2] - dz
    sel = P[(np.abs(P[:, 1] - yf) < 0.10) & (Z > lo) & (Z < hi)]
    x0 = -29.5
    edges = np.arange(x0, 29.5 + bw, bw)
    h, _ = np.histogram(sel[:, 0], bins=edges)
    return {"x0": x0, "h": h.astype(np.float64)}


def fit_lattice(prof: dict[float, np.ndarray], span: tuple[float, float],
                pitch_lo=2.0, pitch_hi=2.8) -> tuple[float, float, float]:
    """在两面剖面上联合搜索 (pitch, phase)，使落在栅格线上的密度**相对局部背景**最大。

    四个要点（都是踩出来的）：
      1. **只在观测跨度内搜索**：在没观测到的区间上比分数，等于用零给所有相位做同样的稀释，
         但会让"格点数多的相位"占便宜，实测把相位拉偏 0.65 m；
      2. **先粗后精**：粗搜 0.02/0.05，再在最优附近 0.005/0.02 精修；
      3. 用两面**联合**打分，而不是单面自相关：自相关会被层板边沿/货箱带的
         0.6–1.4 m 纹理误导（实测给出 1.40 m，真值 2.4 m）；
      4. **打分必须除以局部背景密度（2026-10-30 改）**：原来用"格点上的平均密度"，
         剖面密度沿 x 不均匀时（近端密、远端疏），更短的节距能把更多格点塞进密区、分数更高 ——
         实测 A+B 合并后给出 **2.395**（真值 2.400），24 bay 累积 0.12 m 偏差。
         改成"格点密度 / 该处 ±1.5 m 中位密度"后，节距不再受密度分布影响。
    """
    bw = 0.05
    xlo, xhi = span
    win = max(5, int(round(1.5 / bw)))          # 局部背景窗（±1.5 m）

    # 预先算好每面剖面的平滑曲线与局部背景（与 pitch/phase 无关，避免重复计算）
    prep: list[tuple[float, np.ndarray, np.ndarray]] = []
    for pr in prof.values():
        h = np.asarray(pr["h"], float)
        s = smooth(h, 3)
        bg = np.empty_like(s)
        half = win
        for i in range(s.size):                 # 滑动中位数（尺度小，直接算）
            lo, hi = max(0, i - half), min(s.size, i + half + 1)
            bg[i] = max(float(np.median(s[lo:hi])), 1.0)
        prep.append((pr["x0"], s, bg))

    def score(pitch: float, phase: float) -> float:
        n_pts = 0
        acc = 0.0
        for x0, s, bg in prep:
            idx = np.round((np.arange(phase, xhi, pitch) - x0) / bw).astype(int)
            idx = idx[(idx >= 0) & (idx < s.size)]
            if idx.size:
                acc += float((s[idx] / bg[idx]).sum())
                n_pts += idx.size
        return acc / max(n_pts, 1)              # 每个格点的"相对背景"密度

    best = (BAY_PITCH_HINT, xlo, -1.0)
    for p in np.arange(pitch_lo, pitch_hi + 1e-9, 0.02):
        for ph in np.arange(xlo, xlo + p, 0.05):
            s = score(p, ph)
            if s > best[2]:
                best = (float(p), float(ph), s)
    p0, ph0, _ = best
    for p in np.arange(max(pitch_lo, p0 - 0.06), min(pitch_hi, p0 + 0.06) + 1e-9, 0.005):
        for ph in np.arange(ph0 - 0.5, ph0 + 0.5, 0.02):
            s = score(p, ph)
            if s > best[2]:
                best = (float(p), float(ph), s)
    return best


def refine_by_peaks(prof: dict, pitch: float, phase: float,
                    iters: int = 3) -> tuple[float, float, int]:
    """用**实测立杆峰**做加权最小二乘重拟合，替掉纯网格搜索的结果。

    为什么需要：网格搜索的打分是"各格点位置的相对密度均值"，而剖面密度沿 x 不均匀，
    实测 A+B 合并后给出 2.395、B 单独给出 2.395（真值 2.400），24 bay 累积 0.12 m。
    立杆是真实证据：检出峰 → 指派到最近格点 → 加权最小二乘拟合 x_k = phase + k·pitch。
    实测把 B 的中心误差从 0.121 m 压到 0.082 m、A+B 中位从 0.096 压到 0.040。
    注意本世界的立杆**每 4.8 m 才有一根**（隔一个 bay），所以 2.4 m 格点的一部分证据来自
    层板接缝，节距拟合结果会稳定地落在 2.397 左右（真值 2.400，偏差 0.1%）。
    """
    if not prof:
        return pitch, phase, 0
    tol = 0.45
    n_used = 0
    for _ in range(iters):
        peaks: list[tuple[float, float]] = []
        for pr in prof.values():
            h = smooth(pr["h"], 3)
            if h.size < 5:
                continue
            thr = 0.30 * float(np.percentile(h, 90))
            for i in range(2, h.size - 2):
                if h[i] >= h[i - 1] and h[i] >= h[i + 1] and h[i] > thr:
                    peaks.append((float(pr["x0"] + i * 0.05), float(h[i])))
        if len(peaks) < 8:
            return pitch, phase, n_used
        peaks.sort()
        merged: list[list[float]] = []          # 0.18 m 内的峰按峰高加权合并（两面看同一根立杆）
        for x, w in peaks:
            if merged and x - merged[-1][0] < 0.18:
                x0, w0 = merged[-1]
                tot = w0 + w
                merged[-1] = [(x0 * w0 + x * w) / tot, tot]
            else:
                merged.append([x, w])
        ks, xs, ws = [], [], []
        for x, w in merged:
            k = round((x - phase) / pitch)
            if abs(x - (phase + k * pitch)) < tol:
                ks.append(float(k)); xs.append(x); ws.append(w)
        if len(ks) < 8:
            return pitch, phase, n_used
        k = np.asarray(ks); x = np.asarray(xs); w = np.asarray(ws)
        A = np.vstack([np.ones_like(k), k]).T
        sol, *_ = np.linalg.lstsq(A * np.sqrt(w)[:, None], x * np.sqrt(w), rcond=None)
        new_phase, new_pitch = float(sol[0]), float(sol[1])
        if not (1.8 <= new_pitch <= 3.0):
            return pitch, phase, n_used
        pitch, phase = new_pitch, new_phase
        n_used = len(ks)
        tol = 0.25
    return pitch, phase, n_used


def posts_from_lattice(pitch: float, phase: float, span: tuple[float, float]) -> np.ndarray:
    return np.arange(phase, span[1], pitch)


# ---------------------------------------------------------------- ③ 层板
def detect_levels(P: np.ndarray, dz: float, yf: float) -> list[float]:
    Z = P[:, 2] - dz
    sel = P[(np.abs(P[:, 1] - yf) < 0.10) & (Z > 0.02) & (Z < 3.0)]
    if sel.shape[0] < 100:
        return []
    h, e = np.histogram(sel[:, 2] - dz, bins=np.arange(0.0, 3.0, 0.02))
    hs = smooth(h, 5)
    thresh = hs.max() * 0.45
    levels, i = [], 1
    while i < len(hs) - 1:
        if hs[i] >= thresh and hs[i] >= hs[i - 1] and hs[i] >= hs[i + 1]:
            levels.append(float(e[i] + 0.01))
            i += 15                                   # 跳过一个峰宽
        else:
            i += 1
    return levels


# ---------------------------------------------------------------- 组装 + 打分
def discover(P: np.ndarray) -> dict:
    dz = calibrate_floor_z(P)
    faces = detect_faces(P, dz)
    aisles = detect_aisles(P, dz, faces)
    result = {"dz": dz, "faces": faces, "aisles": []}
    for a in aisles:
        prof = {}
        for f in (a["lo"], a["hi"]):
            pr = face_profile(P, dz, f["y"], 0.30, 2.45)
            if pr["h"].sum() >= 50:
                prof[f["y"]] = pr
        if not prof:
            continue
        # 跨度取两面"确有连续货架"的交集（保守）：端墙那几十个点不会被算成货架
        ext = [rack_extent(prof[f["y"]]) for f in (a["lo"], a["hi"]) if f["y"] in prof]
        # 交集偏保守，末端会切掉半个真 bay（实测少 1 个），所以各放一个 bay 宽；
        # 真正把关的是下面"单元内必须有实测支撑"（外推单元的支撑接近 0）。
        xlo = max(e[0] for e in ext) - BAY_PITCH_HINT / 2
        xhi = min(e[1] for e in ext) + BAY_PITCH_HINT / 2
        # 剖面的 x 索引原点与跨度（只在观测跨度内拟合，否则相位会被未观测区稀释带偏）
        x0 = next(iter(prof.values()))["x0"]
        pitch, phase, score = fit_lattice(prof, (max(xlo, x0), xhi))
        p2, ph2, n_peak = refine_by_peaks(prof, pitch, phase)
        if n_peak >= 8:
            pitch, phase = p2, ph2
        posts = posts_from_lattice(pitch, phase, (max(xlo, x0), xhi))
        bays = []
        cell_support = []
        for k in range(len(posts) - 1):
            cx = 0.5 * (posts[k] + posts[k + 1])
            if xlo - 0.3 <= cx <= xhi + 0.3:
                # 单元内必须有实测支撑：否则是格点外推到货架尽头之外（实测多出 1 个假 bay）
                sup = 0.0
                for pr in prof.values():
                    i0 = int(round((posts[k] - pr["x0"]) / 0.05))
                    i1 = int(round((posts[k + 1] - pr["x0"]) / 0.05))
                    i0, i1 = max(i0, 0), min(i1, len(pr["h"]))
                    if i1 > i0:
                        sup += float(pr["h"][i0:i1].sum())
                bays.append({"center_x": float(cx), "center_y": float(a["center_y"]),
                             "x0": float(posts[k]), "x1": float(posts[k + 1]),
                             "support": sup})
                cell_support.append(sup)
        if cell_support:
            # 阈值不能用"中位数比例"：远端 bay 只被远距离看到，支撑天然只有一半
            # （实测 463–516 vs 近端 1100–1750），用它会把真 bay 删掉；
            # 而货架尽头之外的单元支撑接近 0（只有地面溢出/噪声）。
            # 所以用"绝对下限 20 点 + 2% 中位数"取大者。
            thresh = max(20.0, 0.02 * float(np.median(cell_support)))
            n_before = len(bays)
            bays = [b for b in bays if b["support"] >= thresh]
            if len(bays) != n_before:
                print(f"[SEG-1] 剔除 {n_before - len(bays)} 个无支撑单元"
                      f"（阈值 {thresh:.0f}，外推到货架尽头之外）")
        levels = detect_levels(P, dz, a["lo"]["y"])
        yaw = 0.5 * (a["lo"]["yaw_deg"] + a["hi"]["yaw_deg"])
        result["aisles"].append({
            "center_y": a["center_y"], "gap": a["gap"], "pitch": pitch,
            "phase": phase, "lattice_score": score, "posts": posts.tolist(),
            "bays": bays, "n_bays": len(bays), "levels": levels, "yaw_deg": yaw,
            "x_obs": [xlo, xhi], "inner_ratio": a["inner_ratio"],
            "face_lo_y": a["lo"]["y"], "face_hi_y": a["hi"]["y"],
        })
    return result


def score(disc: dict, truth_path: Path, margin: float = 1.5) -> dict:
    """只在这里读真值。打分口径严格按 SPEC §7.1 三条。"""
    sg = json.loads(Path(truth_path).read_text(encoding="utf-8"))
    tbays = sg["bays"]
    out = {"per_aisle": [], "overall": {}}
    all_err: list[float] = []
    all_yaw: list[float] = []
    n_disc = n_truth = 0
    for a in disc["aisles"]:
        cy = a["center_y"]
        cand = [b for b in tbays if abs(b["aisle_center_y"] - cy) < 0.8]
        if not cand:
            out["per_aisle"].append({"center_y": round(cy, 2), "matched": False,
                                     "note": "真值里没有对应巷道"})
            continue
        xlo, xhi = a["x_obs"]
        tsel = [b for b in cand if xlo - margin <= b["center_x"] <= xhi + margin]
        errs = []
        for b in tsel:
            if not a["bays"]:
                break
            d = [math.hypot(d_["center_x"] - b["center_x"], d_["center_y"] - b["aisle_center_y"])
                 for d_ in a["bays"]]
            errs.append(min(d))
        n_disc += a["n_bays"]
        n_truth += len(tsel)
        all_err += errs
        all_yaw.append(abs(a["yaw_deg"]))
        out["per_aisle"].append({
            "center_y": round(cy, 2), "aisle": cand[0]["aisle"], "matched": True,
            "n_discovered": a["n_bays"], "n_truth_in_span": len(tsel),
            "pitch": round(a["pitch"], 3), "levels": [round(v, 2) for v in a["levels"]],
            "center_err_median_m": round(float(np.median(errs)), 3) if errs else None,
            "center_err_max_m": round(float(np.max(errs)), 3) if errs else None,
            "yaw_deg": round(a["yaw_deg"], 2),
        })
    med = float(np.median(all_err)) if all_err else float("nan")
    count_err = abs(n_disc - n_truth) / max(n_truth, 1)
    yaw_err = float(np.median(all_yaw)) if all_yaw else float("nan")
    ok = (med < 0.15) and (count_err < 0.05) and (yaw_err < 3.0)
    out["overall"] = {
        "center_err_median_m": round(med, 4) if all_err else None,
        "count_err_ratio": round(count_err, 4),
        "yaw_err_median_deg": round(yaw_err, 3) if all_yaw else None,
        "n_discovered": n_disc, "n_truth_in_span": n_truth,
        "criteria": {"center<0.15m": med < 0.15 if all_err else False,
                     "count<5%": count_err < 0.05, "yaw<3deg": yaw_err < 3.0 if all_yaw else False},
        "result": "PASS" if ok else "FAIL",
    }
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cloud", nargs="*", default=[str(ROOT / "results" / "seg1_path_A.npz")])
    ap.add_argument("--truth", default=str(ROOT / "world" / "slot_grid.json"))
    ap.add_argument("--out", default=str(ROOT / "results" / "seg1.json"))
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()

    paths = [Path(p) for p in args.cloud]
    missing = [p for p in paths if not p.exists()]
    if missing:
        print(f"[SEG-1] 缺输入点云 {missing} -> 未跑")
        print("        先跑 make seg1-capture（边开边拼一条巷道的点云）")
        return 2
    P, poses = load_clouds(paths)
    n0 = P.shape[0]
    P = filter_self_hits(P, poses)
    print(f"[SEG-1] 输入 {len(paths)} 个云，共 {n0} 点"
          + (f"，剔自击点后 {P.shape[0]} 点（路径半径 0.70 m）" if P.shape[0] != n0 else ""))
    if P.shape[0] < 20000:
        print(f"[SEG-1] ⚠️ 点太少（{P.shape[0]}）：单帧/短程点云恢复不出 bay 网格，结果仅供参考")

    disc = discover(P)
    # 只保留**机器人真正开进去过**的巷道：横通道会让雷达沿其他巷道看进去很远，
    # 也会在货架外侧（y≈-18 那种）凑出"间距 3.2 + 内部为空"的假巷道。
    # SEG-1 考核的是"开过巷道、发现它的 bay 网格"，没走过的巷不算发现。
    if poses is not None and len(poses):
        kept = []
        for a in disc["aisles"]:
            py = poses[:, 1]
            sel = np.abs(py - a["center_y"]) < 0.7
            n_pose = int(sel.sum())
            span = float(poses[sel, 0].max() - poses[sel, 0].min()) if n_pose else 0.0
            a["path_poses"] = n_pose
            a["path_span_m"] = round(span, 1)
            if n_pose >= 15 and span >= 5.0:
                kept.append(a)
            else:
                print(f"[SEG-1] 剔除未驶入的巷道候选 y={a['center_y']:+.2f}"
                      f"（路径点 {n_pose} 个 / 跨度 {span:.1f} m）")
        disc["aisles"] = kept
    print(f"[SEG-1] 地面标定 dz={disc['dz']:+.3f} m  发现货架面 {len(disc['faces'])} 个  "
          f"巷道 {len(disc['aisles'])} 条")
    for f in disc["faces"]:
        print(f"        面 y={f['y']:+7.2f}  n={f['n']:6d}  z[{f['z_lo']:.2f},{f['z_hi']:.2f}]  "
              f"yaw={f['yaw_deg']:+.2f}°  x[{f['x_lo']:+.1f},{f['x_hi']:+.1f}]")
    for a in disc["aisles"]:
        print(f"        巷道 y={a['center_y']:+.2f} 宽 {a['gap']:.2f}  节距 {a['pitch']:.2f} m  "
              f"相位 {a['phase']:+.2f}  bay {a['n_bays']}  层板 {[round(v,2) for v in a['levels']]}  "
              f"朝向 {a['yaw_deg']:+.2f}°  路径 {a.get('path_span_m', 0):.0f} m")

    # 结构自检：不靠肉眼看图。立杆必须等距，bay 中心必须是相邻立杆的中点。
    for a in disc["aisles"]:
        posts = np.asarray(a["posts"])
        if posts.size >= 2:
            gaps = np.diff(posts)
            print(f"[SEG-1] 自检 巷道 y={a['center_y']:+.2f}: 立杆 {posts.size} 个，"
                  f"间距 min/max = {gaps.min():.3f}/{gaps.max():.3f} m（应≈节距 {a['pitch']:.3f}）")
        bad = [b for b in a["bays"]
               if abs(0.5 * (b["x0"] + b["x1"]) - b["center_x"]) > 1e-6]
        if bad:
            print(f"[SEG-1] ⚠️ 自检失败：{len(bad)} 个 bay 的中心不在相邻立杆中点")

    res = {"discovery": disc, "score": score(disc, Path(args.truth)) if disc["aisles"] else None}
    o = res["score"]["overall"] if res["score"] else None
    if o:
        # 顶层扁平字段：验收脚本（scripts/accept.py）直接读这几个，别让它去猜嵌套结构
        res.update({
            "pass": o["result"] == "PASS",
            "median_center_err_m": o["center_err_median_m"],
            "count_err_ratio": o["count_err_ratio"],
            "yaw_err_deg": o["yaw_err_median_deg"],
            "n_bays_discovered": o["n_discovered"],
            "n_bays_truth_in_span": o["n_truth_in_span"],
            "n_aisles_observed": len(disc["aisles"]),
            "coverage_note": (f"只观测了 {len(disc['aisles'])} 条巷道（本次 A 巷）；"
                              "其余巷道未抓取，数量误差按观测范围内的真值计算"),
        })
    if o:
        print(f"\n[SEG-1] 与真值对账：中位中心误差 {o['center_err_median_m']} m（<0.15）  "
              f"数量误差 {100 * o['count_err_ratio']:.1f}%（<5%）  朝向 {o['yaw_err_median_deg']}°（<3°）")
        print(f"        发现 {o['n_discovered']} / 真值(观测范围内) {o['n_truth_in_span']}")
        for pa in res["score"]["per_aisle"]:
            if pa.get("matched"):
                print(f"        巷道 {pa['aisle']} y={pa['center_y']:+.2f}: "
                      f"{pa['n_discovered']} vs {pa['n_truth_in_span']}  "
                      f"中位 {pa['center_err_median_m']} m  最大 {pa['center_err_max_m']} m  "
                      f"节距 {pa['pitch']}  层板 {pa['levels']}  朝向 {pa['yaw_deg']}°")
        print(f"[SEG-1] RESULT: {o['result']}")
    else:
        print("\n[SEG-1] 没发现成对的货架面 -> 无法打分（未跑）")

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(res, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"-> {out}")

    if args.plot:
        plot(P, disc, Path(str(out).replace(".json", "_grid.png")), Path(args.truth))
    return 0 if (o and o["result"] == "PASS") else 1


def plot(P: np.ndarray, disc: dict, out: Path, truth_path: Path | None = None) -> None:
    """结构图。审阅靠它，所以**只在有观测的位置画**：

    踩过：把立杆画成贯穿全图的 axvline，看起来像"外推到没有点云的地方"，
    视觉复查会把它当成错误（验证时确实被指出）。所以线段都限制在观测跨度内。
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(17, 5.5))
    sub = P[np.random.choice(P.shape[0], min(80000, P.shape[0]), replace=False)]
    ax.scatter(sub[:, 0], sub[:, 1], s=0.12, c="#888888", alpha=0.28, linewidths=0)
    truth = None
    if truth_path and Path(truth_path).exists():
        truth = json.loads(Path(truth_path).read_text(encoding="utf-8"))["bays"]
    for a in disc["aisles"]:
        xlo, xhi = a["x_obs"]
        half = a["gap"] / 2.0
        cy = a["center_y"]
        ax.hlines(cy, xlo, xhi, color="red", lw=0.8, alpha=0.55)
        # 立杆：只在观测跨度内、只跨巷道宽度
        for p in a["posts"]:
            if xlo - 0.3 <= p <= xhi + 0.3:
                ax.vlines(p, cy - half, cy + half, color="orange", lw=0.6, alpha=0.7,
                          label="lattice post (bay boundary)")
        if a["bays"]:
            bx = [b["center_x"] for b in a["bays"]]
            by = [b["center_y"] for b in a["bays"]]
            ax.plot(bx, by, "r.", ms=5, label="discovered bay center")
        if truth:
            tx = [b["center_x"] for b in truth
                  if abs(b["aisle_center_y"] - cy) < 0.8 and xlo - 1.5 <= b["center_x"] <= xhi + 1.5]
            ty = [b["aisle_center_y"] for b in truth
                  if abs(b["aisle_center_y"] - cy) < 0.8 and xlo - 1.5 <= b["center_x"] <= xhi + 1.5]
            ax.plot(tx, ty, "+", color="lime", ms=7, mew=0.9, label="truth bay center")
    ax.set_aspect("equal")
    ax.set_xlim(-34.5, 34.5)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("SEG-1: discovered bay grid vs truth (grey = accumulated cloud)")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"-> {out}")


if __name__ == "__main__":
    sys.exit(main())
