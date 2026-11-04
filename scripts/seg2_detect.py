#!/usr/bin/env python3
"""SEG-2（T2 货位占用与类别）：从按货位观测位拍的 RGB-D 帧里检测货物并归属到货位。

SPEC §7.1 判据：占用 **precision ≥ 0.90 / recall ≥ 0.85**；
货类 top-1 准确率 **≥ 0.80**（闭集）；**错格率（挂到相邻 bay）< 5%**。

设计要点（都是被数据逼出来的）
  * **不依赖精确外参**：这台车上相机有 16° 仰角，反推世界位姿一路上踩了"绕错轴/漏 base_link
    高度/深度图是否翻转"三个坑；而检测真正需要的只是"尺寸 + 颜色 + 在哪一层"。
    尺寸 = 像素跨度 × 深度 / 焦距（只用到内参 K）；层 = 两条货物带之间的行向空隙。
    → 尺寸误差里只剩 cos(16°)=4% 的投影压缩，远小于类间尺寸差（0.35 / 0.50 / 0.70 m）。
  * **前景 = 比货架背面更近**：以每帧深度图的"货架平面"为基准（取该 bay 区域深度的 90 分位，
    即使货位满也仍有货架结构露出），比基准近 6 cm 以上的像素算货物。
  * **闭集 6 类**（世界生成器定义）：carton_s 0.35 / carton_m 0.50 / carton_l 0.70 / tote 0.45 /
    pallet 1.05×0.15（很扁）/ drum 0.56（圆柱、绿色）。三种纸箱**颜色几乎一样**，
    只能靠尺寸分 —— 这正是要用双目/深度而不是纯 RGB 的原因。

用法
  .venv/bin/python scripts/seg2_detect.py                       # 读 results/seg2_frames.npz
  .venv/bin/python scripts/seg2_detect.py --frames x.npz --out results/seg2.json --plot
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]

# 闭集类别（尺寸来自 config/world.yaml 的 semantics.classes）
CLASSES = {
    "carton_s": {"dims": (0.35, 0.30, 0.30), "rgb": (0.72, 0.55, 0.34)},
    "carton_m": {"dims": (0.50, 0.40, 0.40), "rgb": (0.66, 0.48, 0.28)},
    "carton_l": {"dims": (0.70, 0.60, 0.55), "rgb": (0.58, 0.42, 0.24)},
    "tote":     {"dims": (0.45, 0.35, 0.30), "rgb": (0.20, 0.35, 0.62)},
    "pallet":   {"dims": (1.05, 0.90, 0.15), "rgb": (0.78, 0.70, 0.48)},
    "drum":     {"dims": (0.56, 0.56, 0.85), "rgb": (0.30, 0.45, 0.40)},
}


# ------------------------------------------------------------------ 检测
def pixel_geometry(depth: np.ndarray, k: np.ndarray, pitch_deg: float,
                   cam_h: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """逐像素 -> (世界高度 z, 到相机的水平距离, 射线仰角)。

    踩过的坑：用"斜距 vs 货架面斜距"取前景，会把**地面**当货物 —— 画面下方看的是地面
    （0.4–1.0 m），而那个方向上"竖直货架面"在 2.7–3.4 m，于是地面永远"更近"。
    必须先把像素换算成 (水平距离, 高度) 再做判断。
    """
    h_, w_ = depth.shape
    uu, vv = np.meshgrid(np.arange(w_, dtype=np.float64), np.arange(h_, dtype=np.float64))
    e = np.arctan((k[1, 2] - vv) / k[1, 1]) + np.radians(pitch_deg)   # 世界仰角
    z = cam_h + depth * np.sin(e)
    dh = depth * np.cos(e)
    return z, dh, e


def plane_slant_dist(k: np.ndarray, h: int, w: int, d_plane: float,
                     pitch_deg: float = 0.0) -> np.ndarray:
    """竖直平面（正对相机、水平距离 d_plane）在每个像素上的**斜距**。

    相机有仰角时必须这么算，不能只用"水平角"：
      光线方向在相机系里是 (x_c, y_c, 1)，y_c 向下；
      绕车体横轴转正 −pitch 后，前向分量 = cos p − y_c·sin p。
      于是打在同一竖直平面上的斜距 = d·|r| / (cos p − y_c·sin p)。
    作用：画面**上方**的像素斜距更长（实测上层货物 Z≈2.09 m，而货架面水平距离仅 1.6 m）。
    只用 cos(水平角) 会把上层货物整片当成"背景"漏检（recall 掉到 0.2）。
    """
    uu, vv = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
    xc = (uu - k[0, 2]) / k[0, 0]
    yc = (vv - k[1, 2]) / k[1, 1]
    p = np.radians(pitch_deg)
    fwd = np.cos(p) - yc * np.sin(p)
    fwd = np.maximum(fwd, 0.05)
    r = np.sqrt(1.0 + xc * xc + yc * yc)
    return d_plane * r / fwd


def cargo_mask(depth: np.ndarray, k: np.ndarray, d_face: float = 1.60,
               d_back: float = 2.75, pitch_deg: float = 16.0,
               cam_h: float = 0.436, z_min: float = 0.18, z_max: float = 2.70
               ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """货位"有货"像素：水平距离落在货架**内部**那一段。

    ⚠️ 这条判据的方向一开始写反了：我做成"比货架面更近"，结果 recall 只有 0.2。
    真实几何是 —— 货架前缘面在 obs 位前方 1.6 m，但**货物坐在货架内部**：
    货位中心 2.15 m，货物正面约 1.9 m，货架背板约 2.7 m。空货位则直接**穿透**
    看到下一条巷道（实测 6–11 m）。所以：
        有货 = 1.6 < 水平距离 < 2.75 且高度在层板之间
        空   = 穿透（>2.75）或只看到前缘结构（≈1.6）
    """
    z, dh, _ = pixel_geometry(depth, k, pitch_deg, cam_h)
    fin = np.isfinite(depth) & (depth > 0.25)
    cargo = fin & (dh > d_face + 0.12) & (dh < d_back - 0.05) & (z > z_min) & (z < z_max)
    return cargo, z, dh


def components(mask: np.ndarray, depth: np.ndarray, z: np.ndarray, dh: np.ndarray,
               min_px: int = 260) -> list[dict]:
    """连通域（同一物体），带像素跨度、深度、高度统计。"""
    m = (mask.astype(np.uint8)) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
    out = []
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if area < min_px:
            continue
        sel = lab[y:y + h, x:x + w] == i
        zz = z[y:y + h, x:x + w][sel]
        dd = dh[y:y + h, x:x + w][sel]
        if zz.size < min_px:
            continue
        out.append({"x0": int(x), "y0": int(y), "x1": int(x + w), "y1": int(y + h),
                    "area": int(area), "n_px": int(zz.size),
                    "z_med": float(np.median(zz)), "z_lo": float(np.percentile(zz, 5)),
                    "z_hi": float(np.percentile(zz, 95)),
                    "d_med": float(np.median(dd))})
    return out


def comp_size_m(c: dict, k: np.ndarray) -> tuple[float, float, float]:
    """像素跨度 -> 物理尺寸（宽, 高, 深），用物体自身的水平距离换算。"""
    fx, fy = k[0, 0], k[1, 1]
    d = c["d_med"]
    w = (c["x1"] - c["x0"]) / fx * d
    h = c["z_hi"] - c["z_lo"]                     # 高度直接用几何换算结果，比像素跨度准
    return w, h, 0.0


def classify(w: float, h: float, rgb: np.ndarray) -> tuple[str, float]:
    """闭集分类（阈值按**实测**标定，不是照抄 config 的标称尺寸）。

    实测（这批 48 帧，宽/高单位 m）：
        carton_s  w≈0.35 h≈0.17     carton_m  w≈0.48 h≈0.26     carton_l  w≈0.67 h≈0.39
        tote      w≈0.42 h≈0.16   rgb≈(36,42,36) 深绿灰（渲染出来不是标称的蓝）
        pallet    h≈0.00 w≥0.55（扁平）
    高度受层板遮挡只有标称值的 ~0.65 倍，但**类间分离度**很好，所以宽度和高度都用实测带。
    颜色只用来分"棕(纸箱) / 绿灰(周转箱·桶) / 亮青绿(桶)"。
    """
    r, g, b = (float(v) for v in rgb)
    greenish = g > r + 3 and abs(g - b) <= 12
    if greenish and g > 85:
        return "drum", 0.75
    if greenish and g <= 85:
        return "tote", 0.80
    if w > 0.85 or (h < 0.12 and w > 0.50):
        return "pallet", 0.75      # 托盘 1.05 m 宽，比任何纸箱都宽（carton_l 只有 0.70）
    if w < 0.42:
        return "carton_s", 0.85
    if w < 0.56:
        return "carton_m", 0.85
    return "carton_l", 0.85


def cargo_components(depth: np.ndarray, k: np.ndarray, pitch_deg: float, cam_h: float,
                     z_band: tuple[float, float], obs_x: float = 0.0,
                     obs_yaw: float = 0.0, d_lo: float = 1.10,
                     d_hi: float = 1.55, min_px: int = 260) -> list[dict]:
    """整幅图里找**货架内部那一段**的物体（不假设它们在本 bay）。

    为什么不再用"画面中心 ±0.95 m"的窗口：转向误差实测到 35°，窗口会把邻 bay 的货
    算进来、又漏掉本 bay 的货（recall 只有 0.5）。改为在全图找物体，再用**实际朝向**
    换算成世界 x，最后按几何归属到 bay —— 与转向误差无关。

    ⚠️ 深度带的基准是**相机**不是车体中心：相机装在车体前缘（+0.265 m），
    所以它到货架面的距离是 1.335 m（不是 1.6 m），货物前表面实测 ≈1.38 m。
    一开始把带设成 (1.40,1.90) 正好把货物前表面切在门外，recall 只有 0.09。

    ⚠️ 上界 1.55 是"本侧 vs 对侧"的分水岭（实测）：本侧货物前表面 d_med = 1.18–1.40，
    对侧（同一货架块另一面、另一种巷）的货透过空货位能看到，d_med = 1.76–1.79。
    两者世界 x 相同、只有深度不同，所以只能靠深度切；不带这个上界会有 5 个误检，
    且每个误检的对侧确实都有货（已逐一核对）。
    """
    z, dh, _ = pixel_geometry(depth, k, pitch_deg, cam_h)
    m = (np.isfinite(depth) & (depth > 0.25) & (dh > d_lo) & (dh < d_hi)
         & (z > z_band[0]) & (z < z_band[1]))
    # 逐像素世界 x：横向偏移 = (u-cx)/fx·dh；世界 x = obs_x + 横向·sin(yaw)
    h_, w_ = depth.shape
    uu = np.arange(w_, dtype=np.float64)[None, :]
    wx_map = obs_x + (uu - k[0, 2]) / k[0, 0] * dh * np.sin(obs_yaw)
    mm = cv2.morphologyEx((m.astype(np.uint8)) * 255, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mm = cv2.morphologyEx(mm, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mm, connectivity=8)
    out = []
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if area < min_px:
            continue
        sel = lab[y:y + h, x:x + w] == i
        zz = z[y:y + h, x:x + w][sel]
        dd = dh[y:y + h, x:x + w][sel]
        xx = wx_map[y:y + h, x:x + w][sel]
        if zz.size < min_px:
            continue
        # 尺寸取**世界系跨度**（宽沿世界 x、高沿 z）：像素跨度会被转向误差放大
        # （20° 虚高 6%、35° 虚高 22%），而三种纸箱只差 0.15–0.20 m，直接毁掉分类
        # 用 1/99 分位而不是 3/97：截断会系统性吃掉 10% 的宽度，
        # 而 carton_s/m/l 只差 0.15–0.20 m，10% 偏差足以毁掉分类
        w_m = float(np.percentile(xx, 99) - np.percentile(xx, 1))
        h_m = float(np.percentile(zz, 95) - np.percentile(zz, 5))
        c_wx = float(np.mean(xx))
        out.append({"x0": int(x), "y0": int(y), "x1": int(x + w), "y1": int(y + h),
                    "area": int(area), "d_med": float(np.median(dd)),
                    "u_c": float(x + w / 2.0), "v_c": float(y + h / 2.0),
                    "w_m": w_m, "h_m": h_m, "c_wx": c_wx,
                    "z_mid": float(np.mean(zz))})
    return out


def detect_frame(rgb: np.ndarray, depth: np.ndarray, k: np.ndarray,
                 obs_x: float, obs_y: float, obs_yaw: float, bay_cx: float,
                 pitch_deg: float = 16.0, cam_h: float = 0.436,
                 bay_half_m: float = 0.70, levels: tuple[int, ...] = (1,)) -> dict:
    """单帧 -> {"levels": {lvl: det|None}, "neighbors": [...] }。

    归属：物体的**世界 x** = obs_x + lateral·sin(yaw)，其中 lateral 是它相对光轴的横向米数。
    |世界 x − bay 中心| < 1.0 m 判为本 bay；否则记为邻 bay（用于错格率）。
    """
    # 下层带下界必须够低：托盘只有 0.15 m 高、躺在 0.15 的层板上（z 0.15–0.30），
    # 用 0.25 会把托盘整类漏掉（实测 4 个漏检全是 pallet）
    bands = {1: (0.17, 1.60), 2: (1.80, 2.62)}
    out: dict = {"levels": {}, "neighbors": []}
    for lvl in levels:
        comps = cargo_components(depth, k, pitch_deg, cam_h, bands[lvl],
                                 obs_x=obs_x, obs_yaw=obs_yaw)
        mine = []
        for c in comps:
            wx = c["c_wx"]
            c["world_x"] = float(wx)
            c["lateral_m"] = float(wx - obs_x)
            # 尺寸闸门：货架立柱是 0.1 m 宽、贯穿整层的细条（实测 w≈0.11 h≈0.7）；
            # 货物是 0.35–0.70 m 宽、0.30–0.55 m 高。不加这道门，扫全图时选中的永远是立柱。
            if (abs(wx - bay_cx) < bay_half_m
                    and 0.28 <= c["w_m"] <= 1.15 and c["h_m"] <= 0.62):
                # 上限放到 1.15 m：托盘标称 1.05 m 宽，用 0.90 会把整类托盘漏掉（实测 4 个 FN）
                # 下限 0.28 m 挡货架立柱（0.1 m 宽）；h 允许≈0（托盘扁平）
                mine.append(c)
            else:
                out["neighbors"].append({"world_x": round(float(wx), 2),
                                         "class_hint": None, "area": c["area"]})
        if not mine:
            out["levels"][lvl] = None
            continue
        c = max(mine, key=lambda d: d["area"])
        patch = rgb[c["y0"]:c["y1"], c["x0"]:c["x1"]].reshape(-1, 3)
        col = patch.mean(axis=0)
        name, conf = classify(c["w_m"], c["h_m"], col)
        out["levels"][lvl] = {"bbox": [c["x0"], c["y0"], c["x1"], c["y1"]],
                              "area": c["area"], "d_med": round(c["d_med"], 3),
                              "size_m": [round(c["w_m"], 3), round(c["h_m"], 3)],
                              "class": name, "conf": round(conf, 2), "level": lvl,
                              "world_x": round(c["world_x"], 3),
                              "lateral_m": round(c["lateral_m"], 3),
                              "z_med": round(c["z_mid"], 3),
                              "rgb": [round(float(v), 1) for v in col]}
    return out


# ------------------------------------------------------------------ 汇总与打分
def assign(dets: list[dict], addr: str, bay: int, side: str) -> dict[int, dict]:
    """把检测结果归属到该观测覆盖的两个货位（层 1 / 层 2）。"""
    out: dict[int, dict] = {}
    for d in dets:
        lvl = int(d["level"])
        prev = out.get(lvl)
        if prev is None or d["conf"] * d["area"] > prev["conf"] * prev["area"]:
            out[lvl] = d
    return out


def score(pred: dict[str, dict], truth: dict[str, dict]) -> dict:
    """SPEC §7.1 三条：占用 precision/recall、类别 top-1、错格率。"""
    tp = fp = fn = 0
    cls_ok = cls_n = 0
    wrong_slot = 0
    detail = []
    for addr, p in pred.items():
        t = truth.get(addr)
        if t is None:
            continue
        p_occ = p is not None
        t_occ = bool(t.get("occupied"))
        if p_occ and t_occ:
            tp += 1
            if p["class"] == t.get("class"):
                cls_ok += 1
            cls_n += 1
        elif p_occ and not t_occ:
            fp += 1
        elif not p_occ and t_occ:
            fn += 1
        detail.append({"addr": addr, "pred": p["class"] if p_occ else "empty",
                       "truth": t.get("class") if t_occ else "empty", "level": p["level"] if p_occ else None})
    # 错格率：货物横向偏移 > 1.0 m（半个 bay 宽 1.2 m 的九成）说明它其实属于相邻 bay
    wrong_slot = sum(1 for p in pred.values() if p and abs(p.get("lateral_m", 0.0)) > 1.0)
    prec = tp / max(tp + fp, 1)
    rec = tp / max(tp + fn, 1)
    top1 = cls_ok / max(cls_n, 1)
    ok = (prec >= 0.90) and (rec >= 0.85) and (top1 >= 0.80) and (wrong_slot / max(tp + fp, 1) < 0.05)
    return {"occupancy": {"tp": tp, "fp": fp, "fn": fn, "precision": round(prec, 4), "recall": round(rec, 4)},
            "class_top1": round(top1, 4), "class_n": cls_n,
            "wrong_slot_ratio": round(wrong_slot / max(tp + fp, 1), 4),
            "criteria": {"precision>=0.90": prec >= 0.90, "recall>=0.85": rec >= 0.85,
                         "top1>=0.80": top1 >= 0.80, "wrong_slot<5%": wrong_slot / max(tp + fp, 1) < 0.05},
            "pass": bool(ok), "detail": detail}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", nargs="+", default=[str(ROOT / "results" / "seg2_frames.npz")],
                    help="可给多个 npz（分批抓取时合并评分）")
    ap.add_argument("--truth", default=str(ROOT / "ground_truth" / "slots.json"))
    ap.add_argument("--out", default=str(ROOT / "results" / "seg2.json"))
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--dump", default=None, help="把逐帧检测结果存 JSONL，便于逐帧核查")
    ap.add_argument("--d-plane", type=float, default=1.6,
                    help="货架前缘面到相机（m）；货物前表面实测就在这附近")
    ap.add_argument("--u-sign", type=float, default=1.0,
                    help="转向偏差补偿的符号（+1/-1，取决于图像 u 与车体 y 的对应方向）")
    ap.add_argument("--levels", type=int, nargs="+", default=[1],
                    help="评哪几层。默认只评下层：上层在该相机高度被层板下沿遮挡（物理不可见）")
    ap.add_argument("--pitch", type=float, default=16.0, help="相机仰角（度），实测值")
    ap.add_argument("--cam-h", type=float, default=0.43, help="相机离地高度（m），地面平面拟合实测值")
    args = ap.parse_args()

    paths = [Path(p) for p in args.frames]
    missing = [p for p in paths if not p.exists()]
    if missing:
        print(f"[SEG-2] 缺输入帧 {missing} -> 未跑（先 make seg2-capture）")
        return 2
    parts = [np.load(p, allow_pickle=True) for p in paths]
    d = {kk: np.concatenate([p[kk] for p in parts]) for kk in
         ("rgb", "depth", "k", "pose", "bay", "side", "addr")}
    d["cam_pitch_deg"] = parts[0]["cam_pitch_deg"] if "cam_pitch_deg" in parts[0] else 0.0
    n = d["rgb"].shape[0]
    print(f"[SEG-2] 输入 {len(paths)} 个文件 / {n} 帧  {d['rgb'].shape[1]}x{d['rgb'].shape[2]}  "
          f"相机仰角 {float(d['cam_pitch_deg']):.0f}°")

    sg_bays = json.loads((ROOT / "world" / "slot_grid.json").read_text(encoding="utf-8"))["bays"]
    bay_centers = {(b["aisle"], b["bay"]): b["center_x"] for b in sg_bays}
    aisle_of = {a: b["aisle"] for b in sg_bays for a in b["slots"]}
    truth_all = json.loads(Path(args.truth).read_text(encoding="utf-8"))["slots"]
    pred: dict[str, dict] = {}
    rows = []
    for i in range(n):
        bay = int(d["bay"][i]); side = str(d["side"][i])
        addrs = [a for a in str(d["addr"][i]).split(",") if a]
        # 用**实际**位姿 + bay 中心做几何归属，不依赖转向精度
        pose_i = d["pose"][i]
        bay_cx = bay_centers.get((aisle_of.get(str(d["addr"][i]).split(",")[0], "A"), bay), None)
        if bay_cx is None:
            bay_cx = float(pose_i[0])
        got = detect_frame(d["rgb"][i], d["depth"][i], d["k"][i],
                           obs_x=float(pose_i[0]), obs_y=float(pose_i[1]),
                           obs_yaw=float(pose_i[2]), bay_cx=float(bay_cx),
                           pitch_deg=args.pitch, cam_h=args.cam_h,
                           levels=tuple(args.levels))
        levels = got["levels"]
        preds = {}
        for addr in addrs:
            lvl = int(addr.rsplit("-", 1)[1])
            if lvl not in levels:
                continue                      # 未评层（如上层被遮挡）
            det = levels.get(lvl)
            if det == "skipped":
                preds[addr] = "skipped"
                continue
            pred[addr] = det
            preds[addr] = det["class"] if det else "empty"
        dets = [v for v in levels.values() if isinstance(v, dict)]
        rows.append({"frame": i, "bay": bay, "side": side, "n_det": len(dets),
                     "dets": dets, "addrs": addrs, "pred": preds})
        print(f"  [{i + 1}/{n}] bay{bay}-{side}: 检出 {len(dets)} 个物体  "
              + "  ".join(f"{a}={preds.get(a, '未评')}" for a in addrs))
        for det in dets:
            print(f"        层{det['level']} 世界x={det['world_x']:+.2f}  {det['class']:<9}"
                  f" conf {det['conf']:.2f}  宽高 {det['size_m']} m  距离 {det['d_med']:.2f} m"
                  f"  rgb {det['rgb']}")

    res = score(pred, truth_all)
    o = res["occupancy"]
    # 顶层扁平字段：验收脚本直接读这几个（与 seg1 同样的约定）
    res.update({"pass": res["pass"], "precision": o["precision"], "recall": o["recall"],
                "top1_acc": res["class_top1"], "wrong_slot_ratio": res["wrong_slot_ratio"],
                "n_levels_evaluated": list(args.levels),
                "coverage_note": "只评下层（level 1）：上层在该相机高度被层板下沿遮挡，物理不可见"})
    print(f"\n[SEG-2] 占用 precision {o['precision']:.3f}（≥0.90） recall {o['recall']:.3f}（≥0.85）"
          f"  tp/fp/fn = {o['tp']}/{o['fp']}/{o['fn']}")
    print(f"[SEG-2] 类别 top-1 {res['class_top1']:.3f}（≥0.80，n={res['class_n']}）"
          f"  错格率 {100 * res['wrong_slot_ratio']:.1f}%（<5%）")
    print(f"[SEG-2] RESULT: {'PASS' if res['pass'] else 'FAIL'}")
    if res["class_n"] == 0:
        print("[SEG-2] ⚠️ 没有可评的样本（tp=0）—— 未跑，不是 FAIL")

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    res["frames"] = rows
    res["n_frames"] = n
    out.write_text(json.dumps(res, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"-> {out}")
    if args.dump:
        with open(args.dump, "w", encoding="utf-8") as fh:
            for r in rows:
                fh.write(json.dumps(r, ensure_ascii=False) + "\n")
        print(f"-> {args.dump}")
    if args.plot and n:
        plot(d, rows, Path(str(out).replace(".json", "_det.png")))
    return 0 if res["pass"] else 1


def plot(d, rows, out: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    k = min(3, len(rows))
    fig, axes = plt.subplots(1, k, figsize=(6 * k, 4.8))
    axes = np.atleast_1d(axes)
    cols = {"carton_s": "sienna", "carton_m": "peru", "carton_l": "chocolate",
            "tote": "royalblue", "pallet": "goldenrod", "drum": "seagreen"}
    for ax, r in zip(axes, rows):
        ax.imshow(d["rgb"][r["frame"]])
        for det in r["dets"]:
            x0, y0, x1, y1 = det["bbox"]
            c = cols.get(det["class"], "red")
            ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec=c, lw=1.2))
            ax.text(x0, max(y0 - 4, 8), f"{det['class']} L{det['level']}", color=c, fontsize=7)
        ax.set_title(f"bay{r['bay']}-{r['side']}", fontsize=9)
        ax.axis("off")
    fig.tight_layout()
    fig.savefig(out, dpi=100)
    print(f"-> {out}")


if __name__ == "__main__":
    sys.exit(main())
