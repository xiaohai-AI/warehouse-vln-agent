"""地面带 2D 光流 → 沿轴位移（无 ROS 依赖，可离线/合成数据自检）。

几何
----
相机高 h、焦距 f（px）、主点 (cx, cy)、俯仰 0，朝 +x 前进。地面点 (X, Y, 0) 投影：
    v − cy = f·h / X          →      X = f·h / (v − cy)
前进 Δx 后同一地面点的 X' = X − Δx，令 r = v − cy，则
    X = f·h/r,   X' = f·h/(r + Δv)
    Δx = X − X' = f·h·(1/r − 1/(r+Δv))          ← **精确反解，不做小量近似**

横向（麦克纳姆侧移）同样能测，而且更简单：地面点 u = cx − f·Y/X（Y=向左为正），
侧移 ΔY 时 du = +f·ΔY/X，而 X = f·h/r，故
    ΔY = du·X/f = **+du·h/r**      （Y 向左为正，符合 ROS 车体系）
**这一条是被实测逼出来的**：S4 第一次跑整条路线，轴向腿 99.5–99.9%、侧移腿 0.3%
（odom 日志里 yaw 一直 0.0° = 车没转、是横着走的），损失全在侧移上。
线性近似 Δx ≈ Δv·f·h/r² 会**系统性高估**（自检实测尺度 0.911，即 ~10% 偏大，
且随每帧步长增大：3.5 cm 时 5.7%、12 cm 时 10.2%）—— 所以这里必须用精确式。
每个特征独立给一个估计，取中位数（错误匹配是少数派，中位数对它们鲁棒）。
(v−cy)² 随行号增大 → 画面下沿（近处地面）权重最大，正是有尺度信息的那一段。

为什么不用视差/深度：掠射角地面点的双目深度误差 ∝ d²/(f·b)，越远越糟；
而 2D 光流只用到"行坐标变化"，完全不需要深度。
"""
from __future__ import annotations

import math

import cv2
import numpy as np

# 单帧超过这个位移一定是错配（0.35 m/s@10 Hz 只有 3.5 cm）
MAX_STEP_M = 0.5
# 到主点的最小行距：更靠近地平线的点几乎没有尺度信息
MIN_ROW_PX = 25.0


def focal_px(width_px: int, hfov_deg: float) -> float:
    return (width_px / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)


def band_mask(shape: tuple[int, int], band: float) -> np.ndarray:
    """画面底部 band 比例的地面带掩膜（0.40 = 最后 40% 行）。"""
    m = np.zeros(shape, np.uint8)
    m[int(shape[0] * (1.0 - band)):, :] = 255
    return m


def track_points(prev_gray: np.ndarray, gray: np.ndarray, *,
                 band: float = 0.40, mask: np.ndarray | None = None,
                 max_corners: int = 220, min_features: int = 8):
    """地面带上的 KLT 追踪，返回 (p0, p1) 像素坐标；不够则返回 None。

    抽出来是为了让「单轴公式」和「旋转补偿联合解算」共用同一份追踪，
    否则两条路径的特征分布不同，A/B 就没法归因到算法本身。
    """
    if mask is None:
        mask = band_mask(prev_gray.shape, band)
    pts = cv2.goodFeaturesToTrack(prev_gray, maxCorners=max_corners, qualityLevel=0.01,
                                  minDistance=5, blockSize=7, mask=mask)
    if pts is None or len(pts) < min_features:
        return None
    nxt, st, _ = cv2.calcOpticalFlowPyrLK(
        prev_gray, gray, pts, None, winSize=(21, 21), maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    if nxt is None or st is None:
        return None
    ok = st.reshape(-1) == 1
    p0 = pts.reshape(-1, 2)[ok]
    p1 = nxt.reshape(-1, 2)[ok]
    if len(p0) < min_features:
        return None
    return p0, p1


def solve_motion(p0: np.ndarray, p1: np.ndarray, *,
                 h: float, f: float, cx: float, cy: float,
                 dyaw: float = 0.0, max_range: float = 8.0,
                 min_inliers: int = 8, mad_k: float = 3.0,
                 h_lat: float | None = None) -> dict | None:
    """**旋转补偿的联合解算**：一次给出 (dx, dy)，并把已旋转的那部分从光流里去掉。

    与 `estimate_forward` 的差别（这是 S5 横向假位移的根因）：
      * 老式把 dx（径向反解）与 dy（行坐标）**当两件互不相关的事**算，且假设帧间**纯平移**。
        机器人边走边转、或转弯时，旋转在图像里表现为一个大范围的水平偏移，
        会被 dy 通道整个吞掉 —— 实测约 3 mm/帧，58 m 直腿累积 1.3 m。
      * 这里用精确投影把每个点还原成地面坐标：
            X = f·h/(v − cy),      Y = −(u − cx)·X/f        （Y 向左为正）
        已知帧间偏航 ψ 时，同一点在两帧的相机系坐标满足
            (X₁,Y₁) = R(−ψ)·((X₀,Y₀) − (dx,dy))   →   (dx,dy) = (X₀,Y₀) − R(ψ)·(X₁,Y₁)
        每个点给一个 (dx,dy)，再做鲁棒中心（中位 + MAD 剔外点 + 按距离加权）。

    旋转量优先用 IMU 角速度积分（`dyaw` 参数）；给 0 就退化成"忽略旋转"的老行为，
    便于自检里做 A/B。
    """
    if p0 is None or p1 is None or len(p0) < min_inliers:
        return None
    r0 = p0[:, 1] - cy
    r1 = p1[:, 1] - cy
    keep = (r0 > MIN_ROW_PX) & (r1 > MIN_ROW_PX)
    if keep.sum() < min_inliers:
        return None
    rr0, rr1 = r0[keep], r1[keep]
    X0 = f * h / rr0
    X1 = f * h / rr1
    rng_ok = (X0 > 0.15) & (X0 < max_range) & (X1 > 0.15) & (X1 < max_range)
    if rng_ok.sum() < min_inliers:
        return None
    rr0, X0, X1 = rr0[rng_ok], X0[rng_ok], X1[rng_ok]
    # 横向用独立标定常数：横向位移 ∝ h/r（对主点行 cy 的偏置是一次敏感），
    # 前进 ∝ h·(1/r − 1/r')（**二次**敏感）—— 同一个 h 在两条轴需要的修正量不同，
    # 实测横向 1.075（三次冒烟 1.074/1.078/1.075，非常稳定）→ h_lat = 0.456/1.075 = 0.424。
    # 不传 h_lat 时退化成单常数（合成自检就是这样，两轴同时精确）。
    hl = h if h_lat is None else h_lat
    Y0 = -(p0[keep][rng_ok, 0] - cx) * hl / rr0
    Y1 = -(p1[keep][rng_ok, 0] - cx) * hl / rr1
    c, s = math.cos(dyaw), math.sin(dyaw)
    # 把 t1 的坐标转回 t0 的朝向（去掉旋转），剩下的差值全是平移
    Xr = c * X1 - s * Y1
    Yr = s * X1 + c * Y1
    dx_i = X0 - Xr
    dy_i = Y0 - Yr
    ok_step = np.abs(dx_i) < MAX_STEP_M
    if ok_step.sum() < min_inliers:
        return None
    dx_i, dy_i, w = dx_i[ok_step], dy_i[ok_step], rr0[ok_step] ** 2      # 近处（r 大）更可信
    mdx, mdy = float(np.median(dx_i)), float(np.median(dy_i))
    res = np.hypot(dx_i - mdx, dy_i - mdy)
    mad = float(np.median(np.abs(res - np.median(res)))) * 1.4826
    thr = max(mad_k * mad, 0.004)
    inl = res <= thr
    if inl.sum() < min_inliers:
        return None
    wn = w[inl] / w[inl].sum()
    return dict(dx=float((wn * dx_i[inl]).sum()),
                dy=float((wn * dy_i[inl]).sum()),
                n_tracked=int(len(p0)),
                n_in=int(inl.sum()),
                resid_m=float(np.median(res[inl])),
                rot_comp_deg=float(math.degrees(dyaw)))


def estimate_forward(prev_gray: np.ndarray, gray: np.ndarray, *,
                     h: float, f: float, cy: float, band: float = 0.40,
                     mask: np.ndarray | None = None,
                     max_corners: int = 220, min_features: int = 8,
                     cx: float | None = None, h_lat: float | None = None) -> dict | None:
    """一帧的地面运动估计（前进 + 侧移）。**不补偿旋转** —— 保留它用于 A/B 对照。

    返回 {dx, dy, n_feat, med_dy_px, med_du_px, n_tracked} ；不可用时返回 None。

    `dy` 是**车体系向左**的位移（ROS 约定）；不传 cx 时只算前进分量（dy=None）。
    """
    tr = track_points(prev_gray, gray, band=band, mask=mask,
                      max_corners=max_corners, min_features=min_features)
    if tr is None:
        return None
    p0, p1 = tr
    r = p0[:, 1] - cy
    r2 = p1[:, 1] - cy
    keep = (r > MIN_ROW_PX) & (r2 > MIN_ROW_PX)
    if keep.sum() < min_features:
        return None
    rr, rr2 = r[keep], r2[keep]
    # 精确式：Δx = f·h·(1/r − 1/r')，r' = 前进后的行距（单帧步长再大也不失准）
    dx = f * h * (1.0 / rr - 1.0 / rr2)
    dv = p1[keep, 1] - p0[keep, 1]
    du = p1[keep, 0] - p0[keep, 0]
    # 侧移：ΔY = +du·h_lat/r（Y 向左为正；符号被合成自检抓过一次，别再改回去）
    # **横向有独立的标定常数**：前进是 r 的二次敏感、横向是一次敏感，同一个 h 在两条轴上
    # 需要的修正量不同（实测：轴向 h=0.456 → 比 0.994；横向用 0.456 → 比 1.074，
    # 故 h_lat = 0.456/1.074 = 0.425）。见 DECISIONS D13。
    lat = du * (h_lat if h_lat is not None else h) / rr
    good = np.abs(dx) < MAX_STEP_M
    if good.sum() < min_features - 1:
        return None
    out = dict(dx=float(np.median(dx[good])),
               n_feat=int(good.sum()),
               med_dy_px=float(np.median(np.abs(dv[good]))),
               med_du_px=float(np.median(np.abs(du[good]))),
               n_tracked=int(len(p0)),
               dy=float(np.median(lat[good])) if cx is not None else None)
    return out


def floor_fraction(gray: np.ndarray, band: float, max_corners: int = 600) -> float:
    """全图特征里落在底面带的比例（量化"地面信号是否被周期货架淹没"）。"""
    full = cv2.goodFeaturesToTrack(gray, maxCorners=max_corners, qualityLevel=0.01,
                                   minDistance=5, blockSize=7)
    if full is None or len(full) == 0:
        return float("nan")
    cut = int(gray.shape[0] * (1.0 - band))
    inband = sum(1 for p in full if p[0][1] >= cut)
    return inband / len(full)


def fit_scale(est: list[float], truth: list[float]) -> float:
    """最优尺度 s = Σ(t·e)/Σ(e²)；s≈1 说明几何常数（h/f）标定对了。"""
    e = np.asarray(est, dtype=float)
    t = np.asarray(truth, dtype=float)
    m = np.isfinite(e) & np.isfinite(t)
    e, t = e[m], t[m]
    if len(e) < 3 or float((e ** 2).sum()) <= 1e-12:   # 单参数拟合，3 个点即可
        return float("nan")
    return float((t * e).sum() / float((e ** 2).sum()))
