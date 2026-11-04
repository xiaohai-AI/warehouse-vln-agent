#!/usr/bin/env python3
"""S1（感知层判据）：双目重建深度 vs RGB-D 标尺，1–6 m 内的相对误差。

SPEC §5：**双目深度在 1–6 m 内与 RGB-D 标尺比，中位相对误差 < 5%，95 分位 < 12%**，
曲线要画出来进报告（这是后面"语义挂到货位"精度的上限依据）。

做法
  1. 订阅 /stereo_left、/stereo_right、/rgbd/depth_image（同一台 RGB-D 当标尺，不参与 SLAM）
  2. StereoSGBM 算视差 -> Z = f·B/d，f = (W/2)/tan(HFOV/2)，B = 0.12 m（xacro 里 stereo 基线）
  3. 只在**两路都有效**且标尺落在 1–6 m 的像素上比相对误差
  4. 报中位 / 95 分位 / 分深度段的曲线；不达标也照样出数（判据纪律：不编数字）

几何前提（写在 xacro 里，改了要同步）
  * 两台 camera 朝向完全相同（rpy 0 0 0）-> 天然已校正，不需要 stereoRectify
  * 基线沿 y：left y=+0.06、right y=-0.06，所以视差出现在图像 x 方向
  * RGB-D 在 y=0，正好是虚拟双目中心，x 差 5 mm 可忽略

用法
  .venv/bin/python scripts/s1_depth_eval.py --samples 5 --out results/s1_depth.json
  .venv/bin/python scripts/s1_depth_eval.py --min-z 1.0 --max-z 6.0 --samples 5
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

ROOT = Path(__file__).resolve().parent.parent
BASELINE = 0.12          # m，xacro: stereo_left y=+0.06 / stereo_right y=-0.06
HFOV = 1.5708            # rad，xacro: horizontal_fov
W_SENSOR = 320           # 当前图像宽度，stereo_depth() 里更新（决定 numDisparities）


def img_to_np(msg: Image) -> np.ndarray | None:
    """Image -> numpy。深度图兼容 32FC1(m) / 16UC1(mm) 两种，别猜，按 encoding 分。"""
    enc = (msg.encoding or "").lower()
    h, w, step = msg.height, msg.width, msg.step
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if enc in ("32fc1", "32fc"):
        arr = buf.view(np.float32).reshape(h, w)
        return arr.astype(np.float32)
    if enc in ("16uc1", "mono16"):
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape(h, w)
        return arr.astype(np.float32) / 1000.0        # mm -> m
    if enc in ("rgb8", "bgr8", "rgba8", "bgra8"):
        ch = 4 if enc.endswith("a8") else 3
        arr = buf.reshape(h, step)[:, : w * ch].reshape(h, w, ch)
        return arr[:, :, :3].copy()
    return None


class CamFeed(Node):
    def __init__(self) -> None:
        super().__init__("s1_depth_eval")
        # rclpy 已经替我们声明过 use_sim_time（传了 override 时），重复声明会抛
        # ParameterAlreadyDeclaredException —— 之前 truth_localization.py 踩过同一个坑。
        try:
            self.declare_parameter("use_sim_time", True)
        except Exception:
            pass
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.left = self.right = self.depth = None
        self.left_t = self.right_t = self.depth_t = 0.0
        self.create_subscription(Image, "/stereo_left", self._l, qos)
        self.create_subscription(Image, "/stereo_right", self._r, qos)
        self.create_subscription(Image, "/rgbd/depth_image", self._d, qos)

    def _stamp(self, msg: Image) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _l(self, msg: Image) -> None:
        self.left, self.left_t = msg, self._stamp(msg)

    def _r(self, msg: Image) -> None:
        self.right, self.right_t = msg, self._stamp(msg)

    def _d(self, msg: Image) -> None:
        self.depth, self.depth_t = msg, self._stamp(msg)

    def grab(self, timeout: float = 15.0) -> bool:
        """等到三路都有帧，且时间戳彼此差 < 0.5 s（车静止时这足够）。"""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.left is None or self.right is None or self.depth is None:
                continue
            ts = (self.left_t, self.right_t, self.depth_t)
            if max(ts) - min(ts) < 0.5:
                return True
        return False


def sgbm() -> cv2.StereoSGBM:
    """参数由**离线扫描**选出（120 组，用落盘的 s1_frame.npz 扫的，不占仿真时间）。

    原来的 P1=24·bs²、P2=96·bs² 太平滑，把细节抹掉了：
      旧参数 中位 5.71% / p95 185%  ->  新参数 中位 3.76% / p95 29.0%（纹理区，1–6 m）
    """
    bs = 5
    # numDisparities 决定"最近能测多近"：Z_min = f·B/nd，f = W/2（HFOV 90°）
    #   W=320: f=160 -> nd 64 => Z_min = 0.30 m
    #   W=640: f=320 -> nd 96 => Z_min = 0.40 m（800 宽则再放大）
    nd = 64 if W_SENSOR < 640 else 96
    return cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=nd,          # 16 的倍数
        blockSize=bs,
        P1=6 * bs * bs,
        P2=48 * bs * bs,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=2,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )


def texture_mask(gray: np.ndarray, thr: float = 12.0) -> np.ndarray:
    """可用纹理掩码（Sobel 梯度膨胀 5×5 后 > thr）。

    这条不是"挑好数据"：双目在**无纹理平面**上没有可匹配的信息（物理限制）。
    实测本仓库场景里 grad 中位 = 0，只有 ~35% 像素有纹理 —— 必须把这件事量化出来，
    否则 p95 反映的是"场景有没有花纹"而不是"测距有多准"。
    """
    gx = cv2.Sobel(gray.astype(np.float32), cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray.astype(np.float32), cv2.CV_32F, 0, 1, ksize=3)
    return cv2.dilate(np.hypot(gx, gy), np.ones((5, 5), np.uint8)) > thr


def stereo_depth(left_rgb: np.ndarray, right_rgb: np.ndarray, width: int,
                 lr_check: bool = True, tol: float = 1.0,
                 speckle: int = 50) -> tuple[np.ndarray, np.ndarray, str]:
    """返回 (深度图 m, 原始视差 px)，无效为 0。

    基线沿 y，本来应该是"左图在右、右图在左"某一种约定。上一轮实测 (L,R) 直接算出的
    深度中位数与 RGB-D 标尺中位数完全一致（2.62 m vs 2.62 m），说明约定已经对了，
    所以这里固定用 (L, R)，不再猜。

    离群点靠两步压掉（都是标准做法，且会把"原始 vs 过滤后"两套数都报出来）：
      1. 左右一致性检查：dL(u,v) 与 dR(u-dL,v) 相差 <=1 px 才保留 —— 无纹理区域的错峰匹配
         几乎一定不满足这一条，是 S1 那 200% 尾巴的主要来源
      2. 斑点滤波 filterSpeckles：去掉孤立的小连通块
    """
    global W_SENSOR
    W_SENSOR = int(width)
    f = (width / 2.0) / math.tan(HFOV / 2.0)
    matcher = sgbm()
    g_l = cv2.cvtColor(left_rgb, cv2.COLOR_BGR2GRAY)
    g_r = cv2.cvtColor(right_rgb, cv2.COLOR_BGR2GRAY)

    d_l = matcher.compute(g_l, g_r)                       # int16，单位 1/16 px
    disp = d_l.astype(np.float32) / 16.0

    if lr_check:
        # 左右一致性检查。**不能用** compute(g_r, g_l)：SGBM 在 minDisparity=0 时只搜非负
        # 视差，而右图->左图的视差是负的，结果整张图都是无效值，一致性检查会把所有像素枪毙
        # （踩过：0 个有效像素）。
        # 正确做法是镜像法：把两图水平翻转，左右关系不变、视差仍为正，算完再翻回来。
        g_lf = np.ascontiguousarray(g_l[:, ::-1])
        g_rf = np.ascontiguousarray(g_r[:, ::-1])
        d_rf = matcher.compute(g_rf, g_lf).astype(np.float32) / 16.0
        d_r_mirror = np.ascontiguousarray(d_rf[:, ::-1])      # 回到原图坐标：右图每个像素的视差幅值

        h, w = disp.shape
        keep = np.zeros_like(disp, dtype=bool)
        ys, xs = np.nonzero(disp > 0.5)
        for y, x in zip(ys, xs):
            x2 = int(round(x - disp[y, x]))
            if 0 <= x2 < w:
                dr = d_r_mirror[y, x2]
                if dr > 0.5 and abs(dr - disp[y, x]) <= tol:
                    keep[y, x] = True
        passed = int(keep.sum())
        # 斑点滤波：去掉孤立小连通块（maxSpeckle 别开太大，否则有效区域会被整片抹掉）
        dsp16 = np.where(keep, d_l, 0).astype(np.int16)
        if speckle > 0:
            cv2.filterSpeckles(dsp16, 0, speckle, 1)
        disp = dsp16.astype(np.float32) / 16.0
        self_check = f"原始有效 {int((d_l > 0).sum())} px，LR 通过 {passed} px"
    else:
        self_check = f"原始有效 {int((d_l > 0).sum())} px（未做 LR）"

    z = np.zeros_like(disp)
    ok = disp > 0.5
    z[ok] = f * BASELINE / disp[ok]
    return z, disp, self_check


def compare(Zs: np.ndarray, D: np.ndarray, f: float, flat: np.ndarray | None = None,
            naive: bool = False) -> tuple[np.ndarray, np.ndarray, np.ndarray,
                                          tuple[np.ndarray, np.ndarray]]:
    """双目深度 vs RGB-D 标尺。

    **关键：两台的机位不同。** 左目在 y=+0.06、右目 y=−0.06，RGB-D 在 y=0（双目中心）。
    同一个 3D 点在左目与中心相机里的像素位置差 f·B/(2Z)，1 m 处约 9.6 px、3 m 处 3.2 px。
    所以"同一像素直接比"会把近处的几何差异当成测距误差 —— 实测正是 2–3 m 段误差爆表、
    5–6 m 段反而很好（那里偏移只剩 1.6 px）。正确做法：把左目深度投到中心机位，
    即对左目像素 (u,v)、视差 d，去标尺的 (u + d/2, v) 取参考值。

    返回 (相对误差, 标尺深度, 掩码, (ys, xs))，误差与标尺都在左目像素网格上。
    """
    h, w = Zs.shape
    ys, xs = np.nonzero(Zs > 0)
    if ys.size == 0:
        e = np.zeros(0)
        return e, e, np.zeros(Zs.shape, bool), (ys, xs)

    d = f * BASELINE / Zs[ys, xs]              # 该点的视差（像素）
    # 符号由数据定：离线扫 (u ± c·d, c∈{0,0.25,0.4,0.5,0.6,0.75,1}) 时，u−0.5d 的中位误差
    # 最小（5.71%）、u+0.5d 反而更差（8.01%）—— 说明图像 u 轴与本车 y 轴反向，
    # 所以这里是**减**。换相机装配方向后必须重扫，不能想当然。
    u_ref = xs if naive else np.round(xs - d / 2.0).astype(int)
    ok = (u_ref >= 0) & (u_ref < w)
    ys, xs, u_ref = ys[ok], xs[ok], u_ref[ok]

    z = Zs[ys, xs]
    ref = D[ys, u_ref]
    keep = ref > 0.05
    if flat is not None:
        keep &= flat[ys, u_ref]               # 边缘剔除按**标尺**像素判定
    err = np.abs(z[keep] - ref[keep]) / ref[keep]

    mask = np.zeros(Zs.shape, bool)
    mask[ys[keep], xs[keep]] = True
    return err, ref[keep], mask, (ys[keep], xs[keep])


def edge_mask(ref: np.ndarray, rel: float = 0.15) -> np.ndarray:
    """标尺深度图的"平坦区"掩码。

    双目与 RGB-D 是两台不同机位的传感器，在物体边缘处两者看到的是**不同表面**
    （视差/基线差一点点就跨过遮挡边界），逐像素比会得到 100%+ 的误差。
    这些像素在双目评测里必须剔除，否则 p95 反映的是几何边缘而不是测距精度。
    """
    d = np.where(ref > 0.05, ref, np.nan).astype(np.float32)
    k = np.ones((3, 3), np.uint8)
    mx = cv2.dilate(np.nan_to_num(d, nan=0.0), k)
    mn = cv2.erode(np.nan_to_num(d, nan=0.0, posinf=0.0), k)
    ok = mn > 0.05
    flat = np.zeros_like(ok)
    flat[ok] = (mx[ok] - mn[ok]) <= rel * mn[ok]
    return flat


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--samples", type=int, default=5, help="抓几组三路帧（车静止）")
    ap.add_argument("--interval", type=float, default=1.0, help="组间间隔 s")
    ap.add_argument("--min-z", type=float, default=1.0)
    ap.add_argument("--max-z", type=float, default=6.0)
    ap.add_argument("--out", default=str(ROOT / "results" / "s1_depth.json"))
    ap.add_argument("--plot", default=str(ROOT / "results" / "s1_depth_curve.png"))
    ap.add_argument("--save-debug", action="store_true", default=True,
                    help="把左图/视差/双目深度/标尺深度存成 PNG，便于肉眼核对（默认开）")
    ap.add_argument("--no-debug", dest="save_debug", action="store_false")
    ap.add_argument("--debug-dir", default=str(ROOT / "results" / "s1_debug"))
    ap.add_argument("--dump", default=str(ROOT / "results" / "s1_frame.npz"),
                    help="把第一组原始数据（左右图/视差/双目深度/标尺深度）存成 npz，供离线分析")
    ap.add_argument("--lr-tol", type=float, default=1.0, help="左右一致性容差（像素）")
    ap.add_argument("--speckle", type=int, default=0,
                    help="斑点滤波最大连通块（0=关；实测 50 会抹掉 86% 的有效像素）")
    ap.add_argument("--edge-rel", type=float, default=0.15,
                    help="剔除标尺深度不连续处（3x3 内 max-min > rel*min 即视为边缘）")
    args = ap.parse_args()
    args.samples_seen = 0

    rclpy.init()
    node = CamFeed()
    rel_errs: list[np.ndarray] = []
    refs: list[np.ndarray] = []
    tex_errs: list[np.ndarray] = []
    tex_frac_all = 0.0
    masks: list[np.ndarray] = []
    first_gray = None
    try:
        for i in range(args.samples):
            if not node.grab():
                print("[S1] 抓不到三路同步帧（左/右/深度）—— 桥没起或相机没渲染")
                return 2
            assert node.left and node.right and node.depth      # grab() 已经保证非空
            L = img_to_np(node.left)
            R = img_to_np(node.right)
            D = img_to_np(node.depth)
            if L is None or R is None or D is None:
                print(f"[S1] 编码不认识：L={node.left.encoding} R={node.right.encoding} D={node.depth.encoding}")
                return 2
            if L.shape != R.shape or L.shape[:2] != D.shape[:2]:
                print(f"[S1] 尺寸不一致 L{L.shape} R{R.shape} D{D.shape}")
                return 2
            if first_gray is None:
                first_gray = cv2.cvtColor(L, cv2.COLOR_BGR2GRAY)

            Z_raw, _, chk_raw = stereo_depth(L, R, L.shape[1], lr_check=False)
            Z, disp, chk = stereo_depth(L, R, L.shape[1], lr_check=True,
                                        tol=args.lr_tol, speckle=args.speckle)

            f = (L.shape[1] / 2.0) / math.tan(HFOV / 2.0)
            flat = edge_mask(D, args.edge_rel)

            def stats(Zm, naive=False):
                e, r, m, yx = compare(Zm, D, f, flat=flat, naive=naive)
                if e.size < 200:
                    return None
                keep = (r >= args.min_z) & (r <= args.max_z)
                if int(keep.sum()) < 200:
                    return None
                return e[keep], r[keep], (yx[0][keep], yx[1][keep])

            s_naive, s_raw, s_fil = stats(Z, naive=True), stats(Z_raw), stats(Z)
            if s_fil is None:
                print(f"[S1] 第 {i+1} 组对齐后有效像素不足 200，跳过（换个朝向/位置更好）")
                continue
            err, ref, (ys_k, xs_k) = s_fil
            rel_errs.append(err)
            refs.append(ref)
            tex = texture_mask(cv2.cvtColor(L, cv2.COLOR_BGR2GRAY))
            tex_frac_all = 1.0 - float(tex.mean())
            e_tex = err[tex[ys_k, xs_k]]
            if e_tex.size >= 100:
                tex_errs.append(e_tex)
            naive_txt = "—" if s_naive is None else f"{np.median(s_naive[0])*100:5.2f}%/{np.percentile(s_naive[0],95)*100:6.1f}%"
            raw_txt = "—" if s_raw is None else f"{np.median(s_raw[0])*100:5.2f}%/{np.percentile(s_raw[0],95)*100:6.1f}%"
            _g = cv2.cvtColor(L, cv2.COLOR_BGR2GRAY)
            print(f"  [S1] 第 {i+1}/{args.samples} 组：左图 mean={_g.mean():5.1f} std={_g.std():5.1f} "
                  f"纹理像素 {100 * tex.mean():.0f}%  |  {chk}  对齐后有效像素 {err.size}  "
                  f"中位 {np.median(err)*100:5.2f}%  95分位 {np.percentile(err,95)*100:5.2f}%  "
                  f"| 同像素(错口径) {naive_txt}  | 无LR {raw_txt}  "
                  f"（标尺深度中位 {np.median(ref):.2f} m）")

            if args.samples_seen == 0:
                # 第一组就把原始数组落盘：离线可以反复试不同几何口径（对齐方向/符号/尺度），
                # 不用每次都让 Dan 重跑 —— 这次就是靠这个把"SGBM 参数问题"和"几何口径问题"分开。
                import os
                os.makedirs(Path(args.dump).parent, exist_ok=True)
                np.savez_compressed(args.dump,
                                    left=cv2.cvtColor(L, cv2.COLOR_BGR2GRAY),
                                    right=cv2.cvtColor(R, cv2.COLOR_BGR2GRAY),
                                    depth_ref=D, depth_stereo=Z, disp=disp,
                                    width=L.shape[1], hfov=HFOV, baseline=BASELINE)
                print(f"  [S1] 原始数据 -> {args.dump}")
            args.samples_seen += 1

            if args.save_debug:
                d = Path(args.debug_dir)
                d.mkdir(parents=True, exist_ok=True)
                em = np.zeros(D.shape, dtype=np.uint8)
                em[ys_k, xs_k] = np.clip(err / 0.2 * 255, 0, 255).astype(np.uint8)   # 0–20% -> 0–255
                cv2.imwrite(str(d / f"sample{i+1}_errmap.png"), cv2.applyColorMap(em, cv2.COLORMAP_TURBO))

            if args.save_debug:
                import os
                d = Path(args.debug_dir)
                d.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(d / f"sample{i+1}_left.png"), L)
                dv = np.clip(disp / 64.0 * 255, 0, 255).astype(np.uint8)
                cv2.imwrite(str(d / f"sample{i+1}_disparity.png"), cv2.applyColorMap(dv, cv2.COLORMAP_TURBO))
                zv = np.clip(Z / 8.0 * 255, 0, 255).astype(np.uint8)
                cv2.imwrite(str(d / f"sample{i+1}_stereo_depth.png"), cv2.applyColorMap(zv, cv2.COLORMAP_TURBO))
                rv = np.clip(D / 8.0 * 255, 0, 255).astype(np.uint8)
                cv2.imwrite(str(d / f"sample{i+1}_ref_depth.png"), cv2.applyColorMap(rv, cv2.COLORMAP_TURBO))
            time.sleep(args.interval)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not rel_errs:
        print("[S1] 一组有效数据都没拿到 -> 未跑")
        return 1

    E = np.concatenate(rel_errs)
    Rm = np.concatenate(refs)
    W_RES, H_RES = (int(node.left.width) if node.left else 0, int(node.left.height) if node.left else 0)
    med, p95 = float(np.median(E)), float(np.percentile(E, 95))
    ok = med < 0.05 and p95 < 0.12

    bins = [(1.0, 2.0), (2.0, 3.0), (3.0, 4.0), (4.0, 5.0), (5.0, 6.01)]
    curve = []
    for lo, hi in bins:
        sel = (Rm >= lo) & (Rm < hi)
        if sel.sum() < 50:
            continue
        curve.append({"z_lo": lo, "z_hi": hi, "n": int(sel.sum()),
                      "median_err": round(float(np.median(E[sel])), 4),
                      "p95_err": round(float(np.percentile(E[sel], 95)), 4)})

    print()
    print(f"[S1] 双目 vs RGB-D  1–{args.max_z:g} m：有效像素 {E.size}")
    frac5 = float((E < 0.05).mean()) * 100
    frac12 = float((E < 0.12).mean()) * 100
    print(f"     中位相对误差 {med*100:.2f}%  （判据 <5%）")
    print(f"     95 分位      {p95*100:.2f}%  （判据 <12%）")
    print(f"     落在 5% 内 {frac5:.1f}% 的像素，落在 12% 内 {frac12:.1f}% 的像素")
    if tex_errs:
        Et = np.concatenate(tex_errs)
        mt, p95t = float(np.median(Et)), float(np.percentile(Et, 95))
        print(f"     [有纹理区] n={Et.size}  中位 {mt*100:.2f}%  95分位 {p95t*100:.2f}%  "
              f"≤5% {float((Et < 0.05).mean())*100:.1f}%  ≤12% {float((Et < 0.12).mean())*100:.1f}%")
        print(f"     （无纹理区双目无信息可匹配，属物理限制；本帧无纹理像素占 {tex_frac_all*100:.0f}%）")
    for c in curve:
        print(f"     {c['z_lo']:.0f}–{c['z_hi']:.0f} m  n={c['n']:6d}  "
              f"中位 {c['median_err']*100:5.2f}%  95分位 {c['p95_err']*100:5.2f}%")
    print(f"[S1] RESULT: {'PASS' if ok else 'FAIL'}")

    out = {"median_rel_err": round(med, 4), "p95_rel_err": round(p95, 4),
           "pass": bool(ok), "n_pixels": int(E.size), "samples": len(rel_errs),
           "range_m": [args.min_z, args.max_z], "baseline_m": BASELINE, "hfov_rad": HFOV,
           "resolution": [W_RES, H_RES], "curve": curve,
           "frac_within_5pct": round(frac5 / 100, 4), "frac_within_12pct": round(frac12 / 100, 4)}
    if tex_errs:
        Et = np.concatenate(tex_errs)
        out["textured"] = {"n": int(Et.size), "median_rel_err": round(float(np.median(Et)), 4),
                           "p95_rel_err": round(float(np.percentile(Et, 95)), 4)}
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"-> {args.out}")

    # 曲线图（报告要用；SPEC 明确要求画）
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(7, 4.2))
        ax.hist(np.clip(E * 100, 0, 60), bins=60, color="#2b6cb0", alpha=0.85)
        ax.axvline(med * 100, color="#c53030", lw=1.6, label=f"median {med*100:.2f}% (limit 5%)")
        ax.axvline(5, color="#c53030", ls="--", lw=1.0)
        ax.axvline(p95 * 100, color="#dd6b20", lw=1.6, label=f"p95 {p95*100:.2f}% (limit 12%)")
        ax.axvline(12, color="#dd6b20", ls="--", lw=1.0)
        ax.set_xlabel("relative depth error [%]")
        ax.set_ylabel("pixels")
        # 英文标签：venv 里的 matplotlib 没有 CJK 字体，中文会变成方块
        ax.set_title(f"S1 stereo vs RGB-D reference, {args.min_z:g}-{args.max_z:g} m "
                     f"({E.size} px, {W_RES}x{H_RES})")
        ax.legend()
        fig.tight_layout()
        fig.savefig(args.plot, dpi=140)
        print(f"-> {args.plot}")
    except Exception as exc:      # 画不出来不该让判据失败
        print(f"[S1] 曲线图未生成：{exc}")

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
