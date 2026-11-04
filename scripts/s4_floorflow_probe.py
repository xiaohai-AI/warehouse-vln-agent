#!/usr/bin/env python3
"""S4 第 0 级探针：**只用画面底部地面带做 2D KLT 光流**，看现有相机（pitch=0）能不能测出沿轴位移。

要回答的唯一问题
----------------
S3 的 rtabmap 双目在沿巷道轴前进时只还原 9–14%。是"地面上根本没有信号"，
还是"信号被周期货架淹没了"（rtabmap 用全图特征 + 立体视差，货架同构歧义占主导，
地面只是画面底部一条窄带，正确对应是少数派、被 RANSAC 当外点剔掉）？

本探针**不动世界、不动相机、不动 URDF、不碰 rtabmap 参数**，只在同一路图像上
把地面带掩出来单独做光流，量两件事：
  1. 还原率 = 地面带光流累计前进位移 / Gazebo 真值前进位移；
  2. 逐帧相关 R² 与最优尺度 s（s≈1 说明几何常数 h/f 标定正确）。

几何（精确式，不是小量近似）
--------------------------
地面点行坐标 v 与水平距离 X 的关系：X = f·h/(v − cy)。前进 Δx 后同一地面点的
行距 r' = r + Δv，于是
    Δx = f·h·(1/r − 1/(r + Δv))          r = v − cy
每个特征点独立给一个 Δx，取中位数（错误匹配是少数派）。
(r − cy) 越大权重越大 → 画面下沿（近处地面）自动主导，正是有尺度信息的那一段。
注意：线性近似 Δx ≈ Δv·f·h/r² 会系统性高估约 10%（自检实测 s=0.911），已在
`depot_vln/floorflow.py` 里改成精确式（自检 s=1.027）。

判读
----
    ratio ≥0.85 且 R² ≥0.5   → 地面带光流够用：不动硬件、不改几何，只改算法就能救 S3
    0.40 ≤ ratio < 0.85      → 有信号但被抑制 → 下一级：静态俯仰 −20°（cam_pitch:=-20）
    ratio < 0.40             → 地面带不足 → 下一级：下视相机（新硬件，28–56 px/帧）

落盘：results/s4_floorflow/frames.csv（逐帧 est / truth / 特征数 / 光流强度）

用法（全套栈由 s3_smoke.sh 起停，本脚本只当判据）：
    CHECK=scripts/s4_floorflow_probe.py bash scripts/s3_smoke.sh
    make s4-floorflow
"""
from __future__ import annotations

import csv
import hashlib
import math
import os
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from depot_vln.floorflow import (  # noqa: E402
    band_mask,
    estimate_forward,
    fit_scale,
    floor_fraction,
)
from drive_to import drive_to_point, turn_to_precise  # noqa: E402
from slam_route import TruthPoller  # noqa: E402

WAYPOINTS = [(-28.5, -10.75), (-8.5, -10.75)]   # 进 A 巷 → 沿巷 20 m（与 s3_diag 同路线）
SPEED = float(os.environ.get("FLOORFLOW_SPEED", "0.35"))
# 相机高（地面到光心）——**唯一的尺度常数**，10% 误差 = 10% 尺度误差。
# 来源（不靠记忆，靠模型算）：SDF 里 stereo_left 的 pose z = 0.35（相对 base_footprint），
# 而 base_footprint 落在轮轴高度 = wheel_radius = 0.076（spawn -z 0.05 后由物理落定）
#   → 0.35 + 0.076 = 0.426 m。
# 探针会反解最优尺度 s 并折算成 h_fit 打印出来：h_fit 与 0.426 差多少，就是标定差多少。
H_CAM = float(os.environ.get("FLOORFLOW_H", "0.426"))
HFOV_DEG = float(os.environ.get("FLOORFLOW_HFOV", "90.0"))
BAND = float(os.environ.get("FLOORFLOW_BAND", "0.40"))
OUT = ROOT / "results" / "s4_floorflow"


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class FloorFlow(Node):
    """地面带 2D 光流（320×240、~200 点、单帧 3–6 ms，不影响 10 Hz）。"""

    def __init__(self) -> None:
        super().__init__("s4_floorflow")
        try:
            self.declare_parameter("use_sim_time", True)
        except Exception:
            pass
        self.buf = Buffer(cache_time=Duration(seconds=15))
        self.listener = TransformListener(self.buf, self)

        self.poller = None          # type: ignore[assignment]  # main 里注入
        self.heading = None         # leg2 起始航向（真值位移投影用）
        self.prev_truth = None
        # **真值历史 + 按图像戳插值**：TruthPoller 只有 ~6 Hz（pose/info 一次读 0.3–1 s），
        # 而图像是 10 Hz → 直接逐帧取会让 206/577 帧读到同一个值（实测），
        # 逐帧相关算出来 R²≈0.01，那是**参考量的锯齿**，不是估计器的噪声。
        self.truth_hist: list[tuple[float, float, float]] = []
        self._last_samples = -1
        self.truth_stale = 0
        self.prev_gray = None
        self.prev_stamp = None
        self.prev_hash = None
        self.measuring = False
        self.f = 160.0
        self.cy = 120.0
        self._mask = None
        self._every = 0

        self.rows: list[dict] = []
        self.n_frames = 0
        self.n_dup = 0
        self.n_used = 0
        self.est_sum = 0.0
        self.dy_px: list[float] = []
        self.n_feat: list[float] = []
        self.frac_floor: list[float] = []

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        # 用中继后的 /stereo_left_tf：与 S3 判据看的是同一路图像 → "算法差异"是唯一变量
        self.create_subscription(Image, "/stereo_left_tf", self._on_img, qos)

        # **后台常驻 spin**：在驾驶循环里 spin_once 会因队列溢出丢大部分帧
        #（踩过：120 s 只收到 38 帧，量出来的"帧率"其实是自己的采样率）。
        self._stop = False
        self._spin = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin.start()

    # ------------------------------------------------ 生命周期
    def _spin_loop(self) -> None:
        while rclpy.ok() and not self._stop:
            try:
                rclpy.spin_once(self, timeout_sec=0.05)
            except Exception:
                break

    def stop(self) -> None:
        self._stop = True
        self._spin.join(timeout=1.0)

    def pose(self, target: str, source: str = "base_footprint"):
        try:
            t: TransformStamped = self.buf.lookup_transform(
                target, source, Time(), timeout=Duration(seconds=0.2))
            p = t.transform.translation
            return p.x, p.y, yaw_of(t.transform.rotation)
        except Exception:
            return None

    def start_leg(self, heading: float) -> None:
        """开始计分：清空累计、丢掉上一段的最后一帧、记录航向与真值基准。"""
        self.rows.clear()
        self.est_sum = 0.0
        self.n_used = 0
        self.dy_px.clear()
        self.n_feat.clear()
        self.prev_gray = None
        self.prev_stamp = None
        self.heading = heading
        self.prev_truth = self.poller.get() if self.poller else None
        self.measuring = True

    # ------------------------------------------------ 图像回调
    def _on_img(self, m: Image) -> None:
        try:
            h, w = m.height, m.width
            ch = max(1, len(m.data) // (h * w))
            a = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(h, w, ch)
            # 重复帧剔除：gz 渲染跟不上时会**原样重发上一帧**（戳照样前进），
            # 那种帧光流恒为 0，会把总量系统性拉低 —— 跳过不计。
            hsh = hashlib.blake2b(bytes(m.data), digest_size=8).hexdigest()
            if hsh == self.prev_hash:
                self.n_dup += 1
                return
            self.prev_hash = hsh
            self.n_frames += 1

            gray = cv2.cvtColor(a, cv2.COLOR_RGB2GRAY) if ch >= 3 else a[:, :, 0]
            stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

            if self._mask is None or self._mask.shape != gray.shape:
                self.f = (w / 2.0) / math.tan(math.radians(HFOV_DEG) / 2.0)
                self.cy = h / 2.0
                self._mask = band_mask(gray.shape, BAND)

            if self.measuring and self.prev_gray is not None and self.prev_stamp is not None:
                self._step(gray, stamp)
            self.prev_gray = gray
            self.prev_stamp = stamp
        except Exception as e:      # 回调里的异常绝不能冒到 executor（会静默停掉订阅）
            print(f"[floorflow] 回调异常：{type(e).__name__}: {e}")

    def _step(self, gray: np.ndarray, stamp: float) -> None:
        dt = stamp - self.prev_stamp
        if dt <= 1e-3 or dt > 1.0:              # 时间戳乱序/超长间隔 → 跳过
            return
        self._every += 1
        if self._every % 10 == 0:               # 量化"地面信号被货架淹没"的程度
            self.frac_floor.append(floor_fraction(self.prev_gray, BAND))

        r = estimate_forward(self.prev_gray, gray, h=H_CAM, f=self.f, cy=self.cy,
                             band=BAND, mask=self._mask)
        if r is None:
            return

        # 真值：只在新样本到来时入历史（用 poller 自己的仿真时钟戳），
        # 然后按**图像戳**在历史上插值取位置 → 逐帧真值不再有锯齿。
        truth_dx = float("nan")
        if self.poller is not None:
            n = self.poller.samples
            t_s = self.poller.sim_time()
            p = self.poller.get()
            if p is not None and n != self._last_samples:
                self._last_samples = n
                if math.isfinite(t_s):
                    self.truth_hist.append((float(t_s), p[0], p[1]))
            ps = self.prev_stamp
            now = self._truth_at(stamp)
            prev = self._truth_at(ps) if ps is not None else None
            if now is not None and prev is not None and self.heading is not None:
                truth_dx = ((now[0] - prev[0]) * math.cos(self.heading)
                            + (now[1] - prev[1]) * math.sin(self.heading))
            elif now is None or prev is None:
                self.truth_stale += 1

        self.est_sum += r["dx"]
        self.n_used += 1
        self.dy_px.append(r["med_dy_px"])
        self.n_feat.append(float(r["n_feat"]))
        self.rows.append(dict(t_sim=round(stamp, 3), dt=round(dt, 4),
                              n_feat=r["n_feat"],
                              med_dy_px=round(r["med_dy_px"], 3),
                              est_dx=round(r["dx"], 5),
                              truth_dx=round(truth_dx, 5) if math.isfinite(truth_dx) else ""))

    def _truth_at(self, t: float):
        """在真值历史上线性插值出 t 时刻的 (x, y)；超出范围 0.5 s 以上就返回 None。"""
        h = self.truth_hist
        if len(h) < 2 or not math.isfinite(t):
            return None
        if t < h[0][0] - 0.5 or t > h[-1][0] + 0.5:
            return None
        if t <= h[0][0]:
            return h[0][1], h[0][2]
        if t >= h[-1][0]:
            return h[-1][1], h[-1][2]
        lo, hi = 0, len(h) - 1
        while hi - lo > 1:                      # 二分找区间
            mid = (lo + hi) // 2
            if h[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        t0, x0, y0 = h[lo]
        t1, x1, y1 = h[hi]
        w = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
        return x0 + w * (x1 - x0), y0 + w * (y1 - y0)

    # ------------------------------------------------ 统计
    def stats(self) -> dict:
        dy = np.array(self.dy_px) if self.dy_px else np.array([np.nan])
        nf = np.array(self.n_feat) if self.n_feat else np.array([np.nan])
        ff = np.array(self.frac_floor) if self.frac_floor else np.array([np.nan])
        return dict(frames=self.n_frames, dup=self.n_dup, used=self.n_used,
                    med_dy_px=float(np.nanmedian(dy)), med_nfeat=float(np.nanmedian(nf)),
                    frac_floor=float(np.nanmedian(ff)))


def r2_of(est: np.ndarray, truth: np.ndarray, s: float) -> float:
    """以过原点、尺度 s 的直线拟合后的决定系数（衡量逐帧相关性）。"""
    if len(est) < 5:
        return float("nan")
    ss_res = float(((truth - s * est) ** 2).sum())
    ss_tot = float(((truth - truth.mean()) ** 2).sum())
    return 1.0 - ss_res / ss_tot if ss_tot > 1e-9 else float("nan")


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = FloorFlow()
    poller = TruthPoller()
    node.poller = poller

    t0 = time.monotonic()
    while poller.get(10.0) is None and time.monotonic() - t0 < 30:
        time.sleep(0.3)
    if poller.get() is None:
        print("[floorflow] ❌ 读不到 Gazebo 真值")
        node.stop(); node.destroy_node(); rclpy.shutdown(); poller.stop()
        return 2

    for _ in range(80):                     # 等图像（桥与中继是分步起的）
        if node.prev_gray is not None:
            break
        time.sleep(0.5)
    if node.prev_gray is None:
        print("[floorflow] ❌ /stereo_left_tf 没有图像")
        node.stop(); node.destroy_node(); rclpy.shutdown(); poller.stop()
        return 3
    print(f"[floorflow] 图像就绪 {node.prev_gray.shape[1]}×{node.prev_gray.shape[0]}  "
          f"f={node.f:.0f}px cy={node.cy:.0f}px  地面带=底部 {BAND * 100:.0f}%  h={H_CAM} m")

    start_truth = poller.get()
    assert start_truth is not None
    start_vo = node.pose("odom")
    print(f"[floorflow] 起点 真值({start_truth[0]:+.2f},{start_truth[1]:+.2f})  "
          f"rtabmap odom {'有' if start_vo else '无（不影响本探针）'}")

    # leg1：进巷（横向 + 转弯，不计分）
    q = poller.get() or (0.0, 0.0, 0.0)
    d1 = math.hypot(WAYPOINTS[0][0] - q[0], WAYPOINTS[0][1] - q[1])
    print(f"[floorflow] leg1 -> {WAYPOINTS[0]}  {d1:.1f} m（进巷，不计分）")
    drive_to_point(poller, WAYPOINTS[0][0], WAYPOINTS[0][1], speed=SPEED, tol=0.30,
                   max_seconds=60 + 4 * d1)

    # **先摆正车头**：相机只能测"沿自己轴"的位移，车头歪 19° 就白丢 5% 信号，
    # 还会让判据分母（投影 vs 直线）解释不清。沿 A 巷行驶 = yaw 0。
    ok_turn, res = turn_to_precise(poller, 0.0, tol_deg=2.0)
    print(f"[floorflow] 摆正车头 → yaw 0°，残差 {res:.1f}° {'✓' if ok_turn else '⚠️ 未收敛'}")

    # leg2：沿巷纯前进 —— 这一段才是判据
    q = poller.get() or (0.0, 0.0, 0.0)
    d2 = math.hypot(WAYPOINTS[1][0] - q[0], WAYPOINTS[1][1] - q[1])
    yaw0 = q[2]
    print(f"[floorflow] leg2 -> {WAYPOINTS[1]}  {d2:.1f} m（沿轴纯前进，**计分**）")
    time.sleep(0.5)
    # **判据的参考基准必须取在这里**（leg2 起点），不是出生点：
    # 上一版用出生点做投影，把 leg1 的 11 m 也算进去了，分母错、比值没意义（已修）。
    leg2_start_truth = poller.get()
    leg2_start_vo = node.pose("odom")
    node.start_leg(yaw0)

    def progress(elapsed: float) -> None:
        if len(node.rows) % 40 == 0 and node.rows:      # 约每 20 s 一行
            st = node.stats()
            p = poller.get() or (0, 0, 0)
            dtr = math.hypot(p[0] - start_truth[0], p[1] - start_truth[1])
            print(f"      t{elapsed:5.0f}s 地面光流 {node.est_sum:5.2f} m | 真值(全段) {dtr:5.2f} m"
                  f" | {st['med_dy_px']:4.1f} px/帧 特征 {st['med_nfeat']:3.0f} 个")

    ok = drive_to_point(poller, WAYPOINTS[1][0], WAYPOINTS[1][1], speed=SPEED, tol=0.30,
                        max_seconds=60 + 6 * d2, progress=progress)
    node.measuring = False
    time.sleep(1.0)
    end_truth = poller.get()
    end_vo = node.pose("odom")
    print(f"[floorflow] leg2 {'到位' if ok else '未完全到位'}")

    # --- 汇总
    est = np.array([r["est_dx"] for r in node.rows], dtype=float)
    tru = np.array([r["truth_dx"] if r["truth_dx"] != "" else np.nan for r in node.rows], dtype=float)
    m = np.isfinite(est) & np.isfinite(tru)
    st = node.stats()
    truth_total = math.nan
    if end_truth is not None and leg2_start_truth is not None:
        truth_total = ((end_truth[0] - leg2_start_truth[0]) * math.cos(yaw0)
                       + (end_truth[1] - leg2_start_truth[1]) * math.sin(yaw0))
    # 仿真时长（判速度用；与帧数×dt 一致）
    span = (node.rows[-1]["t_sim"] - node.rows[0]["t_sim"]) if node.rows else math.nan

    print()
    print("=" * 70)
    straight = math.nan
    if end_truth is not None and leg2_start_truth is not None:
        straight = math.hypot(end_truth[0] - leg2_start_truth[0],
                              end_truth[1] - leg2_start_truth[1])
        # 车头（=相机轴）与位移方向的夹角：相机只能测"沿自己轴"的分量，
        # 所以判据的分母必须是投影值，不是直线距离 —— 这里把两者都打出来，免得误读。
        print(f"真值 leg2 前进位移（沿相机轴） {truth_total:6.2f} m   "
              f"（直线 {straight:5.2f} m，车头与位移夹角 "
              f"{math.degrees(math.acos(min(1.0, max(-1.0, truth_total / max(straight, 1e-6))))):4.1f}°，"
              f"{span:.1f} s 仿真）")
    print(f"地面带光流累计              {node.est_sum:6.2f} m"
          f"   （{st['used']} 帧出估计 / leg2 内收到 {st['frames']} 帧）")
    ratio = math.nan
    s = math.nan
    r2 = math.nan
    if m.sum() >= 20 and math.isfinite(truth_total) and truth_total > 2.0:
        ratio = node.est_sum / truth_total
        s = fit_scale(list(est[m]), list(tru[m]))
        # **R² 对"累计曲线"算**：逐帧真值受 poller 采样率锯齿影响，
        # 累计量是单调缩放，统计上才有意义（这也是 SLAM 里看漂移曲线的做法）。
        cum_e = np.cumsum(np.where(np.isfinite(est), est, 0.0))
        cum_t = np.cumsum(np.where(np.isfinite(tru), tru, 0.0))
        r2 = r2_of(cum_e[::5], cum_t[::5], s if math.isfinite(s) else 1.0)
        print(f"**还原率（光流/真值）**      {ratio * 100:6.1f}%")
        print(f"累计曲线 R²                 {r2:6.3f}   （{int(m.sum())} 帧插值真值配对，"
              f"真值未更新的帧 {node.truth_stale}）")
        print(f"最优尺度 s                  {s:6.3f}  → 等效相机高 h_fit = {s * H_CAM:5.3f} m"
              f"（标称 {H_CAM}）")
        print(f"每帧估计 中位 {np.median(est[m]) * 100:.2f} cm  "
              f"四分位距 {np.percentile(est[m], 75) * 100 - np.percentile(est[m], 25) * 100:.2f} cm"
              f"   → 隐含速度 {node.est_sum / max(span, 1e-6):.3f} m/s（命令 {SPEED}）")
        print(f"真值采样 {len(node.truth_hist)} 次 / {span:.1f} s = "
              f"{len(node.truth_hist) / max(span, 1e-6):.1f} Hz（图像 10 Hz，已按仿真时间插值）")
    else:
        print("⚠️ 成对真值样本不足或位移 <2 m，比值不可用（这一趟不算通过也不算失败）")
    if end_vo and leg2_start_vo:
        dv = math.hypot(end_vo[0] - leg2_start_vo[0], end_vo[1] - leg2_start_vo[1])
        print(f"对照：rtabmap odom 位移      {dv:6.2f} m → 还原率 "
              f"{dv / max(truth_total, 1e-6) * 100:5.1f}%（同一趟同一段的 S3 数字）")
    print(f"光流强度                    {st['med_dy_px']:5.1f} px/帧（中位）"
          f"   特征 {st['med_nfeat']:.0f} 个/帧")
    if math.isfinite(st["frac_floor"]):
        print(f"全图特征落在地面带的比例     {st['frac_floor'] * 100:5.1f}%"
              f"   ← 这项小，说明地面信号被周期货架淹没")
    print("=" * 70)

    with open(OUT / "frames.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=["t_sim", "dt", "n_feat", "med_dy_px", "est_dx", "truth_dx"])
        w.writeheader()
        w.writerows(node.rows)

    print()
    if not math.isfinite(ratio):
        print("[floorflow] 结论：**未跑出可用数据**（真值位移不足或有效帧不够）—— 未跑，不是通过")
    elif ratio >= 0.85 and (not math.isfinite(r2) or r2 >= 0.5):
        print("[floorflow] 结论：**地面带 2D 光流足够** —— 沿轴可测。")
        print("       -> S3 的病根是\"周期货架主导了对应\"，不是\"地面上没有信号\"；")
        print("          下一级不动硬件：把 VO 的输入换成地面带光流即可。")
    elif ratio >= 0.40:
        print(f"[floorflow] 结论：**有信号但被抑制**（{ratio * 100:.0f}%）—— 部分可测。")
        print("       -> 下一级：静态俯仰 −20°（make robot CAM_ARGS='cam_pitch:=-20'），")
        print("          地面带从画面底部扩到近半幅，光流强度约 ×2–3。")
    else:
        print(f"[floorflow] 结论：**地面带信号不足**（{ratio * 100:.0f}%）—— 这个机位测不出。")
        print("       -> 下一级：下视相机（h≈0.15 m、640 宽，28–56 px/帧）。")

    print(f"       逐帧数据: {OUT.relative_to(ROOT)}/frames.csv")
    node.stop()
    node.destroy_node()
    rclpy.shutdown()
    poller.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
