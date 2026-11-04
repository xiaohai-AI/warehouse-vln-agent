#!/usr/bin/env python3
"""S3 冒烟检查：视觉里程计出不出 odom、频率多少、走一段后**尺度对不对**。

为什么单独做：S3 的整条路线要跑 40 分钟以上（203 m + 双目渲染），
如果视觉里程计根本没出 odom、或者尺度错了（右目 P[3] 写错、baseline 当 1 m），
那 40 分钟全是白跑。这个检查只开 6 m 就能判定：

  1. /odom 有没有发布者、频率多少（要 ≥8 Hz，相机 10 Hz）；
  2. 开 6 m 后，rtabmap 的 odom 位移 vs **Gazebo 真值**位移 —— 比值应在 1.00 ± 0.15；
  3. yaw 增量是否一致（±5°）。

判据数字一律用真值（/world/depot/pose/info），不用 odom 自评。
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from drive_to import drive_to_point  # noqa: E402
from slam_route import TruthPoller  # noqa: E402

DIST_M = 6.0


class Sink(Node):
    def __init__(self) -> None:
        super().__init__("s3_check")
        try:
            self.declare_parameter("use_sim_time", True)
        except Exception:
            pass
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.odom: Odometry | None = None
        self.n = 0
        self.stamps: list[float] = []
        self.t0 = time.monotonic()
        # **必须是视觉里程计自己的话题**：/odom 上还有 mecanum_odom（~250 Hz 轮式），
        # 读错话题会得到"odom 报 36.9 m / 真值 2.0 m"这种离谱比值（车顶着货架空转）。
        self.create_subscription(Odometry, "/odometry/visual", self._cb, qos)

    def _cb(self, m: Odometry) -> None:
        self.odom = m
        self.n += 1
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if not self.stamps or t > self.stamps[-1]:
            self.stamps.append(t)


def main() -> int:
    rclpy.init()
    node = Sink()
    poller = TruthPoller()

    t0 = time.monotonic()
    while poller.get(10.0) is None and time.monotonic() - t0 < 30:
        time.sleep(0.3)

    # 1) 等 odom 出数并测频率
    t_wait = time.monotonic()
    while node.odom is None and time.monotonic() - t_wait < 40:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.odom is None:
        print("[s3] ❌ 40 s 内没收到 /odom —— 视觉里程计没起来（看 results/s3_smoke_odom.log）")
        node.destroy_node(); rclpy.shutdown(); poller.stop()
        return 2
    n0, tw = node.n, time.monotonic()
    s0 = len(node.stamps)
    while time.monotonic() - tw < 5.0:
        rclpy.spin_once(node, timeout_sec=0.05)
    wall_dt = time.monotonic() - tw
    hz_wall = (node.n - n0) / wall_dt
    # **判据用仿真时间**：相机是 10 Hz（仿真时钟），墙钟会随 RTF 一起被拉低。
    # 拿墙钟比"≥8 Hz"在 RTF<0.8 时必然误判 —— 实测 RTF≈0.2 时 6 Hz 仿真 = 1.2 Hz 墙钟。
    new = node.stamps[s0:]
    hz_sim = (len(new) - 1) / max(new[-1] - new[0], 1e-6) if len(new) >= 3 else 0.0
    rtf = hz_wall / hz_sim if hz_sim > 0.5 else 0.0
    print(f"[s3] /odometry/visual：仿真时钟 {hz_sim:.1f} Hz（相机 10 Hz，判据 ≥8）"
          f" / 墙钟 {hz_wall:.1f} Hz  → RTF≈{rtf:.2f}")

    def odom_xyyaw():
        m = node.odom
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return p.x, p.y, yaw

    # 2) 记录起点（odom 与真值）
    rclpy.spin_once(node, timeout_sec=0.2)
    o0 = odom_xyyaw()
    p0 = poller.get()
    if p0 is None:
        print("[s3] ❌ 读不到真值位姿")
        node.destroy_node(); rclpy.shutdown(); poller.stop()
        return 2
    print(f"[s3] 起点：odom ({o0[0]:+.3f},{o0[1]:+.3f}) yaw {math.degrees(o0[2]):+.1f}°  "
          f"真值 ({p0[0]:+.2f},{p0[1]:+.2f})")

    # 3) 沿 **+y**（横通道方向）开 6 m。
    # 踩过：出生点 (-31,0) 沿 +x 直走会撞进两条巷道之间的货架块 —— 车顶着货架空转，
    # 真值只走 2.04 m 而轮式 odom 报 36.9 m，看起来像"尺度差 18 倍"，实际是驱动被挡。
    # 横通道在 y 方向是通的（x=±31.5 在所有巷道 y 上都验过）。
    tx, ty = p0[0], p0[1] + DIST_M
    ok = drive_to_point(poller, tx, ty, speed=0.30, tol=0.20, max_seconds=300)
    time.sleep(1.0)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)
    o1 = odom_xyyaw()
    p1 = poller.get()
    if p1 is None:
        print("[s3] ❌ 终点读不到真值")
        node.destroy_node(); rclpy.shutdown(); poller.stop()
        return 2

    d_odom = math.hypot(o1[0] - o0[0], o1[1] - o0[1])
    d_truth = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    ratio = d_odom / max(d_truth, 1e-6)
    if d_truth < 0.6 * DIST_M:
        print(f"[s3] ⚠️ 真值只走了 {d_truth:.2f} m（应 ≈{DIST_M} m）—— 车被挡住了，"
              f"这一轮的尺度比不可用")
    dyaw = math.degrees(o1[2] - o0[2])
    print(f"[s3] 走了 {d_truth:.2f} m（真值）  odom 报 {d_odom:.2f} m  "
          f"尺度比 {ratio:.3f}（判据 1.00±0.15）  odom yaw 增量 {dyaw:+.1f}°")
    ok2 = ok and abs(ratio - 1.0) <= 0.15
    print(f"[s3] 结果：{'PASS —— 视觉里程计可用，可以跑整条路线' if ok2 else 'FAIL —— 先看日志再跑长路线'}")
    if not ok:
        print("[s3]   （注：闭环驱动本身没到位，尺度比可能不可信）")
    node.destroy_node()
    rclpy.shutdown()
    poller.stop()
    return 0 if ok2 else 1


if __name__ == "__main__":
    sys.exit(main())
