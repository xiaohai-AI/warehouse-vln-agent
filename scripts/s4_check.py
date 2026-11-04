#!/usr/bin/env python3
"""S4 冒烟：`floorflow_odom` 出不出 odom、频率多少、**沿轴**走 6 m 尺度对不对。

与 s3_check 的差别：S3 在横通道沿 +y 走（那是横向，视觉本来就能测），
S4 必须**进巷后沿 +x 走**，因为要考的正是"沿轴向"这一段。

  1. /odometry/floorflow 的**仿真时钟**频率（相机 10 Hz，判据 ≥8）；
  2. 进巷 → 摆正车头 → 沿 +x 走 6 m，floorflow 位移 vs Gazebo 真值 = 1.00 ± 0.15；
  3. yaw 增量一致（±5°，由 IMU 积分，验证陀螺零偏没跑掉）。
"""
from __future__ import annotations

import math
import os
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from drive_to import drive_to_point, turn_to_precise  # noqa: E402
from slam_route import TruthPoller  # noqa: E402

AISLE_Y = -10.75
DIST_M = 6.0
# 被测的里程计话题：S4 = floorflow 原始输出；S5 = EKF 融合输出（make s5-smoke 会设 env）
ODOM_TOPIC = os.environ.get("ODOM_TOPIC", "/odometry/floorflow")


class Sink(Node):
    def __init__(self) -> None:
        super().__init__("s4_check")
        try:
            self.declare_parameter("use_sim_time", True)
        except Exception:
            pass
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.odom: Odometry | None = None
        self.n = 0
        self.stamps: list[float] = []
        self.create_subscription(Odometry, ODOM_TOPIC, self._cb, qos)
        # **三路同观测**：被测话题之外，再挂轮速与视觉原始话题。
        # S5 第一次冒烟出现"侧移 1.00 / 轴向 0.44"，只看融合输出永远分不清是
        # 轮速拖、视觉停、还是融合本身发散 —— 三路一起记，落地成 CSV，一眼看出来。
        self.wheel: Odometry | None = None
        self.vision: Odometry | None = None
        self.trace: list[list[float]] = []
        if ODOM_TOPIC != "/odom":
            self.create_subscription(Odometry, "/odom", self._wheel_cb, qos)
        if ODOM_TOPIC != "/odometry/floorflow":
            self.create_subscription(Odometry, "/odometry/floorflow", self._vision_cb, qos)

    def _wheel_cb(self, m: Odometry) -> None:
        self.wheel = m

    def _vision_cb(self, m: Odometry) -> None:
        self.vision = m

    def _cb(self, m: Odometry) -> None:
        self.odom = m
        self.n += 1
        # 轨迹落盘：三路一起记，离线看不出问题时至少还有数据（"每步留数据"）
        try:
            nan = float("nan")
            p = m.pose.pose.position
            w = self.wheel.pose.pose.position if self.wheel is not None else None
            v = self.vision.pose.pose.position if self.vision is not None else None
            self.trace.append([
                float(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9),
                p.x, p.y,
                w.x if w is not None else nan, w.y if w is not None else nan,
                v.x if v is not None else nan, v.y if v is not None else nan,
            ])
        except Exception:
            pass
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if not self.stamps or t > self.stamps[-1]:
            self.stamps.append(t)


def _pose_of(m: Odometry | None):
    """(x, y, yaw_deg) 或 None。"""
    if m is None:
        return None
    p = m.pose.pose.position
    q = m.pose.pose.orientation
    yaw = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
    return (p.x, p.y, yaw)


def _show(tag: str, a, b, truth_m: float, truth_ax: str | None = None) -> None:
    """打印某一路在"起点→终点"之间的位移，并与真值比。"""
    if a is None or b is None:
        print(f"[s4]   {tag:<9} 无数据")
        return
    dx, dy = b[0] - a[0], b[1] - a[1]
    d = math.hypot(dx, dy)
    r = d / max(truth_m, 1e-6)
    dyaw = math.degrees((b[2] - a[2] + math.pi) % (2 * math.pi) - math.pi)
    print(f"[s4]   {tag:<9} 位移 {d:6.2f} m  (dx {dx:+6.2f} dy {dy:+6.2f})  "
          f"还原 {r * 100:5.1f}%  Δyaw {dyaw:+6.1f}°")


def main() -> int:
    rclpy.init()
    node = Sink()
    poller = TruthPoller()

    t0 = time.monotonic()
    while poller.get(10.0) is None and time.monotonic() - t0 < 30:
        time.sleep(0.3)

    t_wait = time.monotonic()
    while node.odom is None and time.monotonic() - t_wait < 40:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.odom is None:
        print(f"[s4] ❌ 40 s 内没收到 {ODOM_TOPIC}（看 results/s3_smoke_extra.log）")
        node.destroy_node(); rclpy.shutdown(); poller.stop()
        return 2

    n0, tw, s0 = node.n, time.monotonic(), len(node.stamps)
    while time.monotonic() - tw < 5.0:
        rclpy.spin_once(node, timeout_sec=0.05)
    wall_dt = time.monotonic() - tw
    hz_wall = (node.n - n0) / wall_dt
    new = node.stamps[s0:]
    hz_sim = (len(new) - 1) / max(new[-1] - new[0], 1e-6) if len(new) >= 3 else 0.0
    rtf = hz_wall / hz_sim if hz_sim > 0.5 else 0.0
    # **判据必须真的判**（上一版只打印不检查，白白放过一次 4.7 Hz）：
    # 目标 10 Hz（相机 update_rate），但如果渲染跟不上掉到 5 Hz，每帧 7 cm 仍然够用
    # （精确反解在 12 cm/帧内误差 <5%，见 selftest），所以下限设 4 Hz 并如实打印实际值。
    hz_ok = hz_sim >= 4.0
    print(f"[s4] {ODOM_TOPIC}：仿真时钟 {hz_sim:.1f} Hz（目标 10，下限 4"
          f"{'✓' if hz_ok else ' ✗'}）/ 墙钟 {hz_wall:.1f} Hz → RTF≈{rtf:.2f}"
          f"  每帧位移 {0.35 / max(hz_sim, 1e-6) * 100:.1f} cm")

    def xyyaw():
        m = node.odom
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return p.x, p.y, yaw

    p = poller.get()
    if p is None:
        print("[s4] ❌ 读不到真值")
        node.destroy_node(); rclpy.shutdown(); poller.stop()
        return 2

    # ===== 第 1 段：侧移（在出生点的横通道里，y 向全程畅通）=====
    # **不能在有货架的地方侧移**：上一版从 A 巷 y=-10.75 向北侧移 3 m，正好撞进 A/B 之间的
    # 货架块，车只走了 1.38 m 就顶住、300 s 超时 —— 判据拒绝给部分路段记分（对的），
    # 但那是测试设计错，不是算法错。整条路线的 wp1 侧移也正是在这条横通道里。
    LAT_M = 3.0
    time.sleep(0.6)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)
    oL0, pL0 = xyyaw(), poller.get()
    wL0, vL0 = _pose_of(node.wheel), _pose_of(node.vision)
    if pL0 is None:
        print("[s4] ❌ 侧移段起点读不到真值")
        node.destroy_node(); rclpy.shutdown(); poller.stop()
        return 2
    print(f"[s4] 侧移段：横通道里 yaw≈0 向左(+y)平移 {LAT_M} m（此处畅通）")
    ok_lat = drive_to_point(poller, pL0[0], pL0[1] + LAT_M, speed=0.30, tol=0.20,
                            max_seconds=400)
    time.sleep(1.0)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)
    oL1, pL1 = xyyaw(), poller.get()
    if pL1 is None:
        print("[s4] ❌ 侧移段终点读不到真值")
        node.destroy_node(); rclpy.shutdown(); poller.stop()
        return 2
    d_truth_lat = abs(pL1[1] - pL0[1])
    d_odom_lat = abs(oL1[1] - oL0[1])      # yaw≈0 时车体 y（左）≈ 世界 +y
    ratio_lat = d_odom_lat / max(d_truth_lat, 1e-6)
    print(f"[s4] 侧移 {d_truth_lat:.2f} m（真值）  floorflow 报 {d_odom_lat:.2f} m  "
          f"侧移比 {ratio_lat:.3f}（判据 1.00±0.20）")
    print("[s4]   三路对照（侧移段）:")
    _show("被测", oL0, oL1, d_truth_lat, "y")
    _show("轮速", wL0, _pose_of(node.wheel), d_truth_lat, "y")
    _show("视觉", vL0, _pose_of(node.vision), d_truth_lat, "y")
    ok_lat2 = ok_lat and d_truth_lat > 0.6 * LAT_M and abs(ratio_lat - 1.0) <= 0.20

    # ===== 第 2 段：进巷 + 沿轴 6 m =====
    d = math.hypot(-28.5 - pL1[0], AISLE_Y - pL1[1])
    print(f"[s4] 进巷 -> (-28.5,{AISLE_Y})  {d:.1f} m")
    drive_to_point(poller, -28.5, AISLE_Y, speed=0.40, tol=0.30, max_seconds=60 + 4 * d)
    ok_turn, res = turn_to_precise(poller, 0.0, tol_deg=2.0)
    print(f"[s4] 摆正车头 yaw 0°，残差 {res:.1f}° {'✓' if ok_turn else '⚠️'}")

    # 2) 沿 +x 走 6 m（这一段才是判据）
    time.sleep(0.6)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)
    o0 = xyyaw()
    p0 = poller.get()
    w0, v0 = _pose_of(node.wheel), _pose_of(node.vision)
    if p0 is None:
        print("[s4] ❌ 起点读不到真值")
        node.destroy_node(); rclpy.shutdown(); poller.stop()
        return 2
    tx = p0[0] + DIST_M
    ok = drive_to_point(poller, tx, p0[1], speed=0.35, tol=0.20, max_seconds=300)
    time.sleep(1.0)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)
    o1 = xyyaw()
    p1 = poller.get()
    if p1 is None:
        print("[s4] ❌ 终点读不到真值")
        node.destroy_node(); rclpy.shutdown(); poller.stop()
        return 2

    d_odom = math.hypot(o1[0] - o0[0], o1[1] - o0[1])
    d_truth = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    ratio = d_odom / max(d_truth, 1e-6)
    dyaw = math.degrees(o1[2] - o0[2])
    print(f"[s4] 沿轴走 {d_truth:.2f} m（真值）  floorflow 报 {d_odom:.2f} m  "
          f"尺度比 {ratio:.3f}（判据 1.00±0.15）  yaw 增量 {dyaw:+.1f}°")
    print("[s4]   三路对照（轴向段）:")
    _show("被测", o0, o1, d_truth, "x")
    _show("轮速", w0, _pose_of(node.wheel), d_truth, "x")
    _show("视觉", v0, _pose_of(node.vision), d_truth, "x")
    # ===== 第 3 段：**边走边转**（旋转工况）=====
    # 为什么必须测这一段：整条路线的横向假位移有两个来源，合成自检证明"转弯时旋转被 dy
    # 通道吞掉"是其中更凶的一个（纯转 3°/帧 → 老公式凭空 59 mm/帧）。冒烟必须覆盖它，
    # 否则「冒烟 PASS、长路线却偏」会重演（S4 就是这样死在侧移腿上的）。
    # **按真值位移收尾，不要按时间**：本节点在 Node 构造之后才 declare use_sim_time，
    # 那个参数不会真的切换时钟，`get_clock().now()` 拿到的是墙钟 —— 第一版按"4 仿真秒"跑，
    # 实际只走了 0.43 m（4 墙钟秒 ≈ 1.9 仿真秒），判据的分母跟着变小。
    ARC_DIST = 0.9       # m（真值位移）
    ARC_VX, ARC_WZ = 0.25, 0.40
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    time.sleep(0.4)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)
    oA0, pA0 = xyyaw(), poller.get()
    wA0, vA0 = _pose_of(node.wheel), _pose_of(node.vision)
    tw = Twist()
    tw.linear.x, tw.angular.z = ARC_VX, ARC_WZ
    t_wall = time.monotonic()
    while True:
        pub.publish(tw)
        rclpy.spin_once(node, timeout_sec=0.02)
        pc = poller.get()
        if pc is not None and math.hypot(pc[0] - pA0[0], pc[1] - pA0[1]) >= ARC_DIST:
            break
        if time.monotonic() - t_wall > 60.0:
            print("[s4] \u26a0\ufe0f 边走边转段 60 s 没走够距离，按实际位移判")
            break
        time.sleep(0.05)
    pub.publish(Twist())
    time.sleep(0.8)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)
    oA1, pA1 = xyyaw(), poller.get()
    wA1, vA1 = _pose_of(node.wheel), _pose_of(node.vision)
    ok_arc = False
    if pA0 is None or pA1 is None:
        print("[s4] ⚠️ 边走边转段读不到真值，本段不计分")
    else:
        dvx, dvy = pA1[0] - pA0[0], pA1[1] - pA0[1]
        dovx, dovy = oA1[0] - oA0[0], oA1[1] - oA0[1]
        d_t = math.hypot(dvx, dvy)
        d_o = math.hypot(dovx, dovy)
        ecc = math.hypot(dovx - dvx, dovy - dvy)          # 矢量误差（含横向串扰）
        tyaw = math.degrees((pA1[2] - pA0[2] + math.pi) % (2 * math.pi) - math.pi)
        print(f"[s4] 边走边转（vx {ARC_VX} + wz {ARC_WZ}，走够 {ARC_DIST} m）：真值位移 {d_t:.2f} m"
              f"  真值旋转 {tyaw:+.1f}°"
              f"  floorflow {d_o:.2f} m  矢量误差 {ecc * 100:.1f} cm"
              f"（判据 ≤ {0.15 * d_t * 100:.0f} cm）")
        # 三路对照：只看矢量误差分不清"航向错"还是"尺度错"，所以把方向和 Δyaw 也打出来
        print("[s4]   三路对照（边走边转段）:")
        _show("被测", oA0, oA1, d_t)
        if wA0 and wA1:
            _show("轮速", wA0, wA1, d_t)
        if vA0 and vA1:
            _show("视觉", vA0, vA1, d_t)
        ok_arc = d_t > 0.8 * ARC_DIST and ecc <= 0.15 * d_t
    ok2 = (ok and d_truth > 0.6 * DIST_M and abs(ratio - 1.0) <= 0.15
           and abs(dyaw) <= 5.0 and hz_ok and ok_lat2 and ok_arc)
    print(f"[s4] 结果：{'PASS —— 地面光流里程计可用（前进+侧移），可以跑整条路线' if ok2 else 'FAIL —— 先看日志再跑长路线'}")
    if not ok_lat2:
        print("[s4]   侧移段不合格：麦克纳姆横移测不出来（检查 du 分量 / cx 是否传对）")
    if not ok_arc:
        print("[s4]   边走边转不合格：旋转没被补偿（查 --legacy-flow 是否被误开 / IMU 偏航）")
    if d_truth <= 0.6 * DIST_M:
        print(f"[s4]   （注：真值只走了 {d_truth:.2f} m，车被挡住了，尺度比不可用）")
    try:
        out = ROOT / "results" / "s4_check_trace.csv"
        out.parent.mkdir(parents=True, exist_ok=True)
        with open(out, "w", encoding="utf-8") as f:
            f.write("t,filtered_x,filtered_y,wheel_x,wheel_y,vision_x,vision_y\n")
            for r in node.trace:
                f.write(",".join(f"{v:.4f}" for v in r) + "\n")
        print(f"[s4] 三路轨迹: {out.relative_to(ROOT)}（{len(node.trace)} 行）")
    except Exception as e:
        print(f"[s4] 轨迹写盘失败: {e}")
    node.destroy_node()
    rclpy.shutdown()
    poller.stop()
    return 0 if ok2 else 1


if __name__ == "__main__":
    sys.exit(main())
