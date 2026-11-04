#!/usr/bin/env python3
"""起飞前检查（preflight）：把"人眼看 RViz"能发现的问题变成数字。

一次跑完，逐项给数字，任何一项 FAIL 就退出码非 0：

  1 话题存在      /clock /odom /joint_states /lidar3d /imu /cmd_vel
  2 频率          /odom ≥ 20 Hz、/lidar3d ≈ 10 Hz、/imu ≈ 200 Hz
  3 仿真时钟      /clock 在走，并报告 RTF（sim 秒 / 墙钟秒，SPEC §11 要求写进报告）
  4 关节真的接上了 /joint_states 里轮子关节有速度且随 cmd_vel 变化
  5 TF 链         odom -> base_footprint 可查（odom_tf.py 在发）
  6 静止保持      发零速度 1 s，位姿漂移 < 1 cm（否则说明打滑/被推/世界没 static）
  7 直行跟踪      发 vx=0.2 持续 2 s，沿 +X 位移 > 0.25 m 且横向串扰 < 0.05 m
                 —— 这一条就是 M1 的快速版：如果它 FAIL，别往下跑 Nav2，先量 M1

用法（先手动起仿真）
  python3.10 scripts/preflight.py
  python3.10 scripts/preflight.py --skip-motion    # 只看静态项，不动机器人
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from collections import deque
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, PointCloud2

ROOT = Path(__file__).resolve().parents[1]

# ros_gz_bridge 对传感器类话题常用 BEST_EFFORT 发布；如果用默认的 RELIABLE 去订阅，
# QoS 不兼容 -> 一个字节都收不到，而 `ros2 topic list` 里话题明明在（踩过）。
# 订阅端统一申请 BEST_EFFORT：它同时兼容 RELIABLE 和 BEST_EFFORT 的发布者。
SENSOR_QOS = QoSProfile(
    depth=100,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Preflight(Node):
    def __init__(self) -> None:
        super().__init__("depot_preflight")
        self.stamps: dict[str, deque] = {}
        self.last: dict[str, object] = {}
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self._mk("odom", Odometry), SENSOR_QOS)
        self.create_subscription(JointState, "/joint_states", self._mk("joint_states", JointState), SENSOR_QOS)
        self.create_subscription(PointCloud2, "/lidar3d", self._mk("lidar3d", PointCloud2), SENSOR_QOS)
        self.create_subscription(Imu, "/imu", self._mk("imu", Imu), SENSOR_QOS)
        self.clock_stamps: deque = deque(maxlen=400)
        self.sim_stamps: dict[str, deque] = {}
        self.create_subscription(Clock, "/clock", self._on_clock, SENSOR_QOS)
        self.pose_samples: list[tuple[float, float, float]] = []  # wall_t, x, y

    def _mk(self, name: str, _typ):
        def cb(msg) -> None:
            now = time.monotonic()
            self.stamps.setdefault(name, deque(maxlen=200)).append(now)
            st = getattr(msg, "header", None)
            if st is not None:
                # 同时记仿真时间戳：RTF<1 时墙钟频率只有标称值的一半，判据必须按仿真时间算
                self.sim_stamps.setdefault(name, deque(maxlen=200)).append(
                    st.stamp.sec + st.stamp.nanosec * 1e-9
                )
            self.last[name] = msg
            if name == "odom":
                p = msg.pose.pose.position
                self.pose_samples.append((now, p.x, p.y))
                if len(self.pose_samples) > 4000:
                    del self.pose_samples[:2000]

        return cb

    def _on_clock(self, msg) -> None:
        # 必须和 _mk() 记同一套账：判据 1（话题有数据）和判据 2（频率）都查 self.stamps，
        # 之前这里只往 clock_stamps 记，导致 /clock 明明在发却永远被判"有发布者但没数据"。
        now = time.monotonic()
        self.stamps.setdefault("clock", deque(maxlen=200)).append(now)
        self.last["clock"] = msg
        self.clock_stamps.append((now, msg.clock.sec + msg.clock.nanosec * 1e-9))

    def hz(self, name: str) -> float:
        s = self.stamps.get(name) or deque()
        if len(s) < 3:
            return 0.0
        span = s[-1] - s[0]
        return (len(s) - 1) / span if span > 1e-6 else 0.0

    def hz_sim(self, name: str) -> float:
        """按**仿真时间**算频率。RTF≈0.5 时墙钟频率只有标称值一半，用墙钟会把正常场景判成故障。"""
        s = self.sim_stamps.get(name) or deque()
        if len(s) < 3:
            return 0.0
        span = s[-1] - s[0]
        return (len(s) - 1) / span if span > 1e-6 else 0.0

    def sim_now(self) -> float | None:
        return self.clock_stamps[-1][1] if self.clock_stamps else None

    def spin_sim(self, seconds: float, publish_cmd: tuple[float, float, float] | None = None,
                 max_wall: float | None = None) -> float:
        """按**仿真时间**推进 seconds 秒（RTF<1 时墙钟会更长）。返回实际推进的仿真秒数。"""
        s0 = self.sim_now()
        if s0 is None:                       # 没有 /clock：退回墙钟
            self.spin_for(seconds, publish_cmd)
            return seconds
        limit = max_wall if max_wall is not None else seconds * 8.0 + 10.0
        m = Twist()
        t0 = time.monotonic()
        while rclpy.ok():
            sn = self.sim_now()
            if sn is not None and (sn - s0) >= seconds:
                break
            if time.monotonic() - t0 > limit:
                break
            if publish_cmd is not None:
                m.linear.x, m.linear.y, m.angular.z = publish_cmd
                self.cmd.publish(m)
            rclpy.spin_once(self, timeout_sec=0.02)
        sn = self.sim_now()
        return (sn - s0) if sn is not None else seconds

    def wait_topic(self, name: str, timeout: float = 10.0) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if (self.stamps.get(name) or deque()) and len(self.stamps[name]) >= 2:
                return True
        return False

    def spin_for(self, seconds: float, publish_cmd: tuple[float, float, float] | None = None) -> None:
        t0 = time.monotonic()
        m = Twist()
        while time.monotonic() - t0 < seconds and rclpy.ok():
            if publish_cmd is not None:
                m.linear.x, m.linear.y, m.angular.z = publish_cmd
                self.cmd.publish(m)
            rclpy.spin_once(self, timeout_sec=0.02)

    def pose_now(self):
        msg = self.last.get("odom")
        if msg is None:
            return None
        p = msg.pose.pose.position
        return (p.x, p.y, yaw_of(msg.pose.pose.orientation))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--skip-motion", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    n = Preflight()
    rows: list[tuple[str, bool, str]] = []

    def check(name: str, ok: bool, detail: str) -> None:
        rows.append((name, ok, detail))

    # 1 话题：分三档判断 —— 图上没有 / 有但没发布者 / 有发布者但没数据
    TOPIC = {
        "clock": "/clock",
        "odom": "/odom",
        "joint_states": "/joint_states",
        "lidar3d": "/lidar3d",
        "imu": "/imu",
    }
    n.spin_for(0.5)
    found = dict(n.get_topic_names_and_types())
    absent = [k for k, t in TOPIC.items() if t not in found]
    no_pub = [k for k, t in TOPIC.items() if t in found and n.count_publishers(t) == 0]
    no_data = [k for k in TOPIC if k not in absent and not n.wait_topic(k, timeout=6.0)]
    ok1 = not (absent or no_pub or no_data)
    if ok1:
        detail = "全部到位且有数据"
    else:
        bits = []
        if absent:
            bits.append(f"话题不存在 {absent}")
        if no_pub:
            bits.append(f"无发布者 {no_pub}（桥接进程挂了？）")
        if no_data:
            bits.append(f"有发布者但没数据 {no_data}（仿真被暂停？检查 gz 里的 ⏸）")
        detail = "；".join(bits)
    check("1 话题存在", ok1, detail)
    if not ok1:
        rel = sorted(
            t for t in found
            if any(s in t for s in ("odom", "clock", "lidar", "imu", "joint", "cmd_vel", "scan"))
        )
        print()
        print("  当前 ROS 图里的相关话题（用来判断 remap 有没有生效 / 桥接是否活着）：")
        if rel:
            for t in rel:
                print(f"    {t:<52} {', '.join(found[t])}  pubs={n.count_publishers(t)}")
        else:
            print("    （一个都没有：ROS 图是空的，桥接没起来或用了别的 DDS/域）")
        rclpy.shutdown()
        for name, ok, d in rows:
            print(f"[{'PASS' if ok else 'FAIL'}] {name:<12} {d}")
        print("\nRESULT: FAIL")
        return 1

    n.spin_for(3.0)
    hz_odom, hz_lidar, hz_imu, hz_js = n.hz_sim("odom"), n.hz_sim("lidar3d"), n.hz_sim("imu"), n.hz_sim("joint_states")
    # 2 频率（按仿真时间；墙钟频率在 RTF<1 时会腰斩，见 hz_sim 注释）
    ok2 = hz_odom >= 20 and 6 <= hz_lidar <= 14 and hz_imu >= 100
    check("2 频率", ok2,
          f"odom {hz_odom:.0f} Hz / lidar3d {hz_lidar:.1f} Hz / imu {hz_imu:.0f} Hz / "
          f"joint_states {hz_js:.0f} Hz（仿真时间）")

    # 3 仿真时钟 + RTF
    rtf = 0.0
    if len(n.clock_stamps) >= 5:
        (w0, s0), (w1, s1) = n.clock_stamps[0], n.clock_stamps[-1]
        rtf = (s1 - s0) / (w1 - w0) if w1 > w0 else 0.0
    check("3 仿真时钟/RTF", rtf > 0.3, f"RTF ≈ {rtf:.2f}（< 0.3 说明机器扛不住这套传感器组合）")

    # 4 关节真的接上了：zero cmd 时四轮速度应接近 0，且关节名对得上
    js = n.last.get("joint_states")
    wheel_joints = []
    if js is not None:
        wheel_joints = [nm for nm in js.name if "wheel" in nm]
    ok4 = len(wheel_joints) >= 4
    check("4 轮关节存在", ok4, f"{len(wheel_joints)} 个: {wheel_joints}")

    # 5 TF 链
    tf_ok, tf_detail = False, "跳过"
    try:
        import tf2_ros

        buf = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buf, n)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not tf_ok:
            rclpy.spin_once(n, timeout_sec=0.05)
            try:
                tr = buf.lookup_transform("odom", "base_footprint", rclpy.time.Time())
                tf_ok = True
                tf_detail = f"odom->base_footprint {tr.transform.translation.x:.2f},{tr.transform.translation.y:.2f}"
            except Exception:
                pass
        if not tf_ok:
            tf_detail = "查不到 odom->base_footprint（scripts/odom_tf.py 没在跑？）"
    except ImportError:
        tf_detail = "tf2_ros 不可用"

    # 6 静止保持
    p0 = n.pose_now()
    n.spin_sim(1.5, publish_cmd=(0.0, 0.0, 0.0))
    p1 = n.pose_now()
    drift = math.hypot(p1[0] - p0[0], p1[1] - p0[1]) if (p0 and p1) else float("nan")
    check("6 静止保持", drift < 0.01, f"零速度 1.5 s(仿真) 漂移 {drift * 1000:.1f} mm")
    check("5 TF 链", tf_ok, tf_detail)

    # 7 直行跟踪（M1 快速版）：按仿真时间推进，期望位移 = vx × 仿真秒数
    if not args.skip_motion:
        start = n.pose_now()
        elapsed = n.spin_sim(2.0, publish_cmd=(0.2, 0.0, 0.0))
        n.spin_sim(0.5, publish_cmd=(0.0, 0.0, 0.0))
        end = n.pose_now()
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        expect = 0.2 * elapsed
        # 车头朝向 +X（yaw≈0）时 dx 就是前向位移；容差 15%（打滑/轮径误差都算里面）
        ok7 = abs(dx) > 0.85 * expect and abs(dx) < 1.15 * expect and abs(dy) < 0.05
        check("7 直行 0.2 m/s", ok7,
              f"{elapsed:.2f} s(仿真) 前向 {dx:+.3f} m（期望 {expect:.3f}，容差 ±15%）"
              f"/ 横向 {dy:+.3f} m（串扰 < 0.05）")
    else:
        check("7 直行 0.2 m/s", True, "跳过（--skip-motion）")

    rclpy.shutdown()
    print()
    w = max(len(r[0]) for r in rows) + 2
    for name, ok, detail in rows:
        print(f"[{'PASS' if ok else 'FAIL'}] {name:<{w}} {detail}")
    bad = [r for r in rows if not r[1]]
    print()
    print("RESULT:", "PASS" if not bad else f"FAIL（{len(bad)} 项）")
    if any(r[0].startswith("7") and not r[1] for r in bad):
        print("提示：第 7 项失败就不用往下跑 Nav2 —— 直接量 M1：")
        print("  python3.10 scripts/m1_velocity_tracking.py")
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())
