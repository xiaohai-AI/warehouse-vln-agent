#!/usr/bin/env python3
"""把 gz 的雷达话题转成 slam_toolbox / Nav2 能吃的 /scan。

三种模式：
  --type scan   原样转发 LaserScan，只改写 frame_id
  --type cloud  转发 PointCloud2，只改写 frame_id（交给 pointcloud_to_laserscan 切片）
  --type ring   从多线 LaserScan 里**抽出一层**当单线 2D 激光（本机默认走这条）

为什么要 ring 模式：gz 的多线激光在 LaserScan 话题里是把所有线拍平成一维
（count = 水平采样数 × 线数），angle_min/max/step 只描述**一圈**。整包丢给
slam_toolbox 会得到几何错乱的一圈。这里按"一圈的长度"把数据切回成一圈圈，
再挑最像水平的那圈（能看最远的那圈）输出。数据量也从上百 KB 降到几百个数。

为什么要改 frame_id：gz 的 header.frame_id 是作用域名字
  LaserScan   : "depot_bot::base_footprint::lidar3d"
  PointCloud2 : "depot_bot/base_footprint/lidar3d"
TF 树里没有这些（我们的链是 base_footprint -> lidar3d_link，雷达在 0.55 m），
不改写下游会一直以"transform 找不到"丢帧。

QoS：订阅端 BEST_EFFORT（对齐 gz 桥），发布端 RELIABLE —— RELIABLE 发布者对
BEST_EFFORT / RELIABLE 订阅者都兼容，下游用哪种都能收。
"""
from __future__ import annotations

import argparse
import statistics
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2


def finite(r) -> bool:
    return r == r and r != float("inf") and r > 0.05


def median_range(seq) -> float:
    vals = [r for r in seq if finite(r)]
    return statistics.median(vals) if vals else 0.0


def smoothness(seq) -> float:
    """相邻射线差的中位数。正确切出来的一圈应该是连续的墙 -> 差很小。"""
    vals = [r for r in seq if finite(r)]
    if len(vals) < 8:
        return float("inf")
    return statistics.median(abs(vals[i + 1] - vals[i]) for i in range(len(vals) - 1))


class RingPicker:
    """定位"一圈"在数据里的排布方式（分块 or 交错），并挑最像水平的那一圈。"""

    def __init__(self) -> None:
        self.layout: str | None = None      # "block" 或 "stride"
        self.ring: int = 0
        self.n_angles: int = 0
        self.beams: int = 1
        self.report: str = ""

    def setup(self, msg: LaserScan) -> bool:
        n = len(msg.ranges)
        inc = msg.angle_increment
        if n < 32 or inc <= 0:
            return False
        n_angles = int(round((msg.angle_max - msg.angle_min) / inc)) + 1
        if n_angles < 8 or n % n_angles != 0:
            n_angles = n                     # 本来就是单线
        k = max(1, n // n_angles)
        self.n_angles, self.beams = n_angles, k

        cands: list[tuple[float, float, str, int]] = []   # (平滑度, -中位距离, 布局, 环号)
        for layout in ("block", "stride"):
            for r in range(k):
                seq = (msg.ranges[r * n_angles:(r + 1) * n_angles] if layout == "block"
                       else msg.ranges[r::k])
                if len(seq) < n_angles // 2:
                    continue
                cands.append((smoothness(seq), -median_range(seq), layout, r))
        if not cands:
            return False
        # 先用平滑度定布局（只有正确的切法才会切出连续的墙面），再在正确布局里挑看得最远的一环
        best = min(c[0] for c in cands)
        same = sorted((c for c in cands if c[0] <= max(best * 1.5, best + 1e-9)),
                      key=lambda c: c[1])
        _, negmed, self.layout, self.ring = same[0]
        self.report = (f"count={n} 每圈={n_angles} 线数={k} -> 布局={self.layout} "
                       f"环={self.ring}（中位距离 {-negmed:.2f} m，平滑度 {same[0][0]:.3f}）")
        return True

    def extract(self, msg: LaserScan) -> LaserScan | None:
        if self.layout is None:
            return None
        if self.layout == "block":
            seq = msg.ranges[self.ring * self.n_angles:(self.ring + 1) * self.n_angles]
        else:
            seq = msg.ranges[self.ring::self.beams]
        if len(seq) < 8:
            return None
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_increment = msg.angle_increment
        out.angle_max = out.angle_min + out.angle_increment * (len(seq) - 1)
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = list(seq)
        out.intensities = []
        return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--type", choices=("scan", "cloud", "ring"), default="ring")
    ap.add_argument("--in-topic", default="/lidar3d")
    ap.add_argument("--out-topic", default="/scan")
    ap.add_argument("--frame-id", default="lidar3d_link",
                    help="改写成 TF 树里真实存在的 frame")
    args = ap.parse_args()

    rclpy.init()
    n = Node("scan_relay")
    sub_qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
    pub_qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
    stats = {"n": 0}
    picker = RingPicker()

    if args.type == "cloud":
        pub = n.create_publisher(PointCloud2, args.out_topic, pub_qos)

        def cb(msg: PointCloud2) -> None:      # type: ignore[misc]
            src = msg.header.frame_id
            msg.header.frame_id = args.frame_id
            pub.publish(msg)
            stats["n"] += 1
            if stats["n"] % 10 == 1:
                n.get_logger().info(
                    f"{stats['n']} 帧  {src!r} -> {args.frame_id!r}  "
                    f"{msg.width}x{msg.height} step={msg.point_step} "
                    f"({msg.width * msg.height * msg.point_step / 1024:.0f} KB)")

        msg_type = PointCloud2
    else:
        pub = n.create_publisher(LaserScan, args.out_topic, pub_qos)

        def cb(msg: LaserScan) -> None:
            src = msg.header.frame_id
            if args.type == "ring":
                if picker.layout is None:
                    if not picker.setup(msg):
                        n.get_logger().warn("解析不出环结构，跳过这一帧")
                        return
                    n.get_logger().info("锁定布局：" + picker.report)
                out = picker.extract(msg)
                if out is None:
                    return
            else:
                out = msg
            out.header.frame_id = args.frame_id
            pub.publish(out)
            stats["n"] += 1
            if stats["n"] % 10 == 1:
                valid = sum(1 for r in out.ranges if finite(r))
                n.get_logger().info(
                    f"{stats['n']} 帧 {src!r} -> {args.frame_id!r}  输出 {len(out.ranges)} 条"
                    f"（有效 {valid}），中位距离 {median_range(out.ranges):.2f} m")

        msg_type = LaserScan

    n.create_subscription(msg_type, args.in_topic, cb, sub_qos)
    n.get_logger().info(
        f"relay[{args.type}] {args.in_topic} -> {args.out_topic} (frame -> {args.frame_id})")
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
