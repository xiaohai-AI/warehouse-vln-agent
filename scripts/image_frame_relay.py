#!/usr/bin/env python3
"""把 gz 相机图像的 header.frame_id 改写成 TF 树里真实存在的 frame。

为什么必须有这一步（和 `scan_relay.py` 同源的问题）：
gz/ignition 发布的图像 header.frame_id 是**作用域名**，形如

    depot_bot/base_footprint/stereo_left

而 TF 树里是 `base_footprint -> stereo_left_link`（xacro 里的 joint 名）。
rtabmap 的 stereo_odometry 拿到图像后会 `lookupTransform(base_footprint -> 图像 frame)`，
于是刷屏：

    WARN MsgConversion.cpp: getTransform() "depot_bot/base_footprint/stereo_left"
    passed to lookupTransform argument source_frame does not exist

而且它只发 /odometry/visual 的 publisher、一条 odom 都不出（冒烟里看到的正是这个）。

订阅 QoS 用 BEST_EFFORT（对 RELIABLE 发布者也兼容），发布用 RELIABLE
（RELIABLE 发布者能被 BEST_EFFORT 和 RELIABLE 两类订阅者接收，rtabmap 两种都可能）。

用法
  .venv/bin/python scripts/image_frame_relay.py \
      --in-left /stereo_left  --out-left /stereo_left_tf  --frame-left stereo_left_link \
      --in-right /stereo_right --out-right /stereo_right_tf --frame-right stereo_right_link
"""
from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu


class ImageFrameRelay(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("image_frame_relay")
        try:
            self.declare_parameter("use_sim_time", True)
        except Exception:
            pass
        sub_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        pub_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.n = {"L": 0, "R": 0, "I": 0}
        for side, in_t, out_t, frame in (
            ("L", args.in_left, args.out_left, args.frame_left),
            ("R", args.in_right, args.out_right, args.frame_right),
        ):
            pub = self.create_publisher(Image, out_t, pub_qos)
            self.create_subscription(
                Image, in_t, self._mk_cb(pub, getattr(self, "n"), side, frame), sub_qos)
            self.get_logger().info(f"{in_t} -> {out_t}  frame_id: {frame}")
        # IMU 也要改 frame：gz 发的是 `depot_bot/base_footprint/imu`，TF 里只有 `imu_link`。
        # S3 是"双目**惯性**"里程计 —— IMU 不接上，rtabmap 就是纯视觉，沿光轴 3 cm/帧
        # 的位移根本不足以被视觉分辨（实测 leg2 还原 0~22%）。见 TROUBLESHOOTING §27。
        if args.in_imu:
            pub_i = self.create_publisher(Imu, args.out_imu, pub_qos)
            self.create_subscription(
                Imu, args.in_imu, self._mk_imu_cb(pub_i, args.frame_imu), sub_qos)
            self.get_logger().info(
                f"{args.in_imu} -> {args.out_imu}  frame_id: {args.frame_imu}")

    def _mk_imu_cb(self, pub, frame):
        def cb(m: Imu) -> None:
            m.header.frame_id = frame
            # **必须补协方差**：gz 的 IMU 三个 covariance 阵列全 0。robot_localization 会
            # 求测量协方差的逆 -> 全 0 是奇异矩阵 -> 状态里出现 NaN
            # （"Critical Error, NaNs were detected in the output state"）。
            # 这也解释了 S2 为什么"加 IMU 没改善"：那时 /imu 的 frame_id 是 gz 作用域名，
            # TF 查不到，EKF 静默丢掉这一路 —— 零协方差从没真正进过滤波器。
            m.orientation_covariance = [-1.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0]   # -1 = 不提供绝对姿态
            m.angular_velocity_covariance = [1e-4, 0.0, 0.0,
                                             0.0, 1e-4, 0.0,
                                             0.0, 0.0, 1e-4]
            m.linear_acceleration_covariance = [1e-2, 0.0, 0.0,
                                                0.0, 1e-2, 0.0,
                                                0.0, 0.0, 1e-2]
            pub.publish(m)
            self.n["I"] += 1
        return cb

    def _mk_cb(self, pub, counter, side, frame):
        def cb(m: Image) -> None:
            m.header.frame_id = frame
            pub.publish(m)
            counter[side] += 1
            if counter[side] % 100 == 0:
                self.get_logger().info(f"{side}: 已转发 {counter[side]} 帧")
        return cb


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--in-left", default="/stereo_left")
    ap.add_argument("--out-left", default="/stereo_left_tf")
    ap.add_argument("--frame-left", default="stereo_left_link")
    ap.add_argument("--in-right", default="/stereo_right")
    ap.add_argument("--out-right", default="/stereo_right_tf")
    ap.add_argument("--frame-right", default="stereo_right_link")
    ap.add_argument("--in-imu", default="", help="IMU 输入话题（空=不中继）")
    ap.add_argument("--out-imu", default="/imu_tf")
    ap.add_argument("--frame-imu", default="imu_link")
    args = ap.parse_args()
    rclpy.init()
    node = ImageFrameRelay(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
