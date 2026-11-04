#!/usr/bin/env python3
"""把 /odom 转成 TF（odom -> base_footprint）。

为什么不用 MecanumDrive 自带 TF：它会和 Nav2/robot_localization 的 TF 源打架。
本项目只留一个 TF 源，且它只读 /odom，不发任何控制指令。
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


class OdomTf(Node):
    def __init__(self) -> None:
        super().__init__("odom_tf")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("parent_frame", "odom")
        self.declare_parameter("child_frame", "base_footprint")
        self.broadcaster = TransformBroadcaster(self)
        # BEST_EFFORT 订阅：兼容桥接侧可能用的 best-effort 发布（否则一条都收不到）
        qos = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.sub = self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.on_odom, qos
        )
        self.count = 0

    def on_odom(self, msg: Odometry) -> None:
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.get_parameter("parent_frame").value
        t.child_frame_id = self.get_parameter("child_frame").value
        # odom 的 pose 是 base_link 的（z 有轮径高度），base_footprint 贴地
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        q = msg.pose.pose.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) or 1.0
        t.transform.rotation.x = q.x / norm
        t.transform.rotation.y = q.y / norm
        t.transform.rotation.z = q.z / norm
        t.transform.rotation.w = q.w / norm
        self.broadcaster.sendTransform(t)
        self.count += 1
        if self.count % 500 == 0:
            self.get_logger().info(f"odom->base_footprint 已发布 {self.count} 次")


def main() -> None:
    rclpy.init()
    node = OdomTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
