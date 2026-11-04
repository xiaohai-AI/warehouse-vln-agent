#!/usr/bin/env python3
"""完美定位对照：用仿真真值持续修正 map->odom。

**这是实验装置，不是产品代码。** 用途是把两件事分开量：
  1. 控制器能不能把"侧移入库位"做到 0.10 m（M2 想判的能力）；
  2. 我们的定位够不够用（没有雷达时，轮式 odom 看不见横向漂移 —— 实测长距离段
     车向北飘、顶在货架面上，odom 却以为走了 3.6 m "正常"）。

原理：TF 链是 map -> odom -> base_footprint。轮式 odom 给 odom->base，
这里算 map->odom = truth_base ∘ (odom->base)^-1，等价于"把定位误差实时抵消掉"。
所以 Nav2 看到的是真实位姿，闭环里只剩控制误差。

记录要求：任何用这个模式跑出来的成绩，都必须同时标注"完美定位"，
不能当成真实定位能力的证据。
"""
from __future__ import annotations

import math
import re
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.parameter import Parameter
from tf2_ros import TransformBroadcaster

ROBOT, WORLD = "depot_bot", "depot"


def run(cmd, timeout=8.0):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout).stdout
    except subprocess.TimeoutExpired:
        return ""


def truth_pose():
    txt = run(["ign", "topic", "-e", "-t", f"/world/{WORLD}/pose/info", "-n", "1"])
    for block in txt.split('name: "'):
        if not block.startswith(ROBOT + '"'):
            continue
        mp = re.search(r"position \{\s*x: (-?[\d.eE+-]+)\s*y: (-?[\d.eE+-]+)", block, re.S)
        mq = re.search(r"orientation \{\s*x: (-?[\d.eE+-]+)\s*y: (-?[\d.eE+-]+)\s*"
                       r"z: (-?[\d.eE+-]+)\s*w: (-?[\d.eE+-]+)", block, re.S)
        if mp and mq:
            x, y = float(mp.group(1)), float(mp.group(2))
            qx, qy, qz, qw = (float(g) for g in mq.groups())
            return x, y, math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return None


class TruthLoc(Node):
    def __init__(self) -> None:
        # 必须用仿真时间：否则 TF 盖的是墙钟戳（1793121343），而 base_footprint 的
        # TF 来自仿真时间（334 s），planner 会一路报
        # "Lookup would require extrapolation into the past"。
        super().__init__("truth_localization",
                         parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        # rclpy 里只给 override 不够：get_clock() 只有在参数被**声明**后才返回 ROSClock，
        # 否则 TF 会盖墙钟戳（1793121340），而 base_footprint 是仿真时间（514 s）-> 外推报错。
        try:
            self.declare_parameter("use_sim_time", True)
        except Exception:
            pass          # override 已经声明过，忽略
        self.odom = None
        self.create_subscription(
            Odometry, "/odom", self.cb,
            QoSProfile(depth=20, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE))
        self.br = TransformBroadcaster(self)
        self.n_ok = 0
        self.n_fail = 0
        self.timer = self.create_timer(0.2, self.tick)

    def cb(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def tick(self) -> None:
        if self.odom is None:
            return
        tp = truth_pose()
        if tp is None:
            self.n_fail += 1
            return
        ox, oy, oyaw = self.odom
        tx, ty, tyaw = tp
        # map->odom = truth_base ∘ (odom->base)^-1
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        dx, dy = -ox, -oy
        mox = tx + c * dx - s * dy
        moy = ty + s * dx + c * dy
        moyaw = tyaw - oyaw

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = mox
        t.transform.translation.y = moy
        t.transform.rotation.z = math.sin(moyaw / 2)
        t.transform.rotation.w = math.cos(moyaw / 2)
        self.br.sendTransform(t)
        self.n_ok += 1
        if self.n_ok % 25 == 1:
            # 打时间戳自证用仿真时间（期望 ~/= 仿真秒，而不是 17xxxxxxxxx 的墙钟）
            self.get_logger().info(
                f"{self.n_ok} 次更新  stamp={t.header.stamp.sec}.{t.header.stamp.nanosec // 10**6:03d}  "
                f"map->odom=({mox:+.2f},{moy:+.2f},{math.degrees(moyaw):+.1f}°)  失败 {self.n_fail}")


def main() -> int:
    rclpy.init()
    n = TruthLoc()
    n.get_logger().info("完美定位模式启动：map->odom 由仿真真值置换（仅用于分离定位/控制误差）")
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
