#!/usr/bin/env python3
"""odom 的 yaw 对不对：正反各转一次，和 Gazebo 真值比。

为什么单独查这个
  M2 用 Nav2 打 map 坐标的目标，闭环用的是 /odom。如果 odom 的 yaw 符号或比例不对，
  Nav2 会在一个"错的朝向"里闭环：它自己认为到了（odom 口径误差 0.06 m），
  实际差 1.85 m、朝向差 156°（实测）。所以先把 yaw 单独量出来。

判据
  符号必须一致（cmd 正转，真值也正转）；比例 |比例-1| < 5%。

用法（需要 ROS 环境 + 仿真在跑）
  make odom-yaw
"""
from __future__ import annotations

import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

sys.path.insert(0, "scripts")
from m1_truth_check import publish, sim_now, true_pose_full  # noqa: E402  复用取真值的逻辑


class OdomWatch(Node):
    def __init__(self) -> None:
        super().__init__("odom_yaw_watch")
        self.yaw = None
        self.create_subscription(
            Odometry, "/odom", self.cb,
            QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE))

    def cb(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def get(self, timeout: float = 5.0):
        t0 = time.monotonic()
        while self.yaw is None and time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.yaw if self.yaw is not None else float("nan")

    def pump(self, seconds_sim: float, wz: float) -> None:
        """按仿真时间推进；读不到 /clock 时按墙钟兜底（RTF≈0.5 时给 3 倍余量）。

        之前写成 "t is None -> break"：ign topic 偶发超时会让"转 2 s"实际只转一瞬
        （实测真值只 +0.9°，看起来像底盘不转，其实是测试自己退出了）。
        """
        s0 = sim_now()
        t_wall0 = time.monotonic()
        wall_budget = seconds_sim * 3.0 + 5.0
        while True:
            t = sim_now()
            if s0 is not None and t is not None:
                if (t - s0) >= seconds_sim:
                    break
            elif time.monotonic() - t_wall0 > wall_budget:
                break
            if time.monotonic() - t_wall0 > wall_budget:
                break
            publish(0.0, 0.0, wz)
            rclpy.spin_once(self, timeout_sec=0.02)
        publish(0.0, 0.0, 0.0)


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def main() -> int:
    if sim_now() is None:
        print("读不到 /clock：仿真没在跑？")
        return 2
    rclpy.init()
    w = OdomWatch()
    w.get()
    rows = []
    for label, wz in (("正的 wz=+0.5", +0.5), ("反的 wz=-0.5", -0.5)):
        o0, t0 = w.get(), true_pose_full()
        w.pump(2.0, wz)
        time.sleep(0.4)
        o1, t1 = w.get(), true_pose_full()
        if t0 is None or t1 is None:
            print(f"{label}: 读不到真值")
            return 2
        d_odom = wrap(o1 - o0)
        d_true = wrap(t1[2] - t0[2])
        ratio = d_odom / d_true if abs(d_true) > 1e-6 else float("nan")
        rows.append((label, math.degrees(d_true), math.degrees(d_odom), ratio))
        print(f"{label}: 真值 Δyaw={math.degrees(d_true):+7.1f}°  odom Δyaw={math.degrees(d_odom):+7.1f}°  "
              f"比例={ratio:+.3f}")
    ok = True
    for label, dt_, do_, ratio in rows:
        sign_ok = (dt_ > 0) == (do_ > 0)
        scale_ok = abs(ratio - 1.0) < 0.05
        ok &= sign_ok and scale_ok
        print(f"  [{label}] 符号 {'OK' if sign_ok else '不一致'} / 比例 "
              f"{'OK' if scale_ok else f'偏差 {abs(ratio-1)*100:.0f}%'}")
    print()
    print("RESULT:", "PASS" if ok else "FAIL")
    w.destroy_node()
    rclpy.try_shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
