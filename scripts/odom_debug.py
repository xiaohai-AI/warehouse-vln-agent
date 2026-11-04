#!/usr/bin/env python3
"""轮角诊断：把 /joint_states 里四个轮子的「逐帧增量」和「累计转角」打出来。

背景：odom 的 dy 在长距离上会莫名其妙累积十几米（dy = r/4·(−fl+fr+rl−rr)），
      而真值横向只动了 0.1 m 量级。这个脚本用来定位到底是哪个轮子的角度在跳，
      还是消息本身有问题（长度、顺序、整数溢出、纳米级抖动）。

用法：
  .venv/bin/python scripts/odom_debug.py --seconds 20
  # 想同时看车动起来的表现：另开终端跑 map_drive，再跑本脚本

判读：
  - 静止时 |Δ| 应全在 1e-3 rad 量级；出现 0.1 rad 以上的孤立跳变 = 抓到了
  - 四轮累计转角应该接近（前向直行时），某一路明显偏大/偏负 = 符号或名字映射错了
  - `bad` 计数 = |Δ| > 0.5 rad 的帧数（缠绕/溢出候选）
"""
from __future__ import annotations

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

ORDER = ("wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint")
R, HALF = 0.076, 0.41


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--topic", default="/joint_states")
    args = ap.parse_args()

    rclpy.init()
    n = Node("odom_debug")
    qos = QoSProfile(depth=20, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)

    st = {
        "prev": None, "tot": [0.0, 0.0, 0.0, 0.0], "dx": 0.0, "dy": 0.0, "dyaw": 0.0,
        "n": 0, "bad": 0, "shape": None, "events": 0, "t_printed": time.monotonic(),
    }

    def cb(msg: JointState) -> None:
        idx = {nm: i for i, nm in enumerate(msg.name)}
        if not all(j in idx for j in ORDER):
            if st["shape"] is None:
                n.get_logger().warn(f"名字里缺少轮子：{list(msg.name)}")
                st["shape"] = True
            return
        if st["shape"] is None:
            st["shape"] = True
            n.get_logger().info(
                f"name({len(msg.name)})={list(msg.name)}  "
                f"position({len(msg.position)}) velocity({len(msg.velocity)})")
        pos = [float(msg.position[idx[j]]) if idx[j] < len(msg.position) else float("nan")
               for j in ORDER]
        if st["prev"] is not None and all(p == p for p in pos):
            d = [p - q for p, q in zip(pos, st["prev"])]
            # 与 mecanum_odom 完全一致的解缠绕
            dw = [((x + math.pi) % (2 * math.pi)) - math.pi for x in d]
            for i, v in enumerate(dw):
                st["tot"][i] += v
            st["dx"] += R / 4 * sum(dw)
            st["dy"] += R / 4 * (-dw[0] + dw[1] + dw[2] - dw[3])
            st["dyaw"] += R / (4 * HALF) * (-dw[0] + dw[1] - dw[2] + dw[3])
            if any(abs(v) > 0.5 for v in dw):
                st["bad"] += 1
                if st["events"] < 12:
                    st["events"] += 1
                    n.get_logger().warn(
                        f"大增量 {[round(v, 3) for v in dw]}  raw={[round(v, 3) for v in d]}  "
                        f"pos={[round(p, 2) for p in pos]}")
        st["prev"] = pos
        st["n"] += 1

        now = time.monotonic()
        if now - st["t_printed"] >= 2.0:
            st["t_printed"] = now
            sp = [round(v, 2) for v in pos]
            n.get_logger().info(
                f"n={st['n']:5d} pos={sp}  累计转角(rad)={[round(v, 1) for v in st['tot']]}  "
                f"积分 dx={st['dx']:+.2f} dy={st['dy']:+.2f} dyaw={math.degrees(st['dyaw']):+.1f}°  "
                f"bad={st['bad']}")

    n.create_subscription(JointState, args.topic, cb, qos)
    n.get_logger().info(f"订阅 {args.topic}，采 {args.seconds:.0f} s")
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.seconds:
        rclpy.spin_once(n, timeout_sec=0.1)
    print(f"\n[总结] 帧数={st['n']}  大增量帧={st['bad']}")
    print(f"  累计转角(rad): {[round(v, 2) for v in st['tot']]}")
    print(f"  轮角积分: dx={st['dx']:+.3f} dy={st['dy']:+.3f} dyaw={math.degrees(st['dyaw']):+.1f}°")
    n.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
