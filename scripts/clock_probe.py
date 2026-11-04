#!/usr/bin/env python3
"""最小 ROS 探针：只看一个话题到底有没有数据、QoS 是否匹配。

为什么单独写：preflight 用 BEST_EFFORT/VOLATILE 订阅 /clock 收不到数据，
需要判断是"桥没转发"还是"QoS 不兼容"，这两种情况的修法完全不同。

用法（必须 source ROS）：
  python3 scripts/clock_probe.py                 # 默认 /clock
  python3 scripts/clock_probe.py --topic /odom --type nav_msgs/msg/Odometry
"""
from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def profile(name: str, kind: str, depth: int = 10) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE if kind == "reliable" else ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL if kind == "transient" else DurabilityPolicy.VOLATILE,
    )


class Probe(Node):
    def __init__(self, topic: str, typ: str) -> None:
        super().__init__("clock_probe")
        self.topic = topic
        import importlib
        pkg, _, cls = typ.rpartition("/")            # "rosgraph_msgs/msg/Clock" -> pkg="rosgraph_msgs/msg"
        mod = importlib.import_module(pkg.replace("/", "."))
        msg_type = getattr(mod, cls)
        self.hits = {"best_effort": 0, "reliable": 0, "transient": 0}
        self.create_subscription(msg_type, topic, self._cb("best_effort"), profile("best_effort", "best_effort"))
        self.create_subscription(msg_type, topic, self._cb("reliable"), profile("reliable", "reliable"))
        self.create_subscription(msg_type, topic, self._cb("transient"), profile("transient", "transient"))

    def _cb(self, kind: str):
        def cb(msg) -> None:
            self.hits[kind] += 1
        return cb


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/clock")
    ap.add_argument("--type", default="rosgraph_msgs/msg/Clock")
    ap.add_argument("--seconds", type=float, default=6.0)
    args = ap.parse_args()

    rclpy.init()
    n = Probe(args.topic, args.type)
    rclpy.spin_once(n, timeout_sec=0.2)

    pubs = {}
    for name, types in n.get_topic_names_and_types():
        if name == args.topic:
            pubs[name] = types
    n_pub = n.count_publishers(args.topic)
    print(f"话题          {args.topic}  {pubs.get(args.topic, '（图上没有）')}")
    print(f"发布者数      {n_pub}")

    t0 = time.monotonic()
    while time.monotonic() - t0 < args.seconds and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.05)

    print(f"订阅 {args.seconds:.0f}s 收到：")
    for kind, cnt in n.hits.items():
        print(f"  {kind:12s} {cnt}")
    ok = any(v > 0 for v in n.hits.values())
    if not ok:
        print("\n三种 QoS 组合都收不到 -> 桥接没在转发（不是订阅端 QoS 的问题）")
    else:
        missing = [k for k, v in n.hits.items() if v == 0]
        if missing:
            print(f"\n能收到，但 {missing} 收不到 -> 订阅端 QoS 要和发布端对齐")
    print("\nRESULT:", "PASS" if ok else "FAIL")
    n.destroy_node()
    rclpy.try_shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
