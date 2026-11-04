#!/usr/bin/env python3
"""发静态占据图 + 恒定 map->odom，供 Nav2 的 static_layer 使用。

为什么自己发而不是起 nav2_map_server：
  M2 阶段不需要 AMCL（没有雷达就无从定位），也不需要 map_server 的生命周期管理。
  本地化直接用里程计（map->odom 恒等），漂移在一条巷道的十几米内可忽略；
  真正的定位/建图属于 P4，等雷达桥修好再上 slam_toolbox。
"""
from __future__ import annotations

import sys

import rclpy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


def read_pgm(path: str) -> tuple[bytes, int, int]:
    with open(path, "rb") as f:
        data = f.read()
    # P5 / 注释 / 宽 高 / 255
    parts, idx = [], 0
    while len(parts) < 4:
        while data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b"#":
            while data[idx:idx + 1] not in (b"\n", b""):
                idx += 1
            continue
        start = idx
        while not data[idx:idx + 1].isspace():
            idx += 1
        parts.append(data[start:idx])
    while data[idx:idx + 1].isspace():
        idx += 1
    w, h = int(parts[1]), int(parts[2])
    return data[idx:idx + w * h], w, h


def _truth_minus_odom(n, timeout: float = 25.0):
    """算出让 pose_map(robot) == pose_world(robot) 的 map->odom 偏移。

    地图在世界坐标系（pgm 的 origin 就是世界坐标），odom 是机器人自己积分的坐标系，
    所以必须满足 map = R(Δ)·odom + t，即 t = truth − R(Δ)·odom，Δ = truth_yaw − odom_yaw。
    车在出生点时该式退化成 (−31, 0, 0)，与旧的写死值一致。

    返回 (x, y, yaw) 或 None（读不到就回退固定值）。
    """
    import math
    import time
    from pathlib import Path

    import rclpy
    from nav_msgs.msg import Odometry

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from slam_route import model_pose    # 与 drive_to / p6 共用同一套真值读取

    odom: dict[str, float] = {}

    def _cb(m) -> None:
        q = m.pose.pose.orientation
        odom["x"] = m.pose.pose.position.x
        odom["y"] = m.pose.pose.position.y
        odom["yaw"] = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    sub = n.create_subscription(Odometry, "/odom", _cb, 20)

    # **同步读，不要 TruthPoller 线程**：踩过 —— 后台线程 + 主线程 spin_once 混用时
    # 25 s 内既没拿到 odom 也没拿到真值，静默回退到固定偏移，导致整条路线错位 44 m。
    # 这里 odom 用 spin_once 等、真值用阻塞调用（每次 ~0.1–0.5 s），逻辑短且可诊断。
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout and not odom:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.destroy_subscription(sub)

    truth = None
    for attempt in range(1, 6):
        truth = model_pose()
        if truth is not None:
            break
        n.get_logger().warn(f"真值第 {attempt} 次读取失败，重试…")
        time.sleep(0.5)

    if truth is None or not odom:
        n.get_logger().warn(f"读数不全：odom={bool(odom)} truth={truth is not None}")
        return None
    n.get_logger().info(
        f"读数 OK：odom=({odom['x']:+.3f},{odom['y']:+.3f},{math.degrees(odom['yaw']):+.1f}deg) "
        f"truth=({truth[0]:+.3f},{truth[1]:+.3f},{math.degrees(truth[2]):+.1f}deg)")
    tx, ty, tyaw = float(truth[0]), float(truth[1]), float(truth[2])
    d = tyaw - odom["yaw"]
    c, s = math.cos(d), math.sin(d)
    ox = tx - (c * odom["x"] - s * odom["y"])
    oy = ty - (s * odom["x"] + c * odom["y"])
    return ox, oy, d


def main() -> int:
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--yaml", default="maps/depot_static.yaml")
    ap.add_argument("--frame", default="map")
    ap.add_argument("--odom-frame", default="odom")
    # 出生点：odom 从 (0,0) 起算，但机器人在世界/图里的位置是 (spawn_x, spawn_y)。
    # 不补这个偏移，Nav2 会以为车站在 map 的原点（本世界 (0,0) 在货架带里）->
    # 规划器判定起点在障碍中，一路 recoveries 却不走（实测 recoveries=14、落点误差 18.5 m）。
    ap.add_argument("--spawn-x", type=float, default=-31.0)
    ap.add_argument("--spawn-y", type=float, default=0.0)
    ap.add_argument("--spawn-yaw", type=float, default=0.0)
    # **默认改为按车当前真值位姿算偏移**（2026-11-03 事故）：
    # 写死 (-31,0) 的前提是"起栈时车正好在出生点"。一旦车在别处（手动开过、
    # 撞完停在货架边），pose_map = odom + (-31,0) 整体错位 → 世界坐标的目标被
    # 当成地图坐标解释 → 车越过目标还在开（实测越过 C-L-10-1 仍继续）。
    # 改为 offset = 真值位姿 − odom 位姿，任何起车位置都成立；
    # 车确实在出生点时该式退化为 (-31,0)，与旧行为完全一致（M2/SLAM 判据不受影响）。
    ap.add_argument("--from-truth", dest="from_truth", action="store_true", default=True)
    ap.add_argument("--fixed-spawn", dest="from_truth", action="store_false",
                    help="强制用 --spawn-* 固定值（旧行为，仅供对照）")
    ap.add_argument("--no-tf", action="store_true",
                    help="只发 /map，不发布 map->odom（交给定位节点）")
    args = ap.parse_args()

    yaml = {}
    for line in open(args.yaml, encoding="utf-8"):
        if ":" in line:
            k, v = line.split(":", 1)
            yaml[k.strip()] = v.strip()
    res = float(yaml["resolution"])
    origin = [float(x) for x in yaml["origin"].strip("[]").split(",")]
    img, w, h = read_pgm(str(__import__("pathlib").Path(args.yaml).with_suffix(".pgm")))

    rclpy.init()
    n = Node("static_map_pub")
    qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = n.create_publisher(OccupancyGrid, "/map", qos)

    if args.no_tf:
        n.get_logger().info("--no-tf：不发布 map->odom（由定位节点负责）")
        _no_tf = True
    else:
        _no_tf = False
    tf = None if _no_tf else StaticTransformBroadcaster(n)
    t = TransformStamped()
    t.header.stamp = n.get_clock().now().to_msg()
    t.header.frame_id = args.frame
    t.child_frame_id = args.odom_frame
    ox, oy, oyaw = args.spawn_x, args.spawn_y, args.spawn_yaw

    if args.from_truth:
        got = _truth_minus_odom(n, timeout=25.0)
        if got is None:
            n.get_logger().warn("读不到 真值位姿//odom —— 回退到 --spawn-* 固定值（可能错位！）")
        else:
            ox, oy, oyaw = got
            n.get_logger().info(f"按真值算 map->odom = ({ox:.3f}, {oy:.3f}, {oyaw:.4f} rad)")

    t.transform.translation.x = ox
    t.transform.translation.y = oy
    t.transform.rotation.z = __import__("math").sin(oyaw / 2)
    t.transform.rotation.w = __import__("math").cos(oyaw / 2)
    if tf is not None:
        tf.sendTransform(t)

    msg = OccupancyGrid()
    msg.header.frame_id = args.frame
    msg.info = MapMetaData()
    msg.info.resolution = res
    msg.info.width = w
    msg.info.height = h
    msg.info.origin.position.x = origin[0]
    msg.info.origin.position.y = origin[1]
    msg.info.origin.orientation.w = 1.0
    # pgm: 第 0 行是 max y -> OccupancyGrid 要求第 0 行是 min y，需要上下翻转
    flipped = bytearray(w * h)
    for r in range(h):
        flipped[r * w:(r + 1) * w] = img[(h - 1 - r) * w:(h - r) * w]
    # pgm 约定：0=占据 254=自由 205=未知 -> OccupancyGrid: 100=占据 0=自由 -1=未知
    msg.data = [-1 if v == 205 else (100 if v == 0 else 0) for v in flipped]

    n.get_logger().info(f"发布静态图 {w}x{h} @ {res} m，origin=({origin[0]}, {origin[1]})，"
                        f"{args.frame}->{args.odom_frame} = ({ox}, {oy}, {oyaw} rad)")
    def tick():
        msg.header.stamp = n.get_clock().now().to_msg()
        pub.publish(msg)
    timer = n.create_timer(1.0, tick)
    tick()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
