#!/usr/bin/env python3
"""按真值锚定 map->odom（20 Hz 发布 / 1 Hz 真值校正），让 Nav2 不依赖轮式里程计积分。

# 为什么需要（2026-11-03 实测）

mecanum 轮式里程计在 20 m 行程内漂：**横向 1.10 m、航向 15.6°**（更早一次 113°）。
后果不是"精度差一点"，而是：车横向漂进货架块、贴着货架面滑行，Nav2 一直认为"还差一点没到"
→ 一路开过头 16 m（真值 x=+5，目标 x=−10.8），430 s 超时。

纯 odom 定位（LOC=static）在长距离时必然踩这个坑；AMCL 是正式解，但本场景
（仿真 + 静态世界 + 语义导航演示）用真值锚定最直接：把 map->odom 按（真值 − odom）
以 1 Hz 校正、20 Hz 发布（stamp 后移 0.3 s），Nav2 的估计误差被限制在"1 Hz 内的漂移"（毫米级）。

**定位精度本身不由这里代表**：S5/EKF（ATE 0.23 m）与 AMCL 是 P4 的正经评估；
本脚本是仿真里给"驾驶 + 语义 + 读取编号"这条链路提供干净位姿的手段，
与 P6 demo 的 `POSE=truth` 同源。

用法（由 nav_to.sh / m2_static_run.sh 以 LOC=truth 启动）
  .venv/bin/python scripts/truth_map_odom.py
  .venv/bin/python scripts/truth_map_odom.py --rate 20 --truth-rate 1.0
"""
from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--rate", type=float, default=20.0,
                    help="map->odom 发布频率（Hz）；Nav2 控制器按 20 Hz 查 TF，2 Hz 会 extrapolate")
    ap.add_argument("--truth-rate", type=float, default=1.0,
                    help="真值读取频率（Hz）；每次读/pose/info 约 0.1 s，别开太高")
    ap.add_argument("--transform-tolerance", type=float, default=0.3,
                    help="map->odom 的 stamp 向后 post-date 的秒数（AMCL 同款做法）")
    ap.add_argument("--frame", default="map")
    ap.add_argument("--odom-frame", default="odom")
    args = ap.parse_args()

    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from tf2_ros import TransformBroadcaster

    from slam_route import model_pose

    rclpy.init()
    # **必须用仿真时间**：Nav2 的 costmap 以 /clock 的时间戳查询 TF；
    # 如果这里用墙钟（默认），map->odom 的 stamp 会是 1793905xxx，而请求是 56.x，
    # 于是 planner 永远报 "Lookup would require extrapolation into the past"。
    n = Node("truth_map_odom",
             parameter_overrides=[Parameter("use_sim_time", value=True)])
    br = TransformBroadcaster(n)

    state: dict[str, tuple[float, float, float] | None] = {"odom": None, "truth": None}
    lock = threading.Lock()

    def on_odom(m) -> None:
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with lock:
            state["odom"] = (p.x, p.y, yaw)

    n.create_subscription(Odometry, "/odom", on_odom, 20)

    stop = threading.Event()

    def truth_loop() -> None:
        period = 1.0 / max(0.2, args.truth_rate)
        while not stop.is_set():
            try:
                p = model_pose()
            except Exception:  # noqa: BLE001
                p = None
            if p is not None:
                with lock:
                    state["truth"] = p
            time.sleep(period)

    th = threading.Thread(target=truth_loop, daemon=True)
    th.start()

    stats = {"ok": 0, "skip": 0}

    def publish() -> None:
        with lock:
            o, t = state["odom"], state["truth"]
        if o is None or t is None:
            stats["skip"] += 1
            return
        d = t[2] - o[2]
        c, s = math.cos(d), math.sin(d)
        tx = t[0] - (c * o[0] - s * o[1])
        ty = t[1] - (s * o[0] + c * o[1])
        msg = TransformStamped()
        # 必须 post-date：控制器查 TF 的时间通常比"现在"新几十 ms，
        # 只发 now 的 stamp 会永远 extrapolate（2026-11-05 实测：车只原地转）。
        msg.header.stamp = (n.get_clock().now()
                            + Duration(seconds=args.transform_tolerance)).to_msg()
        msg.header.frame_id = args.frame
        msg.child_frame_id = args.odom_frame
        msg.transform.translation.x = tx
        msg.transform.translation.y = ty
        msg.transform.rotation.z = math.sin(d / 2.0)
        msg.transform.rotation.w = math.cos(d / 2.0)
        br.sendTransform(msg)
        stats["ok"] += 1

    timer = n.create_timer(1.0 / max(0.5, args.rate), publish)
    n.get_logger().info(
        f"truth_map_odom 启动：{args.frame}->{args.odom_frame} @ {args.rate:.1f} Hz，"
        f"真值 @ {args.truth_rate:.1f} Hz，stamp 后移 {args.transform_tolerance:.2f} s")

    # 顺带每 5 s 报一次残差（真值 − 估计），便于在日志里一眼看出校正是否生效
    def report() -> None:
        with lock:
            o, t = state["odom"], state["truth"]
        if o is None or t is None:
            n.get_logger().warn(f"等数据：odom={state['odom'] is not None} truth={state['truth'] is not None}")
            return
        n.get_logger().info(
            f"已发布 {stats['ok']} 次（跳过 {stats['skip']}）| "
            f"truth=({t[0]:+.2f},{t[1]:+.2f},{math.degrees(t[2]):+.1f}deg) "
            f"odom=({o[0]:+.2f},{o[1]:+.2f},{math.degrees(o[2]):+.1f}deg)")

    n.create_timer(5.0, report)

    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        th.join(timeout=2.0)
        n.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
