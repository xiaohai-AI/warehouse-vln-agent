#!/usr/bin/env python3
"""沿一段直线边开边抓点云，累积成世界系稠密点云（SEG-1 的输入）。

为什么必须累积：16 线雷达单帧打到一面货架只有 300–400 点，立杆基本打不到；
单帧云恢复不出 0.15 m 精度的 bay 中心（实测：真值立杆 x=-28.8+2.4k，单帧对齐得分个位数）。
真实系统也是"开过巷道把点云拼起来"，所以这不是作弊，是正确输入。

每帧用**当时的真值位姿**投到世界系（TruthPoller 50 Hz，0.28 m/s × 4 Hz 下拼接误差 ~0.01 m）。
注意：这是为了让 SEG-1 只考核"结构发现"，把位姿问题隔离出去（位姿误差已由 SLAM-1 单独考）。

用法
  .venv/bin/python scripts/capture_cloud_path.py --to 25,-10.75 --out results/seg1_path_A.npz
  .venv/bin/python scripts/capture_cloud_path.py --to 25,-10.75 --speed 0.30 --hz 4 --max 300000
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from capture_cloud import cloud_to_xyz  # noqa: E402
from depot_vln.extrinsics import SELF_R, SENSOR_XYZ, to_world  # noqa: E402
from slam_route import TruthPoller, cmd  # noqa: E402



class Sink(Node):
    def __init__(self) -> None:
        super().__init__("seg1_path_capture")
        try:
            self.declare_parameter("use_sim_time", True)
        except Exception:
            pass
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.msg: PointCloud2 | None = None
        self.create_subscription(PointCloud2, "/lidar3d/points", self._cb, qos)

    def _cb(self, m: PointCloud2) -> None:
        self.msg = m


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--to", required=True, help='终点 "x,y"')
    ap.add_argument("--out", default=str(ROOT / "results" / "seg1_path.npz"))
    ap.add_argument("--speed", type=float, default=0.28)
    ap.add_argument("--hz", type=float, default=4.0, help="每秒存几帧")
    ap.add_argument("--tol", type=float, default=0.20)
    ap.add_argument("--max", type=int, default=400000, help="点数上限（超了分箱抽稀）")
    ap.add_argument("--max-seconds", type=float, default=420.0)
    ap.add_argument("--aisle-y", type=float, default=None,
                    help="巷道中线 y（给了就检查起点是否在中线附近，防斜穿货架）")
    args = ap.parse_args()

    tx, ty = (float(v) for v in args.to.split(","))
    poller = TruthPoller()
    t0 = time.monotonic()
    while poller.get(10.0) is None and time.monotonic() - t0 < 30:
        time.sleep(0.3)
    p0 = poller.get()
    if p0 is None:
        print("[path] 读不到真值位姿 —— 仿真没起？")
        return 2
    # 起手安全检查：开直线去终点，若起点不在巷道中线附近就会斜穿货架
    if args.aisle_y is not None and abs(p0[1] - args.aisle_y) > 0.6:
        print(f"[path] 起点 y={p0[1]:+.2f} 不在巷道中线 y={args.aisle_y:+.2f} 附近（>0.6 m）"
              f" —— 拒绝开直线，先 make drive-to X=-28.5 Y={args.aisle_y}")
        poller.stop()
        return 2
    print(f"[path] 起点 ({p0[0]:+.2f},{p0[1]:+.2f}) -> 终点 ({tx:+.2f},{ty:+.2f})  "
          f"{math.hypot(tx - p0[0], ty - p0[1]):.1f} m @ {args.speed} m/s")

    rclpy.init()
    node = Sink()
    clouds: list[np.ndarray] = []
    stamps: list[float] = []
    poses: list[tuple[float, float, float]] = []
    t_next = 0.0
    t_start = time.monotonic()
    last = p0
    t_moved = time.monotonic()
    try:
        while True:
            now = time.monotonic()
            if now - t_start > args.max_seconds:
                print("[path] 超时，按已抓到的数据收尾")
                break
            rclpy.spin_once(node, timeout_sec=0.01)
            p = poller.get()
            if p is None:
                continue
            dx, dy = tx - p[0], ty - p[1]
            d = math.hypot(dx, dy)
            if d < args.tol:
                cmd(0.0, 0.0, 0.0)
                break
            if math.hypot(p[0] - last[0], p[1] - last[1]) > 0.05:
                last, t_moved = p, now
            elif now - t_moved > 10.0:
                print("[path] 真值 10 s 没变化但一直在发命令 —— 中止")
                return 1

            v = min(args.speed, 0.7 * d + 0.03)
            wx, wy = v * dx / d, v * dy / d
            c, s = math.cos(p[2]), math.sin(p[2])
            cmd(c * wx + s * wy, -s * wx + c * wy, 0.0)

            if now >= t_next and node.msg is not None:
                pts = cloud_to_xyz(node.msg)
                if pts.size:
                    pts = pts[np.linalg.norm(pts, axis=1) > SELF_R]
                    clouds.append(to_world(pts, p).astype(np.float32))
                    poses.append(p)
                    stamps.append(node.msg.header.stamp.sec + node.msg.header.stamp.nanosec * 1e-9)
                t_next = now + 1.0 / args.hz
                if len(clouds) % 20 == 0:
                    print(f"[path] 已抓 {len(clouds)} 帧  位置 ({p[0]:+.2f},{p[1]:+.2f})  "
                          f"剩余 {d:.1f} m")
            time.sleep(0.005)
    finally:
        cmd(0.0, 0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()
        poller.stop()

    if not clouds:
        print("[path] 一帧都没抓到 -> 未跑")
        return 2
    P = np.concatenate(clouds, axis=0)
    if P.shape[0] > args.max:                       # 分箱抽稀，保持空间均匀
        key = np.round(P[:, :3] / 0.05).astype(np.int64)
        _, idx = np.unique(key, axis=0, return_index=True)
        P = P[np.sort(idx)]
        print(f"[path] 抽稀 {P.shape[0]} 点（上限 {args.max}）")
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out, points=P.astype(np.float32),
                        stamps=np.array(stamps, np.float64),
                        poses=np.array(poses, np.float64),
                        sensor_xyz=np.array(SENSOR_XYZ, np.float64))
    print(f"[path] {len(clouds)} 帧 / {P.shape[0]} 点 -> {out}")
    print(f"[path] 世界范围 x[{P[:,0].min():+.2f},{P[:,0].max():+.2f}] "
          f"y[{P[:,1].min():+.2f},{P[:,1].max():+.2f}] z[{P[:,2].min():+.2f},{P[:,2].max():+.2f}]")
    return 0


if __name__ == "__main__":
    sys.exit(main())
