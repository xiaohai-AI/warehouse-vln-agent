#!/usr/bin/env python3
"""抓一帧 3D 点云（+ 位姿）落盘，给 SEG-1（bay 网格发现）当离线输入。

为什么单独存：SEG-1 的结构提取要反复调算法，每次调都去起仿真/订阅话题太浪费。
存一帧 → 离线随便重算，和 S1 用 `--dump` 落 npz 是同一套纪律。

存什么
  points   (N,3) float32，**雷达系**（frame_id 见输出，本仓是 lidar3d_link）
  stamp    该帧时间戳（仿真秒）
  pose     (x, y, yaw) 机器人真值位姿（gz /world/depot/pose/info），便于投到世界系对账
  frame    原始 header.frame_id

用法
  .venv/bin/python scripts/capture_cloud.py                      # 1 帧 -> results/seg1_cloud.npz
  .venv/bin/python scripts/capture_cloud.py --frames 5 --out results/seg1_cloud5.npz
"""
from __future__ import annotations

import argparse
import math
import re
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROOT_ = Path(__file__).resolve().parents[1]
if str(ROOT_) not in sys.path:
    sys.path.insert(0, str(ROOT_))
from depot_vln.extrinsics import SELF_R, SENSOR_XYZ, to_world  # noqa: E402

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

ROOT = Path(__file__).resolve().parents[1]


def truth_pose() -> tuple[float, float, float] | None:
    """gz 真值位姿。只用 /world/<world>/pose/info —— 别换通道（见技能库里的坑）。"""
    try:
        txt = subprocess.run(["ign", "topic", "-e", "-t", "/world/depot/pose/info", "-n", "1"],
                             capture_output=True, text=True, timeout=8).stdout
    except Exception:
        return None
    for block in txt.split('name: "'):
        if not block.startswith("depot_bot\""):
            continue
        mp = re.search(r"position \{\s*x: (-?[\d.eE+-]+)\s*y: (-?[\d.eE+-]+)", block, re.S)
        mq = re.search(r"orientation \{\s*x: (-?[\d.eE+-]+)\s*y: (-?[\d.eE+-]+)\s*"
                       r"z: (-?[\d.eE+-]+)\s*w: (-?[\d.eE+-]+)", block, re.S)
        if mp:
            x, y = float(mp.group(1)), float(mp.group(2))
            yaw = 0.0
            if mq:
                qx, qy, qz, qw = (float(g) for g in mq.groups())
                yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
            return x, y, yaw
    return None


def cloud_to_xyz(msg: PointCloud2) -> np.ndarray:
    """PointCloud2 -> (N,3)。手写解析，不依赖 sensor_msgs_py（本机环境不一定有）。

    只取 x/y/z 三个 float32 字段；按 offset 取，兼容 row padding。
    """
    fields = {f.name: (f.offset, f.datatype) for f in msg.fields}
    if not all(k in fields for k in ("x", "y", "z")):
        raise RuntimeError(f"点云缺少 x/y/z 字段，实际字段：{list(fields)}")
    off = {k: fields[k][0] for k in ("x", "y", "z")}
    n = msg.width * msg.height
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    need = msg.point_step
    buf = buf[: n * need].reshape(n, need)
    xyz = np.empty((n, 3), np.float32)
    for i, k in enumerate(("x", "y", "z")):
        xyz[:, i] = buf[:, off[k]: off[k] + 4].copy().view(np.float32).ravel()
    ok = np.isfinite(xyz).all(axis=1)
    xyz = xyz[ok]
    rng = np.linalg.norm(xyz, axis=1)
    return xyz[(rng > 0.2) & (rng < 30.0)]          # 去掉自身与远处噪点


class CloudSink(Node):
    def __init__(self) -> None:
        super().__init__("seg1_cloud_capture")
        try:
            self.declare_parameter("use_sim_time", True)   # rclpy 可能已自动声明
        except Exception:
            pass
        qos = QoSProfile(depth=3, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.msg: PointCloud2 | None = None
        self.create_subscription(PointCloud2, "/lidar3d/points", self._cb, qos)

    def _cb(self, m: PointCloud2) -> None:
        self.msg = m

    def wait(self, timeout: float = 20.0) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.msg is not None:
                return True
        return False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=1, help="连续存几帧（同一位姿，用于看噪声）")
    ap.add_argument("--interval", type=float, default=1.0)
    ap.add_argument("--out", default=str(ROOT / "results" / "seg1_cloud.npz"))
    args = ap.parse_args()

    pose = truth_pose()
    if pose is None:
        print("[cloud] 读不到真值位姿：仿真没起？（这一帧没有位姿就不能投世界系对账）")
        return 2

    rclpy.init()
    node = CloudSink()
    frames, stamps, frame_ids = [], [], []
    try:
        for i in range(args.frames):
            node.msg = None
            if not node.wait():
                print(f"[cloud] 等不到 /lidar3d/points（第 {i+1} 帧）—— 仿真/桥没起")
                return 2
            pts = cloud_to_xyz(node.msg)
            frames.append(pts)
            stamps.append(node.msg.header.stamp.sec + node.msg.header.stamp.nanosec * 1e-9)
            frame_ids.append(node.msg.header.frame_id)
            r = np.linalg.norm(pts, axis=1)
            print(f"  [cloud] 第 {i+1}/{args.frames} 帧：{pts.shape[0]} 点  "
                  f"距离 min/中位/max = {r.min():.2f}/{np.median(r):.2f}/{r.max():.2f} m  "
                  f"frame={node.msg.header.frame_id}")
            time.sleep(args.interval)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    P_all = np.concatenate(frames, axis=0)
    # 同时存世界系版本：分析端（SEG-1）要的是世界系；只存雷达系会让人以为 y=±1.6 是绝对位置
    np.savez_compressed(out,
                        points=P_all.astype(np.float32),                  # 雷达系
                        points_world=to_world(P_all, pose),               # 世界系
                        stamps=np.array(stamps, np.float64),
                        frame=str(frame_ids[0]),
                        pose=np.array(pose, np.float64),
                        frames=np.array([len(f) for f in frames], np.int32))
    P = np.concatenate(frames, axis=0)
    print(f"[cloud] 位姿 真值 ({pose[0]:+.2f},{pose[1]:+.2f}) yaw={math.degrees(pose[2]):+.1f}°")
    print(f"[cloud] 共 {P.shape[0]} 点 -> {out}")
    print(f"[cloud] 坐标范围 x[{P[:,0].min():+.2f},{P[:,0].max():+.2f}] "
          f"y[{P[:,1].min():+.2f},{P[:,1].max():+.2f}] z[{P[:,2].min():+.2f},{P[:,2].max():+.2f}]")
    return 0


if __name__ == "__main__":
    sys.exit(main())
