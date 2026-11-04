#!/usr/bin/env python3
"""SEG-2 抓取：开到每个 bay 的观测位，朝左/右货架面各拍一帧 RGB-D。

为什么按货位观测位拍：SPEC §7.1 的 T2 明确"在每个 bay 的观测位拍帧 → 检测 → 3D 质心 → 归属最近 bay"。
观测位和货位地址来自 `world/slot_grid.json`（世界摘要，运行时输入之一）；
**占用/类别真值在 `ground_truth/slots.json`，本脚本绝不读它**（评测阶段才读）。

用法
  .venv/bin/python scripts/seg2_capture.py --bays 1-3            # 试点：3 个 bay / 6 帧
  .venv/bin/python scripts/seg2_capture.py --bays 1-24           # 全 A 巷：48 帧
  .venv/bin/python scripts/seg2_capture.py --bays 1-24 --aisle C # 换巷

每帧存：rgb(uint8) / depth(float32 m) / K(3x3) / 真值位姿 / 该帧覆盖的货位地址 / 相机外参。
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from drive_to import align_to, drive_to_point, turn_to, turn_to_precise  # noqa: E402  # noqa: F401
from slam_route import TruthPoller, cmd  # noqa: E402

# 相机在 base_link 里的位置（xacro: cam_z=0.35；x 前移 0.10，与雷达同位）
CAM_XYZ = (0.10, 0.0, 0.35 - 0.076)


def parse_range(s: str) -> list[int]:
    out: list[int] = []
    for part in s.split(","):
        part = part.strip()
        if "-" in part[1:]:
            a, b = part.split("-", 1)
            out += list(range(int(a), int(b) + 1))
        else:
            out.append(int(part))
    return out


class Cam(Node):
    def __init__(self) -> None:
        super().__init__("seg2_capture")
        try:
            self.declare_parameter("use_sim_time", True)
        except Exception:
            pass
        qos = QoSProfile(depth=3, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.rgb: Image | None = None
        self.depth: Image | None = None
        self.info: CameraInfo | None = None
        self._last_stamp = -1.0        # 已取过的最新一帧（防重复取缓存帧）
        self.create_subscription(Image, "/rgbd/image", self._rgb, qos)
        self.create_subscription(Image, "/rgbd/depth_image", self._depth, qos)
        self.create_subscription(CameraInfo, "/rgbd/camera_info", self._info, qos)

    def _rgb(self, m: Image) -> None:
        self.rgb = m

    def _depth(self, m: Image) -> None:
        self.depth = m

    def _info(self, m: CameraInfo) -> None:
        self.info = m

    def latest_stamp(self) -> float:
        m = self.rgb or self.depth
        err = 0.0
        if m is None:
            return self._last_stamp
        return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 + err

    def mark(self) -> float:
        """把"当前已知最新帧"设为闸门：之后只接受比它新的帧。

        为什么不用 node.get_clock()：如果 use_sim_time 没真正生效，那是**墙钟**（10⁹ 秒量级），
        而图像戳是仿真秒 —— 比较永远不成立，于是每个观测都"取图失败"（踩过）。
        只用图像戳自身判新旧，与时钟无关。
        """
        self._last_stamp = max(self._last_stamp, self.latest_stamp())
        return self._last_stamp

    def fresh(self, timeout: float = 8.0, after: float | None = None
              ) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
        """取一对时间对齐的 RGB/深度（差 <0.2 s）+ 内参。

        **必须比上一帧新**（`after`/`_last_stamp`）：踩过的坑 —— 只检查 RGB 与深度彼此时间一致，
        于是在相机桥出图变慢时反复读到同一帧缓存：三个不同机位存下来的图像逐像素相同，
        而且画面中心深度 8.9 m（是转向前的旧视角），下游全都白算。
        """
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.rgb and self.depth and self.info:
                ts = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9  # noqa: E731
                newer = (ts(self.rgb) > self._last_stamp + 1e-6) and \
                        (after is None or ts(self.rgb) >= after)
                if newer and abs(ts(self.rgb) - ts(self.depth)) < 0.2:
                    self._last_stamp = ts(self.rgb)
                    rgb = image_to_np(self.rgb)
                    dep = depth_to_np(self.depth)
                    k = np.asarray(self.info.k, np.float64).reshape(3, 3)
                    return rgb, dep, k
        return None


def image_to_np(m: Image) -> np.ndarray:
    if m.encoding in ("rgb8", "bgr8"):
        a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, m.step // m.width)[:, : m.width * 3]
        a = a.reshape(m.height, m.width, 3)
        return a[:, :, ::-1].copy() if m.encoding == "bgr8" else a
    raise RuntimeError(f"未处理的彩图编码 {m.encoding}")


def depth_to_np(m: Image) -> np.ndarray:
    if m.encoding == "32FC1":
        return np.frombuffer(bytes(m.data), np.float32).reshape(m.height, m.width).copy()
    if m.encoding == "16UC1":                       # gz 有时给毫米
        d = np.frombuffer(bytes(m.data), np.uint16).reshape(m.height, m.width).astype(np.float32)
        return d / 1000.0
    raise RuntimeError(f"未处理的深度编码 {m.encoding}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--aisle", default="A")
    ap.add_argument("--bays", default="1-3", help='如 "1-3" 或 "1,2,5"')
    ap.add_argument("--out", default=str(ROOT / "results" / "seg2_frames.npz"))
    ap.add_argument("--speed", type=float, default=0.26)
    ap.add_argument("--settle", type=float, default=0.8, help="转向后等几秒让图像稳定")
    ap.add_argument("--max-seconds", type=float, default=900.0)
    ap.add_argument("--pitch", type=float, default=0.0,
                    help="相机仰角（度）。必须与 make robot 时的 cam_pitch 一致，"
                         "否则反投影会错（SEG-2 用 16）")
    args = ap.parse_args()

    sg = json.loads((ROOT / "world" / "slot_grid.json").read_text(encoding="utf-8"))
    bays = [b for b in sg["bays"] if b["aisle"] == args.aisle]
    want = set(parse_range(args.bays))
    bays = [b for b in bays if b["bay"] in want]
    if not bays:
        print(f"[seg2] A 巷没有这些 bay：{args.bays}")
        return 2
    aisle_y = bays[0]["aisle_center_y"]
    # 每个 bay 两个观测方向（L / R），观测位就是 bay 中心 x
    views = []
    for b in bays:
        for side, yaw in (("L", math.pi / 2), ("R", -math.pi / 2)):
            addr = [s for s in b["slots"] if f"-{side}-" in s]
            # slot_grid 的 slots 是地址字符串，用 slots 字典查层高确认
            views.append({"bay": b["bay"], "x": b["center_x"], "y": aisle_y,
                          "yaw": yaw, "side": side, "addr": addr})
    print(f"[seg2] 巷道 {args.aisle}（y={aisle_y:+.2f}）bay {[b['bay'] for b in bays]} -> "
          f"{len(views)} 个观测（每 bay 左/右各一帧）")

    poller = TruthPoller()
    t0 = time.monotonic()
    while poller.get(10.0) is None and time.monotonic() - t0 < 30:
        time.sleep(0.3)
    if poller.get() is None:
        print("[seg2] 读不到真值位姿 —— 仿真没起？")
        return 2

    # 前置：先把车送到 A 巷西口。
    # 踩过：仿真重启后车在出生点 (-31,0)，直接从那里开向 bay 1 是**斜穿货架端面**，
    # 顶住不动 -> 每个观测都"超时未到"，6 个观测全跳过、一帧没抓到。
    p0 = poller.get()
    if p0 is not None and abs(p0[1] - aisle_y) > 0.6:
        print(f"[seg2] 车不在巷内（y={p0[1]:+.2f}），先送到 A 巷西口 (-28.5,{aisle_y:+.2f})")
        drive_to_point(poller, -31.0, aisle_y, args.speed, 0.20, 180.0)
        drive_to_point(poller, -28.5, aisle_y, args.speed, 0.12, 90.0)

    rclpy.init()
    node = Cam()
    frames = []
    t_start = time.monotonic()
    try:
        # 先等第一帧，确认相机链路真的通（否则白开一路）
        if node.fresh(20.0) is None:
            print("[seg2] 拿不到 RGB-D（相机桥没起？）-> 未跑")
            return 2
        for i, v in enumerate(views):
            if time.monotonic() - t_start > args.max_seconds:
                print(f"[seg2] 到时间上限，收在 {i}/{len(views)} 个观测")
                break
            # 超时按距离算：固定 120 s 在 40 m 行程上会失败（0.26 m/s 需 154 s），
            # 前两次因此"超时未到 bay 1"。
            p_now = poller.get()
            dist = math.hypot(v["x"] - p_now[0], v["y"] - p_now[1]) if p_now else 60.0
            budget = 2.2 * dist / max(args.speed, 0.05) + 25.0
            ok = drive_to_point(poller, v["x"], v["y"], args.speed, 0.12, budget)
            if not ok:
                print(f"[seg2] 到 bay {v['bay']} 观测位失败，跳过")
                continue
            # 转向必须"停稳再校正"：turn_to 是纯 P，误差大时冲过头（10°）、误差小时被静摩擦
            # 卡住（2–4° 超时）。SEG-2 的横向归属全靠实际朝向，残差直接变成 ±0.5 m 定位散布，
            # 所以这里用脉冲式对齐（见 drive_to.align_to）。
            ok_turn, yaw_err_deg = turn_to_precise(poller, v["yaw"], tol_deg=2.0)
            cmd(0.0, 0.0, 0.0)
            # 只接受转向完成之后拍的帧（否则可能拿到转向前的画面）
            node.mark()
            got = node.fresh(15.0)
            if got is None:
                print(f"[seg2] bay {v['bay']}-{v['side']} 取图失败"
                      f"（闸门 {node._last_stamp:.2f}，最新收到 {node.latest_stamp():.2f}）"
                      f" —— 若两者相同说明相机桥没在出新帧")
                continue
            rgb, dep, k = got
            p = poller.get() or (0.0, 0.0, 0.0)
            frames.append({"rgb": rgb, "depth": dep, "k": k,
                           "pose": np.array(p, np.float64), "bay": v["bay"],
                           "side": v["side"], "addr": v["addr"],
                           "yaw_err_deg": float(yaw_err_deg)})
            fin = np.isfinite(dep) & (dep > 0.1)
            h, w = dep.shape
            core = dep[h // 4: 3 * h // 4, w // 4: 3 * w // 4]
            core = core[np.isfinite(core) & (core > 0.1)]
            core_med = float(np.median(core)) if core.size else float("nan")
            warn = "  ⚠️ 中心深度 >4 m，可能没对着货架" if core_med > 4.0 else ""
            print(f"  [seg2] {i + 1}/{len(views)}  bay {v['bay']}-{v['side']}  "
                  f"位姿 ({p[0]:+.2f},{p[1]:+.2f}) yaw={math.degrees(p[2]):+.1f}°  "
                  f"深度有效 {100 * fin.mean():.0f}%  中位 {np.median(dep[fin]):.2f} m  "
                  f"转向残差 {yaw_err_deg:4.1f}°  货位 {v['addr']}{warn}")
    finally:
        cmd(0.0, 0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()
        poller.stop()

    if not frames:
        print("[seg2] 一帧都没抓到 -> 未跑")
        return 2
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    # 存成扁平数组 + 每帧索引（npz 里放 list 会丢类型）
    np.savez_compressed(
        out,
        rgb=np.stack([f["rgb"] for f in frames]),
        depth=np.stack([f["depth"] for f in frames]).astype(np.float32),
        k=np.stack([f["k"] for f in frames]),
        pose=np.stack([f["pose"] for f in frames]),
        bay=np.array([f["bay"] for f in frames], np.int32),
        side=np.array([f["side"] for f in frames]),
        addr=np.array([",".join(f["addr"]) for f in frames]),
        yaw_err_deg=np.array([f["yaw_err_deg"] for f in frames], np.float64),
        cam_xyz=np.array(CAM_XYZ, np.float64),
        cam_pitch_deg=np.array(args.pitch, np.float64),
    )
    print(f"[seg2] {len(frames)} 帧 -> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
