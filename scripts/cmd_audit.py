#!/usr/bin/env python3
"""测量"车在横着走还是正着走"：滑移角（slip angle）审计。

# 判据口径

车体坐标系里，速度矢量与车头（+x）的夹角：

    slip = atan2(vy_body, vx_body)

* slip ≈ 0°   → 车头朝哪就走哪（正常行驶）
* slip ≈ 45°  → 边横移边前进（"斜着走"，Dan 反映的现象）
* slip ≈ 90°  → 纯横移（车头完全不对准运动方向）

MPPI 的 Omni 运动模型允许 vy，若朝向惩罚（PathAngleCritic）太弱，
最优解就是"车头不动、横移贴路径"——这时 slip 会显著非零。
**这才是判断"该不该改权重"的证据，不能靠看截图。**

用法（另开一个终端，在 nav 开始行驶之后跑）
  .venv/bin/python scripts/cmd_audit.py --seconds 60
  .venv/bin/python scripts/cmd_audit.py --seconds 90 --csv results/nav_cmd_log.csv

不需要 source ROS 之外的东西；只订阅 /odom（并可选 /cmd_vel），不发任何控制。
"""
from __future__ import annotations

import argparse
import math
import statistics
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--csv", default="")
    ap.add_argument("--vel-threshold", type=float, default=0.03,
                    help="低于此速度的样本不计入（停着时的角度是噪声）")
    args = ap.parse_args()

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    rows: list[tuple[float, float, float, float, float]] = []

    class Audit(Node):
        def __init__(self) -> None:
            super().__init__("cmd_audit")
            qos = QoSProfile(depth=20)
            qos.reliability = ReliabilityPolicy.BEST_EFFORT
            self.create_subscription(Odometry, "/odom", self._cb, qos)

        def _cb(self, m) -> None:
            tw = m.twist.twist.linear
            ang = m.twist.twist.angular.z
            q = m.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            rows.append((time.time(), tw.x, tw.y, ang, yaw))

    rclpy.init()
    node = Audit()
    print(f"[audit] 采样 {args.seconds:.0f}s（订阅 /odom）… 让车跑起来")
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not rows:
        print("[audit] ✗ 没收到 /odom —— 仿真起了吗？")
        return 2

    moving = [(x, y) for (_t, x, y, _w, _yaw) in rows
              if math.hypot(x, y) >= args.vel_threshold]
    print(f"[audit] 样本 {len(rows)} 帧，其中在动 {len(moving)} 帧")
    if not moving:
        print("[audit] 车一直没动（或速度低于阈值）")
        return 0

    slips = [abs(math.degrees(math.atan2(y, x))) for x, y in moving]
    med = statistics.median(slips)
    p90 = sorted(slips)[int(0.9 * (len(slips) - 1))]
    crab = sum(1 for s in slips if s > 20.0) / len(slips) * 100

    vx_med = statistics.median([abs(x) for x, _ in moving])
    vy_med = statistics.median([abs(y) for _, y in moving])
    print(f"[audit] 滑移角：中位 {med:.1f}° / 90 分位 {p90:.1f}°  "
          f"（>20° 占 {crab:.0f}%）")
    print(f"[audit] 速度分量中位：|vx| {vx_med:.3f}  |vy| {vy_med:.3f} m/s")

    if med > 20.0:
        print("[audit] ✗ 结论：**在横着走**（车头没对准运动方向）"
              " → 提高 MPPI 的 PathAngleCritic 权重 / 降低 vy_max")
    elif med > 8.0:
        print("[audit] ⚠️ 结论：轻微侧偏（可接受但不理想）")
    else:
        print("[audit] ✓ 结论：车头与运动方向基本一致（正常行驶）")

    if args.csv:
        p = Path(args.csv)
        if not p.is_absolute():
            p = ROOT / p
        p.parent.mkdir(parents=True, exist_ok=True)
        with p.open("w", encoding="utf-8") as f:
            f.write("t,vx_body,vy_body,wz,yaw_deg,slip_deg\n")
            base = rows[0][0]
            for t, x, y, w, yaw in rows:
                slip = math.degrees(math.atan2(y, x)) if math.hypot(x, y) > 1e-6 else 0.0
                f.write(f"{t - base:.3f},{x:.4f},{y:.4f},{w:.4f},"
                        f"{math.degrees(yaw):.2f},{slip:.2f}\n")
        print(f"[audit] CSV → {p}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
