#!/usr/bin/env python3
"""记录真值轨迹到 ground_truth/poses.csv（P4 的 ATE/RPE 基准）。

为什么不用 ROS 桥：Fortress 的真实位姿在 gz 话题 /world/depot/pose/info（gz.msgs.Pose_V），
Humble 的 ros_gz_bridge 不支持 Pose_V，所以直接从 gz transport 侧抓文本流解析。
这一点必须写清楚 —— 拿 /odom 当"真值"是自我循环。

  python3.10 scripts/record_gt_poses.py                # Ctrl-C 结束
  python3.10 scripts/record_gt_poses.py --duration 300
"""
from __future__ import annotations

import argparse
import csv
import re
import signal
import subprocess
import sys
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
POSE_TOPIC = "/world/depot/pose/info"

# gz 文本流形如：
#   header { stamp { sec: 12 nsec: 500000000 } }
#   pose { name: "depot_bot" id: 7 position { x: 1 y: 2 z: 0.1 } orientation { x: 0 y: 0 z: 0 w: 1 } }
POSE_RE = re.compile(
    r"header\s*\{\s*stamp\s*\{\s*sec:\s*(\d+)\s*nsec:\s*(\d+)\s*\}.*?"
    r'pose\s*\{[^}]*name:\s*"depot_bot".*?'
    r"position\s*\{\s*x:\s*(-?[\d.eE+-]+)\s*y:\s*(-?[\d.eE+-]+)\s*z:\s*(-?[\d.eE+-]+)\s*\}.*?"
    r"orientation\s*\{\s*x:\s*(-?[\d.eE+-]+)\s*y:\s*(-?[\d.eE+-]+)\s*z:\s*(-?[\d.eE+-]+)\s*w:\s*(-?[\d.eE+-]+)\s*\}",
    re.S,
)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=0.0, help="0 = 直到 Ctrl-C")
    ap.add_argument("--out", default=str(ROOT / "ground_truth" / "poses.csv"))
    args = ap.parse_args()

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    t0 = datetime.now()
    deadman = [False]

    def _stop(*_a):
        deadman[0] = True

    signal.signal(signal.SIGINT, _stop)

    proc = subprocess.Popen(
        ["ign", "topic", "-e", "-t", POSE_TOPIC],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        bufsize=1,
    )
    rows = 0
    print(f"[gt] 从 {POSE_TOPIC} 抓 depot_bot 位姿 -> {out}")
    fh = open(out, "w", newline="", encoding="utf-8")
    w = csv.writer(fh)
    w.writerow(["t", "x", "y", "z", "qx", "qy", "qz", "qw", "vx", "vy", "wz"])
    buf = ""
    try:
        assert proc.stdout is not None
        for line in proc.stdout:
            if deadman[0]:
                break
            if (datetime.now() - t0).total_seconds() > args.duration > 0:
                break
            buf += line
            if line.strip().endswith("}"):
                m = POSE_RE.search(buf)
                if m:
                    sec, nsec, x, y, z, qx, qy, qz, qw = m.groups()
                    t = int(sec) + int(nsec) * 1e-9
                    # 速度列留空：真值位姿只保证位姿，速度由后处理差分
                    w.writerow([f"{t:.6f}", x, y, z, qx, qy, qz, qw, "", "", ""])
                    rows += 1
                    if rows % 200 == 0:
                        print(f"  {rows} 帧  t={t:.1f}s", flush=True)
                buf = ""
    finally:
        fh.close()
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()
    print(f"[gt] 写 {rows} 帧 -> {out}")
    if rows == 0:
        print("提示：仿真没跑，或 world 名不是 depot（POSE_TOPIC 里写死了 /world/depot）")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
