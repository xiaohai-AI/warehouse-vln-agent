#!/usr/bin/env python3
"""建图时开路用：沿世界 +X 走一段，按**仿真时间**推进、到里程就停。

为什么不用 teleop：无头环境没人按键；而且建图路线要可复跑（P4 的 200 m 固定路线同一套机制）。

用法
  .venv/bin/python scripts/map_drive.py --vx 0.6 --distance 20
  .venv/bin/python scripts/map_drive.py --vx 0.6 --distance 20 --lateral 1.0   # 斜着走，多看一点
"""
from __future__ import annotations

import argparse
import math
import re
import subprocess
import sys
import time

W = "depot_bot"
WORLD = "depot"


def run(cmd, timeout=8.0):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout).stdout
    except subprocess.TimeoutExpired:
        return ""


def sim_now():
    txt = run(["ign", "topic", "-e", "-t", "/clock", "-n", "1"])
    m = re.search(r"^sim \{\s*sec: (\d+)\s*nsec: (\d+)", txt, re.S | re.M)
    return None if not m else int(m.group(1)) + int(m.group(2)) * 1e-9


def pose():
    txt = run(["ign", "topic", "-e", "-t", f"/world/{WORLD}/pose/info", "-n", "1"])
    for block in txt.split('name: "'):
        if block.startswith(W + '"'):
            m = re.search(r"position \{\s*x: (-?[\d.eE+-]+)\s*y: (-?[\d.eE+-]+)", block, re.S)
            if not m:
                continue
            x, y = float(m.group(1)), float(m.group(2))
            yaw = 0.0
            mq = re.search(r"orientation \{\s*x: (-?[\d.eE+-]+)\s*y: (-?[\d.eE+-]+)\s*"
                           r"z: (-?[\d.eE+-]+)\s*w: (-?[\d.eE+-]+)", block, re.S)
            if mq:
                qx, qy, qz, qw = (float(g) for g in mq.groups())
                yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
            return x, y, yaw
    return None


def publish(vx, vy=0.0, wz=0.0):
    run(["ign", "topic", "-t", f"/model/{W}/cmd_vel", "-m", "ignition.msgs.Twist",
         "-p", f"linear: {{x: {vx}, y: {vy}}}, angular: {{z: {wz}}}"], timeout=3)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vx", type=float, default=0.0,
                    help="前向速度。默认 0（不动）—— 必须显式给，避免 --turn-to 之后误开")
    ap.add_argument("--lateral", type=float, default=0.0, help="同时给的侧移速度（m/s）")
    ap.add_argument("--distance", type=float, default=20.0, help="沿 +X 走多少米")
    ap.add_argument("--lat-distance", type=float, default=0.0,
                    help="侧移多少米就停（用 --vx 0 --lateral ±v 时用来进巷道）")
    ap.add_argument("--turn-to", type=float, default=None,
                    help="先原地转到这个朝向（度）。沿巷道行驶前先把车头摆正，"
                         "否则 vx 会朝侧墙推、轮子空转把 odom 喂成假里程")
    ap.add_argument("--hold-yaw", type=float, default=None,
                    help="行驶中保持的朝向（度）")
    ap.add_argument("--hold-y", type=float, default=None,
                    help="横向保持的目标 y（世界坐标）。给上之后用真值闭环压住 y，"
                         "避免沿巷道行驶时漂到货架上蹭轮、把 odom 喂成假里程")
    ap.add_argument("--max-seconds", type=float, default=180.0, help="墙钟保护上限")
    args = ap.parse_args()

    if sim_now() is None:
        print("读不到 /clock —— 仿真没在跑？")
        return 2
    p0 = pose()
    if p0 is None:
        print("读不到真值位姿")
        return 2

    x0, y0 = p0[0], p0[1]
    t_wall0 = time.monotonic()

    def wrap(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))

    # 阶段 1：原地摆正车头（这一步很关键，见 docs/TROUBLESHOOTING.md §19）
    if args.turn_to is not None:
        target = math.radians(args.turn_to)
        t_rot = time.monotonic()
        miss = 0
        while True:
            p = pose()
            if p is None:
                miss += 1
                if miss >= 5:
                    print("转向中止：连续 5 次读不到真值位姿")
                    break
                time.sleep(0.3)
                continue
            miss = 0
            err = wrap(target - p[2])
            if abs(err) < math.radians(3.0):
                break
            if time.monotonic() - t_rot > 40.0:
                print(f"转向超时，停在 {math.degrees(p[2]):+.1f}°（目标 {args.turn_to:+.0f}°）")
                break
            publish(0.0, 0.0, max(-0.35, min(0.35, 1.5 * err)))
            time.sleep(0.05)
        publish(0.0, 0.0, 0.0)
        time.sleep(0.5)
        p_rot = pose()
        if p_rot is not None:
            print(f"摆正车头：yaw={math.degrees(p_rot[2]):+.1f}°"
                  f"（目标 {args.turn_to:+.0f}°）", flush=True)
        t_wall0 = time.monotonic()
    if args.distance == 0 and args.lat_distance <= 0:
        print("只做转向/不移动（--distance 0 且无 --lat-distance）", flush=True)
        return 0

    step = 0
    while True:
        p = pose()
        if p is None:
            break
        done = p[0] - x0
        lat = p[1] - y0
        if args.distance >= 0 and done >= args.distance:
            break
        if args.distance < 0 and done <= args.distance:
            break
        if args.lat_distance > 0 and abs(lat) >= args.lat_distance:
            break
        if time.monotonic() - t_wall0 > args.max_seconds:
            print(f"到墙钟上限 {args.max_seconds:.0f}s，停在 +{done:.1f} m")
            break
        vy = args.lateral
        wz = 0.0
        if args.hold_yaw is not None:
            err = wrap(math.radians(args.hold_yaw) - p[2])
            if abs(err) < math.radians(60.0):        # 偏差太大就别硬掰，先停下人工看
                wz = max(-0.2, min(0.2, 1.0 * err))
        if args.hold_y is not None:
            # 比例保持：误差 0.2 m 时给 0.05 m/s，限幅 ±0.12 m/s
            vy = max(-0.12, min(0.12, 0.25 * (args.hold_y - p[1])))
        publish(args.vx, vy, wz)
        if step % 40 == 0:
            print(f"  +{done:5.1f} / {args.distance:.0f} m  y偏移 {lat:+5.2f}"
                  f"  x={p[0]:+7.2f} y={p[1]:+6.2f}  vy={vy:+.3f}", flush=True)
        step += 1
        time.sleep(0.05)
    publish(0.0, 0.0, 0.0)
    p1 = pose() or p0
    print(f"开完：+{p1[0]-x0:.2f} m / 侧移 {p1[1]-y0:+.2f} m"
          f"（x {x0:+.2f} -> {p1[0]:+.2f}, y {y0:+.2f} -> {p1[1]:+.2f}）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
