#!/usr/bin/env python3
"""一次跑完"转到巷道朝向 -> 移到起点 -> 转到入库朝向 -> 侧移入库"，全程闭环。

为什么要一个进程跑完（而不是几条命令拼起来）：
  实测命令之间会出现"没人命令的位移"（一次 45° 对角线 1.6 m，正好是摩擦模型里的
  自由滚动方向）。分段执行时这种漂移发生在间隙里，既看不见也补不回来，
  于是每一段的起点都是错的。合并成一个进程后，每 50 ms 闭环修正一次。

关键做法：**世界系速度 -> 车体系**。目标是世界坐标，车头可能有几度偏差，
如果直接给车体系速度就会走出斜线；这里用当前真值 yaw 把世界速度转进车体系，
于是"往世界 -x 走"永远是往世界 -x 走，跟车头朝向无关。

用法（M2 的入库动作）：
  .venv/bin/python scripts/dock_sequence.py --x -18.0 --y -3.05 --aisle-y -2.15
"""
from __future__ import annotations

import argparse
import math
import re
import subprocess
import time

W, WORLD = "depot_bot", "depot"


def run(cmd, timeout=8.0):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout).stdout
    except subprocess.TimeoutExpired:
        return ""


def pose():
    txt = run(["ign", "topic", "-e", "-t", f"/world/{WORLD}/pose/info", "-n", "1"])
    for block in txt.split('name: "'):
        if not block.startswith(W + '"'):
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


def cmd_body(vx: float, vy: float, wz: float) -> None:
    run(["ign", "topic", "-t", f"/model/{W}/cmd_vel", "-m", "ignition.msgs.Twist",
         "-p", f"linear: {{x: {vx:.4f}, y: {vy:.4f}}}, angular: {{z: {wz:.4f}}}"], timeout=3)


def cmd_world(vx_w: float, vy_w: float, yaw: float, wz: float = 0.0) -> None:
    """把世界系速度转到车体系再发。"""
    c, s = math.cos(yaw), math.sin(yaw)
    cmd_body(c * vx_w + s * vy_w, -s * vx_w + c * vy_w, wz)


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def turn_to(target_yaw: float, max_s: float = 90.0, tol_deg: float = 2.5) -> float:
    t0 = time.monotonic()
    miss = 0
    while time.monotonic() - t0 < max_s:
        p = pose()
        if p is None:
            miss += 1
            if miss > 8:
                break
            time.sleep(0.2)
            continue
        miss = 0
        err = wrap(target_yaw - p[2])
        if abs(err) < math.radians(tol_deg):
            break
        cmd_body(0.0, 0.0, max(-0.35, min(0.35, 1.6 * err)))
        time.sleep(0.05)
    cmd_body(0.0, 0.0, 0.0)
    time.sleep(0.6)
    p = pose()
    if p is None:
        print("  转向阶段结束，但读不到位姿（后面的判据会不准）", flush=True)
        return target_yaw
    print(f"  转向完成 yaw={math.degrees(p[2]):+.1f}°（目标 {math.degrees(target_yaw):+.0f}°）", flush=True)
    return p[2]


def drive_world(target_xy: tuple[float, float], speed: float = 0.15,
                tol: float = 0.03, max_s: float = 180.0, hold_yaw: float | None = None) -> None:
    """朝世界坐标点闭环推进（只按距离收敛，不关心朝向）。"""
    t0 = time.monotonic()
    miss = 0
    while time.monotonic() - t0 < max_s:
        p = pose()
        if p is None:
            miss += 1
            if miss > 10:
                break
            time.sleep(0.2)
            continue
        miss = 0
        dx, dy = target_xy[0] - p[0], target_xy[1] - p[1]
        d = math.hypot(dx, dy)
        if d < tol:
            break
        # 距离越近越慢，避免冲过
        v = min(speed, 0.5 * d + 0.03)
        wz = 0.0
        if hold_yaw is not None:
            wz = max(-0.2, min(0.2, 1.0 * wrap(hold_yaw - p[2])))
        cmd_world(v * dx / d, v * dy / d, p[2], wz)
        time.sleep(0.05)
    cmd_body(0.0, 0.0, 0.0)
    time.sleep(0.6)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, default=-18.0, help="入库位世界 x")
    ap.add_argument("--y", type=float, default=-3.05, help="入库位世界 y")
    ap.add_argument("--aisle-y", type=float, default=-2.15, help="巷道中线 y（先到这里再侧移）")
    ap.add_argument("--speed", type=float, default=0.15)
    ap.add_argument("--dock-yaw", type=float, default=None,
                    help="入库时车头朝向（度）。默认不给 = 不转朝向，直接在巷道中线上侧移入库。"
                         "原因：注入的各向异性摩擦 fdir1 在车身转 90° 后方向对不上，"
                         "实测车头 0° 时平移精度 4 cm、转到 -90° 后平移完全失效（见 TROUBLESHOOTING §20）")
    args = ap.parse_args()

    p0 = pose()
    if p0 is None:
        print("读不到真值位姿 —— 仿真没起？")
        return 2
    print(f"起点 真值 x={p0[0]:+.2f} y={p0[1]:+.2f} yaw={math.degrees(p0[2]):+.1f}°", flush=True)

    print("[1/4] 转到沿巷朝向（0°）", flush=True)
    turn_to(0.0)
    print(f"[2/4] 移到起点 ({args.x:+.2f}, {args.aisle_y:+.2f})", flush=True)
    # 先横移到巷道中线（横通道内横向自由），再沿巷走 —— 直接斜插会切过货架角
    if abs(p0[1] - args.aisle_y) > 0.15:
        drive_world((p0[0], args.aisle_y), speed=args.speed, hold_yaw=0.0)
        p_mid = pose()
        if p_mid is not None:
            print(f"    已进巷道中线 x={p_mid[0]:+.2f} y={p_mid[1]:+.2f}", flush=True)
    drive_world((args.x, args.aisle_y), speed=args.speed, hold_yaw=0.0)
    p = pose()
    if p is None:
        print("    到位检查读不到位姿", flush=True)
        return 2
    print(f"    到位 x={p[0]:+.2f} y={p[1]:+.2f}", flush=True)
    if args.dock_yaw is None:
        print("[3/4] 跳过转向（不转朝向，直接侧移入库）", flush=True)
    else:
        print(f"[3/4] 转到入库朝向（{args.dock_yaw:+.0f}°）", flush=True)
        turn_to(math.radians(args.dock_yaw))
    print(f"[4/4] 侧移入库到 ({args.x:+.2f}, {args.y:+.2f})", flush=True)
    p_hold = pose()
    hold = math.radians(args.dock_yaw) if args.dock_yaw is not None else (
        p_hold[2] if p_hold is not None else 0.0)
    drive_world((args.x, args.y), speed=args.speed, hold_yaw=hold)

    p1 = pose()
    if p1 is None:
        print("终点读不到位姿，判据无效")
        return 2
    err = math.hypot(p1[0] - args.x, p1[1] - args.y)
    print()
    print(f"终点 真值 x={p1[0]:+.2f} y={p1[1]:+.2f} yaw={math.degrees(p1[2]):+.1f}°")
    print(f"落点误差 {err:.3f} m（判据 < 0.10）")
    print(f"RESULT: {'PASS' if err < 0.10 else 'FAIL'}")
    return 0 if err < 0.10 else 1


if __name__ == "__main__":
    raise SystemExit(main())
