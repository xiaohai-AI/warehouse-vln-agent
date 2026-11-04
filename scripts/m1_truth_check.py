#!/usr/bin/env python3
"""M1 真值复核：cmd_vel 与 Gazebo 真实位移是否一致 —— 不看轮速、不看 odom。

为什么必须有这一步
  `scripts/m1_velocity_tracking.py` 用 /odom 的前后差分算速度，而我们的 /odom 是
  从四个轮的累积转角积分出来的。轮子转多少 ↔ 指令是什么，本来就是插件的运动学反解，
  所以那样量等于"自己给自己打分"（SPEC §4.2 的原话）。
  这里换成真值位姿 `/world/depot/pose/info`：车在世界上**实际**挪了多远，才是速度跟踪。

只用 gz 侧工具（ign topic / ign service），不需要 ROS 环境：
  .venv/bin/python scripts/m1_truth_check.py
  .venv/bin/python scripts/m1_truth_check.py --seconds 3 --start -26 -2.15
"""
from __future__ import annotations

import argparse
import json
import math
import re
import subprocess
import sys
import time

W = "depot_bot"
WORLD = "depot"
AISLE_C = (-26.0, -2.15)     # C 巷中间：左右各 1.6 m 自由空间，够侧移测试


def run(cmd: list[str], timeout: float = 8.0) -> str:
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout).stdout
    except subprocess.TimeoutExpired:
        return ""


def sim_now() -> float | None:
    txt = run(["ign", "topic", "-e", "-t", "/clock", "-n", "1"])
    m = re.search(r"^sim \{\s*sec: (\d+)\s*nsec: (\d+)", txt, re.S | re.M)
    if not m:
        return None
    return int(m.group(1)) + int(m.group(2)) * 1e-9


def true_pose(name: str = W) -> tuple[float, float] | None:
    txt = run(["ign", "topic", "-e", "-t", f"/world/{WORLD}/pose/info", "-n", "1"])
    for block in txt.split('name: "'):
        if not block.startswith(name + '"'):
            continue
        mx = re.search(r"position \{\s*x: (-?[\d.eE+-]+)\s*y: (-?[\d.eE+-]+)", block, re.S)
        if mx:
            return float(mx.group(1)), float(mx.group(2))
    return None


def true_pose_full(name: str = W) -> tuple[float, float, float] | None:
    """真值 (x, y, yaw)。yaw 从 pose/info 的四元数换算。"""
    txt = run(["ign", "topic", "-e", "-t", f"/world/{WORLD}/pose/info", "-n", "1"])
    for block in txt.split('name: "'):
        if not block.startswith(name + '"'):
            continue
        mp = re.search(r"position \{\s*x: (-?[\d.eE+-]+)\s*y: (-?[\d.eE+-]+)", block, re.S)
        mq = re.search(r"orientation \{\s*x: (-?[\d.eE+-]+)\s*y: (-?[\d.eE+-]+)\s*"
                       r"z: (-?[\d.eE+-]+)\s*w: (-?[\d.eE+-]+)", block, re.S)
        if mp and mq:
            x, y = float(mp.group(1)), float(mp.group(2))
            qx, qy, qz, qw = (float(g) for g in mq.groups())
            yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
            return x, y, yaw
    return None


def teleport(x: float, y: float) -> None:
    run(["ign", "service", "-s", f"/world/{WORLD}/set_pose",
         "--reqtype", "ignition.msgs.Pose", "--reptype", "ignition.msgs.Boolean",
         "--timeout", "2000",
         "--req", f'name: "{W}" position: {{x: {x}, y: {y}, z: 0.10}}'])
    time.sleep(2.0)


def publish(vx: float, vy: float, wz: float) -> None:
    run(["ign", "topic", "-t", f"/model/{W}/cmd_vel", "-m", "ignition.msgs.Twist",
         "-p", f"linear: {{x: {vx}, y: {vy}}}, angular: {{z: {wz}}}"], timeout=3)


def drive_collect(vx: float, vy: float, wz: float, seconds: float) -> list[tuple[float, float, float]]:
    """按仿真时间驱动 seconds 秒，过程中连续采 (仿真时刻, x, y)。

    采样放在运动**中间**：不在起步瞬间取起点、也不在停车后取终点 ——
    否则 dt 会把加减速段和"停车后滑行"算进去，速度被系统性低估（实测能低 30%）。
    """
    t_end = (sim_now() or 0.0) + seconds
    samples: list[tuple[float, float, float]] = []
    while True:
        t = sim_now()
        if t is None or t >= t_end:
            break
        p = true_pose()
        if p is not None:
            samples.append((t, p[0], p[1]))
        publish(vx, vy, wz)
        time.sleep(0.05)
    publish(0.0, 0.0, 0.0)
    return samples


def velocity_from(samples: list[tuple[float, float, float]], skip_frac: float = 0.25):
    """丢掉起步段（默认前 25%）后用首尾差分算速度，返回 (vx, vy, dt)。"""
    if len(samples) < 6:
        return 0.0, 0.0, 0.0
    k = int(len(samples) * skip_frac)
    s = samples[k:]
    t0, x0, y0 = s[0]
    t1, x1, y1 = s[-1]
    dt = t1 - t0
    if dt <= 0:
        return 0.0, 0.0, 0.0
    return (x1 - x0) / dt, (y1 - y0) / dt, dt


def one_case(name: str, vx: float, vy: float, wz: float, *, seconds: float,
             start: tuple[float, float], tol: dict[str, float]) -> dict:
    teleport(*start)
    s0 = sim_now()
    if s0 is None:
        print(f"{name}: 读不到 /clock，跳过")
        return {"name": name, "ok": False}
    samples = drive_collect(vx, vy, wz, seconds)
    mx, my, dt = velocity_from(samples)
    res = {"name": name, "dt": dt, "mvx": mx, "mvy": my, "n": len(samples)}
    ok = True
    print(f"{name:9s} cmd=({vx:.2f},{vy:.2f},{wz:.2f})  中段 {dt:.2f} s(仿真)，{len(samples)} 个真值采样")
    if vx:
        err = (mx - vx) / vx
        ok &= abs(err) <= tol["vx"]
        print(f"          实测 vx={mx:+.3f}  误差 {err*100:+.1f}%   (容差 ±{tol['vx']*100:.0f}%)")
    if vy:
        err = (my - vy) / vy
        ok &= abs(err) <= tol["vy"]
        print(f"          实测 vy={my:+.3f}  误差 {err*100:+.1f}%   (容差 ±{tol['vy']*100:.0f}%)")
    if vx and vy:
        cmd_dir = math.degrees(math.atan2(vy, vx))
        m_dir = math.degrees(math.atan2(my, mx))
        d = (m_dir - cmd_dir + 180) % 360 - 180
        ok &= abs(d) <= tol["dir"]
        print(f"          合成方向误差 {d:+.1f}°   (容差 ±{tol['dir']:.0f}°)")
    res["ok"] = ok
    return res


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=3.0, help="每个用例跑多少**仿真**秒")
    ap.add_argument("--start", nargs=2, type=float, default=list(AISLE_C), metavar=("X", "Y"))
    args = ap.parse_args()
    start = (args.start[0], args.start[1])

    if sim_now() is None:
        print("读不到 /clock —— 仿真没在跑？")
        return 2

    tol = {"vx": 0.10, "vy": 0.25, "dir": 15.0}   # 与 SPEC §4.2 M1 判据一致
    print("== M1 真值复核（Gazebo 位姿，不看轮速/odom）==")
    rows = [
        one_case("vx_only", 0.40, 0.00, 0.00, seconds=args.seconds, start=start, tol=tol),
        one_case("vy_only", 0.00, 0.30, 0.00, seconds=args.seconds, start=start, tol=tol),
        one_case("wz_only", 0.00, 0.00, 0.50, seconds=args.seconds, start=start, tol=tol),
        one_case("diag", 0.30, 0.30, 0.00, seconds=args.seconds, start=start, tol=tol),
    ]
    bad = [r for r in rows if not r.get("ok")]
    out = {
        "criterion": "M1-truth",
        "source": "gazebo_true_pose (world/depot/pose/info), NOT /odom",
        "methods": "mid-motion sampling, first 25% of samples dropped",
        "cases": rows,
        "pass": not bad,
        "tol": tol,
    }
    from pathlib import Path
    Path("results/m1_truth.json").write_text(json.dumps(out, indent=2, ensure_ascii=False) + "\n",
                                            encoding="utf-8")
    print()
    print(f"RESULT: {'PASS' if not bad else f'FAIL（{len(bad)}/{len(rows)}）'}  -> results/m1_truth.json")
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())
