#!/usr/bin/env python3
"""量 MecanumDrive 插件实际下发的轮子角速度，判断"命令速度只兑现 73%"是插件换算错还是打滑。

为什么要单看 /joint_states 的 velocity 字段：
  holonomic_probe 用轮角**位置差分**算平均速度，窗口里夹着起步加速，
  分不清"轮子没转到目标"和"轮子转到了但车滑"。velocity 是瞬时值，没有这个问题。

判读（以 vx=0.2、轮半径 0.076 为例）：
  期望角速度 = 0.2 / 0.076 = 2.63 rad/s
  - 实测 ≈ 2.63，但车只走 0.146 m/s  -> 插件换算正确，是接触打滑（23%）
  - 实测 ≈ 2.00（= 0.2 / 0.10）      -> 插件把轮半径当成了 0.10（参数没读到），换算错
  - 实测 ≈ 1.92                      -> 同上，比例 0.73

用法：
  .venv/bin/python scripts/wheel_speed_check.py                 # 依次测 vx / vy / wz
  .venv/bin/python scripts/wheel_speed_check.py --cmd 0.2
"""
from __future__ import annotations

import argparse
import math
import re
import statistics
import subprocess
import time

W = "depot_bot"
WHEELS = ("wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint")
R = 0.076            # SDF 里轮胎 cylinder 的 radius
LX_LY = 0.41         # 轮距/2 + 轴距/2（实测四轮合成系数）


def run(cmd, timeout=8.0):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout).stdout
    except subprocess.TimeoutExpired:
        return ""


def sim_now():
    txt = run(["ign", "topic", "-e", "-t", "/clock", "-n", "1"])
    m = re.search(r"^sim \{\s*sec: (\d+)\s*nsec: (\d+)", txt, re.S | re.M)
    return None if not m else int(m.group(1)) + int(m.group(2)) * 1e-9


def cmd(vx=0.0, vy=0.0, wz=0.0):
    run(["ign", "topic", "-t", f"/model/{W}/cmd_vel", "-m", "ignition.msgs.Twist",
         "-p", f"linear: {{x: {vx:.4f}, y: {vy:.4f}}}, angular: {{z: {wz:.4f}}}"], timeout=3)


def pose():
    txt = run(["ign", "topic", "-e", "-t", "/world/depot/pose/info", "-n", "1"])
    for block in txt.split('name: "'):
        if not block.startswith(W + '"'):
            continue
        mp = re.search(r"position \{\s*x: (-?[\d.eE+-]+)\s*y: (-?[\d.eE+-]+)", block, re.S)
        mq = re.search(r"orientation \{\s*x: (-?[\d.eE+-]+)\s*y: (-?[\d.eE+-]+)\s*"
                       r"z: (-?[\d.eE+-]+)\s*w: (-?[\d.eE+-]+)", block, re.S)
        if mp:
            yaw = 0.0
            if mq:
                qx, qy, qz, qw = (float(g) for g in mq.groups())
                yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
            return float(mp.group(1)), float(mp.group(2)), yaw
    return None


def wheel_vel_pos():
    """取 /joint_states 的四轮位置（rad），用于算整段轮子滚过的弧长。"""
    txt = run(["ign", "topic", "-e", "-t", "/joint_states", "-n", "1"])
    out = []
    for w in WHEELS:
        m = re.search(rf'name: "{w}".*?position: (-?[\d.eE+-]+)', txt, re.S)
        out.append(float(m.group(1)) if m else float("nan"))
    if any(math.isnan(x) for x in out):
        return None
    return out


def wheel_vel():
    """取一条 /joint_states，读四轮 velocity（rad/s）。"""
    txt = run(["ign", "topic", "-e", "-t", "/joint_states", "-n", "1"])
    out = []
    for w in WHEELS:
        m = re.search(rf'name: "{w}".*?velocity: (-?[\d.eE+-]+)', txt, re.S)
        out.append(float(m.group(1)) if m else float("nan"))
    return out


def measure(label, vx, vy, wz, secs=2.0):
    p0 = pose()
    cmd(vx, vy, wz)
    t0 = time.monotonic()
    s0 = sim_now() or 0.0
    p_start = p0
    # 先跑到稳态
    while time.monotonic() - t0 < (secs + 5) * 4:
        s = sim_now()
        if s is not None and s - s0 >= secs:
            break
        time.sleep(0.1)
    samples = []
    for _ in range(4):
        v = wheel_vel()
        if not any(math.isnan(x) for x in v):
            samples.append(v)
        time.sleep(0.15)
    p1 = pose()
    s1 = sim_now()
    cmd(0.0, 0.0, 0.0)
    time.sleep(0.8)

    if not samples or p1 is None or p0 is None or p_start is None:
        print(f"{label}: 采样失败")
        return
    assert p1 is not None and p_start is not None      # 给类型检查器看的，上面已经判过
    wv = [statistics.median(col) for col in zip(*samples)]
    dt = (s1 - s0) if (s1 is not None and s0) else float("nan")
    dxw = p1[0] - p_start[0]
    dyw = p1[1] - p_start[1]
    c, s_ = math.cos(p_start[2]), math.sin(p_start[2])
    body_x = c * dxw + s_ * dyw
    body_y = -s_ * dxw + c * dyw
    v_body_x = body_x / dt if dt and dt > 0 else float("nan")
    v_body_y = body_y / dt if dt and dt > 0 else float("nan")
    if label == "vx":
        expect = vx / R
        implied = [x * R for x in wv]
        print(f"{label}={vx:+.2f} m/s  期望轮角 {expect:+.2f} rad/s（按 R={R}）")
        print(f"   实测轮角 {['%+.2f' % x for x in wv]}")
        print(f"   按 R=0.076 折合表面速度 {['%+.3f' % x for x in implied]}  中位 {statistics.median(implied):+.3f}")
        print(f"   若按 R=0.100 折算            {statistics.median(wv) * 0.1:+.3f} m/s")
        print(f"   稳态车体位移 vx_body={v_body_x:+.3f} m/s（仿真时差 {dt:.2f}s，"
              f"兑现比 {v_body_x / vx:.2f}）")
    elif label == "vy":
        expect = vy / R
        print(f"{label}={vy:+.2f} m/s  期望轮角幅值 {expect:.2f} rad/s（±，符号交替）")
        print(f"   实测轮角 {['%+.2f' % x for x in wv]}")
        print(f"   折算表面速度 {statistics.median([abs(x) * R for x in wv]):.3f} m/s")
        print(f"   稳态车体位移 vy_body={v_body_y:+.3f} m/s（仿真时差 {dt:.2f}s，"
              f"兑现比 {v_body_y / vy:.2f}）  串扰 vx_body={v_body_x:+.3f}")
    else:
        expect = wz * LX_LY / R
        print(f"{label}={wz:+.2f} rad/s  期望轮角幅值 {expect:.2f} rad/s")
        print(f"   实测轮角 {['%+.2f' % x for x in wv]}")
        print(f"   折算表面速度 {statistics.median([abs(x) * R for x in wv]):.3f} m/s")
        print(f"   实测转向兑现 见 holonomic 探针；本轮位移 ({body_x:+.3f},{body_y:+.3f}) m")
    print()


def turn_to(deg: float, max_s: float = 60.0) -> bool:
    target = math.radians(deg)
    t0 = time.monotonic()
    miss = 0
    while time.monotonic() - t0 < max_s:
        p = pose()
        if p is None:
            miss += 1
            if miss > 8:
                return False
            time.sleep(0.2)
            continue
        miss = 0
        err = math.atan2(math.sin(target - p[2]), math.cos(target - p[2]))
        if abs(err) < math.radians(4.0):
            break
        cmd(0.0, 0.0, max(-0.35, min(0.35, 1.6 * err)))
        time.sleep(0.05)
    cmd(0.0, 0.0, 0.0)
    time.sleep(0.8)
    return True


def sweep(v: float, secs: float) -> int:
    """同一位置转一圈，每个朝向量前向兑现率：朝向相关 or 位置相关，一眼分开。"""
    print(f"{'车头':>6s} {'轮角':>8s} {'车体 vx':>9s} {'兑现比':>7s}  判读")
    rows = []
    for deg in (0, 45, 90, 135, 180, 225, 270, 315):
        if not turn_to(deg):
            print(f"{deg:6.0f}  转向失败")
            continue
        p0 = pose()
        j0 = None
        cmd(v, 0.0, 0.0)
        t0 = time.monotonic()
        s0 = sim_now() or 0.0
        while time.monotonic() - t0 < (secs + 5) * 4:
            s = sim_now()
            if s is not None and s - s0 >= secs:
                break
            time.sleep(0.1)
        samples = []
        for _ in range(3):
            wv = wheel_vel()
            if not any(math.isnan(x) for x in wv):
                samples.append(wv)
            time.sleep(0.15)
        p1 = pose()
        s1 = sim_now()
        cmd(0.0, 0.0, 0.0)
        time.sleep(0.8)
        if p0 is None or p1 is None or not samples or s1 is None:
            print(f"{deg:6.0f}  采样失败")
            continue
        dt = s1 - s0
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        c, s_ = math.cos(p0[2]), math.sin(p0[2])
        vx_body = (c * dx + s_ * dy) / dt if dt > 0 else float("nan")
        wv = [statistics.median(col) for col in zip(*samples)]
        w_surf = statistics.median([abs(x) * R for x in wv])
        ratio = vx_body / v
        rows.append(ratio)
        note = ""
        if ratio < 0.3:
            note = "前向几乎走不动"
        elif ratio < 0.8:
            note = "部分失效"
        else:
            note = "正常"
        print(f"{deg:6.0f} {w_surf:8.3f} {vx_body:+9.3f} {ratio:7.2f}  {note}")
    if rows:
        good = sum(1 for x in rows if x >= 0.8)
        print(f"\n[总结] 正常朝向 {good}/{len(rows)}  最小 {min(rows):.2f}  最大 {max(rows):.2f}")
        print("       若只有部分朝向正常 -> 摩擦锥方向没随车体旋转（fdir1 框架问题）")
        print("       若所有朝向都差     -> 这个位置本身被挡住/压住")
    return 0


def ramp_check(v: float, secs: float) -> int:
    """从静止起步 -> 匀速 -> 停车，一次完整起停：轮子滚了多远 vs 车真走了多远。

    稳态测试（sweep / measure）都只取匀速段，量不到起停损失；
    而 M2 的每一腿都是起停，若起停时空转，误差就会累积成"odom 到了、车没到"。
    """
    print(f"起停检查：vx={v:+.2f} m/s，匀速段 {secs:.1f} 仿真秒\n")
    print(f"{'轮子滚过':>9s} {'真值位移':>9s} {'打滑量':>8s} {'打滑率':>7s} {'轮速兑现':>9s}")
    for i in range(3):
        p0 = pose()
        j0 = wheel_vel_pos()
        s0 = sim_now()
        if p0 is None or j0 is None or s0 is None:
            print("  采样失败"); return 2
        cmd(v, 0.0, 0.0)
        t0 = time.monotonic()
        wsum = 0.0
        while time.monotonic() - t0 < (secs + 5) * 4:
            s = sim_now()
            if s is not None and s - s0 >= secs:
                break
            wv = wheel_vel()
            if not any(math.isnan(x) for x in wv):
                wsum = statistics.mean(wv)
            time.sleep(0.1)
        cmd(0.0, 0.0, 0.0)
        time.sleep(1.2)                      # 等它停稳（含滑行）
        p1 = pose()
        j1 = wheel_vel_pos()
        if p1 is None or j1 is None:
            print("  采样失败"); return 2
        dw = [b - a for a, b in zip(j0, j1)]
        wheel_dist = statistics.mean([abs(x) * R for x in dw])
        # 车体前向位移（起步时的车体系）
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        c, s_ = math.cos(p0[2]), math.sin(p0[2])
        truth_dist = abs(c * dx + s_ * dy)
        slip = wheel_dist - truth_dist
        rate = slip / wheel_dist * 100 if wheel_dist > 1e-6 else float("nan")
        print(f"{wheel_dist:9.3f} {truth_dist:9.3f} {slip:8.3f} {rate:6.1f}% "
              f"{wsum * R / v:9.2f}   (#{i + 1})")
    print("\n判读：打滑率 >10% 说明起停车轮空转，M2 的多段起停会累积成 0.3~0.5 m 偏差")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cmd", type=float, default=0.2, help="线速度（m/s）；转向用 0.5 rad/s")
    ap.add_argument("--secs", type=float, default=2.0, help="跑到稳态用多少仿真秒")
    ap.add_argument("--turn", type=float, default=None,
                    help="先转到这个车头朝向（度）再做测量。测起停/稳态前必须转到空巷方向，"
                         "否则会量到'被货架顶住'而不是'起步打滑'")
    ap.add_argument("--ramp-check", action="store_true",
                    help="量一次完整起停的打滑量（轮子滚过 vs 车真走过）")
    ap.add_argument("--sweep-heading", action="store_true",
                    help="在当前位置把车头转一圈，每个朝向量一次前向兑现率")
    args = ap.parse_args()
    if pose() is None:
        print("读不到真值位姿 —— 仿真没起？")
        return 2
    print(f"轮半径按 SDF 取 R={R}，四轮合成系数 lx+ly={LX_LY}\n")
    if args.turn is not None:
        turn_to(args.turn)
        p_after = pose()
        if p_after is not None:
            print(f"已转到 yaw={math.degrees(p_after[2]):+.1f}°  位置 "
                  f"x={p_after[0]:+.2f} y={p_after[1]:+.2f}\n")
    if args.ramp_check:
        return ramp_check(args.cmd, args.secs)
    if args.sweep_heading:
        return sweep(args.cmd, args.secs)
    measure("vx", args.cmd, 0.0, 0.0, args.secs)
    measure("vy", 0.0, args.cmd, 0.0, args.secs)
    measure("wz", 0.0, 0.0, 0.5, args.secs)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
