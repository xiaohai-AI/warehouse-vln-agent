#!/usr/bin/env python3
"""全向性判定：在固定车头朝向下扫一圈方向，看每个方向"命令 / 轮角 / 真值"对不对得上。

要回答的问题（M2 之后必须定性）：
  这个麦克纳姆在仿真里到底是不是全向的？
  ——如果"某个车头朝向下能走、转个身就走不动"，那是摩擦锥方向没跟着车身转（框架问题，可修）；
  ——如果各方向都只有命令值的 30%~50%，那是接触/摩擦模型本质不保真（要换建模方式）。

用法：
  .venv/bin/python scripts/holonomic_probe.py                 # 当前朝向扫 8 个方向
  .venv/bin/python scripts/holonomic_probe.py --turn 45       # 先转到 45° 再扫同一组方向
  .venv/bin/python scripts/holonomic_probe.py --turn 90 --speed 0.2

判读：理想行应为 `比值≈1.00、角度误差≈0°`。
  两次运行（0° 与 45°/90°）的表如果**不一样**，说明摩擦方向没随车体旋转 -> 框架 bug。
"""
from __future__ import annotations

import argparse
import math
import re
import subprocess
import time

W, WORLD = "depot_bot", "depot"
WHEELS = ("wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint")
R, HALF = 0.076, 0.41


def run(cmd, timeout=8.0):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout).stdout
    except subprocess.TimeoutExpired:
        return ""


def sim_now() -> float | None:
    txt = run(["ign", "topic", "-e", "-t", "/clock", "-n", "1"])
    m = re.search(r"^sim \{\s*sec: (\d+)\s*nsec: (\d+)", txt, re.S | re.M)
    return None if not m else int(m.group(1)) + int(m.group(2)) * 1e-9


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


def joints() -> list[float]:
    txt = run(["ign", "topic", "-e", "-t", "/joint_states", "-n", "1"])
    out = []
    for w in WHEELS:
        m = re.search(r'name: "%s".*?position: (-?[\d.eE+-]+)' % w, txt, re.S)
        out.append(float(m.group(1)) if m else float("nan"))
    return out


def cmd(vx: float, vy: float, wz: float = 0.0) -> None:
    run(["ign", "topic", "-t", f"/model/{W}/cmd_vel", "-m", "ignition.msgs.Twist",
         "-p", f"linear: {{x: {vx:.4f}, y: {vy:.4f}}}, angular: {{z: {wz:.4f}}}"], timeout=3)


def wheel_body_delta(d) -> tuple[float, float]:
    dx = R / 4 * sum(d)
    dy = R / 4 * (-d[0] + d[1] + d[2] - d[3])
    return dx, dy


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def turn_to(deg: float, max_s: float = 90.0) -> None:
    target = math.radians(deg)
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
        err = wrap(target - p[2])
        if abs(err) < math.radians(2.0):
            break
        cmd(0.0, 0.0, max(-0.35, min(0.35, 1.6 * err)))
        time.sleep(0.05)
    cmd(0.0, 0.0, 0.0)
    time.sleep(0.6)


def goto(x: float, y: float, speed: float = 0.15, tol: float = 0.03, max_s: float = 120.0) -> None:
    """闭环回到世界坐标点（世界速度->车体系），用来让每个方向都从同一位置出发。"""
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
        dx, dy = x - p[0], y - p[1]
        d = math.hypot(dx, dy)
        if d < tol:
            break
        vv = min(speed, 0.5 * d + 0.03)
        c, s_ = math.cos(p[2]), math.sin(p[2])
        wx, wy = vv * dx / d, vv * dy / d
        cmd(c * wx + s_ * wy, -s_ * wx + c * wy, 0.0)
        time.sleep(0.05)
    cmd(0.0, 0.0, 0.0)
    time.sleep(0.6)


def one_direction(v: float, ang_deg: float, secs: float):
    a = math.radians(ang_deg)
    p0 = pose()
    j0 = joints()
    s0 = sim_now()
    if p0 is None or s0 is None:
        return None
    p_start = p0
    # 起步段会拉低平均速度（M1 判据当年也是为此丢掉前 25%），这里只取后半程稳态
    mid = None
    t0 = time.monotonic()
    while time.monotonic() - t0 < secs * 6 + 12:
        s = sim_now()
        if s is not None and s - s0 >= secs:
            break
        if mid is None and s is not None and s - s0 >= secs * 0.5:
            pm = pose()
            jm = joints()
            mid = (pm, jm, s)
        cmd(v * math.cos(a), v * math.sin(a), 0.0)
    cmd(0.0, 0.0, 0.0)
    time.sleep(0.6)
    p_end = pose()
    j_end = joints()
    s_end = sim_now()
    if p_end is None or s_end is None:
        return None
    if mid is not None and mid[0] is not None and s_end - mid[2] > 1e-3:
        p0, j0, s0 = mid[0], mid[1], mid[2]      # 用半程之后的稳态窗口
    p1, j1, s1 = p_end, j_end, s_end
    dt = s1 - s0
    wx, wy = p1[0] - p0[0], p1[1] - p0[1]
    c, s_ = math.cos(p0[2]), math.sin(p0[2])
    tx, ty = c * wx + s_ * wy, -s_ * wx + c * wy          # 真值位移投到起点车体系
    d = [b - a_ for a_, b in zip(j0, j1)]
    bx, by = wheel_body_delta(d)
    goto(p_start[0], p_start[1])
    v_true = math.hypot(tx, ty) / dt if dt > 0 else float("nan")
    v_wheel = math.hypot(bx, by) / dt if dt > 0 else float("nan")
    ang_true = math.degrees(math.atan2(ty, tx)) if (tx or ty) else float("nan")
    return v_true, math.degrees(wrap(math.atan2(ty, tx) - a)) if (tx or ty) else float("nan"), v_wheel, dt


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--speed", type=float, default=0.20)
    ap.add_argument("--secs", type=float, default=2.5, help="每个方向走多少仿真秒")
    ap.add_argument("--turn", type=float, default=None, help="先转到这个朝向（度）再扫")
    ap.add_argument("--step", type=float, default=45.0, help="方向步长（度）")
    args = ap.parse_args()

    p = pose()
    if p is None:
        print("读不到真值位姿 —— 仿真没起？")
        return 2
    if args.turn is not None:
        turn_to(args.turn)
    p = pose()
    if p is None:
        print("转向后读不到位姿，判据无效")
        return 2
    print(f"车头 yaw={math.degrees(p[2]):+.1f}°  位置 x={p[0]:+.2f} y={p[1]:+.2f}  "
          f"命令速度 {args.speed} m/s  每方向 {args.secs} 仿真秒\n", flush=True)
    print(f"{'方向':>6s} {'真值速度':>9s} {'比值':>6s} {'角度误差':>8s} {'轮角速度':>9s}")
    ratios = []
    angs = []
    deg = 0.0
    while deg < 360.0:
        r = one_direction(args.speed, deg, args.secs)
        if r is None:
            print(f"{deg:6.0f}  读取失败")
        else:
            v_true, ang_err, v_wheel, _ = r
            ratio = v_true / args.speed if args.speed else float("nan")
            ratios.append(ratio)
            angs.append(abs(ang_err))
            print(f"{deg:6.0f} {v_true:9.3f} {ratio:6.2f} {ang_err:8.1f}° {v_wheel:9.3f}")
        deg += args.step
        time.sleep(0.4)

    cmd(0.0, 0.0, 0.0)
    if ratios:
        good = sum(1 for x in ratios if x >= 0.8)
        print(f"\n[总结] 方向数 {len(ratios)}  比值中位 {sorted(ratios)[len(ratios)//2]:.2f}  "
              f"最小 {min(ratios):.2f}  最大 {max(ratios):.2f}")
        print(f"       速度达 80% 以上的方向: {good}/{len(ratios)}   角度误差中位 {sorted(angs)[len(angs)//2]:.1f}°")
        if good < len(ratios) * 0.7:
            print("       判定：这些方向不能可靠执行 -> 摩擦/接触模型不保真（或 fdir1 框架不对）")
        else:
            print("       判定：全向性基本成立（个别方向偏弱）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
