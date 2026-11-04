#!/usr/bin/env python3
"""把车开到世界坐标里的指定点（可经若干路点），用真值闭环。

为什么单独写：`map_drive.py` 只会"沿世界 +X 走 + 压住 y"，从出生点 (-31,0) 到 A 巷里的
(-20,-10.75) 需要**两段**（先南移 10.75 m、再东进 10 m），否则斜穿会撞货架端面。

⚠️ `--wp` 必须写成 `--wp=x,y`（等号），不能写 `--wp x,y`：
     路点常以负号开头（-31,-10.75），argparse 会把 "-31,-10.75" 当成选项而不是值，
     报 `argument --wp: expected one argument`。

实现要点（都是踩过的）：
  * 真值只读 `/world/depot/pose/info`，用后台线程轮询（同步读会把闭环压到 <1 Hz）；
  * 每个 tick 把**世界系速度**转进车体系（车头有偏差时直接给车体系速度会走斜线）；
  * 真值冻结保护：一直发命令但真值 10 s 不变就中止（别盲开）；
  * 停车必发零速。

用法
  .venv/bin/python scripts/drive_to.py --x -20 --y -10.75 --yaw 0 \\
      --wp=-31,-10.75              # 先到西横通道南口，再进 A 巷（**等号形式**，见下）
  .venv/bin/python scripts/drive_to.py --x -20 --y -10.75 --yaw 0   # 只给终点（直线可达时）
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from slam_route import TruthPoller, cmd  # noqa: E402  复用已验证的真值读取 + cmd_vel 发布


def parse_wp(s: str) -> tuple[float, float]:
    x, y = s.split(",")
    return float(x), float(y)


def _kick(poller: TruthPoller, dx: float, dy: float, d: float,
          amp: float = 0.35, dur: float = 0.25, settle: float = 0.30) -> None:
    """卡住时朝目标方向踢一脚（定长脉冲），然后短暂停稳。

    用途只有一个：破静摩擦。幅度必须够大（0.35 m/s），低于约 0.1 m/s 的连续命令
    会被四轮静摩擦完全吃掉；时长要短（0.25 s），避免把"差一点"踢成"冲过头"。
    """
    p = poller.get()
    if p is None or d < 1e-6:
        return
    wx, wy = amp * dx / d, amp * dy / d
    c, s = math.cos(p[2]), math.sin(p[2])
    bvx, bvy = c * wx + s * wy, -s * wx + c * wy
    t0 = time.monotonic()
    while time.monotonic() - t0 < dur:
        cmd(bvx, bvy, 0.0)
        time.sleep(0.02)
    cmd(0.0, 0.0, 0.0)
    time.sleep(settle)


def drive_to_point(poller: TruthPoller, tx: float, ty: float, speed: float,
                   tol: float, max_seconds: float, progress=None) -> bool:
    """开到 (tx,ty)：闭环 P 控制 + **速度下限** + 卡住才踢一脚。

    两次踩过的坑（都在这条路径上）：
      * 纯 `v = 0.7·d` 衰减：最后 0.2–0.5 m 时命令只剩 0.05–0.2 m/s，被四轮静摩擦顶住
        不动，误差永远收不进容差 —— 实测在进巷最后 3 m 上蹭满 420 s。
      * 改用定长脉冲：横向（麦克纳姆侧移）兑现率只有约 30%，8 个"0.24 m"的脉冲
        实际只走 0.24 m，残差仍 0.55 m。
    现在：命令速度下限 0.15 m/s（保证一定克服静摩擦）、增益 0.8/d 收敛；
    只有真卡住（1.5 s 内位移 <3 cm）才补一个 0.35 m/s × 0.25 s 的踢脚。
    """
    t0 = time.monotonic()
    last_moved = poller.get() or (0.0, 0.0, 0.0)
    t_moved = time.monotonic()
    t_report = 0.0
    while True:
        # 可选进度回调（诊断脚本用它边开边打印三条轨迹的位移比值）
        if progress is not None and time.monotonic() - t_report > 5.0:
            t_report = time.monotonic()
            try:
                progress(time.monotonic() - t0)
            except Exception as e:      # 回调出错绝不能影响开车
                print(f"[drive] progress 回调异常：{e}")
        if time.monotonic() - t0 > max_seconds:
            print(f"[drive] 超时（{max_seconds:.0f} s）未到 ({tx:+.2f},{ty:+.2f})")
            return False
        p = poller.get()
        if p is None:
            time.sleep(0.05)
            continue
        dx, dy = tx - p[0], ty - p[1]
        d = math.hypot(dx, dy)
        if d < tol:
            cmd(0.0, 0.0, 0.0)
            return True
        moved = math.hypot(p[0] - last_moved[0], p[1] - last_moved[1])
        if moved > 0.03:
            last_moved, t_moved = p, time.monotonic()
        elif time.monotonic() - t_moved > 10.0:
            print("[drive] 真值 10 s 没变化但一直在发命令 —— 被顶住或真值冻结，中止")
            return False
        elif time.monotonic() - t_moved > 1.5:
            # 卡住了：踢一脚（幅度必须够大，否则还是被静摩擦吃掉）
            _kick(poller, dx, dy, d, amp=0.35, dur=0.25)
            last_moved, t_moved = poller.get() or p, time.monotonic()
            continue
        v = min(speed, max(0.15, 0.8 * d))          # 下限 0.15：低于它会被静摩擦吃掉
        wx, wy = v * dx / d, v * dy / d
        c, s = math.cos(p[2]), math.sin(p[2])
        cmd(c * wx + s * wy, -s * wx + c * wy, 0.0)     # 世界速度 -> 车体速度
        time.sleep(0.05)


def turn_to(poller: TruthPoller, yaw: float, tol_deg: float = 3.0,
            max_seconds: float = 25.0) -> bool:
    t0 = time.monotonic()
    while time.monotonic() - t0 < max_seconds:
        p = poller.get()
        if p is None:
            time.sleep(0.05)
            continue
        err = math.atan2(math.sin(yaw - p[2]), math.cos(yaw - p[2]))
        if abs(math.degrees(err)) < tol_deg:
            return True
        cmd(0.0, 0.0, max(-0.5, min(0.5, 1.5 * err)))
        time.sleep(0.05)
    print("[drive] 转向超时")
    return False


def align_to(poller: TruthPoller, yaw: float, tol_deg: float = 2.0,
             tries: int = 6, wz: float = 0.25, settle: float = 0.6) -> tuple[bool, float]:
    """脉冲式对齐（转一小段 → 停稳 → 量残差 → 按新残差再来）。

    为什么不用纯 P 闭环：误差 10° 时 wz 打到 0.5 rad/s，车体动量带着冲过头（实测残差 10°）；
    误差 2–4° 时 wz 只有 0.06 rad/s，被四轮静摩擦（mu=3.0）完全吃掉，于是"转不动 → 超时"。
    脉冲给出**足够大的角速度**再停稳测量，两种情况都能收敛。
    返回 (是否达标, 最终残差度数)。
    """
    err_deg = 999.0
    for _ in range(tries):
        p = poller.get()
        if p is None:
            time.sleep(0.05)
            continue
        err = math.atan2(math.sin(yaw - p[2]), math.cos(yaw - p[2]))
        err_deg = abs(math.degrees(err))
        if err_deg <= tol_deg:
            return True, err_deg
        # 脉冲时长：按残差算，下限 0.10 s（再短会被静摩擦吃掉），上限 0.9 s
        dur = max(0.10, min(0.9, abs(err) / wz * 0.85))
        sign = 1.0 if err > 0 else -1.0
        t0 = time.monotonic()
        while time.monotonic() - t0 < dur:
            cmd(0.0, 0.0, sign * wz)
            time.sleep(0.02)
        cmd(0.0, 0.0, 0.0)
        time.sleep(settle)
    p = poller.get()
    if p is not None:
        err = math.atan2(math.sin(yaw - p[2]), math.cos(yaw - p[2]))
        err_deg = abs(math.degrees(err))
    return err_deg <= tol_deg, err_deg


def turn_to_precise(poller: TruthPoller, yaw: float, tol_deg: float = 2.0) -> tuple[bool, float]:
    """两段式转向：P 律粗转（负责 180° 这种大角度）+ 脉冲精修（负责最后几度）。

    只用脉冲：每脉冲最多 0.9 s × 0.25 rad/s ≈ 13°，转 180° 要 14 个脉冲，而预算只有 6 个
    -> 实测残差 58–135°。只用 P 律：大角度冲过头、小角度被静摩擦卡住。必须两段。
    """
    turn_to(poller, yaw, tol_deg=10.0, max_seconds=30.0)
    cmd(0.0, 0.0, 0.0)
    time.sleep(0.6)
    return align_to(poller, yaw, tol_deg=tol_deg, tries=8, wz=0.25, settle=0.6)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, required=True)
    ap.add_argument("--y", type=float, required=True)
    ap.add_argument("--yaw", type=float, default=None, help="终点朝向（度）；不给就不转")
    ap.add_argument("--wp", action="append", default=[], help='中间路点 "x,y"，可重复')
    # 速度：0.28 太保守（20 m 要走 70 仿真秒，GUI 下 RTF≈0.3 就必然超时）。
    # 0.60 m/s 是"行驶"速度；判据 M1 只验到 0.40（vx 误差 <10%），
    # 但 drive_to 不进判据，弯道/末端有 0.8*d 的比例减速，所以取 0.60。
    ap.add_argument("--speed", type=float, default=0.60)
    ap.add_argument("--tol", type=float, default=0.15)
    # 0 = 按每段直线距离自动算（RTF 最差 0.28 也不超时）；给正数则用给定值
    ap.add_argument("--max-seconds", type=float, default=0.0)
    ap.add_argument("--force", action="store_true",
                    help="跳过路径守卫（明知会压货架也要开）")
    args = ap.parse_args()

    print("[drive] ⚠️  开路点对点驱动：**没有在线避障**（不读激光 / 不看 costmap）。")
    print("[drive]    只做开车前的几何预检（path_guard 扫世界 SDF 全部静态碰撞体）。")
    print("[drive]    手动驾驶请用：make nav-to X=.. Y=.. YAW=..（Nav2，带在线避障）")

    poller = TruthPoller()
    t0 = time.monotonic()
    while poller.get(max_age=10.0) is None and time.monotonic() - t0 < 30:
        time.sleep(0.3)
    p0 = poller.get()
    if p0 is None:
        print("[drive] 读不到真值位姿 —— 仿真没起？")
        poller.stop()
        return 2
    print(f"[drive] 起点 ({p0[0]:+.2f},{p0[1]:+.2f}) yaw={math.degrees(p0[2]):+.1f}°")

    waypoints = [parse_wp(w) for w in args.wp] + [(args.x, args.y)]

    # ---- 路径守卫：开车前先算（"斜穿货架端面"是实测踩过两次的坑，见 path_guard.py 文件头）
    from path_guard import check_route, load_layout
    blocks, aisles = load_layout()
    if blocks:
        ok, _bad = check_route((p0[0], p0[1]), waypoints, blocks, aisles, verbose=True)
        if not ok:
            if not args.force:
                print("[drive] 拒绝执行（exit 3）。确认要硬开请加 --force")
                poller.stop()
                return 3
            print("[drive] --force：照开（会擦货架，后果自负）")

    try:
        for i, (wx_, wy_) in enumerate(waypoints):
            # 每段单独算预算：GUI 下 RTF 会掉到 0.3 左右，固定 180 s 会把正常行程判成超时
            p_cur = poller.get()
            leg = math.hypot(wx_ - p_cur[0], wy_ - p_cur[1]) if p_cur else 0.0
            budget = args.max_seconds if args.max_seconds > 0 else 25.0 + leg / max(0.2, args.speed) * 3.6
            print(f"[drive] -> ({wx_:+.2f},{wy_:+.2f})  直线 {leg:.1f} m  预算 {budget:.0f} s")
            if not drive_to_point(poller, wx_, wy_, args.speed, args.tol, budget):
                return 1
        if args.yaw is not None:
            if not turn_to(poller, math.radians(args.yaw), max_seconds=45.0):
                return 1
        p = poller.get()
        if p is not None:
            print(f"[drive] 到达 ({p[0]:+.2f},{p[1]:+.2f}) yaw={math.degrees(p[2]):+.1f}°")
        return 0
    finally:
        cmd(0.0, 0.0, 0.0)
        if poller is not None:
            poller.stop()


if __name__ == "__main__":
    sys.exit(main())
