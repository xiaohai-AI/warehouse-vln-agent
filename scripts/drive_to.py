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


def _turn_rate() -> float:
    """转向角速度（rad/s，**仿真时间**），来自 config/nav.yaml:turn_rate。"""
    import yaml

    p = Path(__file__).resolve().parents[1] / "config" / "nav.yaml"
    try:
        return float((yaml.safe_load(p.read_text(encoding="utf-8")) or {}).get("turn_rate", 0.9))
    except Exception:
        return 0.9


TURN_RATE = _turn_rate()


def turn_to(poller: TruthPoller, yaw: float, tol_deg: float = 3.0,
            max_seconds: float = 25.0, stall_seconds: float = 3.0) -> bool:
    """纯 P 律转向。**误差不再下降就早退**，不把整个预算耗在静摩擦上。

    为什么加早退：四轮静摩擦（mu=3.0）会吃掉小误差对应的微小 wz，残差卡在 2–5° 不动，
    而这里只有 P 律 —— 于是每次转向都跑满 25–30 s 超时。实测每条观测位多花 ~30 s 墙钟
    （SEG-2 全巷采集 274 帧，光这一项就是 2 小时）。精修由 `align_to` 的脉冲负责，
    所以这里"卡住就交棒"是正确行为，不是失败。
    """
    t0 = time.monotonic()
    best = 1e9
    last_progress = t0
    while time.monotonic() - t0 < max_seconds:
        p = poller.get()
        if p is None:
            time.sleep(0.05)
            continue
        err = math.atan2(math.sin(yaw - p[2]), math.cos(yaw - p[2]))
        err_deg = abs(math.degrees(err))
        if err_deg < tol_deg:
            return True
        now = time.monotonic()
        if err_deg < best - 0.3:
            best, last_progress = err_deg, now
        elif now - last_progress > stall_seconds:
            cmd(0.0, 0.0, 0.0)
            return False            # 停住了：交给 align_to 的脉冲精修
        cmd(0.0, 0.0, max(-TURN_RATE, min(TURN_RATE, 1.5 * err)))
        time.sleep(0.05)
    print("[drive] 转向超时")
    return False


def align_to(poller: TruthPoller, yaw: float, tol_deg: float = 2.0,
             tries: int | None = None, wz: float = 0.6, settle: float = 0.35,
             max_seconds: float = 90.0) -> tuple[bool, float]:
    """脉冲式对齐：转一小段 → 停稳 → 量残差 → 按**实测角速度**定下一段长度。

    为什么不能沿用"按仿真角速度算脉冲时长"
    ----------------------------------------
    gz 默认 RTF ≈ 0.22（带相机渲染）。脉冲里写 `dur = |err|/wz` 是按**仿真秒**算的，
    但 `time.sleep(dur)` 是**墙钟秒** —— 于是每个脉冲只转到了预期角度的 22%。
    实测 90° 误差：脉冲 0.9 s 预期 12.9°，实际只有 2.9°；28 s 预算内最多转 58°，
    90° 转不完、180° 更不可能，于是"原地转很久 → 还是歪的 → 丢帧"
    （B 巷实测丢 9/48 帧，正是这个原因）。

    现在每次脉冲后用真值量一下**实际转了多少**，更新 rad/墙钟秒 的估计，再定下一段时长。
    这样与 RTF 无关：仿真跑得快就自动变短，慢就自动变长。

    为什么仍然用"脉冲 + 停稳"而不是连续 P 闭环：
    大误差时连续 P 律（wz 打到 0.5）会带着动量冲过头；小误差时微小 wz 被四轮静摩擦
    （mu=3.0）完全吃掉。脉冲给足角速度、停稳再测量，两种情况都能收敛。
    """
    err_deg = 999.0
    t_start = time.monotonic()
    # 先验：TURN_RATE × RTF 0.22 ≈ 0.2 rad/墙钟秒；第一次脉冲后立刻被实测替换
    rate = TURN_RATE * 0.22
    n = 0
    stalled = 0
    while (tries is not None and n < tries) or \
            (tries is None and time.monotonic() - t_start < max_seconds):
        n += 1
        p = poller.get()
        if p is None:
            time.sleep(0.05)
            continue
        err = math.atan2(math.sin(yaw - p[2]), math.cos(yaw - p[2]))
        err_before = abs(err)
        err_deg = math.degrees(err_before)
        if err_deg <= tol_deg:
            return True, err_deg
        # 脉冲时长按**墙钟角速度**算：rate 是 rad/墙钟秒
        dur = max(0.15, min(5.0, err_before / max(rate, 1e-3) * 0.85))
        sign = 1.0 if err > 0 else -1.0
        t_p = time.monotonic()
        while time.monotonic() - t_p < dur:
            cmd(0.0, 0.0, sign * wz)
            time.sleep(0.02)
        cmd(0.0, 0.0, 0.0)
        time.sleep(settle)
        # 量实际转了多少，更新 rate（EMA，抗一次抖动）
        p2 = poller.get()
        spent = time.monotonic() - t_p
        if p2 is not None and spent > 0.05:
            err_after = math.atan2(math.sin(yaw - p2[2]), math.cos(yaw - p2[2]))
            progress = err_before - abs(err_after)
            if progress > 0.02:
                rate = 0.6 * rate + 0.4 * (progress / spent)
                stalled = 0
            else:
                stalled += 1
                if stalled >= 2:      # 连着两次没转过去：物理上卡住了，别耗预算
                    break
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
    # 粗转只负责"把大角度快速拉近"，剩下的由自适应精修收敛（不再有第二次粗转）
    turn_to(poller, yaw, tol_deg=15.0, max_seconds=15.0)
    cmd(0.0, 0.0, 0.0)
    time.sleep(0.3)
    return align_to(poller, yaw, tol_deg=tol_deg, tries=None, wz=TURN_RATE, settle=0.35)


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
