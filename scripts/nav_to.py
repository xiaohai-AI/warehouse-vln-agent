#!/usr/bin/env python3
"""用 **Nav2** 开到 (x, y[, yaw])：带在线避障（全局 + 局部 costmap）。

# 和 drive_to.py 的分工（选错工具会撞车，写在这里免得再犯）

| 工具 | 控制方式 | 在线避障 | 适用 |
|---|---|---|---|
| `drive_to.py` | 点对点 cmd_vel（开路追航点）| **没有** | 脚本里的精确停位（SEG-2 采集、S3 检查）、离线预检 |
| `nav_to.py`（本文件） | Nav2 `NavigateToPose` 单目标 | **有**（costmap，D31 已修好全局层）| **一切手动/演示驾驶** |

**2026-11-03 事故**：手动用 `make drive-to` 沿 C 巷中线走，正面撞上静态叉车 ——
`drive_to` 不看激光/costmap，路线上有没有东西它不知道。要在线避障就得走 Nav2。

# 为什么用 `ros2 action send_goal` 子进程而不是 nav2_simple_commander

本进程里已经有 `TruthPoller` 的后台 spin（读真值位姿/odom），再塞一个 Navigator 节点
就是"两处 spin 挤一个 context"，实测会 `IndexError: wait set index too big` 并在退出时 SIGABRT。
CLI 是独立进程，干净 —— 这条与 `depot_vln/agent/ros_tools.py::_nav2_goto` 完全一致的实现。

# 用法

  python3 scripts/nav_to.py --x -10.80 --y -2.15 --yaw 90
  python3 scripts/nav_to.py --x -10.80 --y -2.15 --yaw 90 --budget 300 --no-yaw-fix

退出码：0 到达；2 Nav2 报失败/超时；3 目标被拒收（栈没起好）；4 前置条件不满足
"""
from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

ROS_SETUP = "/opt/ros/humble/setup.bash"


def _sh(cmd: str, timeout: float = 20.0) -> str:
    try:
        p = subprocess.run(["bash", "-lc", f"source {ROS_SETUP} && {cmd}"],
                           capture_output=True, text=True, timeout=timeout)
        return ((p.stdout or "") + (p.stderr or "")).strip()
    except subprocess.TimeoutExpired:
        return ""


def nav2_state() -> dict[str, str]:
    """三个关键节点的 lifecycle 状态。READY 只保证 bt_navigator，必须三个都 active。"""
    out = {}
    for ns in ("controller_server", "planner_server", "bt_navigator"):
        s = _sh(f"ros2 lifecycle get /{ns} 2>/dev/null | head -1")
        out[ns] = (s.split()[0] if s.split() else "?")
    return out


def goto(x: float, y: float, yaw: float, budget: float) -> tuple[bool, str]:
    """发一个 NavigateToPose（map 帧；D31：map 帧 = 世界坐标）。被拒收要重试。"""
    goal = ("{pose: {header: {frame_id: map}, pose: {position: {x: %.3f, y: %.3f, z: 0.0}, "
            "orientation: {x: 0.0, y: 0.0, z: %.6f, w: %.6f}}}}"
            % (x, y, math.sin(yaw / 2.0), math.cos(yaw / 2.0)))
    # **必须经 bash -lc 且先 source**：本进程是 .venv/bin/python，PATH 里没有 ros2
    # （踩过：FileNotFoundError: 'ros2'）。frame_id 不加引号，避免和 bash 单引号打架。
    cli = (f"source {ROS_SETUP} && ros2 action send_goal /navigate_to_pose "
           f"nav2_msgs/action/NavigateToPose '{goal}'")
    out = ""
    for attempt in range(1, 4):
        try:
            p = subprocess.run(["bash", "-lc", cli], capture_output=True, text=True,
                               timeout=budget + 30)
        except subprocess.TimeoutExpired:
            return False, f"send_goal 超过 {budget + 30:.0f}s 没结果"
        out = ((p.stdout or "") + (p.stderr or "")).strip()
        if "SUCCEEDED" in out or "ABORTED" in out or "CANCELED" in out:
            return ("SUCCEEDED" in out), out[-600:]
        if "rejected" not in out.lower():
            return False, out[-600:]
        print(f"[nav] Nav2 拒收目标（第 {attempt} 次）—— 栈可能还在激活，5 s 后重试")
        time.sleep(5.0)
    return False, out[-600:]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, required=True)
    ap.add_argument("--y", type=float, required=True)
    ap.add_argument("--yaw", type=float, default=None, help="到达朝向（度）")
    ap.add_argument("--budget", type=float, default=400.0, help="墙钟预算（秒）")
    ap.add_argument("--no-yaw-fix", action="store_true",
                    help="不做末端朝向精修（Nav2 的朝向容差较宽）")
    ap.add_argument("--dry", action="store_true", help="只打印将要发的 action CLI，不发目标")
    args = ap.parse_args()

    # ---- dry：只打印将要发的命令，跳过所有前置检查（离线可验）
    if args.dry:
        yaw_d = math.radians(args.yaw) if args.yaw is not None else 0.0
        goal_d = ("{pose: {header: {frame_id: map}, pose: {position: {x: %.3f, y: %.3f, z: 0.0}, "
                  "orientation: {x: 0.0, y: 0.0, z: %.6f, w: %.6f}}}}"
                  % (args.x, args.y, math.sin(yaw_d / 2.0), math.cos(yaw_d / 2.0)))
        print("[nav] dry：将要执行的命令 ↓")
        print(f"source {ROS_SETUP} && ros2 action send_goal /navigate_to_pose "
              f"nav2_msgs/action/NavigateToPose '{goal_d}'")
        return 0

    # ---- 前置：仿真在跑？Nav2 active？
    if not _sh("pgrep -f '[i]gn gazebo' | head -1"):
        print("[nav] ✗ 仿真没起。先 make sim SIM_ARGS='gui:=true'（headless 用 gui:=false）")
        return 4
    st = nav2_state()
    if any(v != "active" for v in st.values()):
        print(f"[nav] ✗ Nav2 未就绪：{st}")
        print("[nav]   用 `bash scripts/nav_to.sh --x .. --y ..` 会自动起栈；")
        print("[nav]   或手动：STACK_ONLY=1 bash scripts/m2_static_run.sh")
        return 4
    print(f"[nav] Nav2 active（controller/planner/bt_navigator）→ 目标 ({args.x:+.2f},{args.y:+.2f})"
          + (f" yaw {args.yaw:+.0f}°" if args.yaw is not None else "")
          + f"，预算 {args.budget:.0f}s")

    yaw = math.radians(args.yaw) if args.yaw is not None else 0.0
    ok, tail = goto(args.x, args.y, yaw, args.budget)
    if not ok:
        print("[nav] ✗ Nav2 没能到达。末尾输出：")
        print("      " + tail.replace("\n", "\n      "))
        return 2
    print("[nav] Nav2 报告 SUCCEEDED")

    # ---- 末端朝向精修（Nav2 的 goal 朝向容差宽，拍照/verify 需要更准）
    if args.yaw is not None and not args.no_yaw_fix:
        try:
            from drive_to import turn_to_precise  # 与 drive_to 共用同一套 cmd_vel 实现
            from slam_route import TruthPoller

            poller = TruthPoller()
            t0 = time.monotonic()
            while poller.get(max_age=10.0) is None and time.monotonic() - t0 < 20:
                time.sleep(0.3)
            ok2, err = turn_to_precise(poller, yaw, 2.0)
            poller.stop()
            print(f"[nav] 朝向精修：{'✓' if ok2 else '✗'} 残差 {err:.1f}°")
        except Exception as e:  # noqa: BLE001
            print(f"[nav] 朝向精修跳过（{e}）")

    # ---- 实测落点
    try:
        from slam_route import TruthPoller

        poller = TruthPoller()
        t0 = time.monotonic()
        p = None
        while time.monotonic() - t0 < 15:
            p = poller.get()
            if p is not None:
                break
            time.sleep(0.3)
        poller.stop()
        if p is not None:
            err = math.hypot(p[0] - args.x, p[1] - args.y)
            print(f"[nav] 实测落点 ({p[0]:+.2f},{p[1]:+.2f}) yaw={math.degrees(p[2]):+.1f}° "
                  f"偏差 {err:.2f} m")
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
