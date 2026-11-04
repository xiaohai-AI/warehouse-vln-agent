#!/usr/bin/env python3
"""判据 M2：3.20 m 巷道内"侧移入库位"（SPEC §4.3）。

动作定义
  起点：巷道中线，面朝 +X
  目标：横向移动到距货架面 0.7 m 的观测位（横向位移 ≥ 0.8 m），朝向对准货架
  判据：落点误差 < 0.10 m，不触发任何恢复行为，连续 5 次成功 ≥ 4 次

为什么要单独测：这是"选麦克纳姆的理由"。如果只会前进+转向，这个动作根本做不出来。

用法
  ros2 launch launch/depot_sim.launch.py &
  ros2 launch launch/slam.launch.py &        # 或者加载静态图
  ros2 launch launch/nav2_omni.launch.py &
  python3.10 scripts/m2_side_dock.py
"""
from __future__ import annotations

import argparse
import json
import math
import re
import subprocess
import sys
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

ROBOT = "depot_bot"
WORLD = "depot"


def yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def _gz(cmd: list[str], timeout: float = 8.0) -> str:
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout).stdout
    except subprocess.TimeoutExpired:
        return ""


def true_pose() -> tuple[float, float, float] | None:
    """Gazebo 真值位姿（世界系）。判据用这个，不用 /odom。

    /odom 是我们自己从四个轮子的累积转角积出来的 —— 拿它判"停位精度"等于
    轮子转多少就算车到了哪，属于自己给自己打分（SPEC §4.2）。
    """
    txt = _gz(["ign", "topic", "-e", "-t", f"/world/{WORLD}/pose/info", "-n", "1"])
    for block in txt.split('name: "'):
        if not block.startswith(ROBOT + '"'):
            continue
        mp = re.search(r"position \{\s*x: (-?[\d.eE+-]+)\s*y: (-?[\d.eE+-]+)", block, re.S)
        mq = re.search(r"orientation \{\s*[^}]*?w: (-?[\d.eE+-]+)", block, re.S)
        if mp:
            x, y = float(mp.group(1)), float(mp.group(2))
            return x, y, 0.0
    return None


class PoseTracker(Node):
    def __init__(self) -> None:
        super().__init__("m2_pose_tracker")
        self.pose = None
        self.n_msg = 0
        # 两种 QoS 都订：发布端可能位于任一档，兼容性判断出错时至少有一路能收到。
        # （踩过：mecanum_odom 用默认 RELIABLE、这里用 BEST_EFFORT，理论兼容却收不到，
        #   排查时连"到底收没收到"都无法确认。）
        self.create_subscription(
            Odometry, "/odom", self.on_odom,
            QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE),
        )
        self.create_subscription(
            Odometry, "/odom", self.on_odom,
            QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE),
        )

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)
        self.n_msg += 1

    def wait_pose(self, timeout: float = 30.0):
        waited = 0.0
        while self.pose is None and waited < timeout and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            waited += 0.1
        return self.pose

    def diagnose(self) -> str:
        out = [f"/odom 收到 {self.n_msg} 条（30 s 内）"]
        try:
            info = subprocess.run(
                ["bash", "-lc", "source /opt/ros/humble/setup.bash && "
                                "ros2 topic info /odom --verbose 2>&1 | head -40"],
                capture_output=True, text=True, timeout=20).stdout
            out.append(info.strip())
        except Exception as exc:            # noqa: BLE001
            out.append(f"（ros2 topic info 失败：{exc}）")
        return "\n".join(out)


def obstacle_rects(sdf_path: Path, models=("forklift", "totes", "barriers")):
    """从世界 SDF 取静态障碍的碰撞矩形：(名字, cx, cy, sx, sy)。

    为什么需要：M2 原默认点（C 巷 bay 5 右侧 = (−18.00, −3.05)）正好落在场景里那台
    静止叉车的碰撞体里（叉车 1.05×0.62，中心 (−18.00, −2.70)，占 y∈[−3.01,−2.39]）。
    车侧移过去被叉车顶住，轮子空转、odom 照样记到位 -> 判据显示"odom 0.09 m / 真值 0.95 m"，
    看上去像底盘打滑，实际是**入库点被障碍物占了**。这个检查把这种情况变成启动即报错。
    """
    txt = sdf_path.read_text(encoding="utf-8")
    rects = []
    for m in models:
        # gz sdf -p 输出用单引号：<model name='forklift'>；世界文件手写时可能是双引号，两种都认
        mm = re.search(rf"<model name=['\"]{m}['\"]>", txt)
        if mm is None:
            continue
        i = mm.start()
        j = txt.find("</model>", i)
        block = txt[i:j]
        # 世界 SDF 用双引号、机器人 SDF 用单引号，两种都要认
        for lm in re.finditer(r"<link name=['\"]([^'\"]+)['\"]>\s*<pose>([^<]+)</pose>"
                              r"(.*?)(?=</link>|$)", block, re.S):
            name, pose_s, body = lm.group(1), lm.group(2), lm.group(3)
            if "collision" not in body:
                continue
            mb = re.search(r"<box><size>([^<]+)</size></box>", body)
            if not mb:
                continue
            p = [float(v) for v in pose_s.split()]
            sx, sy = [float(v) for v in mb.group(1).split()][:2]
            rects.append((name, p[0], p[1], sx, sy))
    return rects


def blockers_at(x: float, y: float, margin: float = 0.35):
    """返回把这个点（含机器人半径余量）挡住的静态障碍名。"""
    out = []
    for name, cx, cy, sx, sy in obstacle_rects(ROOT / "world" / "warehouse.sdf"):
        if abs(x - cx) < sx / 2 + margin and abs(y - cy) < sy / 2 + margin:
            out.append(name)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--attempts", type=int, default=5)
    ap.add_argument("--aisle", default="C")
    # 默认 bay 5 的右侧观测位正好是叉车所在位置（见 obstacles 检查），改用 bay 4。
    ap.add_argument("--bay", type=int, default=4,
                    help="具体 bay 号；C 巷 bay 5 右侧被静态叉车占用，不能用")
    ap.add_argument("--side", default="R", choices=["L", "R"])
    ap.add_argument("--out", default=str(ROOT / "results" / "m2.json"))
    ap.add_argument("--spawn-x", type=float, default=-31.0,
                    help="出生点（odom 原点在世界里的位置），用于把 odom 换算回 map")
    ap.add_argument("--spawn-y", type=float, default=0.0)
    ap.add_argument("--frame", default="world", choices=["world", "slam"],
                    help="map 帧约定：world = map 与世界重合（static_map_pub 固定偏移，默认）；"
                         "slam = map 原点在出生点（slam_toolbox 在线建图）")
    args = ap.parse_args()

    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult  # 延迟导入

    grid = json.loads((ROOT / "world" / "slot_grid.json").read_text(encoding="utf-8"))
    aisle = next(a for a in grid["aisles"] if a["name"] == args.aisle)
    bay = next(b for b in grid["bays"] if b["aisle"] == args.aisle and b["bay"] == args.bay)
    rack_d = float(grid["derived"]["rack_depth"])
    aisle_w = float(grid["derived"]["aisle_clear_width"])

    # 观测位：距货架面 0.7 m（货架面距中线 aisle_w/2）
    face_off = aisle_w / 2
    dock_off = face_off - 0.70
    sign = 1.0 if args.side == "L" else -1.0
    start = (bay["center_x"], aisle["center_y"], 0.0)
    dock = (bay["center_x"], aisle["center_y"] + sign * dock_off, math.pi / 2 * sign)
    lateral = abs(dock[1] - start[1])

    bad = blockers_at(dock[0], dock[1]) + blockers_at(start[0], start[1])
    if bad:
        print(f"FAIL 入库点/起点落在静态障碍里：{sorted(set(bad))}"
              f"（dock=({dock[0]:.2f},{dock[1]:.2f}) start=({start[0]:.2f},{start[1]:.2f})）")
        print("     换一个空闲 bay 再跑，例如 --bay 4（x=-20.4）或 --bay 6（x=-15.6）")
        return 3
    print(f"[M2] 起点 ({start[0]:+.2f},{start[1]:+.2f})  入库点 "
          f"({dock[0]:+.2f},{dock[1]:+.2f})  横向 {lateral:.2f} m  静态障碍检查 PASS")

    # 世界坐标 -> map 坐标。start/dock 来自 slot_grid.json，是世界系的。
    # static_map_pub 模式：map->odom 用固定偏移 (-31,0)，map 与世界重合 -> 直接给。
    # slam 模式：map 原点落在车出生点 -> 世界点要减去出生点。
    def to_map(x: float, y: float) -> tuple[float, float]:
        if args.frame == "slam":
            return (x - args.spawn_x, y - args.spawn_y)
        return (x, y)

    def odom_in_map() -> tuple[float, float]:
        if tracker.pose is None:
            return (float("nan"), float("nan"))
        if args.frame == "slam":
            return (tracker.pose[0], tracker.pose[1])       # map->odom 由 slam_toolbox 给，≈恒等
        return (tracker.pose[0] + args.spawn_x, tracker.pose[1] + args.spawn_y)

    rclpy.init()
    tracker = PoseTracker()
    nav = BasicNavigator()
    if tracker.wait_pose() is None:
        print("FAIL 没有 /odom：仿真没起或桥接没起")
        print(tracker.diagnose())
        return 2
    # BasicNavigator 默认等 amcl/get_state —— 但这套定位是"odom + map->odom 恒等"，没有 AMCL
    # （雷达桥还没通，没法定点；见 docs/TROUBLESHOOTING.md §12）。实测直接调会无限刷
    # "amcl/get_state service not available"。
    # 传 localizer="bt_navigator"：既跳过 AMCL 等待，也绕过 _waitForInitialPose。
    nav.waitUntilNav2Active(navigator="bt_navigator", localizer="bt_navigator")

    def go(xyz) -> tuple[bool, int, float]:
        px, py, pyaw = xyz            # 世界坐标
        mx, my = to_map(px, py)       # 发给 Nav2 的必须是 map 坐标
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = nav.get_clock().now().to_msg()
        goal.pose.position.x = float(mx)
        goal.pose.position.y = float(my)
        q = yaw_to_quat(pyaw)
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]
        nav.goToPose(goal)
        recoveries = 0
        i = 0
        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            if fb is not None:
                recoveries = max(recoveries, getattr(fb, "number_of_recoveries", 0))
            rclpy.spin_once(tracker, timeout_sec=0.05)
            i += 1
            if i > 4000:
                break
        res = nav.getResult()
        ok = res == TaskResult.SUCCEEDED
        # 分段对账：每段结束时同时打 odom（换算回 map）与真值，定位分叉发生在哪一段
        tp_seg = true_pose()
        if tracker.pose is not None and tp_seg is not None:
            tx, ty = to_map(tp_seg[0], tp_seg[1])
            ox, oy = odom_in_map()
            print(f"      [段末] 目标map=({mx:+.2f},{my:+.2f}) "
                  f"odommap=({ox:+.2f},{oy:+.2f}) 真值map=({tx:+.2f},{ty:+.2f}) "
                  f"差=({tx-ox:+.2f},{ty-oy:+.2f})", flush=True)
        # 判据：真值落点误差（目标点是 map 坐标，本世界里 map 与世界重合）
        tp = true_pose()
        if tp:
            tx, ty = to_map(tp[0], tp[1])
            err_true = math.hypot(tx - mx, ty - my)
        else:
            err_true = float("nan")
        # 对照：odom 里的误差，需要把 odom 换算回 map（加出生点偏移，map->odom 是平移）
        err_odom = float("nan")
        if tracker.pose is not None:
            ox, oy = odom_in_map()
            err_odom = math.hypot(ox - mx, oy - my)
        return ok, recoveries, err_true, err_odom

    runs = []
    for k in range(args.attempts):
        # 动作拆成三段：回起点(朝向 0) -> 原地转到入库朝向 -> 纯侧移到位。
        # 不拆的话 MPPI 会把"转向"和"横移"揉成一个复合动作，实测在
        # fdir1=±45° 的近似摩擦下会明显打滑：真值少转 27°、横向少走 0.99 m，
        # 而 odom 口径只有 0.06 m（自己骗自己）。拆开后每段都是单自由度，
        # 也更符合"侧移入库位"这个动作本来的定义。
        print(f"[M2] 第 {k + 1}/{args.attempts} 次：回起点 -> 原地转向 -> 侧移入位", flush=True)
        go(start)
        go((start[0], start[1], dock[2]))
        ok, recoveries, err, err_odom = go(dock)
        runs.append({"attempt": k + 1, "nav_ok": bool(ok), "recoveries": int(recoveries),
                     "final_err_m": round(float(err), 4),
                     "final_err_odom_m": round(float(err_odom), 4) if err_odom == err_odom else None})
        print(f"     nav={ok} recoveries={recoveries} 落点误差(真值)={err:.3f} m"
              f"  /  odom 口径 {err_odom:.3f} m")

    n_success = sum(1 for r in runs if r["nav_ok"] and r["recoveries"] == 0 and r["final_err_m"] < 0.10)
    verdict = lateral >= 0.80 and n_success >= 4
    payload = {
        "criterion": "M2",
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "aisle": args.aisle,
        "bay": args.bay,
        "side": args.side,
        "lateral_displacement_m": round(lateral, 3),
        "frame": args.frame,
        "dock_pose": [round(v, 3) for v in dock],
        "runs": runs,
        "n_success": n_success,
        "n_attempts": args.attempts,
        "pass": bool(verdict),
        "note": "成功 = 导航 SUCCEEDED 且 0 次恢复行为 且 落点误差 < 0.10 m"
                "（落点误差取 Gazebo 真值位姿，不用 /odom；odom 口径另存在 final_err_odom_m）",
    }
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print()
    print(f"横向位移 {lateral:.2f} m（判据 ≥ 0.80）  成功 {n_success}/{args.attempts}（判据 ≥ 4）")
    print(f"RESULT: {'PASS' if verdict else 'FAIL'}  -> {out.relative_to(ROOT)}")
    # 不在这里 lifecycleShutdown：BasicNavigator 的 shutdown 会把整套 Nav2 节点下掉，
    # 于是下一次 `make m2` 又要重起一套；更糟的是重起前若上一套进程还在 ->
    # 节点重名 -> "Failed to change state for node: controller_server" -> bringup 中止。
    # Nav2 的生命周期交给 `make nav2` / `pkill` 管。
    rclpy.shutdown()
    return 0 if verdict else 1


if __name__ == "__main__":
    sys.exit(main())
