#!/usr/bin/env python3
"""SLAM-1（P4）固定路线驱动 + 轨迹记录。

为什么需要这个脚本
  SLAM-1 的判据不是"能不能建图"，而是"建出来的图/轨迹离真值差多少"，而且必须可复跑。
  所以路线固定、速度固定、每 0.2 s 同时记两份轨迹：
    * 真值：gz 的 /world/depot/pose/info（世界系，判据只认它）
    * 估计：slam_toolbox 的 TF map -> base_footprint（地图帧）
  ATE（t0 对齐后的 RMSE 与终点漂移）由 scripts/slam_metrics.py 从 CSV 算。

路线（都是"巷道中线 + 横通道换巷"，不会切到货架角；总长 ≈199 m）
  (-31, 当前y) -> (-31,-10.75) -> 沿 A 到 x=+27.5 -> 东横通道 -> B（向西）
  -> 西横通道 -> C（向东）到 x=+27.5

用法
  .venv/bin/python scripts/slam_route.py                     # 默认 0.30 m/s，边跑边存 CSV
  .venv/bin/python scripts/slam_route.py --speed 0.25 --out results/slam1_traj.csv
驱动只用 gz 话题（不经 ROS），所以即使 ROS 侧没起也能走；但 TF 估计需要 slam 在跑。
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import re
import subprocess
import sys
import threading
import time

# 必须在 import depot_vln **之前**加仓库根：直接 `python scripts/slam_route.py`
# 时 sys.path[0] 是 scripts/，否则 ModuleNotFoundError: No module named 'depot_vln'。
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener
from depot_vln.catalog import get_catalog

W, WORLD = "depot_bot", "depot"
AISLE_Y = {"A": -10.75, "B": -6.45, "C": -2.15, "D": 2.15, "E": 6.45, "F": 10.75}
# 实际行驶用的 y：C 巷 bay 5 停着一台静态叉车（占 y∈[−3.00,−2.40]），沿中线 −2.15 走
# 机身与叉车只剩 2 cm 余量，会擦到。C 巷改走上半幅 y=−1.60（离叉车 0.8 m、离北货架 1.05 m）。
DRIVE_Y = {"A": -10.75, "B": -6.45, "C": -1.60}
# 东/西横通道的中心线（rack_run ±28.8，墙 ±33.4）
X_EAST, X_WEST = 31.1, -31.1


def _run(cmd: list[str], timeout: float = 8.0) -> str:
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout).stdout
    except subprocess.TimeoutExpired:
        return ""


def sim_now() -> float | None:
    txt = _run(["ign", "topic", "-e", "-t", "/clock", "-n", "1"])
    m = re.search(r"^sim \{\s*sec: (\d+)\s*nsec: (\d+)", txt, re.S | re.M)
    return None if not m else int(m.group(1)) + int(m.group(2)) * 1e-9


def model_pose(name: str = W) -> tuple[float, float, float] | None:
    """任意模型的真值位姿（默认机器人）。**只读** /world/depot/pose/info。

    抽出来是为了让"读某个模型的位姿"只有一份实现：动态障碍脚本要读推车，
    机器人要读自己，两处各写一遍必然分叉。

    **不要**改用 /model/depot_bot/pose（PosePublisher 插件）：实测它发出来的不是世界位姿，
    而是差了约 +31 m 的另一个坐标系下的位姿（踩过：驱动脚本以为车没动，一路朝 -x 撞墙，
    238 m 的路线与地图整条作废，ATE 12.6 m）。
    需要高频就把这个函数放进后台线程轮询（见下面的 TruthPoller），不要换通道。
    """
    txt = _run(["ign", "topic", "-e", "-t", f"/world/{WORLD}/pose/info", "-n", "1"])
    for block in txt.split('name: "'):
        if not block.startswith(name + '"'):
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


def truth_pose() -> tuple[float, float, float] | None:
    """机器人的真值位姿（`model_pose(W)` 的别名，保留原调用点不变）。"""
    return model_pose(W)


class TruthPoller:
    """后台线程持续读真值（pose/info + /clock），主循环只取最新值。

    为什么：`pose/info` 一次读 0.3–1 s，同步读会把闭环压到 0.19–0.77 Hz —— 每 1.3 s
    才修正一次方向，0.35 m/s 下等于每米才纠一回航，路线会飘出中线 1 m。放后台线程后
    主循环只做"取最新值 + 发 cmd_vel"，频率由发布决定。
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._pose: tuple[float, float, float] | None = None
        self._t: float = float("nan")
        self._stamp = 0.0
        self._n = 0
        self._stop = False
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self) -> None:
        while not self._stop:
            p = truth_pose()
            t = sim_now()
            with self._lock:
                if p is not None:
                    self._pose = p
                    self._t = float("nan") if t is None else t
                    self._stamp = time.monotonic()
                    self._n += 1
            time.sleep(0.02)

    def get(self, max_age: float = 5.0) -> tuple[float, float, float] | None:
        with self._lock:
            if self._pose is None or time.monotonic() - self._stamp > max_age:
                return None
            return self._pose

    def sim_time(self) -> float:
        with self._lock:
            return self._t

    @property
    def samples(self) -> int:
        with self._lock:
            return self._n

    def stop(self) -> None:
        self._stop = True


def cmd(vx: float, vy: float, wz: float = 0.0) -> None:
    _run(["ign", "topic", "-t", f"/model/{W}/cmd_vel", "-m", "ignition.msgs.Twist",
          "-p", f"linear: {{x: {vx:.4f}, y: {vy:.4f}}}, angular: {{z: {wz:.4f}}}"], timeout=3)


class Estimator(Node):
    """slam_toolbox 的 map -> base_footprint。拿不到时返回 None（不编数据）。"""

    def __init__(self) -> None:
        super().__init__("slam_route",
                         parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        try:
            self.declare_parameter("use_sim_time", True)
        except Exception:                      # 已经自动声明过
            pass
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        # 后台持续 spin：循环里用 spin_once 只能处理零星几条消息，TF 缓冲追不上，
        # 实测 703 帧里 292 帧拿不到 map->base_footprint。放进 executor 线程后
        # 缓冲始终是最新的，lookup 变成本地计算。
        # 注意：不能用 self.executor —— rclpy 的 Node 自己就有 executor 属性，
        # 赋值会写进 Node 内部、读回来是 None（踩过：AttributeError: 'NoneType' object has no attribute 'add_node'）
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self)
        self._thread = threading.Thread(target=self._exec.spin, daemon=True)
        self._thread.start()

    def shutdown(self):
        # 顺序必须是：停 executor -> join 线程 -> destroy_node。
        # 反了（或省掉 join）会 SIGABRT：`terminate called without an active exception`
        # 实测在路线跑完之后的收尾阶段才崩，导致一次 20 分钟的跑动被误判为"未完成"。
        try:
            self._exec.shutdown(timeout_sec=1.0)
        except Exception:
            pass
        try:
            self._thread.join(timeout=2.0)
        except Exception:
            pass
        try:
            self.destroy_node()
        except Exception:
            pass

    def est_odom(self):
        """`odom -> base_footprint`（**不含** slam 修正）。

        加这一路的理由：S5 的 ATE 0.258 m 里，dx 小、dy 全程 ≈ −0.25 m、yaw 误差 <0.3°、
        单帧最大跳变 0.074 m —— 形状像"横向系统偏置"，不像漂移。只看 map->base 分不清
        这偏差是**里程计自己带的**还是**后端修正引入的**；把这两条链分开记，一次跑完就能定位。
        """
        return self._lookup("odom")

    def est(self):
        """先查 0.3 s 前的时间点，再退回最新。

        踩过：`map->odom`（slam 每 10 ms 发）与 `odom->base_footprint`（odom 50 Hz）的末端
        时间戳不一致，`lookup(Time())` 会抛 "extrapolation into the future"，实测 624 帧里
        有 311 帧因此丢估计。查稍旧的时间点，tf2 会用插值拿到一致的链。
        """
        return self._lookup("map")

    def _lookup(self, parent: str):
        now = self.get_clock().now()
        for back in (0.30, 0.60, 0.0):
            try:
                t = now - rclpy.duration.Duration(seconds=back) if back else rclpy.time.Time()
                tr = self.buf.lookup_transform(parent, "base_footprint", t).transform
                q = tr.rotation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                return tr.translation.x, tr.translation.y, yaw
            except Exception:
                continue
        return None


def build_route(start_x: float, start_y: float) -> list[tuple[float, float]]:
    """当前位姿 -> 西横通道 -> A 巷向东 -> 东横通道 -> B 巷向西 -> 西横通道 -> C 巷向东。

    关键约束：**任何一段都不许斜穿货架**。所以第一步是"保持当前 y 直线开到西横通道"
    （当前 y 一定在某条巷道或横通道里，直线走是安全的），再在横通道内换 y。
    总长约 207 m（SPEC 要求 200 m 固定路线）。
    """
    r: list[tuple[float, float]] = [(X_WEST, start_y)]             # 就地开到西横通道
    r += [(X_WEST, DRIVE_Y["A"])]                                  # 横通道内换到 A 巷高度
    r += [(27.5, DRIVE_Y["A"])]                                    # A 巷向东
    r += [(X_EAST, DRIVE_Y["A"]), (X_EAST, DRIVE_Y["B"])]          # 东横通道换巷
    r += [(27.5, DRIVE_Y["B"]), (-27.5, DRIVE_Y["B"])]             # B 巷向西
    r += [(X_WEST, DRIVE_Y["B"]), (X_WEST, DRIVE_Y["C"])]          # 西横通道换巷
    r += [(-27.5, DRIVE_Y["C"]), (27.5, DRIVE_Y["C"])]             # C 巷向东（走上半幅避叉车）
    return r


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--speed", type=float, default=0.30, help="巡航速度 m/s（打滑随速度超线性）")
    ap.add_argument("--tol", type=float, default=0.35, help="到点容差 m")
    ap.add_argument("--out", default="results/slam1_traj.csv")
    ap.add_argument("--max-seconds", type=float, default=7200.0, help="墙钟上限（安全阀）")
    args = ap.parse_args()

    poller = TruthPoller()
    p0 = None
    for _ in range(60):                     # 首次读数最多等 30 s
        p0 = poller.get(max_age=10.0)
        if p0 is not None:
            break
        time.sleep(0.5)
    if p0 is None:
        print("读不到真值位姿 —— 仿真没起？")
        return 2

    # 出发点自检：判据脚本一律从出生点 (-31,0) 开跑。若真值读到了**别的坐标系**的位姿，
    # 这里会立刻拦下（踩过：PosePublisher 发的是偏移 +31 m 的位姿，脚本以为车没动，
    # 一路朝 -x 撞墙，238 m 路线与地图整条作废）。
    spawn = get_catalog().spawn
    if math.hypot(p0[0] - spawn["x"], p0[1] - spawn["y"]) > 2.0:
        print(f"[route] 起点异常：真值 ({p0[0]:+.2f},{p0[1]:+.2f}) 不在出生点 ({spawn['x']:+.2f},{spawn['y']:+.2f}) 2 m 内")
        print("[route] 真值通道有问题，拒绝开跑（本次不产生数据）")
        return 2

    route = build_route(p0[0], p0[1])
    total = 0.0
    prev = (p0[0], p0[1])
    for x, y in route:
        total += math.hypot(x - prev[0], y - prev[1])
        prev = (x, y)
    print(f"[route] 起点 ({p0[0]:+.2f},{p0[1]:+.2f})  路点 {len(route)} 个  预计 {total:.1f} m")
    for i, (x, y) in enumerate(route):
        print(f"         {i + 1:2d}  ({x:+.1f},{y:+.1f})")

    rclpy.init()
    est_node = Estimator()
    t_start = time.monotonic()

    import os
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    f = open(args.out, "w", newline="", encoding="utf-8")
    wr = csv.writer(f)
    wr.writerow(["t_sim", "wp", "tx", "ty", "tyaw", "ex", "ey", "eyaw", "dist",
                 "ox", "oy", "oyaw"])

    wp = 0
    travelled = 0.0
    last = p0
    last_print = 0.0
    # 真值冻结保护：一直发运动命令、但真值 10 s 没变化 -> 中止（别盲开）
    last_moved = p0
    t_moved = time.monotonic()
    rc = 0
    try:
        while wp < len(route):
            if time.monotonic() - t_start > args.max_seconds:
                print("[route] 到达墙钟上限，提前结束")
                rc = 1
                break
            p = poller.get()
            if p is None:
                time.sleep(0.05)
                continue
            t = poller.sim_time()
            if t is None:
                t = float("nan")
            travelled += math.hypot(p[0] - last[0], p[1] - last[1])
            last = p

            if math.hypot(p[0] - last_moved[0], p[1] - last_moved[1]) > 0.05:
                last_moved, t_moved = p, time.monotonic()
            elif time.monotonic() - t_moved > 10.0:
                print("[route] 真值 10 s 没变化但一直在发运动命令 —— 疑似真值冻结或车被顶住，中止")
                rc = 3
                break

            e = est_node.est()
            o = est_node.est_odom()
            wr.writerow([f"{t:.3f}", wp, f"{p[0]:.4f}", f"{p[1]:.4f}", f"{p[2]:.4f}",
                         "" if e is None else f"{e[0]:.4f}",
                         "" if e is None else f"{e[1]:.4f}",
                         "" if e is None else f"{e[2]:.4f}",
                         f"{travelled:.3f}",
                         "" if o is None else f"{o[0]:.4f}",
                         "" if o is None else f"{o[1]:.4f}",
                         "" if o is None else f"{o[2]:.4f}"])

            tx, ty = route[wp]
            dx, dy = tx - p[0], ty - p[1]
            d = math.hypot(dx, dy)
            if d < args.tol:
                wp += 1
                continue

            v = min(args.speed, 0.6 * d + 0.04)
            c, s = math.cos(p[2]), math.sin(p[2])
            wx, wy = v * dx / d, v * dy / d
            cmd(c * wx + s * wy, -s * wx + c * wy, 0.0)      # 世界速度 -> 车体速度

            # TF 由后台 executor 持续接收，这里不需要再 spin
            if time.monotonic() - last_print > 10.0:
                last_print = time.monotonic()
                print(f"  wp {wp + 1}/{len(route)}  真值 ({p[0]:+7.2f},{p[1]:+7.2f})  "
                      f"走过 {travelled:6.1f} m  "
                      f"估计 {'无' if e is None else f'({e[0]:+7.2f},{e[1]:+7.2f})'}", flush=True)
            time.sleep(0.1)
    finally:
        cmd(0.0, 0.0, 0.0)
        f.close()
        poller.stop()                       # 先停后台读真值的线程，别让它跑到解释器退出
        est_node.shutdown()
        try:
            rclpy.shutdown()
        except Exception:
            pass

    print(f"[route] 完成：路点 {wp}/{len(route)}，真值累计 {travelled:.1f} m，"
          f"墙钟 {time.monotonic() - t_start:.0f} s -> {args.out}")
    if wp >= len(route):
        return 0
    return rc if rc else 1


if __name__ == "__main__":
    sys.exit(main())
