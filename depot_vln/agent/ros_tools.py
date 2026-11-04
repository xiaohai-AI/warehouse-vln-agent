"""P6-2 真工具层：把 agent 的 act / verify 接到仿真上（`resolve` / `plan` 复用地图世界）。

与 `OfflineToolLayer`（离线实现）的差别只有三处，且都在这条边界上：
  * `navigate()` —— **真的开**：`drive_to_point` 闭环到 `plan.obs_pose`，需要时按
    `plan.entry` 做侧移贴靠（全向底盘的优势，不掉头）。
  * `observe()`  —— **真的看**：抓一帧 `/stereo_left` → 送 VLM（qwen3-vl）判占用/类别。
  * `dry=True`   —— 只走 resolve/plan，不动机器人（离线自检、CI 用）。

**口径（必须写进报告，否则会被当成作弊）**
  * 语义层（resolve / plan / verify / recover）**只读 `world/slot_grid.json`（地图）与
    `depot_vln/semantic_graph.py`（P5 产物）+ 现场视觉观测**，从不读 `ground_truth/`。
  * 只有**运动闭环**用真值（`TruthPoller`）做反馈 —— 与 SEG-1 用真值位姿累积点云、路线脚本
    用真值闭环压横向是同一个口径：真值在这里扮演"底层控制器"，不参与任何语义判断。
  * 因此本层**不提供**"用真值判断目标对不对"的任何接口；打分只在 `scripts/p6_demo.py` 里
    用 `score_target()` 做，且结果只打印、不回传给图。
"""
from __future__ import annotations

import base64
import json
import math
import os
import sys
import time
import urllib.request
from pathlib import Path
from typing import Any

from .tools import Observation, ToolResult

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))

# ---- 配置（config/nav.yaml 是唯一来源；这里只做读取与默认值兜底）
_NAV_DEFAULTS = {
    "nominal_speed": 0.35,
    "dock_time": 6.0,
    "tol": 0.12,
    "wall_budget_margin": 1.8,
    "lane_offsets": {},
}


def _nav_cfg() -> dict[str, Any]:
    import yaml

    p = ROOT / "config" / "nav.yaml"
    cfg = dict(_NAV_DEFAULTS)
    if p.exists():
        cfg.update(yaml.safe_load(p.read_text(encoding="utf-8")) or {})
    return cfg


_NAV = _nav_cfg()
NOMINAL_SPEED = float(_NAV["nominal_speed"])
DOCK_TIME = float(_NAV["dock_time"])
LANE_DY: dict[str, float] = {str(k).upper(): float(v) for k, v in (_NAV.get("lane_offsets") or {}).items()}

# world ← odom 的锚定偏移：**从 catalog 派生**（原写死 (-31.0, 0.0)，改世界就错位）
def _world_origin() -> tuple[float, float]:
    from depot_vln.catalog import get_catalog

    s = get_catalog().spawn
    return float(s["x"]), float(s["y"])


WORLD_ORIGIN = _world_origin()


class _OdomPoller:
    """订阅 `/odom` 取**估计**位姿（轮速里程计，odom 系）。与 `TruthPoller` 分开：

    真值给运动闭环与打分用；估计位姿只在 `POSE=odom` 时给 verify 的投影用。
    这样"投影用的位姿"是谁一目了然，不会混着用还不自知。
    """

    def __init__(self, topic: str = "/odom") -> None:
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.node import Node

        self._latest: tuple[float, float, float, float] | None = None
        if not rclpy.ok():
            rclpy.init()
        self.node: Node = rclpy.create_node("p6_odom_pose")
        self.node.create_subscription(Odometry, topic, self._cb, 10)
        # **不在这里起 spin 线程**：`TruthPoller` 已经有一个后台 spin，`grab_pair` 又在主线程
        # spin，三个 spin 挤同一个 rclpy context → `IndexError: wait set index too big`，
        # 退出时还会 `terminate called without an active exception`（SIGABRT / 核心转储）。
        # 改成在 `get()` 里**同步** spin_once：调用点是串行的（取位姿 → 抓图），不需要并发。

    def _cb(self, msg) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._latest = (p.x, p.y, yaw, time.monotonic())

    def get(self, timeout: float = 5.0) -> tuple[float, float, float] | None:
        import rclpy

        t0 = time.monotonic()
        while self._latest is None and time.monotonic() - t0 < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if self._latest is None:
            return None
        return self._latest[0], self._latest[1], self._latest[2]


class SimToolLayer:
    """真工具层。`world` 只需提供 resolve/plan/observe（`depot_vln/world.py:WorldModel`）。"""

    def __init__(
        self,
        world,
        *,
        dry: bool = False,
        vlm_model: str = "qwen3-vl:8b",
        vlm_host: str = "http://127.0.0.1:11434",
        image_topic: str = "/stereo_left",
        speed: float = NOMINAL_SPEED,
        tol: float = 0.12,
        rtf: float = 0.44,
        poller=None,
        lane_dy: dict[str, float] | None = None,
        verify: str = "vlm",
        pose_source: str = "truth",
        act_backend: str = "drive_to",
        save_dir: str | Path | None = None,
    ) -> None:
        self.world = world
        self.dry = dry
        self.vlm_model = vlm_model
        self.vlm_host = vlm_host.rstrip("/")
        self.image_topic = image_topic
        self.speed = speed
        self.tol = tol
        self.rtf = rtf
        self.lane_dy = LANE_DY if lane_dy is None else dict(lane_dy)
        self.verify = verify      # "vlm"（看图）或 "depth"（双目深度门限，见 verify_depth.py）
        # 投影用哪一路位姿："truth"=真值（运动层口径）；"odom"=轮速里程计 + WORLD_ORIGIN
        # （部署口径：机器人只有估计位姿）。两者的差就是"位姿误差把 verify 带偏多少"的量。
        self.pose_source = pose_source
        # act 的后端："drive_to"（图航点 + 点对点控制，D23-1）或 "nav2"（**单目标交给 Nav2**：
        # 静态图里有货架、局部 costmap 有 /scan → 能绕开动态障碍，绕不开才会停/超时）。
        self.act_backend = act_backend
        self._odom = None
        self.save_dir = Path(save_dir) if save_dir else ROOT / "results" / "p6_frames"
        self._poller = poller
        self._current: dict[str, Any] | None = None
        self._done: dict[int, ToolResult] = {}

    # ------------------------------------------------------------ 惰性依赖
    @property
    def poller(self):
        if self._poller is None:
            from slam_route import TruthPoller  # scripts/slam_route.py

            self._poller = TruthPoller()
        return self._poller

    def _pose(self, wait: float = 20.0) -> dict[str, float] | None:
        """取真值位姿。**第一次调用要等**：`TruthPoller` 的后台线程还没跑起来时
        `get()` 立刻返回 None —— 原来没等，于是 `here` 退化成目标点、路线变成 1 段、
        机器人在原地发命令（实测：`转向超时` + 侧移 60 s 超时）。"""
        t0 = time.monotonic()
        while True:
            p = self.poller.get(5.0)
            if p is not None:
                return {"x": round(p[0], 3), "y": round(p[1], 3), "yaw": round(p[2], 4)}
            if time.monotonic() - t0 >= wait:
                return None
            time.sleep(0.2)

    # ------------------------------------------------------------ resolve / plan
    def resolve(self, query: dict[str, Any]) -> tuple[list[dict[str, Any]], str | None]:
        # 只查地图 + 语义图；`slot_grid` 里没有 ground_truth 的占用/类别，所以这里不可能泄真值
        return self.world.resolve(query)

    def plan(self, target: dict[str, Any]) -> dict[str, Any] | None:
        return self.world.plan(target)

    # ------------------------------------------------------------ act（真开）
    def navigate(self, target: dict[str, Any], plan: dict[str, Any] | None, seq: int) -> ToolResult:
        t0 = time.monotonic()
        plan = plan or self.plan(target)
        if plan is None:
            return self._finish(seq, ToolResult(
                ok=False, reason_code="UNREACHABLE", detail="图上没有到该目标的路径",
                elapsed=0.0, seq=seq, state="FAILED"))
        obs = plan.get("obs_pose") or {}
        gx, gy = float(obs.get("x", 0.0)), float(obs.get("y", 0.0))
        gyaw = float(obs.get("yaw", 0.0))

        # **必须逐航段走**：drive_to_point 是点对点控制器，直线直奔会横穿货架排
        # （实测 51 m 直线 363 s 超时，顶在货架上）。航点来自语义图（巷道中线/站台），
        # 走法 = 沿巷道走，与路线脚本同构。
        node = plan.get("node")
        graph = getattr(self.world, "graph", None)
        if self.dry:   # dry：用地图自己记的位姿（真值轮询需要 ROS），才能算出真实航段
            here = {"x": float(self.world.pose.get("x", 0.0)),
                    "y": float(self.world.pose.get("y", 0.0))}
        else:
            here_p = self._pose()
            if here_p is None:
                return self._finish(seq, ToolResult(
                    ok=False, reason_code="MOTION_NO_POSE", seq=seq,
                    detail="拿不到真值位姿（/world/depot/pose/info 没数据？仿真没起？）",
                    elapsed=time.monotonic() - t0, state="FAILED"))
            here = here_p
        route: list[str] = []
        legs: list[tuple[str, tuple[float, float]]] = []
        lane_legs: list[tuple[str, tuple[float, float], float]] = []
        if graph is not None and node:
            route = graph.route((here["x"], here["y"]), node)
            legs = [(n, graph.pos[n]) for n in route]
        _pts = [(here["x"], here["y"])] + [xy for _n, xy in legs]
        total_leg_m = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                          for a, b in zip(_pts, _pts[1:]))
        print(f"[act] seq={seq} → {node} 观测位 ({gx:+.2f},{gy:+.2f}) yaw {gyaw:+.2f} | "
              f"航段 {len(legs)} 段 / {plan['path_length']:.1f} m / RTF≈{self.rtf:.2f} / "
              f"预计墙钟 {total_leg_m / max(0.05, self.speed * self.rtf):.0f}s")

        # 中间航段套用车道偏移；最后一段（进观测位）不套，保证对着货架的距离正确
        for k, (nname, xy) in enumerate(legs):
            dy = 0.0
            if k < len(legs) - 1:
                aisle = nname.split("@")[0].split(":")[-1]
                dy = self.lane_dy.get(aisle, 0.0)
            lane_legs.append((nname, (xy[0], xy[1] + dy), dy))

        if self.dry:
            for i, (nname, xy, dy) in enumerate(lane_legs, 1):   # dry 也要把航段打全
                print(f"[act]   航段 {i}/{len(lane_legs)} → {nname} ({xy[0]:+.2f},{xy[1]:+.2f})"
                      + (f"（车道偏移 {dy:+.2f}）" if dy else ""))
            return self._finish(seq, ToolResult(
                ok=True, reason_code=None, pose=self._map_pose(plan), seq=seq,
                detail=f"[dry] 计划到 {plan['node']}（{plan['path_length']:.1f} m / "
                       f"{len(legs)} 航段，进入动作 {plan['entry']['type']}），未动机器人",
                elapsed=0.0))

        # ---- Nav2 后端：**一个目标**交给规划器（它自己会在静态图上规划、用 /scan 躲障碍）。
        # 为什么不像 drive_to 那样逐航段：逐航段是把"路径规划"塞进航点里；Nav2 的强项正是
        # 规划 + 局部避障，喂一个目标反而能绕开临时障碍（比如横穿的推车）。
        if self.act_backend == "nav2":
            dist = math.hypot(gx - here["x"], gy - here["y"])
            budget = self._wall_budget(dist) * 1.5
            print(f"[act]   Nav2 单目标 → ({gx:+.2f},{gy:+.2f}) 目标朝向 {gyaw:+.2f}，"
                  f"直线 {dist:.1f} m，预算 {budget:.0f}s")
            ok, tail = self._nav2_goto(gx, gy, gyaw, budget)
            p = self._pose()
            if not ok:
                return self._finish(seq, ToolResult(
                    ok=False, reason_code="MOTION_NAV2_FAILED", pose=p, seq=seq,
                    detail=f"Nav2 没能在 {budget:.0f}s 内到达观测位；末尾输出：{tail}",
                    elapsed=time.monotonic() - t0, state="FAILED"))
            print("[act]   Nav2 报告到达；用 drive_to 精修朝向（Nav2 的朝向容差较宽）")
            from drive_to import turn_to_precise as _turn      # scripts/drive_to.py
            try:
                _turn(self.poller, gyaw, 3.0)
            except Exception as e:
                print(f"[act] 摆正车头失败（继续）：{e}")
            here = self._pose() or here
            err = round(math.hypot(here["x"] - gx, here["y"] - gy), 3) if here else None
            return self._finish(seq, ToolResult(
                ok=True, reason_code=None, pose=here, seq=seq,
                detail=f"Nav2 单目标导航到观测位（图上路径 {plan['path_length']:.1f} m），"
                       f"末段误差 {err} m",
                elapsed=time.monotonic() - t0))

        from drive_to import drive_to_point, turn_to_precise      # scripts/drive_to.py

        for i, (nname, xy, dy) in enumerate(lane_legs, 1):
            d = math.hypot(xy[0] - here["x"], xy[1] - here["y"])
            if d < 0.25:                        # 已经在这个站台上
                continue
            leg_budget = self._wall_budget(d)
            print(f"[act]   航段 {i}/{len(legs)} → {nname} ({xy[0]:+.2f},{xy[1]:+.2f}) "
                  f"{d:.1f} m，预算 {leg_budget:.0f}s"
                  + (f"（车道偏移 {dy:+.2f}）" if dy else ""))
            if not drive_to_point(self.poller, xy[0], xy[1], self.speed, 0.20, leg_budget):
                p = self._pose()
                return self._finish(seq, ToolResult(
                    ok=False, reason_code="TIMEOUT", pose=p, seq=seq,
                    detail=f"航段 {i}/{len(legs)}（{nname}）走不过去，卡在 ({p['x']}, {p['y']})"
                           if p else f"航段 {i}（{nname}）走不过去",
                    elapsed=time.monotonic() - t0, state="FAILED"))
            here = self._pose() or here

        # 终点：精确落到观测位（站台坐标通常就等于 obs_pose，这一步是兜底）
        if math.hypot(gx - here["x"], gy - here["y"]) > self.tol:
            if not drive_to_point(self.poller, gx, gy, self.speed, self.tol,
                                  self._wall_budget(math.hypot(gx - here["x"], gy - here["y"]))):
                p = self._pose()
                return self._finish(seq, ToolResult(
                    ok=False, reason_code="TIMEOUT", pose=p, seq=seq,
                    detail=f"最后一段到观测位失败，卡在 {p}", elapsed=time.monotonic() - t0,
                    state="FAILED"))

        # 摆正车头（对着货架面），否则 verify 的视野会歪
        try:
            turn_to_precise(self.poller, gyaw, 3.0)
        except Exception as e:
            print(f"[act] 摆正车头失败（继续）：{e}")

        # **不做侧移贴靠**：`entry.lateral = |slot.y − obs.y|` 是"巷道中线到货位中心"的
        # 距离（A 巷实测 1.88 m），货架面就在中线外 0.6 m —— 硬侧移是撞货架（实测 60 s 超时）。
        # obs_pose 本身就是部署位：站台 + 朝货架 + 1.9 m 视距，320×240/90° 视野足够看一格。
        entry = plan.get("entry") or {}
        print(f"[act] 已在观测位，朝货架（进入动作 {entry.get('type')} / "
              f"货位离心线 {float(entry.get('lateral') or 0):.2f} m，不贴靠）")

        pose = self._pose()
        return self._finish(seq, ToolResult(
            ok=True, reason_code=None, pose=pose, seq=seq,
            detail=f"到达 {plan['node']}（进入动作 {entry.get('type')}）",
            elapsed=time.monotonic() - t0))

    def _wall_budget(self, dist_m: float) -> float:
        """把"仿真距离"换算成**墙钟**预算：RTF≈0.44 时 1 m 要 6.5 墙钟秒（0.35 m/s）。"""
        return dist_m / max(0.05, self.speed * self.rtf) * 1.8 + 20.0

    def _map_pose(self, plan: dict[str, Any]) -> dict[str, float]:
        obs = plan.get("obs_pose") or {}
        return {"x": round(float(obs.get("x", 0.0)), 3),
                "y": round(float(obs.get("y", 0.0)), 3),
                "yaw": round(float(obs.get("yaw", 0.0)), 4)}

    def status(self, seq: int) -> ToolResult:
        return self._done.get(seq) or ToolResult(
            ok=False, reason_code="NO_SUCH_SEQ", seq=seq, state="FAILED",
            detail=f"没有 seq={seq} 的记录")

    def cancel(self, seq: int) -> ToolResult:
        try:
            from slam_route import cmd

            cmd(0.0, 0.0, 0.0)
        except Exception:
            pass
        return self.status(seq)

    def _finish(self, seq: int, res: ToolResult) -> ToolResult:
        self._done[seq] = res
        self._current = None
        return res

    # ------------------------------------------------------------ verify（真看）
    def observe(self, target: dict[str, Any]) -> Observation:
        t0 = time.monotonic()
        if self.dry:
            return Observation(ok=False, reason_code="OBSERVE_DRY",
                               detail="[dry] 未抓帧",
                               elapsed=time.monotonic() - t0)
        addr = target.get("address") or ""
        if self.verify == "depth":
            return self._observe_depth(target, addr, t0)
        png = self._capture_frame(f"obs_{addr or 'x'}".replace("-", "_"))
        if png is None:
            return Observation(ok=False, reason_code="OBSERVE_NO_FRAME",
                               detail=f"{self.image_topic} 抓不到帧",
                               elapsed=time.monotonic() - t0)
        verdict = self._ask_vlm(png, addr)
        if verdict is None:
            return Observation(ok=False, reason_code="OBSERVE_VLM_FAILED",
                               detail=f"VLM 未给出可解析结果（{png.name}）",
                               elapsed=time.monotonic() - t0)
        return Observation(
            ok=True, reason_code=None,
            occupied=verdict.get("occupied"),
            cls=verdict.get("cls"),
            confidence=float(verdict.get("confidence") or 0.0),
            detail=f"{addr}: occupied={verdict.get('occupied')} cls={verdict.get('cls')} "
                   f"（{png.name}，{verdict.get('detail', '')}）",
            elapsed=time.monotonic() - t0,
        )

    def _nav2_goto(self, x: float, y: float, yaw: float,
                   timeout_s: float) -> tuple[bool, str]:
        """给 Nav2 发一个 NavigateToPose 目标，等结果。

        **走 action CLI 而不是 nav2_simple_commander**：本进程里已经有 TruthPoller 的后台
        spin，再塞一个 BasicNavigator 节点就是"三处 spin 挤一个 context"（踩过，会
        `IndexError: wait set index too big` 并在退出时 SIGABRT）。CLI 是独立进程，干净。
        """
        import subprocess

        goal = ("{pose: {header: {frame_id: 'map'}, pose: {position: {x: %.3f, y: %.3f, z: 0.0}, "
                "orientation: {x: 0.0, y: 0.0, z: %.6f, w: %.6f}}}}"
                % (x, y, math.sin(yaw / 2.0), math.cos(yaw / 2.0)))
        argv = ["ros2", "action", "send_goal", "/navigate_to_pose",
                "nav2_msgs/action/NavigateToPose", goal]
        # 被拒收就重试：Nav2 起栈时 lifecycle 逐个激活，早发会被 `Goal was rejected`；
        # 这种失败**必须重试**而不是直接判失败（实测 demo 因此 0.5 s 就报 MOTION_NAV2_FAILED）。
        out = ""
        for attempt in range(1, 4):
            try:
                p = subprocess.run(argv, capture_output=True, text=True,
                                   timeout=timeout_s + 30)
            except subprocess.TimeoutExpired:
                return False, f"ros2 action send_goal 超过 {timeout_s + 30:.0f}s 没结果"
            out = ((p.stdout or "") + (p.stderr or "")).strip()
            if "SUCCEEDED" in out or "ABORTED" in out or "CANCELED" in out:
                return ("SUCCEEDED" in out), out[-500:]
            if "rejected" not in out.lower():
                return False, out[-500:]
            print(f"[act]   Nav2 拒收目标（第 {attempt} 次），5 s 后重试")
            time.sleep(5.0)
        return False, out[-500:]

    def _est_pose(self, wait: float = 20.0) -> dict[str, float] | None:
        """估计位姿 = `/odom`（轮速里程计）+ `WORLD_ORIGIN`（等价 static `map→odom`）。

        `POSE=odom` 时 verify 的投影用这一路，用来回答：**位姿不准时 verify 还准不准**。
        """
        if self._odom is None:
            self._odom = _OdomPoller()
        t0 = time.monotonic()
        while True:
            p = self._odom.get(5.0)
            if p is not None:
                return {"x": round(p[0] + WORLD_ORIGIN[0], 3),
                        "y": round(p[1] + WORLD_ORIGIN[1], 3),
                        "yaw": round(p[2], 4)}
            if time.monotonic() - t0 >= wait:
                return None
            time.sleep(0.2)

    def _observe_depth(self, target: dict[str, Any], addr: str, t0: float) -> Observation:
        """双目深度门限判决（不调 VLM）：只统计目标格开口四边形内、深度在格子内部带里的像素。

        为什么不用"画面中央"：货架通透，空格背后 2.2 m 的货会投到同一像素
        （`docs/TROUBLESHOOTING.md §33.3`）。
        """
        from depot_vln.verify_depth import (F_PX as F_PX_REF, cam_world,
                                            depth_gate_verdict, slot_opening)
        from verify_probe import grab_pair, stereo_depth   # scripts/verify_probe.py

        # 投影用哪一路位姿（`POSE=truth|odom`）；另一路只用于**度量**误差，不参与投影。
        if self.pose_source == "odom":
            pose = self._est_pose()
            truth = self._pose()          # 仅记录误差
            if pose is None:
                return Observation(ok=False, reason_code="OBSERVE_NO_ODOM",
                                   detail="取不到 /odom 位姿（估计位姿），无法投影目标格",
                                   elapsed=time.monotonic() - t0)
        else:
            pose, truth = self._pose(), None
        if pose is None:
            return Observation(ok=False, reason_code="OBSERVE_NO_POSE",
                               detail="取不到车体位姿，无法投影目标格",
                               elapsed=time.monotonic() - t0)
        pitch = float(os.environ.get("CAM_PITCH_DEG", "0") or 0)
        if pitch:
            print(f"[observe] 相机俯仰={pitch:+.1f}°（模型按 0 投影，等效竖直偏 {F_PX_REF * math.tan(math.radians(pitch)):+.1f} px）")
        pose_err = None
        if truth is not None:
            dx, dy = pose["x"] - truth["x"], pose["y"] - truth["y"]
            dyaw = math.degrees(pose["yaw"] - truth["yaw"])
            dyaw = (dyaw + 180.0) % 360.0 - 180.0
            pose_err = {"dx": round(dx, 3), "dy": round(dy, 3),
                        "d_m": round(math.hypot(dx, dy), 3), "dyaw_deg": round(dyaw, 2),
                        "source": self.pose_source}
            print(f"[observe] 投影位姿={self.pose_source} ({pose['x']:+.2f},{pose['y']:+.2f}) "
                  f"yaw {pose['yaw']:+.3f} | 真值 ({truth['x']:+.2f},{truth['y']:+.2f}) "
                  f"yaw {truth['yaw']:+.3f} → 误差 {pose_err['d_m']:.2f} m / "
                  f"{pose_err['dyaw_deg']:+.1f}°")
        slot = (self.world.grid.get("slots") or {}).get(addr)
        if slot is None:
            return Observation(ok=False, reason_code="OBSERVE_NO_SLOT",
                               detail=f"地图里没有 {addr}", elapsed=time.monotonic() - t0)
        cam = cam_world(pose, "L" if slot.get("side") == "L" else "R")
        op = slot_opening(slot, cam)
        if op is None:
            return Observation(ok=False, reason_code="OBSERVE_OUT_OF_VIEW",
                               detail="目标格不在相机前方", elapsed=time.monotonic() - t0)
        left, right, _ = grab_pair("/stereo_left", "/stereo_right")
        if left is None:
            return Observation(ok=False, reason_code="OBSERVE_NO_FRAME",
                               detail="/stereo_left 或 /stereo_right 抓不到帧",
                               elapsed=time.monotonic() - t0)
        depth = stereo_depth(left, right)
        res = depth_gate_verdict(depth, op)
        # **落盘存档**：320×240 的 left/right/depth + 投影参数。这样阈值标定可以在离线做，
        # 不必为了看一个数字再开一次仿真（每轮 3–6 分钟，全是人的时间）。
        try:
            import numpy as np

            dump_dir = ROOT / "results" / "depth_dumps"
            dump_dir.mkdir(parents=True, exist_ok=True)
            tag = f"{addr}_{time.strftime('%Y%m%dT%H%M%S')}"
            np.savez_compressed(
                dump_dir / f"{tag}.npz",
                left=left, right=right, depth=depth,
                quad=np.array(op["quad"], dtype=np.float32),
                meta=np.array([op["d_front"], op["d_back"], pose["x"], pose["y"], pose["yaw"]],
                              dtype=np.float64),
            )
            (dump_dir / f"{tag}.json").write_text(
                json.dumps({"addr": addr, "slot": slot,
                            "pose": pose,                 # 投影实际用到的位姿
                            "pose_source": self.pose_source,
                            "pose_truth": truth,          # 仅度量用
                            "pose_err": pose_err,
                            # 实验口径：模型恒假设 pitch=0，这里记的是**实体相机的实际俯仰**
                            # （`CAM_PITCH` 传进来的），用于事后核对"俯仰误差 → 判决"的关系
                            "cam_pitch_deg": float(os.environ.get("CAM_PITCH_DEG", "0") or 0),
                            "opening": {k: op[k] for k in ("d_front", "d_back", "side")},
                            "verdict": res}, ensure_ascii=False, indent=2, default=float),
                encoding="utf-8")
            print(f"[observe] 深度存档 results/depth_dumps/{tag}.npz")
        except Exception as e:            # 存档失败不能影响演示
            print(f"[observe] 深度存档失败（不影响判决）：{e}")
        verdict = res.get("verdict")
        if verdict == "unseen":
            return Observation(ok=False, reason_code="VERIFY_UNSEEN",
                               detail=f"{addr}: {res.get('why')}",
                               elapsed=time.monotonic() - t0)
        return Observation(
            ok=True, reason_code=None, occupied=(verdict == "occupied"), cls=None,
            confidence=0.0,          # pydantic 字段是 float，不能给 None
            detail=(f"{addr}: {verdict}（{res.get('why')}；窗口 {res.get('win_w_m')} m 宽"
                    + (f"；投影位姿 {pose_err['source']} 误差 {pose_err['d_m']:.2f} m / "
                       f"{pose_err['dyaw_deg']:+.1f}°" if pose_err else "")
                    + "）"),
            elapsed=time.monotonic() - t0)

    # ------------------------------------------------------------ 抓帧
    def _capture_frame(self, tag: str, timeout: float = 15.0):
        """订阅一帧图像并存 PNG。**惰性 import rclpy**，所以 dry 模式不需要 ROS。"""
        try:
            import cv2
            import numpy as np
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import Image
        except Exception as e:
            print(f"[observe] 无法 import ROS/cv2：{e}")
            return None

        holder: dict[str, Any] = {}
        topic = self.image_topic

        class _Grab(Node):
            def __init__(self) -> None:
                super().__init__("p6_grab")
                qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
                self.create_subscription(Image, topic, self._cb, qos)

            def _cb(self, m) -> None:
                holder["m"] = m

        own_init = False
        if not rclpy.ok():
            rclpy.init()
            own_init = True
        node = _Grab()
        try:
            t0 = time.monotonic()
            while time.monotonic() - t0 < timeout and "m" not in holder:
                rclpy.spin_once(node, timeout_sec=0.1)
            m = holder.get("m")
            if m is None:
                return None
            h, w = int(m.height), int(m.width)
            buf = bytes(m.data)
            enc = (m.encoding or "").lower()
            if enc in ("rgb8", "bgr8"):
                img = np.frombuffer(buf, dtype="uint8").reshape(h, w, 3)
                if enc == "rgb8":
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            else:                          # mono 或未知编码：只存灰度
                img = np.frombuffer(buf, dtype="uint8").reshape(h, w)
            self.save_dir.mkdir(parents=True, exist_ok=True)
            out = self.save_dir / f"{time.strftime('%H%M%S')}_{tag}.png"
            cv2.imwrite(str(out), img)
            print(f"[observe] 抓帧 {w}x{h} {m.encoding} → {out.relative_to(ROOT)}")
            return out
        finally:
            node.destroy_node()
            _ = own_init   # 交给进程退出统一 shutdown（重复 shutdown 会抛）

    # ------------------------------------------------------------ VLM
    def _ask_vlm(self, png: Path, addr: str) -> dict[str, Any] | None:
        """问 qwen3-vl：该货位有没有货、是哪一类。返回 {occupied, cls, confidence, detail}。"""
        try:
            b64 = base64.b64encode(png.read_bytes()).decode()
        except Exception as e:
            print(f"[observe] 读图失败：{e}")
            return None
        prompt = (
            "这是仓储机器人在货架前拍的一帧。请只输出 JSON，不要解释：\n"
            '{"occupied": true|false, "cls": "carton_s|carton_m|carton_l|tote|pallet|drum|null",'
            ' "confidence": 0.0-1.0}\n'
            f"要判断的货位是 {addr}（画面中央附近、离相机最近的那一格）。\n"
            "画面中央那一格里有货就 occupied=true 并给出类别；空着就 false、cls=null。"
            "看不清就把 confidence 写低（<0.3），**不要猜**。"
        )
        # `think: False` 必须加：qwen3-vl 是思考型模型，不加会把输出全塞进 `thinking`，
        # `response` 拿到空串（实测就是 `VLM 输出无法解析：\'\'`）。
        body = json.dumps({"model": self.vlm_model, "prompt": prompt,
                           "images": [b64], "stream": False, "think": False,
                           "options": {"temperature": 0}}).encode()
        req = urllib.request.Request(
            f"{self.vlm_host}/api/generate", data=body,
            headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=180) as r:
                raw = json.loads(r.read().decode())
        except Exception as e:
            print(f"[observe] VLM 调用失败（{self.vlm_model}@ollama）：{e}")
            return None
        text = str(raw.get("response") or "")
        obj = _first_json(text)
        if obj is None and raw.get("thinking"):      # 兜底：某些版本把 JSON 留在 thinking 里
            text = str(raw.get("thinking"))
            obj = _first_json(text)
        if obj is None:
            print(f"[observe] VLM 输出无法解析：{text[:200]!r}")
            return None
        cls = obj.get("cls")
        if isinstance(cls, str):
            cls = cls.strip().lower()
            if cls in ("null", "none", ""):
                cls = None
        return {
            "occupied": bool(obj.get("occupied")) if obj.get("occupied") is not None else None,
            "cls": cls,
            "confidence": float(obj.get("confidence") or 0.0),
            "detail": f"{self.vlm_model}",
        }


def _first_json(text: str) -> dict[str, Any] | None:
    import re

    m = re.search(r"\{.*\}", text, re.S)
    if not m:
        return None
    try:
        obj = json.loads(m.group(0))
    except json.JSONDecodeError:
        return None
    return obj if isinstance(obj, dict) else None
