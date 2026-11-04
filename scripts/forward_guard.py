#!/usr/bin/env python3
"""前向安全层：前方扇区有障碍就**压住** /cmd_vel，清了再放行。

为什么要有这一层
  `drive_to_point()` 是点对点闭环，它**看不见障碍** —— 撞上去只会表现为"超时"。
  Nav2 的局部 costmap 本来该管这件事，但本项目的局部层一直订的是错的话题/错的数据类型
  （`/lidar3d` 是 LaserScan，配置里却写 `PointCloud2`；`z_voxels: 40` 也超过上限 16），
  等于局部避障为空（`docs/REPORT.md §5.3`）。
  在修好 Nav2 之前（以及不管用哪套后端），都需要一个**独立于规划器**的安全层。

怎么做的（关键：不侵入驱动逻辑）
  * 订阅 gz 的多线 `LaserScan`（`/lidar3d`），用 `scan_relay.RingPicker` 抽出最像水平的
    那一圈 —— 与 slam 用的是同一套切环逻辑，不另写一份；
  * 只取**前方扇区**（默认 ±25°）的最小距离，带迟滞：
      距 <  stop_dist (0.55 m) → 进入 BLOCKED，开始以 10 Hz 发零速度；
      距 >  resume_dist (0.85 m) 连续 0.3 s → 退出 BLOCKED，停发（让驱动器接管）。
    迟滞是必须的：单阈值会在临界距离上抖成"走-停-走"，实测比不装还慢。
  * 它**永远不会**代替驱动器前进 —— 只在 BLOCKED 时发零速度；CLEAR 时一个字都不发。
    所以它和 `drive_to_point` / Nav2 可以共存，不需要改它们。

用法
  python3 scripts/forward_guard.py --selftest          # 纯逻辑自检，不需要 ROS/仿真
  python3 scripts/forward_guard.py --dry               # 只观察、不发布（调阈值用）
  python3 scripts/forward_guard.py                     # 真压：BLOCKED 时发零速度
  python3 scripts/forward_guard.py --stop-dist 0.6 --sector-deg 30
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))


# ---------------------------------------------------------------- 纯逻辑
def judge(
    ranges: list[float],
    angle_min: float,
    angle_increment: float,
    *,
    sector_deg: float = 25.0,
    stop_dist: float = 0.55,
    resume_dist: float = 0.85,
    was_blocked: bool = False,
    clear_for: float = 0.0,
    clear_hold: float = 0.30,
) -> dict:
    """单帧判决（纯函数，离线可测）。

    返回 {blocked, nearest, bearing_deg, reason}。`bearing_deg` 是最近障碍相对车头的角度
    （负=右，正=左），只用于日志 —— 安全层本身不做绕行，绕行是规划器的事。
    """
    half = math.radians(sector_deg)
    nearest = math.inf
    bearing = 0.0
    for i, r in enumerate(ranges):
        if not (r == r) or r in (math.inf, -math.inf):      # NaN / inf
            continue
        a = angle_min + i * angle_increment
        if abs(a) > half or r <= 0.05:
            continue
        if r < nearest:
            nearest = r
            bearing = a

    if was_blocked:
        # 只有**持续**清空才放行，避免临界距离抖动
        if nearest >= resume_dist and clear_for >= clear_hold:
            return {"blocked": False, "nearest": nearest, "bearing_deg": math.degrees(bearing),
                    "reason": "clear"}
        return {"blocked": True, "nearest": nearest, "bearing_deg": math.degrees(bearing),
                "reason": "hold" if nearest >= resume_dist else "obstacle"}
    if nearest < stop_dist:
        return {"blocked": True, "nearest": nearest, "bearing_deg": math.degrees(bearing),
                "reason": "obstacle"}
    return {"blocked": False, "nearest": nearest, "bearing_deg": math.degrees(bearing),
            "reason": "clear"}


def selftest() -> int:
    """把上面那个函数喂几组合成数据，确认判决符合预期（不需要 ROS）。"""
    cases: list[tuple[str, dict, dict]] = []

    def far(n=40, ang=10.0, step=0.02):
        return {"ranges": [ang] * n, "angle_min": -step * n / 2, "angle_increment": step}

    def block(d=0.40, n=40, step=0.02):
        rs = [10.0] * n
        rs[n // 2] = d                                  # 正前方（角度≈0）一个障碍
        return {"ranges": rs, "angle_min": -step * n / 2, "angle_increment": step}

    cases.append(("前方空旷 → 放行", dict(far()), {"blocked": False}))
    cases.append(("正前方 0.40 m → 压住", dict(block()), {"blocked": True}))
    # 障碍在 +56°（扇区外）：不能因为它而压住
    cases.append(("障碍在扇区外（+56°）→ 放行",
                  dict(ranges=[10.0] * 99 + [0.30], angle_min=-1.0, angle_increment=0.02),
                  {"blocked": False}))
    # 迟滞：已 BLOCKED，距离 0.70 m（在 stop 与 resume 之间）→ 继续压住
    cases.append(("迟滞：0.70 m 且已 BLOCKED → 仍压住",
                  dict(**block(0.70), was_blocked=True, clear_for=5.0),
                  {"blocked": True}))
    cases.append(("迟滞：1.20 m 且已 BLOCKED → 放行", dict(far(ang=1.2), was_blocked=True,
                  clear_for=1.0), {"blocked": False}))
    cases.append(("全 inf（无回波）→ 放行",
                  dict(ranges=[math.inf] * 40, angle_min=-0.4, angle_increment=0.02),
                  {"blocked": False}))
    cases.append(("全 NaN → 放行（不许拿脏数据当障碍）",
                  dict(ranges=[float("nan")] * 40, angle_min=-0.4, angle_increment=0.02),
                  {"blocked": False}))

    bad = 0
    for name, kw, want in cases:
        got = judge(kw.pop("ranges"), kw.pop("angle_min"), kw.pop("angle_increment"), **kw)
        ok = all(got[k] == v for k, v in want.items())
        bad += 0 if ok else 1
        print(f"[{'PASS' if ok else 'FAIL'}] {name:<28} blocked={got['blocked']} "
              f"nearest={got['nearest']:.2f} bearing={got['bearing_deg']:+.1f}°")
    print()
    print(f"结论：{'PASS' if bad == 0 else 'FAIL'} —— {len(cases) - bad}/{len(cases)} 项符合预期")
    return 0 if bad == 0 else 1


# ---------------------------------------------------------------- ROS 侧
def run(args) -> int:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan

    from scan_relay import RingPicker       # 与 slam 用同一套切环逻辑

    rclpy.init()
    node = Node("forward_guard")
    # **必须包 try/except**：rclpy 某些情况下已经替声明过 use_sim_time，裸调用会抛
    # `ParameterAlreadyDeclaredException` → 节点启动即崩。实测代价：本节点曾在**每一次**
    # GUARD=1 的真跑里静默死掉，而 p6_demo.sh 只看到"压住 0 次"，于是把"0 次"误读成
    # "正常行驶不插手"（空结论）。这是本项目第 4 次踩这条。
    try:
        node.declare_parameter("use_sim_time", True)
    except Exception:                    # ParameterAlreadyDeclaredException
        pass
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    picker = RingPicker()
    state = {"blocked": False, "clear_since": 0.0, "n": 0, "events": 0,
             "last_log": 0.0, "nearest": math.inf}

    def on_scan(msg: LaserScan) -> None:
        if not picker.setup(msg):
            return
        ring = picker.extract(msg)
        if ring is None:
            return
        state["n"] += 1
        now = time.monotonic()
        clear_for = 0.0 if state["clear_since"] == 0 else now - state["clear_since"]
        v = judge(
            list(ring.ranges), ring.angle_min, ring.angle_increment,
            sector_deg=args.sector_deg, stop_dist=args.stop_dist,
            resume_dist=args.resume_dist, was_blocked=state["blocked"], clear_for=clear_for,
        )
        if v["blocked"] and not state["blocked"]:
            state["events"] += 1
            node.get_logger().warn(
                f"[GUARD] 前方 {v['nearest']:.2f} m（{v['bearing_deg']:+.0f}°）< "
                f"{args.stop_dist:.2f} m → 压住 /cmd_vel")
        elif not v["blocked"] and state["blocked"]:
            node.get_logger().info(f"[GUARD] 前方已清（{v['nearest']:.2f} m）→ 放行")
        state["blocked"] = v["blocked"]
        state["nearest"] = v["nearest"]
        if v["nearest"] >= args.resume_dist:
            if state["clear_since"] == 0:
                state["clear_since"] = now
        else:
            state["clear_since"] = 0.0

    node.create_subscription(LaserScan, args.topic, on_scan, qos)

    def tick() -> None:
        if not state["blocked"] or args.dry:
            return
        pub.publish(Twist())                    # 全零 = 停
        now = time.monotonic()
        if now - state["last_log"] > 2.0:
            state["last_log"] = now
            node.get_logger().info(
                f"[GUARD] 正在压住（最近 {state['nearest']:.2f} m，已压 {state['events']} 次）")

    node.create_timer(0.10, tick)
    node.get_logger().info(
        f"[GUARD] 就绪：{args.topic} 扇区 ±{args.sector_deg:.0f}° / "
        f"压住 < {args.stop_dist:.2f} m / 放行 > {args.resume_dist:.2f} m / "
        f"{'只观察不发布' if args.dry else '真压'}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            pub.publish(Twist())
        except Exception:
            pass
        node.get_logger().info(f"[GUARD] 退出：共压住 {state['events']} 次，收到 {state['n']} 帧")
        node.destroy_node()
        rclpy.shutdown()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true", help="纯逻辑自检（不需要 ROS）")
    ap.add_argument("--dry", action="store_true", help="只观察不发布")
    ap.add_argument("--topic", default="/lidar3d")
    ap.add_argument("--sector-deg", type=float, default=25.0)
    ap.add_argument("--stop-dist", type=float, default=0.55)
    ap.add_argument("--resume-dist", type=float, default=0.85)
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
