#!/usr/bin/env python3
"""判据 M1：麦克纳姆速度跟踪实测（SPEC §4.2）。

怎么量
  对每个 cmd_vel 用例：先等 settle（默认 2 s，让加速度斜坡走完），然后取 duration 窗口内的
  /odom 位姿做有限差分，得到机器人**本体系**实测速度（用 yaw 把 odom 速度旋回去），
  再和指令比。用位姿差分而不是直接用 twist，是因为 twist 是插件自己算的，用之等于自己给自己打分。

判据（SPEC §4.2 原文）
  cmd (vx=0.4, vy=0)    → vx 误差 < 10%，vy 串扰 < 0.05 m/s
  cmd (vx=0, vy=0.3)    → vy 误差 < 25%
  cmd (ωz=0.5, 线速度 0) → ωz 误差 < 15%
  cmd (vx=0.3, vy=0.3)  → 合成速度方向误差 < 15°

用法
  ros2 launch launch/depot_sim.launch.py &      # 先起仿真（手动）
  python3.10 scripts/m1_velocity_tracking.py    # 再量
  python3.10 scripts/m1_velocity_tracking.py --cases vx vy   # 只跑某几个用例
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

ROOT = Path(__file__).resolve().parents[1]

# 订阅端一律 BEST_EFFORT：ros_gz_bridge 可能用 best-effort 发布传感器/里程计，
# RELIABLE 订阅会静默收不到（QoS 不兼容），看起来就像"仿真没跑"。
SENSOR_QOS = QoSProfile(
    depth=100,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

# name, vx, vy, wz, 判据（"vx"=线性误差比, "vy"=同上, "wz"=角速度误差比, "dir"=合成方向误差(deg)）
CASES = [
    {"name": "vx_only", "cmd": (0.40, 0.00, 0.00), "crit": "vx", "tol": 0.10, "cross": 0.05},
    {"name": "vy_only", "cmd": (0.00, 0.30, 0.00), "crit": "vy", "tol": 0.25, "cross": 0.10},
    {"name": "wz_only", "cmd": (0.00, 0.00, 0.50), "crit": "wz", "tol": 0.15, "cross": None},
    {"name": "diag", "cmd": (0.30, 0.30, 0.00), "crit": "dir", "tol": 15.0, "cross": None},
]


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class M1(Node):
    def __init__(self) -> None:
        super().__init__("m1_velocity_tracking")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, SENSOR_QOS)
        self.samples: list[tuple[float, float, float, float]] = []  # t, x, y, yaw
        self.cmd = (0.0, 0.0, 0.0)

    def on_odom(self, msg: Odometry) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t == 0.0:
            return
        p = msg.pose.pose.position
        self.samples.append((t, p.x, p.y, yaw_of(msg.pose.pose.orientation)))

    def publish_cmd(self) -> None:
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z = self.cmd
        self.pub.publish(m)


def measure(node: M1, cmd: tuple[float, float, float], settle: float, duration: float) -> dict:
    node.cmd = cmd
    node.samples.clear()
    # 先发 settle 秒，把斜坡走完；窗口从 settle 之后开始
    node.get_clock()  # noqa: B018  (确保 rclpy 初始化时间源)
    t0 = None
    while t0 is None and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.samples:
            t0 = node.samples[0][0]
    # settle
    end_settle = (t0 or 0.0) + settle
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        node.publish_cmd()
        if node.samples and node.samples[-1][0] >= end_settle:
            break
    node.samples.clear()
    # 测量窗口
    end = None
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        node.publish_cmd()
        if node.samples:
            end = node.samples[0][0] + duration
            if node.samples[-1][0] >= end:
                break
    # 停止
    node.cmd = (0.0, 0.0, 0.0)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.02)
        node.publish_cmd()

    s = node.samples
    if len(s) < 10:
        return {"ok": False, "reason": f"样本不足（{len(s)} 条 /odom），仿真没跑或 /odom 没桥接"}

    # 逐段有限差分 -> 本体系速度
    vxs: list[float] = []
    vys: list[float] = []
    wzs: list[float] = []
    for (t1, x1, y1, y1aw), (t2, x2, y2, y2aw) in zip(s, s[1:]):
        dt = t2 - t1
        if dt <= 1e-6 or dt > 0.5:
            continue
        vx_w = (x2 - x1) / dt
        vy_w = (y2 - y1) / dt
        c, sn = math.cos(y1aw), math.sin(y1aw)
        vxs.append(c * vx_w + sn * vy_w)
        vys.append(-sn * vx_w + c * vy_w)
        dyaw = math.atan2(math.sin(y2aw - y1aw), math.cos(y2aw - y1aw))
        wzs.append(dyaw / dt)
    if not vxs:
        return {"ok": False, "reason": "差分不出速度（时间戳不连续）"}

    n = len(vxs)
    mean = lambda a: sum(a) / len(a)  # noqa: E731
    res = {
        "n_samples": n,
        "duration_s": round(s[-1][0] - s[0][0], 2),
        "vx_meas": round(mean(vxs), 4),
        "vy_meas": round(mean(vys), 4),
        "wz_meas": round(mean(wzs), 4),
        "vx_std": round(math.sqrt(mean([(v - mean(vxs)) ** 2 for v in vxs])), 4),
        "vy_std": round(math.sqrt(mean([(v - mean(vys)) ** 2 for v in vys])), 4),
        "wz_std": round(math.sqrt(mean([(v - mean(wzs)) ** 2 for v in wzs])), 4),
    }
    return res


def evaluate(case: dict, res: dict) -> dict:
    if not res.get("ok", True) and res.get("reason"):
        return {**res, "pass": False}
    vx_c, vy_c, wz_c = case["cmd"]
    vx, vy, wz = res["vx_meas"], res["vy_meas"], res["wz_meas"]
    out = dict(res)
    out["cmd"] = list(case["cmd"])
    out["case"] = case["name"]

    if case["crit"] == "vx":
        err = abs(vx - vx_c) / abs(vx_c) if vx_c else 0.0
        out.update(metric="vx_rel_err", value=round(err, 4), tol=case["tol"])
        out["pass"] = err < case["tol"] and abs(vy) < (case.get("cross") or 9)
    elif case["crit"] == "vy":
        err = abs(vy - vy_c) / abs(vy_c) if vy_c else 0.0
        out.update(metric="vy_rel_err", value=round(err, 4), tol=case["tol"])
        out["pass"] = err < case["tol"] and abs(vx) < (case.get("cross") or 9)
    elif case["crit"] == "wz":
        err = abs(wz - wz_c) / abs(wz_c) if wz_c else 0.0
        out.update(metric="wz_rel_err", value=round(err, 4), tol=case["tol"])
        out["pass"] = err < case["tol"]
    else:  # dir
        dir_cmd = math.degrees(math.atan2(vy_c, vx_c))
        dir_meas = math.degrees(math.atan2(vy, vx)) if (abs(vx) + abs(vy)) > 1e-6 else float("nan")
        err = abs(dir_meas - dir_cmd)
        err = 180 - abs(err - 180) if err > 180 else err
        out.update(metric="dir_err_deg", value=round(err, 2), tol=case["tol"], dir_cmd=round(dir_cmd, 2),
                   dir_meas=round(dir_meas, 2))
        out["pass"] = err < case["tol"]
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--settle", type=float, default=2.0)
    ap.add_argument("--duration", type=float, default=4.0)
    ap.add_argument("--cases", nargs="*", default=[c["name"] for c in CASES])
    ap.add_argument("--out", default=str(ROOT / "results" / "m1.json"))
    args = ap.parse_args()

    rclpy.init()
    node = M1()
    # 等 /odom 出现（ROS 图还没起来时不要误判成 STUCK）
    waited = 0.0
    while rclpy.ok() and not node.samples and waited < 15.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        node.publish_cmd()
        waited += 0.1
    if not node.samples:
        rclpy.shutdown()
        print("FAIL 15 s 内没有收到 /odom。先确认 `ros2 launch launch/depot_sim.launch.py` 在跑，"
              "且 `ros2 topic hz /odom` 有输出。")
        return 2

    results = []
    for case in CASES:
        if case["name"] not in args.cases:
            continue
        print(f"[M1] 用例 {case['name']:<8} cmd={case['cmd']} ...", flush=True)
        raw = measure(node, case["cmd"], args.settle, args.duration)
        results.append(evaluate(case, raw))

    rclpy.shutdown()

    print()
    print(f"{'case':<10}{'cmd(vx,vy,wz)':<24}{'实测':<28}{'指标':<14}{'值':>8}{'容差':>8}{'':>6}")
    for r in results:
        if "vx_meas" not in r:
            print(f"{r.get('case', '?'):<10} 无效：{r.get('reason')}")
            continue
        meas = f"({r['vx_meas']:+.3f},{r['vy_meas']:+.3f},{r['wz_meas']:+.3f})"
        print(f"{r['case']:<10}{str(tuple(r['cmd'])):<24}{meas:<28}{r['metric']:<14}"
              f"{r['value']:>8}{r['tol']:>8}{'PASS' if r['pass'] else 'FAIL':>6}")

    n_pass = sum(1 for r in results if r.get("pass"))
    verdict = n_pass == len(results) and results
    payload = {
        "criterion": "M1",
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "settle_s": args.settle,
        "duration_s": args.duration,
        "cases": results,
        "n_pass": n_pass,
        "n_total": len(results),
        "pass": bool(verdict),
        "note": "实测速度由 /odom 位姿有限差分得到（不是插件自报的 twist）",
    }
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print()
    print(f"RESULT: {'PASS' if verdict else 'FAIL'}  ({n_pass}/{len(results)})  -> {out.relative_to(ROOT)}")
    if not verdict:
        print("下一步（按 SPEC §4.2 三级补救顺序）：")
        print("  ① 用 xacro 参数 wheel_friction:=isotropic / anisotropic 对比，看是否方向性摩擦问题")
        print("  ② 若仍不达标：改用 4 个 velocity 关节 + 自己写反解，绕过 MecanumDrive")
        print("  ③ 若仍不达标：承认运动学近似，把误差写进报告（不要藏）")
    return 0 if verdict else 1


if __name__ == "__main__":
    sys.exit(main())
