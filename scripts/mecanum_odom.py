#!/usr/bin/env python3
"""麦克纳姆里程计：从四个轮子的实测角速度积分出 /odom（+ TF odom->base_footprint）。

为什么需要这个节点（实测结论，不是设计偏好）
  Fortress 6.18 的 `ignition-gazebo-mecanum-drive-system` 在这个构建里**不发布 odometry**：
  即使 `<odom_publish_frequency>50</odom_publish_frequency>` 写了，publisher 也注册了
  （`ign topic -i` 能看到），但一条消息都不发 —— 官方例程 `mecanum_drive.sdf` 加同样参数
  也一样静默。而 `/joint_states` 里四个轮的 `velocity` 是真实在变的（跟 cmd_vel 一一对应），
  所以里程计从轮速自己算：信息源和插件本该用的完全一样，不是拿真值作弊。

运动学（符号约定由实测标定，见 docs/TROUBLESHOOTING.md §7）
  正解（插件内部，cmd -> 轮速）
      fl = ( vx - vy - (lx+ly)·wz ) / r
      fr = ( vx + vy + (lx+ly)·wz ) / r
      rl = ( vx + vy - (lx+ly)·wz ) / r
      rr = ( vx - vy + (lx+ly)·wz ) / r
  逆解（本节点，轮速 -> cmd 速度）
      vx =  r/4        · ( fl + fr + rl + rr )
      vy =  r/4        · (-fl + fr + rl - rr )
      wz =  r/(4(lx+ly))·(-fl + fr - rl + rr )
  lx = wheelbase/2 = 0.20，ly = track/2 = 0.21，r = 0.076

用法
  ros2 launch launch/depot_sim.launch.py        # 本节点由 launch 起
  python3 scripts/mecanum_odom.py --selftest    # 纯离线自检：正解->逆解 往返一致 + 积分闭式解
"""
from __future__ import annotations

import argparse
import math
import sys

# ---- 底盘常量（与 robot/depot_bot.urdf.xacro 一致；改模型就要改这里）
WHEEL_RADIUS = 0.076
WHEELBASE = 0.40
TRACK = 0.42
JOINT_ORDER = ("wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint")


def wheel_speeds_to_twist(w_fl: float, w_fr: float, w_rl: float, w_rr: float) -> tuple[float, float, float]:
    """逆解：四个轮角速度(rad/s) -> 车体系 (vx, vy, wz)。"""
    r = WHEEL_RADIUS
    half = (WHEELBASE + TRACK) / 2.0          # lx + ly = 0.41
    vx = r / 4.0 * (w_fl + w_fr + w_rl + w_rr)
    vy = r / 4.0 * (-w_fl + w_fr + w_rl - w_rr)
    wz = r / (4.0 * half) * (-w_fl + w_fr - w_rl + w_rr)
    return vx, vy, wz


def twist_to_wheel_speeds(vx: float, vy: float, wz: float) -> tuple[float, float, float, float]:
    """正解：车体系速度 -> 四个轮角速度（用来做自检和交叉验证）。"""
    r = WHEEL_RADIUS
    half = (WHEELBASE + TRACK) / 2.0
    return (
        (vx - vy - half * wz) / r,
        (vx + vy + half * wz) / r,
        (vx + vy - half * wz) / r,
        (vx - vy + half * wz) / r,
    )


def wheel_deltas_to_pose_step(d_fl: float, d_fr: float, d_rl: float, d_rr: float) -> tuple[float, float, float]:
    """累积轮角增量(rad) -> 车体系位移增量 (dx, dy, dyaw)。

    用**累积角位置**做差分而不是"速度 × dt"：轮角是绝对值，差分天然免疫
    /clock 跳变、丢帧和暂停恢复 —— 实测用 dt 积分时 5% 的样本要因为 dt 越界被丢弃。
    """
    r = WHEEL_RADIUS
    half = (WHEELBASE + TRACK) / 2.0
    dx = r / 4.0 * (d_fl + d_fr + d_rl + d_rr)
    dy = r / 4.0 * (-d_fl + d_fr + d_rl - d_rr)
    dyaw = r / (4.0 * half) * (-d_fl + d_fr - d_rl + d_rr)
    return dx, dy, dyaw


def step_pose(x: float, y: float, yaw: float, dx: float, dy: float, dyaw: float):
    """把车体系位移增量按当前 yaw 转到世界系（小步长下用中点 yaw 足够准）。"""
    yaw_mid = yaw + 0.5 * dyaw
    c, s = math.cos(yaw_mid), math.sin(yaw_mid)
    return x + (dx * c - dy * s), y + (dx * s + dy * c), yaw + dyaw


def integrate(x: float, y: float, yaw: float, vx: float, vy: float, wz: float, dt: float):
    """速度 × dt 版本（保留给自检与"没有累积角位置"时的退路）。"""
    return step_pose(x, y, yaw, vx * dt, vy * dt, wz * dt)


def selftest() -> int:
    fails = 0

    def check(name: str, ok: bool, detail: str) -> None:
        nonlocal fails
        print(f"  [{'PASS' if ok else 'FAIL'}] {name:<28} {detail}")
        if not ok:
            fails += 1

    # 1) 正解 -> 逆解 往返
    worst = 0.0
    for vx in (-0.6, 0.0, 0.3, 1.0):
        for vy in (-0.3, 0.0, 0.3):
            for wz in (-1.0, 0.0, 0.5):
                w = twist_to_wheel_speeds(vx, vy, wz)
                b = wheel_speeds_to_twist(*w)
                worst = max(worst, abs(b[0] - vx), abs(b[1] - vy), abs(b[2] - wz))
    check("往返一致 (36 组)", worst < 1e-12, f"max 误差 {worst:.2e}")

    # 2) 实测标定值：vx=0.3 时四轮都应是 +3.9474 rad/s
    w = twist_to_wheel_speeds(0.30, 0.0, 0.0)
    check("vx=0.30 轮速", all(abs(v - 3.94737) < 1e-4 for v in w), f"{[round(v, 4) for v in w]}")

    # 3) 实测标定值：vy=0.30 -> (-,+,+,-)
    w = twist_to_wheel_speeds(0.0, 0.30, 0.0)
    check("vy=0.30 符号", w[0] < 0 and w[1] > 0 and w[2] > 0 and w[3] < 0, f"{[round(v, 3) for v in w]}")

    # 4) 实测标定值：wz=0.50 -> (-,+,-,+)，幅值 2.697
    w = twist_to_wheel_speeds(0.0, 0.0, 0.50)
    ok = w[0] < 0 < w[1] and w[2] < 0 < w[3] and all(abs(abs(v) - 2.69737) < 1e-3 for v in w)
    check("wz=0.50 符号+幅值", ok, f"{[round(v, 3) for v in w]}")

    # 5) 积分闭式解：vx=0.3 走 2 s 应沿 +X 走 0.6 m
    x = y = yaw = 0.0
    for _ in range(200):
        x, y, yaw = integrate(x, y, yaw, 0.30, 0.0, 0.0, 0.01)
    check("积分 vx 2s", abs(x - 0.60) < 1e-9 and abs(y) < 1e-9, f"x={x:.6f} y={y:.6f}")

    # 6) 积分侧移：vy=0.3 走 1 s 应沿 +Y 走 0.3 m，yaw 不变
    x = y = yaw = 0.0
    for _ in range(100):
        x, y, yaw = integrate(x, y, yaw, 0.0, 0.30, 0.0, 0.01)
    check("积分 vy 1s", abs(y - 0.30) < 1e-9 and abs(x) < 1e-9, f"x={x:.2e} y={y:.6f}")

    # 7) 原地转 90°：wz=π/2 走 1 s
    x = y = yaw = 0.0
    for _ in range(100):
        x, y, yaw = integrate(x, y, yaw, 0.0, 0.0, math.pi / 2, 0.01)
    check("积分 wz 1s", abs(yaw - math.pi / 2) < 1e-9 and abs(x) < 1e-9 and abs(y) < 1e-9,
          f"yaw={math.degrees(yaw):.3f}°")

    # 8) 节点实际走的那条路：累积轮角差分（node 主路径，不带 dt）
    x = y = yaw = 0.0
    d = [w * 0.01 for w in twist_to_wheel_speeds(0.30, 0.0, 0.0)]
    for _ in range(200):
        dx, dy, dyaw = wheel_deltas_to_pose_step(*d)
        x, y, yaw = step_pose(x, y, yaw, dx, dy, dyaw)
    check("轮角差分 vx 2s", abs(x - 0.60) < 1e-9 and abs(y) < 1e-9 and abs(yaw) < 1e-9,
          f"x={x:.6f} y={y:.2e} yaw={yaw:.2e}")

    # 9) 轮角差分侧移：vy=0.3 走 1 s
    x = y = yaw = 0.0
    d = [w * 0.01 for w in twist_to_wheel_speeds(0.0, 0.30, 0.0)]
    for _ in range(100):
        dx, dy, dyaw = wheel_deltas_to_pose_step(*d)
        x, y, yaw = step_pose(x, y, yaw, dx, dy, dyaw)
    check("轮角差分 vy 1s", abs(y - 0.30) < 1e-9 and abs(x) < 1e-9 and abs(yaw) < 1e-9,
          f"y={y:.6f} x={x:.2e} yaw={yaw:.2e}")

    print()
    print("RESULT:", "FAIL" if fails else "PASS")
    return 1 if fails else 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true", help="纯离线自检，不连 ROS")
    ap.add_argument("--verbose", action="store_true", help="每 2 s 打印一次速度与位姿")
    ap.add_argument("--odom-topic", default="/odom")
    ap.add_argument("--tf", action="store_true", default=True,
                    help="同时发 TF odom->base_footprint（默认开；关掉时由 odom_tf.py 负责）")
    ap.add_argument("--no-tf", dest="tf", action="store_false")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from tf2_ros import TransformBroadcaster

    qos = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)

    class MecanumOdom(Node):
        def __init__(self) -> None:
            super().__init__("mecanum_odom")
            self.x = self.y = self.yaw = 0.0
            self.vx = self.vy = self.wz = 0.0
            self.last_t: float | None = None
            self.last_pos: list[float] | None = None
            self.n_msg = 0
            self.n_skip = 0
            self.n_jump = 0
            self.pub = self.create_publisher(Odometry, args.odom_topic, 10)
            self.create_subscription(JointState, "/joint_states", self.on_joint, qos)
            self.tfb = TransformBroadcaster(self) if args.tf else None
            self.create_timer(2.0, self.report)

        def on_joint(self, msg: JointState) -> None:
            idx = {n: i for i, n in enumerate(msg.name)}
            if not all(j in idx for j in JOINT_ORDER):
                return
            speeds = [msg.velocity[idx[j]] if idx[j] < len(msg.velocity) else 0.0 for j in JOINT_ORDER]
            self.vx, self.vy, self.wz = wheel_speeds_to_twist(*speeds)

            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if t <= 0.0:
                return

            pos = [msg.position[idx[j]] if idx[j] < len(msg.position) else None for j in JOINT_ORDER]
            if all(p is not None for p in pos):
                pp: list[float] = [float(p) for p in pos if p is not None]
                # 主路径：累积轮角差分 -> 位移增量（不需要 dt）
                if self.last_pos is not None:
                    # 轮角必须先在 ±π 处解缠绕再差分：
                    # /joint_states 的 position 是连续角，超过 π 会翻回 -π（或反之），
                    # 原始差分会出现 ≈±2π 的假增量 -> 一步就注入 ±66° 的假转角，
                    # 而 |Δ|=6.28 远小于下面的 50 rad 跳变阈值，过滤器抓不住。
                    # 实测症状：cmd wz=+0.5 转 2 s，真值 +57°，odom 只记 +40.3°（反向 -23.8°），
                    # 正反不对称 = 缠绕方向不同。
                    d = [((p - q + math.pi) % (2 * math.pi)) - math.pi
                         for p, q in zip(pp, self.last_pos)]
                    if any(abs(v) > 3.0 for v in d):   # 缠绕是 6.28；正常单帧增量 <0.1 rad
                        self.n_jump += 1
                    else:
                        dx, dy, dyaw = wheel_deltas_to_pose_step(*d)
                        self.x, self.y, self.yaw = step_pose(self.x, self.y, self.yaw, dx, dy, dyaw)
                self.last_pos = pp
            else:
                # 退路：只有速度时按 dt 积分（dt 越界丢弃）
                if self.last_t is not None:
                    dt = t - self.last_t
                    if 0.0 < dt < 0.5:
                        self.x, self.y, self.yaw = integrate(self.x, self.y, self.yaw,
                                                             self.vx, self.vy, self.wz, dt)
                    elif dt != 0.0:
                        self.n_skip += 1

            self.last_t = t
            self.n_msg += 1
            self.publish(t)

        def publish(self, t: float) -> None:
            qz, qw = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
            od = Odometry()
            od.header.stamp.sec = int(t)
            od.header.stamp.nanosec = int((t - int(t)) * 1e9)
            od.header.frame_id = "odom"
            od.child_frame_id = "base_footprint"
            od.pose.pose.position.x = self.x
            od.pose.pose.position.y = self.y
            od.pose.pose.orientation.z = qz
            od.pose.pose.orientation.w = qw
            # 横移不可观测是麦克纳姆的固有弱点（SPEC §5）：给一个诚实的、偏大的横向方差
            od.pose.covariance[0] = 0.01
            od.pose.covariance[7] = 0.04
            od.pose.covariance[35] = 0.01
            # 未融合轴也填上大值，别留 0（0 在 RL 里是"绝对确信"）
            od.pose.covariance[14] = od.pose.covariance[21] = 1e-3
            od.pose.covariance[28] = 1e-3
            # **twist 协方差原本全 0**：EKF 融合轮速 vx,vy,vyaw 时同样会踩奇异矩阵。
            if not any(od.twist.covariance):
                od.twist.covariance[0] = od.twist.covariance[7] = 0.01
                od.twist.covariance[35] = 0.01
                od.twist.covariance[14] = od.twist.covariance[21] = 1e-3
                od.twist.covariance[28] = 1e-3
            od.twist.twist.linear.x = self.vx
            od.twist.twist.linear.y = self.vy
            od.twist.twist.angular.z = self.wz
            self.pub.publish(od)

            if self.tfb is not None:
                tf = TransformStamped()
                tf.header = od.header
                tf.child_frame_id = "base_footprint"
                tf.transform.translation.x = self.x
                tf.transform.translation.y = self.y
                tf.transform.rotation.z = qz
                tf.transform.rotation.w = qw
                self.tfb.sendTransform(tf)

        def report(self) -> None:
            if not args.verbose:
                return
            self.get_logger().info(
                f"odom x={self.x:+.3f} y={self.y:+.3f} yaw={math.degrees(self.yaw):+.1f}° | "
                f"v=({self.vx:+.3f},{self.vy:+.3f}) wz={self.wz:+.3f} | "
                f"msgs={self.n_msg} jump={self.n_jump} skip={self.n_skip}"
            )

    rclpy.init()
    node = MecanumOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
