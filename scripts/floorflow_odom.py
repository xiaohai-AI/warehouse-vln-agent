#!/usr/bin/env python3
"""S4 里程计：**地面带 2D 光流给平移 + IMU 给航向**，发布 /odometry/floorflow。

与 S3（rtabmap 双目）的差别只有一处 —— 怎么用图像：
  S3：全图特征 + 立体视差 → 周期货架主导对应，沿轴只还原 1.4%
  S4：只取画面底部地面带 + 2D KLT（不用视差）→ 沿轴还原 93.4%（实测，`make s4-floorflow`）

为什么能测：地面点行距 r = v − cy 与水平距离 X 满足 X = f·h/r；前进 Δx 后
    Δx = f·h·(1/r − 1/(r+Δv))        （精确式，见 depot_vln/floorflow.py）
沿轴信号量级：0.35 m/s@10 Hz、h=0.43 m、地面 0.6–2 m → **2.8–8.9 px/帧**，
而货架（周期 2.4 m、5–10 m 远）只有 0.6–1.1 px —— 所以必须把货架排除在输入之外，
不是"调参"能解决的（DECISIONS D13）。

航向：IMU 陀螺 z 积分（静止 2 s 估零偏），与 S3 的"视觉 + IMU"口径一致；
      `--yaw-from wheel` 可换用 /odom 的航向（对照实验用；实测横移后自己漂 -9.4°，不可用）。

发布：
  /odometry/floorflow        nav_msgs/Odometry（frame=odom，child=base_footprint）
  TF odom->base_footprint    默认发（仿真要 odom_tf:=false，否则与 mecanum_odom 抢 TF）
  /floorflow/status          std_msgs/String（帧率/每帧位移/掉帧数，排障用）

用法（一般由 slam1_run.sh CFG=s4 起）：
  bash -lc 'source /opt/ros/humble/setup.bash && .venv/bin/python scripts/floorflow_odom.py'
  参数：--h 0.456（标定值，见下） --band 0.40 --hfov 90 --yaw-from imu --publish-tf 1

尺度标定：nominal h=0.426 m（SDF 0.35 + 轮轴 0.076），实测 20.01 m 走成 18.68 m
（还原 93.4%）→ **h_eff = 0.426 × 20.01/18.68 = 0.456 m**。默认用标定值；
换了车型/悬挂就得重标（一条 `make s4-floorflow` 就够）。
"""
from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from depot_vln.floorflow import (band_mask, estimate_forward, solve_motion,  # noqa: E402
                                 track_points)


def yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class FloorFlowOdom(Node):
    def __init__(self, a: argparse.Namespace) -> None:
        super().__init__("floorflow_odom")
        try:
            self.declare_parameter("use_sim_time", True)
        except Exception:
            pass
        self.h = a.h
        self.h_lat = a.h_lat
        self.no_lateral = bool(a.no_lateral)
        self.legacy_flow = bool(a.legacy_flow)
        self.yaw_from = a.yaw_from
        self.wheel_yaw = None
        self.cam_x = float(a.cam_x)
        self.cam_y = float(a.cam_y)
        self.band = a.band
        self.hfov = a.hfov
        self.publish_tf = bool(a.publish_tf)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.yaw_at_prev_img = 0.0
        self.frame = "odom"
        self.child = "base_footprint"

        self.f = None
        self.cy = None
        self.cx = None
        self.mask = None
        self.prev_gray = None
        self.prev_stamp = None
        self.n_img = 0
        self.n_used = 0
        self.n_skip = 0
        self.n_used_img = 0
        self.n_retry = 0
        # **在线陀螺零偏估计**（安静样本中位数）：这颗 IMU 的 wz 有 ~0.4 mrad/s 的游走零偏，
        # 6 秒的启动窗口估不准（±0.18 mrad/s），而纯陀螺积分的航向 600 s 会漂 5.6° ——
        # 5.6° × 35 m ≈ 3.4 m 横向，正是 S5 里程计链的残差。
        # 做法：只收集 |wz| < 3 mrad/s 的"安静"样本（静止与匀速直行时 wz≈0 就是零偏），
        # 取最近 30 s（6000 样本）的中位数 → 单调、抗外点、能跟上缓慢游走。
        self._quiet = deque(maxlen=6000)
        self._bias_ready = False
        self.t_start = None
        self._t_first = None
        self._t_last = None

        # IMU：静止段估零偏（前 2 s），之后 wz_used = wz − bias
        self.bias = 0.0
        self._bias_acc: list[float] = []
        self.prev_imu_t = None
        self.t_imu = None
        self.dx_last = 0.0
        self.dy_last = 0.0
        self.dt_last = 0.0

        self.pub = self.create_publisher(Odometry, "/odometry/floorflow", 10)
        # 去偏后的 IMU 角速度：EKF 的航向权威吃这个，而不是带 0.4 mrad/s 游走零偏的原始 /imu
        self.pub_imu = self.create_publisher(Imu, "/imu_debiased", 50)
        self.pub_st = self.create_publisher(String, "/floorflow/status", 10)
        self.tfb = TransformBroadcaster(self) if self.publish_tf else None
        img_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, a.image_topic, self._on_img, img_qos)
        self.create_subscription(Imu, a.imu_topic, self._on_imu, 50)
        if self.yaw_from == "wheel":
            # 航向源：轮式里程计的 yaw（EKF 也把它当绝对偏航用，实测整条路线漂 ≤0.13°）。
            # 自己积分 IMU 会漂 8°/600 s —— 那 8° 会把纯前进的位移转出 2-3 m 的横向误差。
            self.create_subscription(Odometry, "/odom", self._on_wheel, 50)

        self._stop = False
        threading.Thread(target=self._spin_loop, daemon=True).start()
        self.create_timer(5.0, self._report)

    # ------------------------------------------------------------ 线程
    def _spin_loop(self) -> None:
        while rclpy.ok() and not self._stop:
            try:
                rclpy.spin_once(self, timeout_sec=0.05)
            except Exception:
                break

    def stop(self) -> None:
        self._stop = True

    # ------------------------------------------------------------ 回调
    def _on_imu(self, m: Imu) -> None:
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        wz = float(m.angular_velocity.z)
        if t <= 0.0:
            # 有仿真器不发 IMU 戳；退回节点钟（只在没有更好来源时）
            t = self.get_clock().now().nanoseconds * 1e-9
        if self.t_start is None:
            self.t_start = t
        if t - self.t_start < 2.0:                       # 起车前 2 s：估零偏
            self._bias_acc.append(wz)
            if len(self._bias_acc) == 40:
                self.bias = float(np.median(self._bias_acc))
                self.get_logger().info(f"IMU 陀螺零偏 {self.bias * 1000:+.2f} mrad/s（静止 2 s）")
            return
        # 安静样本 → 零偏（先于任何使用）
        if abs(wz) < 3e-3:
            self._quiet.append(wz)
            if len(self._quiet) >= 400 and len(self._quiet) % 200 == 0:
                self.bias = float(np.median(self._quiet))
                if not self._bias_ready:
                    self._bias_ready = True
                    self.get_logger().info(
                        f"IMU 零偏（安静样本中位数）{self.bias * 1000:+.3f} mrad/s")
        self._pub_debiased(m, wz)
        if self.yaw_from != "imu" or self.prev_imu_t is None:
            self.prev_imu_t = t
            return
        dt = t - self.prev_imu_t
        self.prev_imu_t = t
        if not (0.0 < dt < 0.5):
            return
        self.yaw += (wz - self.bias) * dt
        self.t_imu = t

    def _pub_debiased(self, m: Imu, wz: float) -> None:
        """把 (wz − 在线零偏) 发出去，其余字段原样转发（协方差保留，绝不填 0）。"""
        try:
            o = Imu()
            o.header = m.header
            o.orientation = m.orientation
            o.orientation_covariance = m.orientation_covariance
            o.angular_velocity = m.angular_velocity
            o.angular_velocity.z = wz - self.bias
            o.angular_velocity_covariance = m.angular_velocity_covariance
            o.linear_acceleration = m.linear_acceleration
            o.linear_acceleration_covariance = m.linear_acceleration_covariance
            self.pub_imu.publish(o)
        except Exception:
            pass

    def _on_wheel(self, m: Odometry) -> None:
        q = m.pose.pose.orientation
        self.wheel_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self.yaw_from == "wheel":
            self.yaw = self.wheel_yaw

    def _on_img(self, m: Image) -> None:
        try:
            h, w = m.height, m.width
            ch = max(1, len(m.data) // (h * w))
            a = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(h, w, ch)
            gray = cv2.cvtColor(a, cv2.COLOR_RGB2GRAY) if ch >= 3 else a[:, :, 0]
            stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if self.mask is None or self.mask.shape != gray.shape:
                self.f = (w / 2.0) / math.tan(math.radians(self.hfov) / 2.0)
                self.cy = h / 2.0
                self.cx = w / 2.0
                self.mask = band_mask(gray.shape, self.band)
            self.n_img += 1
            self._t_last = stamp
            if self._t_first is None:
                self._t_first = stamp
            r = None
            if self.prev_gray is not None and self.prev_stamp is not None:
                dt = stamp - self.prev_stamp
                if 1e-3 < dt < 1.0:
                    if self.legacy_flow:
                        # 老路径（A/B 对照用）：不补偿旋转、在图像空间取中位数
                        r = estimate_forward(self.prev_gray, gray, h=self.h, f=self.f,
                                             cy=self.cy, band=self.band, mask=self.mask,
                                             cx=self.cx, h_lat=self.h_lat)
                    else:
                        # **新路径**：先用 IMU 的帧间偏航把旋转去掉，再在地面度量空间
                        # 联合解 (dx, dy) + MAD 剔外点。合成自检证明它同时消掉两处偏差：
                        #   ①转弯时旋转被 dy 通道吞掉（纯转 3°/帧 → 老公式凭空 59 mm/帧）
                        #   ②前进时径向膨胀 × 特征左右不均 → 老公式 5 mm/帧 假侧移
                        # 详见 docs/DECISIONS.md D16 与 scripts/s4_vo_selftest.py
                        dpsi = self._yaw_delta(self.yaw_at_prev_img, self.yaw)
                        tr = track_points(self.prev_gray, gray, band=self.band,
                                          mask=self.mask, max_corners=260)
                        r = None if tr is None else solve_motion(
                            tr[0], tr[1], h=self.h, f=self.f, cx=self.cx, cy=self.cy,
                            dyaw=dpsi, h_lat=self.h_lat)
                        if r is not None:
                            r["dpsi"] = dpsi
                            # **旋转中心不在相机上**：车体绕 base 原点转 Δψ 时，相机（前向
                            # 0.26 m、左 0.06 m）额外走了一段圆弧 (R(Δψ)−I)·p。solve_motion
                            # 量到的是**相机点**的位移，必须减掉这段才是车体位移。
                            # 实测漏掉这一项时，边走边转 0.43 m 的位移上矢量误差 22.5 cm，
                            # 与 (R−I)·p 的解析值 0.195 m 同量级同方向。
                            c_, s_ = math.cos(dpsi), math.sin(dpsi)
                            ex = (c_ - 1.0) * self.cam_x - s_ * self.cam_y
                            ey = s_ * self.cam_x + (c_ - 1.0) * self.cam_y
                            r["dx"] -= ex
                            r["dy"] -= ey
                    if self.no_lateral and r is not None:
                        r["dy"] = 0.0
                    if r is not None:
                        self._integrate(r["dx"], r.get("dy") or 0.0, dt, stamp)
                        self.n_used_img += 1
                else:
                    self.n_skip += 1
            # **解算失败时不能换参考帧**：换了就等于把 A→B 这段运动永久丢掉。
            # 实测后果：相机有时出 20 Hz，一半的帧对因基线太短失败 → 轴向尺度掉到
            # 0.928（6.03 m 只报 5.60 m），run-to-run 在 1.004/0.928 之间跳。
            # 正确做法：保留旧参考帧，让下一次成功的一对**跨更大的基线**把这段补回来；
            # 但基线也不能无限长：0.35 m/s 下 0.35 s 就是 0.12 m/帧，已到精确反解的可靠边界
            # （自检：7 cm 误差 0.2%、12 cm 开始退化）。超过 0.35 s 就放弃并重置。
            if self.prev_gray is not None and r is None and (stamp - self.prev_stamp) < 0.35:
                self.n_retry += 1
            else:
                self.prev_gray = gray
                self.prev_stamp = stamp
                self.yaw_at_prev_img = self.yaw
        except Exception as e:
            self.get_logger().warn(f"图像回调异常：{type(e).__name__}: {e}")

    # ------------------------------------------------------------ 积分与发布
    @staticmethod
    def _yaw_delta(a: float, b: float) -> float:
        """帧间偏航增量，折回 (−π, π]。"""
        d = b - a
        while d > math.pi:
            d -= 2.0 * math.pi
        while d <= -math.pi:
            d += 2.0 * math.pi
        return d

    def _integrate(self, dx: float, dy: float, dt: float, stamp: float) -> None:
        """车体系 (dx 前, dy 左) → 世界系。**两个分量都要**：
        S4 第一次跑整条路线时轴向腿 99.5–99.9%、侧移腿 0.3%（麦克纳姆横移在图像里是
        水平流动，只测竖直分量就完全看不见）——见 DECISIONS D13 的实测结果。"""
        self.n_used += 1
        self.dx_last, self.dy_last, self.dt_last = dx, dy, dt
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += dx * c - dy * s
        self.y += dx * s + dy * c
        self._publish(stamp, dx / dt, dy / dt)

    def _publish(self, stamp: float, vx: float, vy: float = 0.0) -> None:
        o = Odometry()
        # **戳用图像戳**：位姿是对应"那一帧"的，用发布时刻打戳会引入一个恒定的
        # 滞后（0.35 m/s × 100 ms ≈ 3.5 cm），在长路线里会变成系统性偏差。
        try:
            o.header.stamp = Time(seconds=float(stamp)).to_msg()
        except Exception:
            o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = self.frame
        o.child_frame_id = self.child
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        qx, qy, qz, qw = yaw_to_quat(self.yaw)
        o.pose.pose.orientation.x, o.pose.pose.orientation.y = qx, qy
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        o.twist.twist.linear.x = vx
        o.twist.twist.linear.y = vy
        o.twist.twist.angular.z = 0.0
        # **必须填协方差**：全零在 robot_localization 里等于"绝对确信"，两路零协方差
        # 来源一起喂会让 EKF 输出 NaN（TF 报 nan nan nan nan）—— 踩过一次。
        # 位姿：地面光流每帧抖动实测 0.09 cm 量级，给 2 cm 标准差（含未建模漂移）。
        # 偏航/横滚/俯仰不参与融合（航向由 IMU+轮速给），给大值表示"不可信"。
        c = o.pose.covariance
        c[0] = c[7] = 4e-4
        c[14] = c[21] = c[28] = 1e-3
        c[35] = 5e-2
        ct = o.twist.covariance
        ct[0] = ct[7] = 4e-4          # vx, vy（0.02 m/s）
        ct[14] = ct[21] = ct[28] = 1e-3
        ct[35] = 1e-2                 # vyaw 不由本节点负责
        self.pub.publish(o)
        if self.tfb is not None:
            t = TransformStamped()
            t.header.stamp = o.header.stamp
            t.header.frame_id = self.frame
            t.child_frame_id = self.child
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.x, t.transform.rotation.y = qx, qy
            t.transform.rotation.z, t.transform.rotation.w = qz, qw
            self.tfb.sendTransform(t)

    def _report(self) -> None:
        # 速率用**图像戳**实测（别用 "帧数/10" 估，也别用节点钟 —— 上一版在第一次上报前
        # 分母≈0，打印出 "58000000.0 Hz" 这种噪声行）
        el = ((self._t_last or 0.0) - (self._t_first or 0.0)) or 1e-6
        hz = (self.n_img - 1) / el if el > 1e-3 else 0.0
        s = (f"[floorflow-odom] 图像 {self.n_img} 帧 / {el:.0f}s = {hz:.1f} Hz "
             f"| 出估计 {self.n_used} | 跳过 {self.n_skip} | 重试 {self.n_retry} "
             f"| 位姿 ({self.x:+.2f},{self.y:+.2f}) yaw {math.degrees(self.yaw):+.1f}° "
             f"| 上一帧 dx {self.dx_last * 100:+.2f} dy {self.dy_last * 100:+.2f} cm"
             f" / {self.dt_last * 1000:.0f} ms")
        self.pub_st.publish(String(data=s))
        self.get_logger().info(s)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--h", type=float, default=0.456,
                    help="前进用相机高（标定：20 m 轴向 → 93.4%%，见文件头）")
    ap.add_argument("--yaw-from", choices=("wheel", "imu"), default="imu",
                    help="帧间偏航来源（只用于旋转补偿的 dψ）。imu=积分陀螺角速度："
                         "0.1 s 一帧内噪声 0.005°、精确；wheel=/odom 的 yaw：实测横移后"
                         "自己漂 -9.4°，不要用于位移。**绝对位置不再依赖它**（见 ekf_s5.yaml："
                         "视觉只提供体坐标速度 vx,vy，世界系旋转由 EKF 的航向权威负责）")
    ap.add_argument("--cam-x", type=float, default=0.26,
                    help="相机相对车体中心的前向偏移（URDF stereo_left_joint: x=0.26 y=+0.06）")
    ap.add_argument("--cam-y", type=float, default=0.06,
                    help="相机相对车体中心的左向偏移（同上）")
    ap.add_argument("--legacy-flow", action="store_true",
                    help="退回老公式（不补偿旋转、图像空间中位数）。仅用于 A/B 对照："
                         "合成自检里它在纯转 3°/帧 时凭空造出 59 mm/帧、纯前进时也有 "
                         "−5 mm/帧 假侧移，见 scripts/s4_vo_selftest.py")
    ap.add_argument("--no-lateral", action="store_true",
                    help="关掉侧移通道（只测前进）。用于隔离横向漏量：实测 odom 累计路径只差 "
                         "+0.4%%，但位置误差 2.8 m —— 距离对、位置错 = 每帧约 3 mm 的假侧移，"
                         "58 m 直腿累积 1.3 m。若关掉后整条路线更好，说明漏量大于收益。")
    ap.add_argument("--h-lat", type=float, default=0.424,
                    help="侧移用相机高（标定：3 m 横移 → 1.074 修正，见 DECISIONS D13）")
    ap.add_argument("--band", type=float, default=0.40, help="地面带 = 画面底部比例")
    ap.add_argument("--hfov", type=float, default=90.0)
    ap.add_argument("--image-topic", default="/stereo_left_tf")
    ap.add_argument("--imu-topic", default="/imu_tf")
    ap.add_argument("--publish-tf", type=int, default=1)
    a = ap.parse_args()

    rclpy.init()
    node = FloorFlowOdom(a)
    try:
        while rclpy.ok():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
