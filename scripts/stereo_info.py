#!/usr/bin/env python3
"""给双目相机发 CameraInfo（rtabmap 的 stereo_odometry 必须要它）。

仿真里的 `/stereo_left` 与 `/stereo_right` 只桥了 Image，没有 camera_info。
而这个世界的相机是理想针孔（无畸变、无旋转），所以：

  * 内参直接从 xacro 的 `cam_width/cam_height` + HFOV 1.5708 rad 算：
    fx = fy = (W/2) / tan(HFOV/2)，cx = W/2，cy = H/2 —— 640×480 时 fx = 320；
  * **右目的 P[3]（Tx）= −fx·baseline**，baseline = 0.12 m（左右目各偏 0.06）；
    rtabmap 用这个平移量把视差换成深度，写错就等于尺度错 8 倍（0.12 → 1.0）。
  * distortion 全 0 → "image_rect" 就等于原始图像，不需要 stereo_image_proc。

用法
  .venv/bin/python scripts/stereo_info.py                 # 默认 640×480@10
  .venv/bin/python scripts/stereo_info.py --width 320 --height 240 --rate 5
"""
from __future__ import annotations

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

HFOV = 1.5708            # 与 xacro/urdf 一致：90°
BASELINE = 0.12          # 左右目各偏 ±0.06 m
FRAME_L = "stereo_left_link"
FRAME_R = "stereo_right_link"


class StereoInfo(Node):
    def __init__(self, w: int, h: int, rate: float, image_topic: str = "/stereo_left") -> None:
        self.image_topic = image_topic
        super().__init__("stereo_info")
        try:
            self.declare_parameter("use_sim_time", True)
        except Exception:
            pass
        self.fx = (w / 2.0) / math.tan(HFOV / 2.0)
        self.w, self.h = w, h
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_l = self.create_publisher(CameraInfo, "/stereo_left/camera_info", qos)
        self.pub_r = self.create_publisher(CameraInfo, "/stereo_right/camera_info", qos)
        # **必须跟图像同戳**：rtabmap 用 approx_sync 把 图像+图像+camera_info+camera_info 配对，
        # 如果 camera_info 用节点自己的钟打戳（use_sim_time 没生效时是墙钟 1.7e9），
        # 跟仿真时间戳差 10⁹ 量级，永远配不上 —— 表现成
        #   "stereo_odometry: Did not receive data since 5 seconds!"
        # 所以这里订阅左目图像，收到一帧就照它的 header 发一对 camera_info。
        self.last_stamp = None
        self.create_subscription(Image, self.image_topic, self._on_img, 10)
        # 收到第一帧图像后就不再用定时器发了：同一戳的 camera_info 发两遍会让 rtabmap
        # 组出重复的"伪同步对"，日志刷
        #   "Detected not valid consecutive stamps (previous=X new=X). This new data is ignored"
        # 然后**静默丢掉那一帧**（长路线上白丢一半里程计更新）。
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"stereo_info: {w}x{h} fx=fy={self.fx:.1f} cx={w/2:.1f} cy={h/2:.1f} "
            f"Tx_right={-self.fx * BASELINE:.2f} (baseline {BASELINE} m)")

    def _mk(self, frame: str, tx: float) -> CameraInfo:
        m = CameraInfo()
        m.header.frame_id = frame
        m.width, m.height = self.w, self.h
        m.distortion_model = "plumb_bob"
        m.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        m.k = [self.fx, 0.0, self.w / 2.0, 0.0, self.fx, self.h / 2.0, 0.0, 0.0, 1.0]
        m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        m.p = [self.fx, 0.0, self.w / 2.0, tx,
               0.0, self.fx, self.h / 2.0, 0.0,
               0.0, 0.0, 1.0, 0.0]
        return m

    def _on_img(self, m: Image) -> None:
        self.last_stamp = m.header.stamp
        self._publish(m.header.stamp)     # 每帧图像恰好配一对 camera_info（同戳、不重复）

    def _tick(self) -> None:
        # 只在还没收到图像时兜底（有图之后完全由 _on_img 驱动，保证一帧一对）
        if self.last_stamp is not None:
            return
        self._publish(self.get_clock().now().to_msg())

    def _publish(self, stamp) -> None:
        now = stamp
        left = self._mk(FRAME_L, 0.0)
        right = self._mk(FRAME_R, -self.fx * BASELINE)
        left.header.stamp = now
        right.header.stamp = now
        self.pub_l.publish(left)
        self.pub_r.publish(right)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--rate", type=float, default=10.0)
    ap.add_argument("--image-topic", default="/stereo_left",
                    help="用哪个话题的图像时间戳给 camera_info 打戳（要与 rtabmap 的输入同源）")
    args = ap.parse_args()
    rclpy.init()
    node = StereoInfo(args.width, args.height, args.rate, args.image_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
