"""传感器外参与坐标变换（无 ROS 依赖，离线分析也能 import）。

为什么单独一个模块：`scripts/capture_cloud.py` 顶部 import rclpy，
离线分析脚本（如 seg1_bay_grid）不能用它的函数；而坐标口径一旦分叉两处实现，
就会出现"单帧云是雷达系、累积云是世界系"这种难以察觉的错。
"""
from __future__ import annotations

import math

import numpy as np

# 雷达在 base_link 里的位置（xacro: lidar_z=0.55, base_link 原点在轮轴高度 zr=0.076）
SENSOR_XYZ: tuple[float, float, float] = (-0.10, 0.0, 0.55 - 0.076)

# 自击点半径：雷达能看到自己的底盘/轮子（就在它下方 0.3–0.6 m），
# 实测这是点云里最大的单个 z 簇，会把"巷道中间为空"的判别器破坏掉。
SELF_R = 0.85


def to_world(pts: np.ndarray, pose) -> np.ndarray:
    """雷达系 -> 世界系。

    pose = (x, y, yaw)，来自 gz 真值 `/world/depot/pose/info`。
    世界 z **不做地面标定**（不同来源的 z 偏移不一样），分析端自己标定。
    """
    x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
    c, s = math.cos(yaw), math.sin(yaw)
    sx, sy, sz = SENSOR_XYZ
    if pts.size == 0:
        return np.zeros((0, 3), np.float32)
    bx, by = pts[:, 0] + sx, pts[:, 1] + sy
    return np.column_stack([x + c * bx - s * by,
                            y + s * bx + c * by,
                            pts[:, 2] + sz]).astype(np.float32)
