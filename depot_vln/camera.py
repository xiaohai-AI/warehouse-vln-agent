"""相机模型：深度图 -> 世界系点云（无 ROS 依赖）。

约定（踩过的坑，写清楚省得再错一次）
  * gz 的 RGB-D 深度按**光学系**发布：u 向右、v 向下、z 沿光轴向前（与 CameraInfo 的 K 一致）。
  * 车体系：x 前、y 左、z 上。
  * 相机相对车体有**仰角** `pitch`（xacro 里 `cam_pitch`，正数=抬头）：
    关节 rpy 是 (0, -pitch, 0)，所以 车体 = R_y(-pitch) · 相机。
    第一次实现绕"光轴 y"旋转，结果点全飞到别处（bay 内只落 640 点且 z 恒为 0.29）。

用法
  P_opt = depth_to_points(depth, K)                    # (N,3) 光学系
  P_w   = optical_to_world(P_opt, pose, cam_xyz, pitch) # (N,3) 世界系
"""
from __future__ import annotations

import math

import numpy as np


def depth_to_points(depth: np.ndarray, K: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """深度图 -> (N,3) 光学系点 + (N,2) 像素坐标。无效像素(depth<=0 或非有限)剔除。"""
    h, w = depth.shape
    uu, vv = np.meshgrid(np.arange(w), np.arange(h))
    ok = np.isfinite(depth) & (depth > 0.0)
    z = depth[ok]
    u, v = uu[ok].astype(np.float64), vv[ok].astype(np.float64)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return np.column_stack([x, y, z]), np.column_stack([u, v])


def calibrate_from_floor(depth: np.ndarray, K: np.ndarray,
                         v_lo: float = 0.70, z_max: float = 3.5) -> tuple[np.ndarray, float]:
    """用地面平面自标定相机：返回 (地面法向 n_cam, 相机离地高度 m)。

    为什么不信标称外参：按 URDF 算相机在 base_link 上方 0.284 m，但实测地面到相机
    0.43–0.48 m（漏了 base_link 离地 0.076 m 与模型原点偏移，实测差 ~0.15 m），
    直接套用会让**一半的点落到地面以下**。地面是场景里最大的平面，它的法向就是相机姿态、
    垂直距离就是安装高度 —— 自标定比信标称值稳。
    """
    P, uv = depth_to_points(depth, K)
    v = uv[:, 1]
    sel = (v > depth.shape[0] * v_lo) & (P[:, 2] < z_max) & (P[:, 2] > 0.2)
    Q = P[sel]
    if Q.shape[0] < 200:
        return np.array([0.0, 1.0, 0.0]), 0.35
    mu = Q.mean(axis=0)
    _, _, vt = np.linalg.svd(Q - mu, full_matrices=False)
    n = vt[-1] / np.linalg.norm(vt[-1])
    if n[1] < 0:                       # 法向朝下（图像 v 增大方向）
        n = -n
    return n, float(n @ mu)


def optical_to_world_calibrated(P_opt: np.ndarray, pose, n_cam: np.ndarray, h: float) -> np.ndarray:
    """用自标定结果把光学系点转到世界系。

    n_cam（地面法向，相机系，指向"下"）-> 世界"下"(0,0,-1)：取最小旋转，
    再叠加真值 yaw；相机中心高度 = h。
    """
    up_cam = -np.asarray(n_cam, dtype=np.float64)
    up_world0 = np.array([0.0, 0.0, 1.0])
    # 最小旋转：Rodrigues
    a = np.cross(up_cam, up_world0)
    s = np.linalg.norm(a)
    c = float(up_cam @ up_world0)
    if s < 1e-8:
        R0 = np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    else:
        k = a / s
        Kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        R0 = np.eye(3) + s * Kx + (1 - c) * (Kx @ Kx)
    yaw = float(pose[2])
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    # 相机的"正前方"是光轴 +z_cam；转完后应对准车体 +x。Rz 只负责朝向，
    # 所以这里再绕世界 z 轴把光轴旋到 +x：
    Pw = (Rz @ (R0 @ P_opt.T)).T
    fwd = Rz @ (R0 @ np.array([0.0, 0.0, 1.0]))
    ang = math.atan2(fwd[1], fwd[0])
    ca, sa = math.cos(-ang), math.sin(-ang)
    Rz2 = np.array([[ca, -sa, 0.0], [sa, ca, 0.0], [0.0, 0.0, 1.0]])
    Pw = (Rz2 @ Pw.T).T
    Pw[:, 0] += float(pose[0])
    Pw[:, 1] += float(pose[1])
    Pw[:, 2] += h                       # 地面在 z=0
    return Pw


def optical_to_world(P_opt: np.ndarray, pose, cam_xyz: tuple[float, float, float],
                     pitch_deg: float = 0.0) -> np.ndarray:
    """光学系点 -> 世界系。

    pose = (x, y, yaw)（gz 真值）。cam_xyz = 相机在 base_link 里的位置。
    """
    p = math.radians(pitch_deg)
    Xc, Yc, Zc = P_opt[:, 0], P_opt[:, 1], P_opt[:, 2]
    # 光学系 -> 相机本体（前/左/上）
    fwd, left, up = Zc, -Xc, -Yc
    # 相机本体 -> base_link：R_y(-pitch)（正 pitch = 抬头）
    bx = fwd * math.cos(p) - up * math.sin(p)
    bz = fwd * math.sin(p) + up * math.cos(p)
    by = left
    # base_link -> 世界
    x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
    c, s = math.cos(yaw), math.sin(yaw)
    wx = x + c * (bx + cam_xyz[0]) - s * (by + cam_xyz[1])
    wy = y + s * (bx + cam_xyz[0]) + c * (by + cam_xyz[1])
    wz = bz + cam_xyz[2]
    return np.column_stack([wx, wy, wz])
