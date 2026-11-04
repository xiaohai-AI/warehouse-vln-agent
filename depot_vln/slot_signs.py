#!/usr/bin/env python3
"""货位编号牌（slot sign）的几何与可读性口径 —— 世界的**观测能力**，不是装饰。

# 为什么要有它（设计决定，对应 LIMITATIONS L1）

`verify` 只能回答"这一格有货 / 空"，**不能回答"这是哪一格"** —— 画面里没有编号，
模型无从判断。后果：机器人开到错的格口时，这个错误在物理上**不可观测**，
而 verify 照样会给出一个看起来正确的结论。

按项目原则「物理不可观测就重构世界（而不是放宽判据）」，给每个货位加一块编号牌，
让相机能直接读出 `C-L-08-1`。这样 L1 从「不可用」变成「可测」。

# 尺寸是**反推**出来的，不是拍脑袋

    验证相机  320×240，**HFOV 1.5708 rad（90°）** ← xacro 里三台相机都是这个值
      f_px    = (320/2)/tan(0.7854) = 160.0 px
    观测距离  2.15 m（= 巷道半宽 1.60 + 货位半深 0.55）
      比例    = f_px / 2.15 = 74.4 px/m
    OCR 下限  ~20 px 字高（保守；OCR/VLM 都能过的量级）
      → 字高 ≥ 20/74.4 = 0.269 m

**这里踩过一次**：最初按 69° FOV 算（以为 108.8 px/m），于是把字高定成 0.26 m ——
实测"8 字符宽 5.53×字高"把牌宽撑到 1.45 m 上限，字被压到 0.215 m，只有 **16 px**，
低于门槛。90° 广角 + 320×240 的组合下，牌的**物理尺寸**是硬约束：

    字高 = 牌宽 × 0.92 / 5.53      （5.53 = 8 字符在 Condensed-Bold 下的宽高比）

取牌宽 **2.00 m**（bay 净宽 2.20，两侧立柱各留 0.20）→ 字高 ≈ **0.33 m** → 2.15 m 处 **24.7 px**。

> 想回到现实尺寸（真实货架标牌 0.10–0.15 m）只有一个办法：**抬相机分辨率**。
> 但 verify 的判据是**像素口径**（blob ≥250 px、窗口宽度按 px 定），换分辨率要重跑
> 全部深度存档，属于"改判据"，不能顺手做。

# 竖直位置：必须落在"窗口之上、视野之内"

* verify 的观测窗口上沿 = `deck_z + 0.70`（判据 `ztop0.85` 对 level 1）
  → 牌子压进窗口会把占用判成假阳性，所以牌底从 `deck_z + 0.87` 起（留 0.17 余量）
* 相机高 0.35 m、俯仰 0、2.15 m 处画面上沿 ≈ 1.45 m
  → level 1 牌顶 1.36 m，整块在视野内

# 牌的朝向（一个未定的点，记在这里免得下次重推）

SDF 的 box 六个面共用一张图。gz 的 box UV 通常让每个面"从外面看都是正的"，
所以左/右两侧都用 `yaw=0`。**如果实测某一侧文字是镜像的**，把
`MIRROR_RIGHT_SIDE` 改成 True（右面 yaw 用 π）即可 —— 三角函数的细节不靠猜。
"""
from __future__ import annotations

import math
from typing import Any

# ---------------------------------------------------------------- 牌面尺寸
CAP_M = 0.34          # 目标字高（米）—— 由相机反推；实际会被牌宽钳到 ≈0.33
PLATE_W = 2.00        # 牌宽（米）—— bay 净宽 2.20，两侧立柱各留 0.20
PLATE_H = 0.40        # 牌高（米）
PLATE_T = 0.008       # 牌厚（米）
HANG_W = 0.03         # 吊带宽度（米）
OUT_M = 0.004         # 牌面离货架面的外凸（米）

# 贴图分辨率：比例必须与 PLATE_W/PLATE_H 一致（2.00/0.40 = 5.0），否则字会被拉变形。
# 只做到"够用"：牌在 2.15 m 处只有 149×30 px，2.6× 超采样已经足够清晰。
# 踩过：最初用 800×160 → 576 张独立贴图解码后 **295 MB 显存**，
# 把 GUI 的世界解析拖慢到触发 gz 的双载入竞态（段错误 in CreateLight）。
# 400×80 后显存 ~74 MB、PNG 总量 2.3 MB → 0.6 MB，画面上看不出差别。
TEX_W, TEX_H = 400, 80

MIN_CAP_PX = 20.0     # OCR 下限（判据用）
MIRROR_RIGHT_SIDE = False   # 若实测右面镜像，改 True（见文件头）

# 观测位到货架面的距离由 layout 给出（obs_distance），这里只做缺省
DEFAULT_OBS_DIST = 2.15
CAM_W, CAM_H = 320, 240
CAM_HFOV = 1.5708     # rad，与 xacro 的 <horizontal_fov> 完全一致（90° 广角）


# ---------------------------------------------------------------- 相机口径
def px_per_m(distance: float, cam_w: int = CAM_W, hfov: float = CAM_HFOV) -> float:
    """给定距离，画面上的像素/米。"""
    f_px = (cam_w / 2.0) / math.tan(hfov / 2.0)
    return f_px / max(1e-6, distance)


def cap_px(distance: float, cap_m: float = CAP_M, **kw: Any) -> float:
    return cap_m * px_per_m(distance, **kw)


def f_px(cam_w: int = CAM_W, hfov: float = CAM_HFOV) -> float:
    return (cam_w / 2.0) / math.tan(hfov / 2.0)


# ---------------------------------------------------------------- 命名
def visual_name(addr: str, kind: str = "slot") -> str:
    """`C-L-08-1` → `sign_slot_C_L_08_1_vis`（SDF 名里不能带连字符）。"""
    return f"sign_{kind}_{addr.replace('-', '_')}_vis"


def addr_from_visual(name: str) -> str | None:
    """`sign_slot_C_L_08_1_vis` → `C-L-08-1`；不是牌面就返回 None。"""
    if not name.startswith("sign_slot_") or not name.endswith("_vis"):
        return None
    parts = name[len("sign_slot_"):-len("_vis")].split("_")
    if len(parts) != 4:
        return None
    a, s, bay, lvl = parts
    return f"{a}-{s}-{int(bay):02d}-{int(lvl)}"


def texture_relpath(addr: str) -> str:
    return f"slot_labels/{addr}.png"


# ---------------------------------------------------------------- 几何
def band_z(deck_z: float) -> tuple[float, float]:
    """牌面的 z 区间（底, 顶）。底 = deck + 0.87（窗口上沿 0.70 之上留 0.17）。"""
    z0 = round(deck_z + 0.87, 4)
    return z0, round(z0 + PLATE_H, 4)


def support_z(deck_z: float, levels: list[dict[str, Any]], top_z: float) -> float:
    """牌子上方能挂靠的构件高度：下一层层板下的横梁底面；顶层则挂到顶横梁。"""
    higher = [float(l["deck_z"]) for l in levels if float(l["deck_z"]) > deck_z + 1e-6]
    if higher:
        return round(min(higher) - 0.15, 4)      # deck_thickness 0.05 + beam 0.10
    return round(top_z, 4)


def sign_geometry(slot: dict[str, Any], levels: list[dict[str, Any]], top_z: float) -> dict[str, Any]:
    """由 slot 记录（来自 world/slot_grid.json）算出牌面/吊带的位姿与尺寸。"""
    px, py = float(slot["pose"]["x"]), float(slot["pose"]["y"])
    ox, oy = float(slot["obs_pose"]["x"]), float(slot["obs_pose"]["y"])
    depth = float(slot.get("slot_depth") or 0.55)   # 半深货位的进深（rack_d/2）
    deck_z = float(slot["deck_z"])

    # 从货位中心指向**本侧巷道**的方向。
    # slot 的 pose.y 是"半深货位"的中点，所以货架面在 py ± depth/2（不是 ± depth）。
    # 踩过一次：写成 ± depth 得到 -0.825，而 rack_block_3 的下表面在 -0.55。
    s = 1.0 if (oy - py) > 0 else -1.0
    y_face = py + s * depth / 2
    # OUT_M 是**外表面**离货架面的距离；牌厚 PLATE_T，所以中心要比外表面回退半个厚度
    # （结果恰好落在货架面上：牌子一半嵌进横梁，看起来像贴上去的）
    y_plate = y_face + s * (OUT_M - PLATE_T / 2)

    z0, z1 = band_z(deck_z)
    zw = support_z(deck_z, levels, top_z)
    hang_h = max(0.0, round(zw - z1, 4))
    yaw = 0.0
    if abs(oy - py) < 1e-9:
        yaw = 0.0
    elif MIRROR_RIGHT_SIDE and s > 0:
        yaw = math.pi

    return {
        "plate_center": [px, y_plate, round((z0 + z1) / 2, 4)],
        "plate_size": [PLATE_W, PLATE_T, PLATE_H],
        "hang_center": [px, y_plate, round(z1 + hang_h / 2, 4)] if hang_h > 1e-6 else None,
        "hang_size": [HANG_W, PLATE_T, hang_h] if hang_h > 1e-6 else None,
        "yaw": yaw,
        "z_band": [z0, z1],
        "y_face": round(y_face, 4),
        "support_z": zw,
        "obs_distance": float(slot.get("obs_distance") or DEFAULT_OBS_DIST),
    }
