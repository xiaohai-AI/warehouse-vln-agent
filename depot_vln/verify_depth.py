"""深度门限 verify：把"目标格里有什么"从"画面中央有什么"里分离出来。

为什么必须这么做（`docs/TROUBLESHOOTING.md §33.3` 的实测）
  货架是**通透的**（没有背板）。空的目标格背后 2.2 m 处、隔壁巷的货会**投影到同一个像素**
  —— `C-L-08-1`（空）那一帧里，`cargo_D_R_08_1`（2.21 m）恰好投在 (167,120)，而 cx,cy=(160,120)。
  于是"看一眼画面中央"得到的结果是背景的货，VLM 报 `occupied, carton_s, conf 0.8` —— 假阳性。
  之前 3 次"判对了"只是因为目标有货时目标在最前面**遮住**了背景，属遮挡顺序的运气。

做法（纯几何 + 双目深度，不依赖 VLM）
  1. 取目标格**开口面**上一块**窄窗口**（默认 ±0.70 m）按位姿投影成图像四边形；
  2. 双目视差给深度 `Z = f*B/disp`（B = 0.12 m 基线，f = 160 px）；
  3. 统计窗口内（再收缩 82% 去掉格框）**比格口更近**（`Z < d_front + 0.40`）的像素所构成的
     **最大连通块**：空格里只剩贴边散落的板条端面，拼不成块（≤133 px）；有货 → 一整块面
     （≥545 px）。用"连通块"而不是"像素总数"，是因为位姿误差会让阈值整体平移，
     散落像素会漏检、而整块面不会（见 `depth_gate_verdict()` 的说明）；
  4. 深度不成立 → `unseen`：看不见就说看不见，不许猜。

**为什么窗口不是整格 2.4 m**（用 `results/depth_dumps/*.npz` 离线批处理得出）
  整格宽时，四边形平移 14 px（≈位姿偏 4°）就把**隔壁格的货**框了进来：空位 0 → 535 px 假阳性。
  收到 ±0.70 m 后，15 种扰动（du=±7/±14 px、dv=±8 px）下空位恒为 **0 像素**、有货最低 **186 px**。
  → 窗口窄于格宽是"抗位姿误差"的关键设计，不是随手取的参数。

**一条判据性推论**（写进报告）：**空位应当输出"看不见货"，而不是"看见了背景"。**
"""
from __future__ import annotations

import math
from typing import Any, Sequence

# 相机（robot/depot_bot.urdf.xacro）：前伸 body_x/2=0.26、高 cam_z=0.35、双目 y=±0.06
F_PX = 160.0          # f = (320/2)/tan(HFOV/2), HFOV=90°
CX, CY = 160.0, 120.0
W_IMG, H_IMG = 320, 240
CAM_FWD = 0.26        # 相机在车体系的前伸
CAM_Y = 0.06          # 左相机在 +y（右相机 -y），基线 = 0.12 m
CAM_Z = 0.35
BASELINE = 0.12

# 观测窗口的竖直带 —— 注意它是**窗口**，不是货位的净空（clear_height 1.6 m）。
# 下沿 = 层板面；上沿 = 层板面 + 0.70。为什么不是 1.15：
#   上沿取 1.15 时，窗口正好顶到"空位正上方那根横梁"在像面上的位置；
#   而窗口是**在开口平面上**投出的四边形，横梁在开口平面**之后**（实测深度 1.6 m、
#   像面行 34–40、z≈1.18 m），靠视差会落进窗口内 → 空位被误判成有货
#   （2026-11-03 实跑 456 px，正是这个机制；`bbox=(120,34,255,40)` 贴在窗口最上沿）。
# 压到 0.85 后：**正常工况 20 份存档 20/20**（原 19/20），且 5 个正例的块全部变大
#   （554→689 / 545→721 / 1266→1455 / 1461→1648 / 1489→1769 px），因为质心下移、
#   按质心收缩时不再啃掉货的底边。见 docs/DECISIONS.md D27 续。
LEVEL1_Z = (0.15, 0.85)
BAY_W = 2.4           # 站台间距 = 一个格的宽度（仅作参考，观测窗口用 WIN_HALF_W）
WIN_HALF_W = 0.70     # 观测窗口水平半宽（见文件头：为什么不用整格宽）

# 判据常数：由 4 个真样本 + 3 个 **odom 位姿** 样本（7 份存档）离线扫描定。
# 空位最大连通块 ≤133 px、有货 ≥545 px（间隔 412 px，4.1 倍）→ 门槛 250 px 留两边余量。
PROT_NEAR = 0.40      # "比格口更近"的余量（m）：要容得下 odom 位姿误差（实测 0.27 m）
PROT_MIN_PX = 250     # 最大连通块的绝对像素门槛
PROT_MIN_FRAC = 0.010 # 连通块占窗口内有效像素的比例门槛

# 判据版本号：写进 demo 产物，让"旧判据跑出来的结论"不会被新判据的统计口径混进判据
# （本轮就发生了：`near=0.15+像素总数` 判据在 odom 位姿下漏检一次，改判据后那条记录
#  仍是失败记录，不能被新规则的 7/7 掩盖掉）。
RULE_ID = "depth/near0.40+blob250/win0.70/inner0.82/ztop0.85"
INNER_K = 0.82        # 窗口收缩比（去掉贴边的格框/板条）
MIN_VALID = 200       # 少于这么多有效深度像素 → unseen


def cam_world(robot: dict[str, float], side: str = "L") -> dict[str, float]:
    """车体位姿 → 相机位姿（水平面）。`side` 只影响相机在车体的左右偏移。"""
    x, y, yaw = float(robot["x"]), float(robot["y"]), float(robot["yaw"])
    dy_lat = CAM_Y if side == "L" else -CAM_Y
    return {
        "x": x + CAM_FWD * math.cos(yaw) - dy_lat * math.sin(yaw),
        "y": y + CAM_FWD * math.sin(yaw) + dy_lat * math.cos(yaw),
        "yaw": yaw,
        "z": CAM_Z,
    }


def project(pt: Sequence[float], cam: dict[str, float]) -> tuple[float, float, float] | None:
    """世界点 → (u, v, 前向距离)。在相机后面返回 None。"""
    dx, dy, dz = pt[0] - cam["x"], pt[1] - cam["y"], pt[2] - cam["z"]
    cy_, sy_ = math.cos(cam["yaw"]), math.sin(cam["yaw"])
    fwd = dx * cy_ + dy * sy_                 # 沿相机光轴
    left = -dx * sy_ + dy * cy_               # 相机左侧
    if fwd <= 0.10:
        return None
    u = CX - F_PX * left / fwd                # 左侧 → 更小的 u
    v = CY - F_PX * dz / fwd                  # 更高 → 更小的 v
    return u, v, fwd


def slot_opening(slot: dict[str, Any], cam: dict[str, float],
                 half_w: float = WIN_HALF_W) -> dict[str, Any] | None:
    """把目标格**开口面窄窗口**投影成图像四边形，并给出格口深度。

    `slot` 来自 `world/slot_grid.json`：{pose:{x,y,z,yaw}, side, slot_depth, ...}
    开口面 = 朝着巷道那一侧的面（L 面格子在 +y，开口朝 −y；R 面相反）。
    """
    pose = slot.get("pose") or {}
    if not pose:
        return None
    sx, sy = float(pose["x"]), float(pose["y"])
    depth = float(slot.get("slot_depth") or 0.55)
    side = slot.get("side") or "L"
    sign = -1.0 if side == "L" else 1.0        # L 面：开口在 −y 一侧
    y_front = sy + sign * depth / 2.0
    z0, z1 = LEVEL1_Z
    corners3d = [
        (sx - half_w, y_front, z0), (sx + half_w, y_front, z0),
        (sx + half_w, y_front, z1), (sx - half_w, y_front, z1),
    ]
    quad, fwds = [], []
    for c in corners3d:
        p = project(c, cam)
        if p is None:
            return None
        quad.append((p[0], p[1]))
        fwds.append(p[2])
    d_front = min(fwds)
    return {"quad": quad, "d_front": d_front, "d_back": d_front + depth,
            "z_band": (z0, z1), "side": side, "half_w": half_w, "win_w_m": 2 * half_w}


def shrink_quad(quad: Sequence[tuple[float, float]], k: float = INNER_K):
    """把四边形按质心收缩 k 倍：剔掉格的边框/立柱/板条端面（它们永远贴着边）。"""
    cx = sum(p[0] for p in quad) / len(quad)
    cy = sum(p[1] for p in quad) / len(quad)
    return [(cx + (u - cx) * k, cy + (v - cy) * k) for u, v in quad]


def quad_mask(quad: Sequence[tuple[float, float]], w: int = W_IMG, h: int = H_IMG):
    """凸四边形 → 布尔掩膜（射线法，逐像素；320×240 规模足够快，不引入 cv2 依赖）。"""
    mask = [[False] * w for _ in range(h)]
    n = len(quad)
    ymin = max(0, int(math.floor(min(p[1] for p in quad))))
    ymax = min(h - 1, int(math.ceil(max(p[1] for p in quad))))
    for v in range(ymin, ymax + 1):
        xs = []
        yc = v + 0.5
        for i in range(n):
            x1, y1 = quad[i]
            x2, y2 = quad[(i + 1) % n]
            if (y1 <= yc < y2) or (y2 <= yc < y1):
                xs.append(x1 + (yc - y1) * (x2 - x1) / (y2 - y1))
        xs.sort()
        for a, b in zip(xs[0::2], xs[1::2]):
            for u in range(max(0, int(math.ceil(a))), min(w - 1, int(math.floor(b))) + 1):
                mask[v][u] = True
    return mask


def largest_blob(mask: Sequence[Sequence[bool]]) -> tuple[int, tuple[int, int, int, int] | None]:
    """最大 4-连通块（纯 Python BFS）。诊断用：看"近处的东西"是一整块还是几条边。"""
    h = len(mask)
    w = len(mask[0]) if h else 0
    seen = [[False] * w for _ in range(h)]
    best: tuple[int, tuple[int, int, int, int] | None] = (0, None)
    for v0 in range(h):
        row = mask[v0]
        for u0 in range(w):
            if not row[u0] or seen[v0][u0]:
                continue
            stack = [(v0, u0)]
            seen[v0][u0] = True
            area = 0
            u_min = u_max = u0
            v_min = v_max = v0
            while stack:
                v, u = stack.pop()
                area += 1
                u_min, u_max = min(u_min, u), max(u_max, u)
                v_min, v_max = min(v_min, v), max(v_max, v)
                for dv, du in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    vv, uu = v + dv, u + du
                    if 0 <= vv < h and 0 <= uu < w and mask[vv][uu] and not seen[vv][uu]:
                        seen[vv][uu] = True
                        stack.append((vv, uu))
            if area > best[0]:
                best = (area, (u_min, v_min, u_max, v_max))
    return best


def depth_gate_verdict(
    depth: Sequence[Sequence[float]],
    opening: dict[str, Any],
    *,
    near: float = PROT_NEAR,
    min_px: int = PROT_MIN_PX,
    min_frac: float = PROT_MIN_FRAC,
    inner_k: float = INNER_K,
    min_valid: int = MIN_VALID,
) -> dict[str, Any]:
    """判决：occupied / empty / unseen（无效深度用 NaN、inf 或 <=0 表示）。

    判据：窗口内**比格口更近**（`Z < d_front + near`）的像素构成的**最大连通块**。
      · 空位：格内是空的、通透，看到的是**背面以远**的背景；剩下的只有贴边散落的板条
        端面，拼不成块 → ≤133 px（7 份存档实测）；
      · 有货：货的正面是一整块面 → ≥545 px。
    只统计收缩 82% 的内区；深度不成立 → `unseen`，不猜。

    **为什么用连通块而不是像素总数、为什么 near=0.40**（都是实测逼出来的）：
      · 用总数时，`near=0.15` 在 odom 位姿误差 0.27 m 的那一格上漏检（阈值随 `d_front`
        平移了 0.19 m，货的正面 1.371 m 落到阈值外 → 0 px）。散落像素与整块面在
        "放宽余量"后仍能分开，靠的就是连通性。
      · `near` 放宽到 0.40 m 仍安全：通透货架的背景在格口之后 0.9–1.2 m，进不来。
    """
    quad = opening["quad"]
    d_front = opening["d_front"]
    # **必须同时吃 list-of-list 和 numpy 数组**：真实链路上传进来的是 np.ndarray，
    # 而 `if depth:` 对 ndarray 会抛 "truth value of an array is ambiguous"
    # （踩过：自检喂 list 通过、真跑到 verify 才炸，白跑两轮仿真）。
    h = int(depth.shape[0]) if hasattr(depth, "shape") else len(depth)
    if hasattr(depth, "shape") and len(depth.shape) > 1:
        w = int(depth.shape[1])
    else:
        w = len(depth[0]) if h else W_IMG
    inner = quad_mask(shrink_quad(quad, inner_k), w, h)
    thr = d_front + near
    inner_px = valid = near_px = 0
    near_mask = [[False] * w for _ in range(h)]
    zs: list[float] = []
    for v in range(h):
        row, irow = depth[v], inner[v]
        for u in range(w):
            if not irow[u]:
                continue
            inner_px += 1
            z = row[u]
            if not (z == z) or z in (math.inf, -math.inf) or z <= 0.0:
                continue
            valid += 1
            if z < thr:
                near_px += 1
                near_mask[v][u] = True
                zs.append(float(z))
    if valid < min_valid:
        return {"verdict": "unseen", "inner_px": inner_px, "valid": valid, "near_px": 0,
                "near_frac": None, "blob": 0, "d_front": round(d_front, 3),
                "win_w_m": opening.get("win_w_m"),
                "why": f"窗口内有效深度只有 {valid} 像素（<{min_valid}），看不见就不猜"}
    area, bbox = largest_blob(near_mask)
    frac = area / valid
    verdict = "occupied" if (area >= min_px and frac >= min_frac) else "empty"
    zs.sort()
    return {"verdict": verdict, "inner_px": inner_px, "valid": valid, "near_px": near_px,
            "near_frac": round(frac, 4), "blob": area, "bbox": bbox,
            "d_front": round(d_front, 3), "win_w_m": opening.get("win_w_m"),
            "z_med_near": (round(zs[len(zs) // 2], 3) if zs else None),
            "why": f"比格口({d_front:.2f}m+{near:.2f}m)更近的最大连通块 {area} px"
                   f"（共 {near_px} px，占 {frac:.1%}；门槛 {min_px} px / {min_frac:.1%}）"}


# ---------------------------------------------------------------- 自检
def _synthetic(fill: float, block: tuple[int, int, int, int] | None = None,
               block_z: float | None = None) -> list[list[float]]:
    img = [[fill] * W_IMG for _ in range(H_IMG)]
    if block is not None and block_z is not None:
        v0, v1, u0, u1 = block
        for v in range(v0, v1):
            for u in range(u0, u1):
                img[v][u] = block_z
    return img


def selftest() -> int:
    """合成用例（含真实链路的 numpy 类型）+ 边框板条用例，确认判决符合物理直觉。"""
    # 相机在巷道上、正对 L 面格子：格子在 +y，开口面 y=-0.55，中心 x=0
    robot = {"x": 0.0, "y": -2.15, "yaw": math.pi / 2}
    cam = cam_world(robot, "L")
    slot = {"pose": {"x": 0.0, "y": -0.275, "z": 0.45}, "side": "L", "slot_depth": 0.55}
    op = slot_opening(slot, cam)
    assert op is not None, "投影失败"
    d_front = op["d_front"]
    us = [p[0] for p in op["quad"]]
    vs = [p[1] for p in op["quad"]]
    u0, u1 = int(min(us)), int(max(us))
    v0, v1 = int(min(vs)), int(max(vs))
    ci, cj = (u0 + u1) // 2, (v0 + v1) // 2
    hw = max(6, int((u1 - u0) * INNER_K / 2 * 0.6))
    hh = max(6, int((v1 - v0) * INNER_K / 2 * 0.6))
    center_block = (cj - hh, cj + hh, ci - hw, ci + hw)
    edge_strip = (v0, v0 + 5, u0, u1)          # 贴着上边的板条（收缩后被剔除）

    cases = [
        ("空位：窗口内全是背景（背面以远）", _synthetic(d_front + 1.2), "empty"),
        ("贴着四边形边的板条（收缩后剔除）",
         _synthetic(d_front + 1.2, edge_strip, d_front - 0.10), "empty"),
        ("散落的薄条：宽 2 px 的行 + 列（放宽余量后仍不成块）",
         _synthetic(d_front + 1.2, (cj - 1, cj + 1, ci - 40, ci + 40), d_front + 0.05), "empty"),
        ("有货：格口中部一大块就在格口面附近",
         _synthetic(d_front + 1.2, center_block, d_front + 0.05), "occupied"),
        ("有货但货凸出格口 0.2 m",
         _synthetic(d_front + 1.2, center_block, d_front - 0.20), "occupied"),
        ("深度不成立（NaN）", [[float("nan")] * W_IMG for _ in range(H_IMG)], "unseen"),
    ]
    # 真实链路的输入类型：numpy float32 + NaN（上面几组是 list，容易掩盖类型 bug）
    try:
        import numpy as np

        arr = np.full((H_IMG, W_IMG), d_front + 1.2, dtype=np.float32)
        arr[center_block[0]:center_block[1], center_block[2]:center_block[3]] = d_front + 0.05
        cases.append(("numpy float32 输入（真实链路类型）+ 有货", arr, "occupied"))
        cases.append(("numpy 全 NaN", np.full((H_IMG, W_IMG), np.nan, dtype=np.float32),
                      "unseen"))
    except ImportError:
        print("[skip] 没有 numpy，跳过真实类型用例")

    print(f"投影：窗口 ±{op['half_w']} m，格口面 d_front={d_front:.2f} m，"
          f"四边形 u∈[{u0},{u1}] v∈[{v0},{v1}]")
    bad = 0
    for name, depth, want in cases:
        got = depth_gate_verdict(depth, op)
        ok = got["verdict"] == want
        bad += 0 if ok else 1
        print(f"[{'PASS' if ok else 'FAIL'}] {name:<38} → {got['verdict']:<9} "
              f"near={got['near_px']}（{got['why']}）")
    print()
    print(f"结论：{'PASS' if bad == 0 else 'FAIL'} —— {len(cases) - bad}/{len(cases)} 项符合预期")
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    import sys

    sys.exit(selftest())
