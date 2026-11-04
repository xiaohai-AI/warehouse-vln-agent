#!/usr/bin/env python3
"""生成**覆盖全场、处处唯一**的地面纹理 world/textures/floor_unique.png。

为什么需要它（S3 视觉里程计的根因）：
  相机朝巷道前进时，货架每隔 2.4 m 完全一样（同一张贴图平铺），画面沿光轴近似"平移不变"，
  特征匹配到任意一格都成立 —— 实测视觉里程计对**横向运动还原 97–104%，
  对沿巷前进只还原 0~3%**。这不是特征强度问题（换 SIFT/帧到帧无效），
  而是景物沿该方向周期性重复这个事实。按项目原则（"物理不可观测就重构世界"）
  要给地面加上**非周期**的视觉线索。

为什么只改地面、不改几何：
  SDF 里地面是一个覆盖全场的 box，而 box 面的 UV 是 0..1 —— 一张图正好铺满整个地面。
  所以只要把这张图换成"处处不一样"的大图，地面就有了全局唯一的视觉指纹，
  而 collision 一个字节都不用动（视觉与碰撞在 SDF 里本来就是分开的）。

为什么对 UV 朝向不敏感：
  box 面 UV 的原点/翻转方向不确定，所以：
    * 随机污渍/裂缝/补丁 —— 朝向无关；
    * 巷道地标线沿 x 走向、且 6 条巷道关于 y=0 对称 —— y 翻转后仍落在同一条巷道上；
      x 翻转不改变"沿 x 的线"。因此无论 UV 怎么翻，地标线都对得上货架。

自检（不过就非零退出）：
  1. 梯度中位 > 5（有足够特征，和 make_textures.py 同一口径）
  2. **唯一性**：随机取 200 个 0.6 m 见方的块，在 5 m 之外找最相似的另一块，
     NCC 最大值必须 < 0.90（否则说明仍有周期性，视觉还是分不清"走了多远"）
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PX_PER_M = 44          # ≈2.3 cm/px：66 m 宽地面 -> 约 2930 px
SEED = 20261026        # 与 slot_grid 同一个种子，可复现


def _value_noise(h: int, w: int, cell_px: int, rng: np.random.Generator) -> np.ndarray:
    """低频值噪声：小网格随机 + 双三次放大，用来做混凝土的大尺度色差。"""
    ch = max(2, h // cell_px + 2)
    cw = max(2, w // cell_px + 2)
    small = rng.random((ch, cw)).astype(np.float32)
    return cv2.resize(small, (w, h), interpolation=cv2.INTER_CUBIC)


def _soft_blob(h: int, w: int, cx: float, cy: float, rx: float, ry: float) -> np.ndarray:
    """软边椭圆掩膜（0..1），做污渍/油渍的柔和边缘。"""
    yy, xx = np.ogrid[:h, :w]
    d = ((xx - cx) / max(rx, 1.0)) ** 2 + ((yy - cy) / max(ry, 1.0)) ** 2
    return np.clip(1.4 - d, 0.0, 1.0).astype(np.float32)


def build(footprint: dict, aisles: list[dict], px_per_m: int, seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    x_min, x_max = footprint["x_min"], footprint["x_max"]
    y_min, y_max = footprint["y_min"], footprint["y_max"]
    sx, sy = x_max - x_min, y_max - y_min
    w, h = int(round(sx * px_per_m)), int(round(sy * px_per_m))

    def to_px(x: float, y: float) -> tuple[int, int]:
        return int(round((x - x_min) * px_per_m)), int(round((y - y_min) * px_per_m))

    # --- 1) 混凝土底：三档尺度噪声叠加（0.4 m / 2.5 m / 12 m）
    base = np.full((h, w, 3), (0.52, 0.51, 0.49), np.float32)
    for cell_m, amp in ((0.10, 0.035), (0.4, 0.045), (2.5, 0.075), (12.0, 0.10)):
        n = _value_noise(h, w, max(2, int(cell_m * px_per_m)), rng) - 0.5
        base += (n * amp)[..., None]
    # 细颗粒
    base += (rng.standard_normal((h, w, 1)).astype(np.float32) * 0.022)
    img = np.clip(base, 0, 1)

    # --- 2) 补丁（修补过的混凝土：色调略变 + 深色勾边），非周期
    for _ in range(46):
        pw = int(rng.uniform(0.8, 4.5) * px_per_m)
        ph = int(rng.uniform(0.8, 4.5) * px_per_m)
        x0 = int(rng.integers(0, max(1, w - pw)))
        y0 = int(rng.integers(0, max(1, h - ph)))
        tone = rng.uniform(-0.06, 0.07)
        img[y0:y0 + ph, x0:x0 + pw] += tone
        cv2.rectangle(img, (x0, y0), (x0 + pw, y0 + ph),
                      tuple(float(v) for v in img[y0, x0] * 0.75), 2)

    # --- 3) 油渍/水渍：软边暗斑
    for _ in range(320):
        cx, cy = rng.uniform(0, w), rng.uniform(0, h)
        r = rng.uniform(0.08, 1.4) * px_per_m
        m = _soft_blob(h, w, cx, cy, r, r * rng.uniform(0.5, 1.8))
        img -= (m * rng.uniform(0.05, 0.22))[..., None]

    # --- 4) 裂缝：随机游走折线（细、深、带分叉）
    for _ in range(130):
        px, py = rng.uniform(0, w), rng.uniform(0, h)
        ang = rng.uniform(0, 2 * np.pi)
        pts = [(px, py)]
        for _ in range(int(rng.integers(8, 40))):
            ang += rng.normal(0, 0.45)
            step = rng.uniform(0.05, 0.25) * px_per_m
            px += np.cos(ang) * step
            py += np.sin(ang) * step
            if not (0 <= px < w and 0 <= py < h):
                break
            pts.append((px, py))
        if len(pts) > 2:
            cv2.polylines(img, [np.array(pts, np.int32)], False,
                          (0.12, 0.12, 0.13), thickness=int(rng.integers(1, 3)))

    # --- 5) 轮胎痕：沿巷道的深色拖痕（长度随机 -> 仍然非周期）
    for aisle in aisles:
        cy = aisle["center_y"]
        for _ in range(24):
            x0 = rng.uniform(x_min + 2, x_max - 6)
            L = rng.uniform(1.0, 7.0)
            off = rng.uniform(-1.1, 1.1)
            for sgn in (-1, 1):
                p0 = to_px(x0, cy + off - 0.18 * sgn)
                p1 = to_px(x0 + L, cy + off - 0.18 * sgn)
                cv2.line(img, p0, p1, (0.30, 0.29, 0.28), thickness=int(0.06 * px_per_m) or 1)

    # --- 6) 巷道地标线（黄）+ bay 刻度（白）：真实仓库的地面标识
    #     沿 x 走向、且巷道关于 y=0 对称 -> 对 UV 翻转不敏感
    yellow = (0.86, 0.72, 0.15)
    white = (0.88, 0.88, 0.86)
    for aisle in aisles:
        cy, half = aisle["center_y"], aisle["width"] / 2.0
        for sgn in (-1, 1):
            y = cy + sgn * (half - 0.16)
            p0, p1 = to_px(aisle["x0"], y), to_px(aisle["x1"], y)
            thick = max(2, int(0.10 * px_per_m))
            cv2.line(img, p0, p1, yellow, thick)
            # 磨损：随机断口，让线本身也不是完美周期
            for _ in range(18):
                xa = rng.uniform(aisle["x0"] + 1, aisle["x1"] - 1)
                xb = xa + rng.uniform(0.15, 1.2)
                cv2.line(img, to_px(xa, y), to_px(min(xb, aisle["x1"]), y),
                         tuple(float(v) for v in img[max(0, p0[1]), 0] * 1.0), thick + 1)
        # bay 刻度：每 2.4 m 一小段白线，垂直巷道
        x = aisle["x0"]
        while x <= aisle["x1"]:
            for sgn in (-1, 1):
                y = cy + sgn * (half - 0.16)
                cv2.line(img, to_px(x - 0.05, y - 0.22), to_px(x + 0.05, y + 0.22), white, 3)
            x += aisle.get("pitch", 2.4)

    # --- 7) 细颗粒（角点用）：GFTT 检测的是角点，光滑混凝土本来角点就少，
    #        所以这里要**密集的小尺寸斑点**（1–3 px ≈ 2–7 cm），不是稀疏的大污渍。
    #        实测 14000 颗时地面在整幅图里占 43% 面积却给不出足够特征，VO 仍靠远处货架。
    for _ in range(120000):
        cx, cy = int(rng.integers(0, w)), int(rng.integers(0, h))
        v = float(rng.uniform(0.30, 0.78))
        cv2.circle(img, (cx, cy), int(rng.integers(1, 3)), (v, v, v * 0.98), -1)
    # 小块漆面剥落 / 胶带残留 / 小坑（0.03–0.25 m 的四边或圆点，也是角点来源）
    for _ in range(3200):
        cx, cy = int(rng.integers(0, w)), int(rng.integers(0, h))
        s = int(rng.uniform(0.03, 0.25) * px_per_m)
        v = float(rng.uniform(0.22, 0.85))
        if rng.random() < 0.5:
            cv2.rectangle(img, (cx, cy), (cx + max(2, s), cy + max(2, int(s * 0.6))), (v, v, v), -1)
        else:
            cv2.circle(img, (cx, cy), max(2, s), (v, v, v), -1)

    return np.clip(img, 0, 1)


def self_check(img: np.ndarray, px_per_m: int, seed: int) -> tuple[bool, str]:
    """① 特征足够 ② 5 m 之外找不到相似的**局部结构**（唯一性）。

    唯一性必须在**高通**图像上判：视觉特征（GFTT/BRIEF、ORB、SIFT）用的是局部梯度结构，
    而原始灰度会被"大家都是灰色混凝土"的低频分量主导，NCC 天然偏高（实测 0.935 全是
    低频色调造成的假相似）。所以先减掉大尺度模糊，只比局部纹理。
    """
    g = cv2.cvtColor((img * 255).astype(np.uint8), cv2.COLOR_RGB2GRAY).astype(np.float32)
    g = g - cv2.GaussianBlur(g, (0, 0), sigmaX=0.25 * px_per_m)     # 去掉 >~0.5 m 的低频
    gx = np.abs(cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3))
    gy = np.abs(cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3))
    grad_med = float(np.median(np.maximum(gx, gy)))
    ok_feat = grad_med > 5.0

    h, w = g.shape
    rng = np.random.default_rng(seed + 7)
    side = int(round(0.6 * px_per_m))
    excl = 5.0 * px_per_m
    worst = 0.0
    n = 200
    for _ in range(n):
        x0 = int(rng.integers(0, max(1, w - side)))
        y0 = int(rng.integers(0, max(1, h - side)))
        patch = g[y0:y0 + side, x0:x0 + side]
        if patch.std() < 1e-3:
            continue
        for _ in range(50):
            x1 = int(rng.integers(0, max(1, w - side)))
            y1 = int(rng.integers(0, max(1, h - side)))
            if abs(x1 - x0) < excl and abs(y1 - y0) < excl:
                continue
            other = g[y1:y1 + side, x1:x1 + side]
            a = patch - patch.mean()
            b = other - other.mean()
            d = (np.linalg.norm(a) * np.linalg.norm(b))
            if d < 1e-6:
                continue
            ncc = float((a * b).sum() / d)
            worst = max(worst, ncc)
    ok_uniq = worst < 0.90
    msg = (f"梯度中位 {grad_med:.1f}（>5）  "
           f"5 m 外最相似块 NCC {worst:.3f}（<0.90）")
    return (ok_feat and ok_uniq), msg


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "world" / "textures" / "floor_unique.png"))
    ap.add_argument("--grid", default=str(ROOT / "world" / "slot_grid.json"))
    ap.add_argument("--px-per-m", type=int, default=0)
    ap.add_argument("--seed", type=int, default=SEED)
    args = ap.parse_args()

    grid = json.loads(Path(args.grid).read_text(encoding="utf-8"))
    ppm = args.px_per_m or PX_PER_M
    img = build(grid["footprint"], grid["aisles"], ppm, args.seed)
    ok, msg = self_check(img, ppm, args.seed)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    bgr = cv2.cvtColor((img * 255).astype(np.uint8), cv2.COLOR_RGB2BGR)
    cv2.imwrite(str(out), bgr)
    h, w = img.shape[:2]
    print(f"[floor] {w}×{h} px  ≈{ppm} px/m（地面 "
          f"{grid['footprint']['x_max'] - grid['footprint']['x_min']:.1f}×"
          f"{grid['footprint']['y_max'] - grid['footprint']['y_min']:.1f} m）"
          f" -> {out}（{out.stat().st_size / 1e6:.1f} MB）")
    print(f"[floor] 自检：{msg} -> {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
