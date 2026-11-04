#!/usr/bin/env python3
"""生成仓库表面纹理（程序化，纯离线）。

为什么需要：视觉为主的仓库如果表面全是纯色平面，双目/视觉 SLAM/货位识别**物理上无信息可匹配**
（实测：左图中位梯度 0、只有 35% 像素有可用梯度；SGBM 在无纹理区只能靠平滑先验"编"深度，
导致 S1 的 p95 达 200%+）。这不是算法能修的，必须让世界本身有纹理。

纹理设计原则
  1. **高频噪声**必须强（每个表面在图像里 1–3 px 尺度上有对比），否则远处匹配不上；
  2. 同时有**结构化元素**（接缝、铆钉、标签、警示条）—— 那是 P5 货位识别要用的语义线索；
  3. 尺寸 512/1024，SDF 的 box 面 UV 是 0..1，所以一张图铺满一个面：
     地面 67 m 面 / 1024 px = 6.5 cm/px（2 m 处约 15 px/纹理像素，够用）；
     货架层板 2.2 m 面 / 512 px = 4 mm/px（很细）。

用法
  .venv/bin/python scripts/make_textures.py                 # -> world/textures/*.png
  .venv/bin/python scripts/make_textures.py --out /tmp/tex --seed 7
"""
from __future__ import annotations

import argparse

import cv2
import numpy as np

ROOT_TEX = "world/textures"


def _rng(seed: int) -> np.random.Generator:
    return np.random.default_rng(seed)


def _value_noise(h: int, w: int, cell: int, rng: np.random.Generator) -> np.ndarray:
    """低频到高频都有的平滑随机场（双线性放大随机格点）。"""
    gh, gw = max(2, h // cell + 2), max(2, w // cell + 2)
    g = rng.random((gh, gw)).astype(np.float32)
    return cv2.resize(g, (w, h), interpolation=cv2.INTER_CUBIC)


def _fractal(h: int, w: int, rng: np.random.Generator, octaves=(4, 8, 16, 32)) -> np.ndarray:
    acc = np.zeros((h, w), np.float32)
    amp = 1.0
    for cell in octaves:
        acc += amp * (_value_noise(h, w, cell, rng) - 0.5)
        amp *= 0.75
    acc /= np.abs(acc).max() + 1e-6
    return acc


def _to_bgr(base: tuple[float, float, float], field: np.ndarray, contrast: float) -> np.ndarray:
    """把 [-1,1] 的随机场变成以 base 颜色为中心的 BGR 图。"""
    h, w = field.shape
    out = np.zeros((h, w, 3), np.float32)
    for i, c in enumerate(base):          # base 是 RGB，输出 BGR
        out[:, :, 2 - i] = np.clip(c + contrast * field, 0.0, 1.0)
    return out


def concrete(h: int = 1024, w: int = 1024, seed: int = 1) -> np.ndarray:
    """地坪：细粒骨料 + 板块接缝 + 油渍。地面是双目最常"看到"的大平面。"""
    rng = _rng(seed)
    fine = _fractal(h, w, rng, octaves=(2, 4, 8, 16))
    grain = (rng.random((h, w)).astype(np.float32) - 0.5) * 0.55     # 骨料颗粒：逐像素对比
    field = 0.45 * fine + grain
    img = _to_bgr((0.42, 0.44, 0.43), field, 0.55)

    # 板块接缝（每 1/4 图一条，深色细线）+ 两侧高光，形成强边
    for frac in (0.25, 0.5, 0.75):
        x = int(w * frac)
        img[:, x - 1:x + 1] *= 0.55
        img[:, x + 1:x + 3] *= 1.25
    # 少量油渍/轮胎痕
    for _ in range(6):
        cx, cy = rng.integers(0, w), rng.integers(0, h)
        blob = _value_noise(h, w, 64, rng)
        mask = np.clip((blob - 0.55) * 3.0, 0, 1)
        img *= (1.0 - 0.22 * mask)[:, :, None]
    return img


def wall_panel(h: int = 512, w: int = 512, seed: int = 2) -> np.ndarray:
    """墙面彩钢板：竖向压型肋 + 螺栓行 + 锈斑。肋提供垂直方向强梯度。"""
    rng = _rng(seed)
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    ribs = 0.5 + 0.5 * np.sin(xx / w * np.pi * 2 * 16)          # 16 道肋
    ribs = (ribs - 0.5) * 2.0
    field = 0.55 * ribs + 0.45 * _fractal(h, w, rng, octaves=(4, 8, 16))
    img = _to_bgr((0.62, 0.64, 0.66), field, 0.35)
    # 螺栓行（两行）+ 高光点
    for y in (int(h * 0.25), int(h * 0.7)):
        for x in range(16, w, 64):
            cv2.circle(img, (x, y), 3, (0.30, 0.30, 0.32), -1)
            cv2.circle(img, (x + 1, y - 1), 1, (0.85, 0.85, 0.88), -1)
    # 锈斑
    for _ in range(10):
        cx, cy = int(rng.integers(0, w)), int(rng.integers(0, h))
        cv2.ellipse(img, (cx, cy), (int(rng.integers(4, 18)), int(rng.integers(3, 12))),
                    float(rng.integers(0, 180)), 0, 360, (0.32, 0.42, 0.58), -1)
    return img


def steel_rack(h: int = 512, w: int = 512, seed: int = 3) -> np.ndarray:
    """货架：喷涂钢 + 铆钉 + 划痕 + 底部黄黑警示条。层板/立柱/横梁共用。"""
    rng = _rng(seed)
    field = 0.5 * _fractal(h, w, rng, octaves=(4, 8, 16, 32)) + \
        0.5 * (rng.random((h, w)).astype(np.float32) - 0.5)
    img = _to_bgr((0.30, 0.34, 0.40), field, 0.30)
    # 铆钉网格
    for y in range(32, h, 96):
        for x in range(32, w, 96):
            cv2.circle(img, (x, y), 4, (0.20, 0.22, 0.26), -1)
            cv2.circle(img, (x, y - 1), 2, (0.62, 0.66, 0.72), -1)
    # 划痕
    for _ in range(40):
        x0, y0 = int(rng.integers(0, w)), int(rng.integers(0, h))
        ln = int(rng.integers(10, 60))
        ang = float(rng.integers(0, 180))
        dx, dy = int(ln * np.cos(np.radians(ang))), int(ln * np.sin(np.radians(ang)))
        cv2.line(img, (x0, y0), (x0 + dx, y0 + dy), (0.55, 0.58, 0.62), 1)
    # 底部警示条（黄黑斜纹）—— 语义线索，也是强纹理
    band = int(h * 0.14)
    yy, xx = np.mgrid[0:band, 0:w]
    stripe = (((xx + yy) // 20) % 2).astype(np.float32)
    for i, channel in enumerate((0.10, 0.72, 0.86)):      # B/G/R：黑 <-> 黄
        img[h - band:, :, i] = channel * stripe + 0.10 * (1 - stripe)
    return img


def carton(h: int = 512, w: int = 512, seed: int = 4) -> np.ndarray:
    """纸箱：瓦楞纸纹 + 封箱胶带 + 印刷标签块。P5 的"箱子"识别靠这些。"""
    rng = _rng(seed)
    corrug = 0.5 + 0.5 * np.sin(np.mgrid[0:h, 0:w][0].astype(np.float32) / h * np.pi * 2 * 48)
    # 纸面颗粒必须逐像素有对比，否则双目在 3–6 m 的箱面上匹配不上（实测梯度中位只有 7）
    grain = (rng.random((h, w)).astype(np.float32) - 0.5) * 0.5
    field = 0.3 * (corrug - 0.5) * 2 + 0.35 * _fractal(h, w, rng, octaves=(8, 16, 32)) + grain
    img = _to_bgr((0.72, 0.56, 0.34), field, 0.22)
    # 胶带（横贯）
    y = int(h * 0.18)
    img[y:y + int(h * 0.07), :] = (0.78, 0.80, 0.72)
    img[y:y + 2, :] *= 0.8
    # 白色标签块 + 假条码
    ly, lx = int(h * 0.55), int(w * 0.12)
    img[ly:ly + int(h * 0.22), lx:lx + int(w * 0.5)] = (0.92, 0.92, 0.90)
    for i in range(24):
        x = lx + 8 + i * 9
        if rng.random() > 0.4:
            img[ly + int(h * 0.06):ly + int(h * 0.17), x:x + 4] = (0.15, 0.15, 0.15)
    return img


def plastic(h: int = 512, w: int = 512, seed: int = 5) -> np.ndarray:
    """周转箱：塑料+喷点+棱边高光。"""
    rng = _rng(seed)
    field = 0.6 * _fractal(h, w, rng, octaves=(8, 16, 32)) + \
        0.4 * (rng.random((h, w)).astype(np.float32) - 0.5)
    return _to_bgr((0.22, 0.34, 0.62), field, 0.30)


def hazard(h: int = 512, w: int = 512, seed: int = 6) -> np.ndarray:
    """警示斜纹（护栏/挡板）—— 最强的方向性纹理，适合给视觉当"特征靶"。

    光有斜纹不够：条纹内部是纯色，逐像素匹配会退化（实测梯度中位 = 0）。
    真实喷漆条纹也有漆面颗粒与磨损，所以叠一层 grain —— 既是真实感也是双目的需求。
    """
    rng = _rng(seed)
    yy, xx = np.mgrid[0:h, 0:w]
    stripe = (((xx + yy) // 24) % 2).astype(np.float32)
    img = np.zeros((h, w, 3), np.float32)
    for i, channel in enumerate((0.10, 0.72, 0.86)):      # B/G/R：黑 <-> 黄
        img[:, :, i] = channel * stripe + 0.10 * (1 - stripe)
    grain = (rng.random((h, w)).astype(np.float32) - 0.5) * 0.30
    wear = _fractal(h, w, rng, octaves=(8, 16, 32)) * 0.10
    img = np.clip(img + (grain + wear)[:, :, None], 0.0, 1.0)
    return img


GENERATORS = {
    "concrete": concrete,
    "wall_panel": wall_panel,
    "steel_rack": steel_rack,
    "carton": carton,
    "plastic": plastic,
    "hazard": hazard,
}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=ROOT_TEX)
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()

    from pathlib import Path
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    bad: list[str] = []
    for name, fn in GENERATORS.items():
        img = fn(seed=args.seed)
        img8 = np.clip(img * 255, 0, 255).astype(np.uint8)
        p = out / f"{name}.png"
        cv2.imwrite(str(p), img8)
        gray = cv2.cvtColor(img8, cv2.COLOR_BGR2GRAY)
        gx = np.abs(cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3))
        gy = np.abs(cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3))
        g = np.maximum(gx, gy)                      # 斜纹取 max，否则单方向中位会是 0
        med = float(np.median(g))
        flag = "" if med > 10 else "  ⚠️ 对比不足（双目会匹配不上）"
        if med <= 10:
            bad.append(name)
        print(f"[tex] {name:<12} {img8.shape[1]}x{img8.shape[0]}  "
              f"均值 {img8.mean():5.1f}  std {img8.std():5.1f}  梯度中位 {med:5.1f}{flag}  -> {p}")
    if bad:
        print(f"[tex] FAIL：{bad} 的梯度中位 ≤10 —— 这些表面在双目里等于没有信息")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
