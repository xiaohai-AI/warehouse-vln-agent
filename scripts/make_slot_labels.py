#!/usr/bin/env python3
"""生成每个货位的编号牌贴图：`world/textures/slot_labels/<地址>.png`。

尺寸/位置口径全部来自 `depot_vln/slot_signs.py`（那里有反推过程）：
字高 0.26 m（2.15 m 处 ≈ 28 px），牌面 1.45×0.34 m，贴图 640×150 px。

渲染后端（自动选，都会打印用了哪个）：
  1. PIL + DejaVu Sans Bold（笔画均匀、OCR 最稳）
  2. OpenCV Hershey DUPLEX（`make world` 已依赖 cv2，一定在）
字高不靠字体参数猜：先画一遍、**量墨迹包围盒**，再按目标像素高度反算字号（两遍收敛）。

用法
  .venv/bin/python scripts/make_slot_labels.py                 # 全量 576 张
  .venv/bin/python scripts/make_slot_labels.py --limit 8        # 只做前 8 个（调试）
  .venv/bin/python scripts/make_slot_labels.py --check          # 只量已有贴图的可读性
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from depot_vln import slot_signs as S  # noqa: E402

OUT_DIR = ROOT / "world" / "textures" / "slot_labels"
GRID = ROOT / "world" / "slot_grid.json"

BG = 236          # 牌面白（略暖，避免过曝成纯白）
INK = 18          # 字色近黑
BORDER = 6        # 边框像素（让牌子在货架前有边界感）

FONTS = [
    # 压缩体优先：8 个字符要在一跨 1.45 m 的牌上做到 0.26 m 字高，
    # 常规 DejaVu Sans Bold 只能做到 0.21 m（宽度先撞上限），Condensed 正好够
    "/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed-Bold.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
    "/usr/share/fonts/truetype/freefont/FreeSansBold.ttf",
]


# ---------------------------------------------------------------- 后端
BACKEND: str | None = None      # "pil" / "cv2" / None
_PIL: Any = None                # (Image, ImageDraw, ImageFont, font_path)
_CV: Any = None                 # (cv2, numpy)


def _init_backend() -> None:
    global BACKEND, _PIL, _CV
    try:
        from PIL import Image, ImageDraw, ImageFont  # type: ignore

        for f in FONTS:
            if Path(f).exists():
                _PIL = (Image, ImageDraw, ImageFont, f)
                BACKEND = "pil"
                return
    except Exception:  # noqa: BLE001
        pass
    try:
        import cv2  # type: ignore
        import numpy as np  # type: ignore

        _CV = (cv2, np)
        BACKEND = "cv2"
    except Exception:  # noqa: BLE001
        BACKEND = None


_init_backend()


def draw_mask(text: str, size: int, box_w: int, box_h: int):
    """在 box_w×box_h 的画布上画字，返回 (ink_mask[box_h,box_w] 0/1, bbox)。"""
    if BACKEND == "pil":
        Image, ImageDraw, ImageFont, font_path = _PIL
        img = Image.new("L", (box_w, box_h), 0)
        d = ImageDraw.Draw(img)
        font = ImageFont.truetype(font_path, max(4, int(size)))
        # 用 anchor="mm" 居中；先量再画
        l, t, r, b = d.textbbox((0, 0), text, font=font)
        d.text(((box_w - (r - l)) / 2 - l, (box_h - (b - t)) / 2 - t), text, font=font, fill=255)
        arr = __import__("numpy").asarray(img, dtype="uint8")
        mask = arr > 96
        ys, xs = mask.nonzero()
        if len(xs) == 0:
            return mask.astype("uint8"), (0, 0, 0, 0)
        return mask.astype("uint8"), (int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max()))

    cv2, np = _CV
    canvas = np.zeros((box_h, box_w), dtype=np.uint8)
    font = cv2.FONT_HERSHEY_DUPLEX
    scale = size / 22.0
    thick = max(1, int(round(size / 14)))
    (tw, th), base = cv2.getTextSize(text, font, scale, thick)
    org = ((box_w - tw) // 2, (box_h + th) // 2)
    cv2.putText(canvas, text, org, font, scale, 255, thick, cv2.LINE_AA)
    mask = canvas > 96
    ys, xs = mask.nonzero()
    if len(xs) == 0:
        return mask.astype("uint8"), (0, 0, 0, 0)
    return mask.astype("uint8"), (int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max()))


def render_label(addr: str):
    """渲染一张牌：返回 (gray[H,W] uint8, 实测墨迹 bbox)。"""
    np = __import__("numpy")
    W, H = S.TEX_W, S.TEX_H
    target_cap = S.CAP_M / S.PLATE_H * H          # 目标字高（像素）
    margin = 0.04 * W      # 牌宽是硬约束，边距从 6% 收到 4%（字高能多 5%）

    # 第一遍：按 22 px 字高画，量出墨迹后按比例反算
    size0 = 22
    m0, bb0 = draw_mask(addr, size0, W, H)
    if bb0[3] <= bb0[1]:
        raise SystemExit(f"[label] 渲染失败：{addr}")
    cap0 = bb0[3] - bb0[1] + 1
    size1 = max(4, int(round(size0 * target_cap / cap0)))

    # 第二遍：目标字高，检查是否超宽；超宽就缩
    mask, bb = None, None
    for _ in range(6):
        mask, bb = draw_mask(addr, size1, W, H)
        w = bb[2] - bb[0] + 1
        if w <= W - 2 * margin:
            break
        size1 = int(size1 * (W - 2 * margin) / w)

    gray = np.full((H, W), BG, dtype=np.uint8)
    gray[mask.astype(bool)] = INK
    gray[:BORDER, :] = INK
    gray[-BORDER:, :] = INK
    gray[:, :BORDER] = INK
    gray[:, -BORDER:] = INK
    return gray, bb


def save_gray(path: Path, gray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if BACKEND == "cv2":
        cv2, _ = _CV
        cv2.imwrite(str(path), gray)
        return
    Image = _PIL[0]
    Image.fromarray(gray).save(path)     # 2D uint8 → 自动按 L 存


# ---------------------------------------------------------------- 主流程
def load_slots() -> dict:
    grid = json.loads(GRID.read_text(encoding="utf-8"))
    return grid["slots"]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--check", action="store_true", help="只量已有贴图，不重新生成")
    args = ap.parse_args()

    if BACKEND is None:
        print("[label] ✗ 既没有 PIL 也没有 cv2 —— 无法渲染。装一个：pip install pillow")
        return 2

    slots = load_slots()
    addrs = sorted(slots)
    if args.limit:
        addrs = addrs[: args.limit]

    if args.check:
        return check(addrs, slots)

    NP = __import__("numpy")
    worst = None
    for i, addr in enumerate(addrs, 1):
        gray, bb = render_label(addr)
        save_gray(OUT_DIR / f"{addr}.png", gray)
        cap_tex = bb[3] - bb[1] + 1
        rec = slots[addr]
        dist = float(rec.get("obs_distance") or S.DEFAULT_OBS_DIST)
        cap_m = cap_tex / S.TEX_H * S.PLATE_H    # 像素 → 米
        px = S.cap_px(dist, cap_m)
        if worst is None or px < worst[1]:
            worst = (addr, px, cap_m, cap_tex)
        if i % 96 == 0 or i == len(addrs):
            print(f"[label] {i}/{len(addrs)}  后端={BACKEND}")

    print(f"[label] 共 {len(addrs)} 张 -> {OUT_DIR}")
    if worst:
        addr, px, cap_m, cap_tex = worst
        print(f"[label] 最窄情形 {addr}：字高 {cap_m:.3f} m / 贴图 {cap_tex} px "
              f"→ 2.15 m 处 {px:.1f} px（门槛 {S.MIN_CAP_PX:.0f}）")
        if px < S.MIN_CAP_PX:
            print("[label] ✗ 字高不够，OCR 不可靠")
            return 1
    return 0


def check(addrs: list[str], slots: dict) -> int:
    np = __import__("numpy")
    missing, rows = [], []
    for addr in addrs:
        p = OUT_DIR / f"{addr}.png"
        if not p.exists():
            missing.append(addr)
            continue
        if BACKEND == "cv2":
            cv2, _ = _CV
            img = cv2.imread(str(p), cv2.IMREAD_GRAYSCALE)
        else:
            Image = _PIL[0]
            img = np.asarray(Image.open(p).convert("L"))
        inner = img[BORDER:-BORDER, BORDER:-BORDER]      # 去掉边框再量字
        mask = inner < 128                               # 深色字
        ys, xs = mask.nonzero()
        if len(xs) == 0:
            rows.append((addr, 0, 0.0, 0.0))
            continue
        cap_tex = int(ys.max() - ys.min() + 1)
        cap_m = cap_tex / S.TEX_H * S.PLATE_H
        dist = float(slots.get(addr, {}).get("obs_distance") or S.DEFAULT_OBS_DIST)
        rows.append((addr, cap_tex, cap_m, S.cap_px(dist, cap_m)))

    if missing:
        print(f"[label] ✗ 缺 {len(missing)} 张贴图，例如 {missing[:4]}")
        return 2
    mn = min(rows, key=lambda r: r[3])
    print(f"[label] 量了 {len(rows)} 张（后端={BACKEND}）")
    print(f"[label] 字高最小：{mn[0]} {mn[1]} px = {mn[2]:.3f} m → 2.15 m 处 {mn[3]:.1f} px"
          f"（门槛 {S.MIN_CAP_PX:.0f}）")
    bad = [r for r in rows if r[3] < S.MIN_CAP_PX]
    if bad:
        print(f"[label] ✗ {len(bad)} 张字高不足：{bad[:5]}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
