#!/usr/bin/env python3
"""离线验证"牌子够不够大"：把贴图按 2.15 m 观测距离投影到 320×240 画面里的真实像素尺寸。

为什么这么做：判"够不够 OCR"不能只看米数，要看**相机里实际有多少像素**。
这里用真实内参（f_px = 160，HFOV 90°，320×240）把牌面缩到等效尺寸，
再拼进一张 320×240 的空白帧，交给视觉模型读 —— 读得出来才算够大。

用法
  .venv/bin/python scripts/project_label_preview.py --addr C-L-08-1
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from depot_vln import slot_signs as S  # noqa: E402

CAM_Z = 0.35


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--addr", default="C-L-08-1")
    ap.add_argument("--dist", type=float, default=None)
    ap.add_argument("--out", default=str(ROOT / "results" / "label_preview.png"))
    args = ap.parse_args()

    grid = json.loads((ROOT / "world" / "slot_grid.json").read_text(encoding="utf-8"))
    rec = grid["slots"][args.addr]
    dist = args.dist or float(rec.get("obs_distance") or S.DEFAULT_OBS_DIST)

    ppm = S.px_per_m(dist)
    w_px = max(1, round(S.PLATE_W * ppm))
    h_px = max(1, round(S.PLATE_H * ppm))
    cap_px = S.cap_px(dist)

    src = cv2.imread(str(ROOT / "world" / "textures" / "slot_labels" / f"{args.addr}.png"),
                     cv2.IMREAD_GRAYSCALE)
    small = cv2.resize(src, (w_px, h_px), interpolation=cv2.INTER_AREA)

    frame = np.full((S.CAM_H, S.CAM_W), 90, dtype=np.uint8)     # 背景当灰色货架
    cy = S.CAM_H // 2 - round((1.22 - CAM_Z) * S.f_px() / dist)  # 牌心在画面里的行
    x0 = (S.CAM_W - w_px) // 2
    y0 = max(0, cy - h_px // 2)
    frame[y0:y0 + h_px, x0:x0 + w_px] = small[: max(0, min(h_px, S.CAM_H - y0))][: small.shape[0]]

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), frame)
    cv2.imwrite(str(out.with_name(out.stem + "_crop.png")), small)

    print(f"[preview] {args.addr} @ {dist:.2f} m：牌 {S.PLATE_W}×{S.PLATE_H} m "
          f"→ {w_px}×{h_px} px，字高 {cap_px:.1f} px")
    print(f"[preview] 整帧 -> {out}")
    print(f"[preview] 裁切  -> {out.with_name(out.stem + '_crop.png')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
