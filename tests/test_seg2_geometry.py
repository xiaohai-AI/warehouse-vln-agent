"""SEG-2 检测的几何不变量（合成数据，不依赖 20 MB 采集）。

这三条都是 2026-11-04 重标定时踩出来的，写成断言防止再退回去：

1. **层带必须由编号牌几何派生**：牌底 = deck + 0.87，level 1 就是 1.02 m。
   带子上界写 1.60 会把整排编号牌含进去 —— 牌子是连续前景，横向连成 2.4 m 大块，
   尺寸闸门（≤1.15 m）于是把所有货判成"没有"（实测新世界 48 帧全空）。
2. **必须先按 bay 横向窗口裁掩码，再做连通域**：不裁的话左右邻 bay 的货会连成
   1.5–3.6 m 的宽块（实测 16 个漏检全是这种）。
3. **深度带上界是"本侧/对侧"分界**：对侧（另一条巷）的货透过空货位能看到，
   d≈1.75–1.95；上界放到 2.35 会多出 6 个假阳性。
"""
from __future__ import annotations

import numpy as np
import pytest

from depot_vln.catalog import get_catalog
from depot_vln.slot_signs import band_z

from scripts.seg2_detect import (  # noqa: E402
    MORPH_CLOSE_H,
    MORPH_CLOSE_V,
    Z_DECK_CLEAR,
    cargo_components,
    classify,
    level_bands,
)


def _plane_depth(h: int, w: int, dist: float) -> np.ndarray:
    """整幅图都是同一个水平距离的竖直平面（pitch=0 时 depth 就是水平距离）。"""
    return np.full((h, w), float(dist), dtype=np.float32)


def _k(h: int, w: int) -> np.ndarray:
    f = w / 2.0 / np.tan(np.radians(45.0))      # HFOV 90°，与 robot xacro 一致
    return np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]], dtype=np.float64)


def test_level_bands_sit_below_sign_plates():
    cat = get_catalog()
    lv1 = next(l for l in cat.levels if l.index == 1)
    lo, hi = level_bands()[1]
    plate_lo, _ = band_z(lv1.deck_z)
    assert lo == pytest.approx(lv1.deck_z + Z_DECK_CLEAR, abs=1e-3)
    assert hi < plate_lo, "层带上界必须压在编号牌底面以下"
    # 下界必须高于台面前沿（否则那条连续横梁会把整幅货架连成一块）
    assert lo > lv1.deck_z + 0.05


def test_bay_window_crops_before_connected_components():
    h, w = 61, 640
    depth = _plane_depth(h, w, 1.45)
    k = _k(h, w)
    cam_h = 0.60                    # 让画面中心落在 level-1 带内
    kwargs = dict(z_band=level_bands()[1], obs_x=0.0, obs_yaw=np.pi / 2,
                  d_lo=1.33, d_hi=1.60)
    whole = cargo_components(depth, k, 0.0, cam_h, **kwargs)
    cropped = cargo_components(depth, k, 0.0, cam_h, bay_cx=0.0, bay_half=0.70, **kwargs)
    assert whole and cropped
    w_whole = max(c["w_m"] for c in whole)
    w_crop = max(c["w_m"] for c in cropped)
    assert w_whole > 2.0, "整幅平面在未裁剪时本来就是一整块（这正是要修的病）"
    assert w_crop <= 1.5, f"按 bay 窗口裁剪后不应超过 1.4 m，实际 {w_crop:.2f}"


def test_far_side_is_rejected_by_depth_gate():
    """对侧货（1.8 m）必须被上界挡住，本侧货（1.45 m）必须能检到。"""
    h, w = 61, 320
    k = _k(h, w)
    band = level_bands()[1]
    near = cargo_components(_plane_depth(h, w, 1.45), k, 0.0, 0.60, z_band=band,
                            d_lo=1.33, d_hi=1.60)
    far = cargo_components(_plane_depth(h, w, 1.80), k, 0.0, 0.60, z_band=band,
                           d_lo=1.33, d_hi=1.60)
    assert near, "本侧平面应当被检出"
    assert not far, "对侧平面（1.80 m）不该被检成本侧货物"


@pytest.mark.parametrize("w_m,h_m,cls", [
    (0.34, 0.15, "carton_s"),
    (0.49, 0.25, "carton_m"),
    (0.68, 0.38, "carton_l"),
    (1.00, 0.00, "pallet"),
])
def test_classify_measured_bands(w_m, h_m, cls):
    # 纸箱都是棕色；用实测过的典型值（宽度、高度）验证分类边界
    assert classify(w_m, h_m, np.array([120.0, 85.0, 45.0]))[0] == cls


def test_classify_separates_tote_and_drum_by_colour():
    assert classify(0.44, 0.14, np.array([49.0, 49.0, 38.0]))[0] == "tote"
    assert classify(0.50, 0.60, np.array([90.0, 120.0, 90.0]))[0] == "drum"


def test_morph_kernel_is_wider_horizontally():
    """一个货位只放一件货：被立柱切断时要能横向接回，纵向保持窄以免粘上层板。"""
    assert MORPH_CLOSE_H > MORPH_CLOSE_V


def test_fragment_merge_runs_before_size_gate():
    """同货位同深度的碎片必须先合并再过尺寸闸门，否则被立柱切开的物体会整块丢掉。"""
    from scripts.seg2_detect import _merge_fragments

    frag = lambda lo, hi, d: {"wx_lo": lo, "wx_hi": hi, "w_m": hi - lo, "h_m": 0.5,  # noqa: E731
                              "d_med": d, "c_wx": (lo + hi) / 2, "area": 400,
                              "x0": 0, "x1": 10, "y0": 0, "y1": 10}
    merged = _merge_fragments([frag(0.0, 0.17, 1.36), frag(0.30, 0.47, 1.39)])
    assert merged["w_m"] == pytest.approx(0.47, abs=1e-6), "同深度碎片应合并成 0.47 m"
    # 前后两排货（深度差 0.5 m）不许合：合起来尺寸会虚高
    not_merged = _merge_fragments([frag(0.0, 0.17, 1.20), frag(0.30, 0.47, 1.75)])
    assert not_merged["w_m"] == pytest.approx(0.17, abs=1e-6)
