#!/usr/bin/env python3
"""给生成的 world/warehouse.sdf 注入 PBR 纹理贴图（按 visual 名字匹配）。

为什么单独一步而不是改 `depot_vln/sdf.py`：
  - SDF 是**生成产物**，这一步同样是生成步骤（幂等、按名字派生），不违反"不手改 SDF"；
  - 一行开关就能 A/B（`make world TEXTURES=off` 回到纯色世界），
    这对"加了纹理到底改善了多少"的对照实验很关键。

匹配规则（见 RULES）：地面→混凝土、墙→彩钢压型板、货架→喷涂钢（带黄黑警示条）、
货箱→纸箱、周转箱→塑料、护栏/立柱护罩→黄黑斜纹。没匹配到的 visual 保持纯色。

⚠️ 已知前提：SDF 的 box 面 UV 是 0..1，所以一张图铺满一个面。
   地面 67 m 面用 1024 px（6.5 cm/px）、货架 2.2 m 面用 512 px（4 mm/px）——设计时按这个选的尺寸。

用法
  .venv/bin/python scripts/apply_world_textures.py                    # 就地改 world/warehouse.sdf
  .venv/bin/python scripts/apply_world_textures.py --check            # 只统计，不写
  .venv/bin/python scripts/apply_world_textures.py --strip            # 去掉所有 pbr（回到纯色）
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from depot_vln.slot_signs import addr_from_visual  # noqa: E402

# (正则, 纹理名) —— 顺序有意义，先匹配先用
RULES: list[tuple[str, str]] = [
    # 地面用"覆盖全场唯一大图"而不是平铺混凝土：见 scripts/make_floor_texture.py
    (r"^floor", "floor_unique"),
    (r"^wall_|^trim_wall", "wall_panel"),
    (r"^column_.*_guard$|^.*_guard$", "hazard"),
    (r"^barrier_.*(body|base|stripe|sign)$", "hazard"),
    (r"^column_", "concrete"),
    (r"^trim_deckrun|^rack_deck|^.*_deck$|^beam|^post|^rack_.*post", "steel_rack"),
    (r"^cargo_", "carton"),
    (r"^obstacle_tote_", "plastic"),
    (r"^obstacle_fl_", "steel_rack"),
]

VIS_START = '<visual name="'
VIS_END = "</visual>"


def pick(name: str, block: str = "") -> str | None:
    # visual 名都带 `_vis` 后缀，规则里写的是物体名 -> 先剥掉
    # （踩过：没剥后缀导致 `..._guard$` / `barrier_..._body$` 全部匹配失败）
    name = re.sub(r"_vis$", "", name)
    for pat, tex in RULES:
        if re.search(pat, name):
            return tex
    # 圆柱形货物（桶/drum）给塑料而不是纸箱
    if name.startswith("cargo_") and "<cylinder>" in block:
        return "plastic"
    return None


def pbr(texture_path: Path, original_material: str) -> str:
    """在原有 <material> 里补一段 <pbr>，保留 ambient/diffuse 作为兜底。

    保留 ambient 很重要：万一贴图加载失败，物体不会变成纯黑（纯黑会让双目更瞎），
    而是退回原来的颜色 —— 这样"加纹理"永远不会让世界比原来更糟。
    """
    inner = original_material
    if inner.startswith("<material>"):
        inner = inner[len("<material>"):]
    if inner.endswith("</material>"):
        inner = inner[: -len("</material>")]
    inner = re.sub(r"<pbr>.*?</pbr>", "", inner, flags=re.S)       # 幂等：先去掉旧的 pbr
    uri = texture_path.resolve().as_uri()
    return (
        "<material>"
        f"<pbr><metal><albedo_map>{uri}</albedo_map>"
        "<roughness>0.85</roughness><metalness>0.05</metalness></metal></pbr>"
        f"{inner}"
        "</material>"
    )


def transform(sdf: str, tex_dir: Path) -> tuple[str, dict[str, int], list[str]]:
    out: list[str] = []
    counts: dict[str, int] = {}
    missing: list[str] = []
    i = 0
    while True:
        j = sdf.find(VIS_START, i)
        if j < 0:
            out.append(sdf[i:])
            break
        out.append(sdf[i:j])
        k = sdf.find(VIS_END, j)
        if k < 0:
            out.append(sdf[j:])
            break
        block = sdf[j:k]
        m_name = re.match(r'<visual name="([^"]+)"', block)
        name = m_name.group(1) if m_name else ""
        # 货位编号牌：**每个地址一张独立贴图**（sign_slot_C_L_08_1_vis -> slot_labels/C-L-08-1.png）
        addr = addr_from_visual(name)
        if addr is not None:
            tex, p = "slot_labels", tex_dir / "slot_labels" / f"{addr}.png"
        else:
            tex = pick(name, block)
            p = tex_dir / f"{tex}.png" if tex else None
        if tex and p is not None:
            if not p.exists():
                missing.append(f"slot_labels/{addr}.png" if addr else tex)
            else:
                m_mat = re.search(r"<material>.*?</material>", block, re.S)
                if m_mat:
                    block = block[: m_mat.start()] + pbr(p, m_mat.group(0)) + block[m_mat.end():]
                    counts[tex] = counts.get(tex, 0) + 1
        out.append(block)
        i = k
    return "".join(out), counts, missing


def strip(sdf: str) -> tuple[str, int]:
    n = len(re.findall(r"<pbr>.*?</pbr>", sdf, flags=re.S))
    return re.sub(r"<pbr>.*?</pbr>", "", sdf, flags=re.S), n


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sdf", default=str(ROOT / "world" / "warehouse.sdf"))
    ap.add_argument("--tex", default=str(ROOT / "world" / "textures"))
    ap.add_argument("--check", action="store_true", help="只统计不写")
    ap.add_argument("--strip", action="store_true", help="去掉所有 pbr，回到纯色世界")
    args = ap.parse_args()

    path = Path(args.sdf)
    sdf = path.read_text(encoding="utf-8")

    if args.strip:
        new, n = strip(sdf)
        if not args.check:
            path.write_text(new, encoding="utf-8")
        print(f"[tex] 去掉 {n} 处 pbr 贴图 -> {path}")
        return 0

    new, counts, missing = transform(sdf, Path(args.tex))
    total = sum(counts.values())
    for tex, c in sorted(counts.items()):
        print(f"[tex] {tex:<12} 注入 {c:4d} 个 visual")
    if missing:
        uniq = sorted(set(missing))
        print(f"[tex] ⚠️ 缺 {len(uniq)} 个纹理文件，例如 {uniq[:6]}")
        print("[tex]    普通纹理 -> scripts/make_textures.py；slot_labels -> scripts/make_slot_labels.py")
        return 2
    if not args.check:
        path.write_text(new, encoding="utf-8")
        print(f"[tex] 共 {total} 个 visual 带贴图 -> {path}（{len(new) / 1024:.0f} KiB）")
    else:
        print(f"[tex] check：将注入 {total} 个 visual（未写盘）")
    return 0 if total else 1


if __name__ == "__main__":
    raise SystemExit(main())
