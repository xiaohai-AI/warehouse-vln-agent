#!/usr/bin/env python3
"""给 gz 生成的机器人 SDF 补回材质颜色。

为什么需要这一步（实测坑）
  xacro 里 `<material name="amr_red"><color rgba="..."/></material>` 是 URDF 的**命名材质**，
  而 `gz sdf -p robot/depot_bot.urdf > robot/depot_bot.sdf` 会**把它整段丢掉** ——
  实测：96 个 `<visual>`、`<diffuse>/<ambient>` 计数 **0**。
  结果 Gazebo GUI 里整台车是默认灰，裙边/斜切车头/轮罩/镜头环全糊成一坨 → 看着像"简化模型"。
  结构其实一个没少，缺的只是颜色。

只改"看起来"
  仅向 `<visual>` 块内插入 `<material>`，**不动** `<collision>` / `<inertial>` / `<pose>` /
  `<joint>` / 传感器 —— 所以物理与量测基准逐字节不变（用 --check 可自证）。

用法
  python3 scripts/patch_robot_materials.py robot/depot_bot.sdf            # 原地补材质（幂等）
  python3 scripts/patch_robot_materials.py robot/depot_bot.sdf --check     # 只检查，不写
  python3 scripts/patch_robot_materials.py robot/depot_bot.sdf --show      # 打印 名称→材质 对照
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

# 颜色取自 robot/depot_bot.urdf.xacro 的 <material> 定义（同一套配色，别在这里另起一套）
# 值 = (rgb, specular, emissive, metalness, roughness)
#   metalness/roughness 走 SDF 的 <pbr>：Ogre2 认这两个，镀铬/钢件才有金属反射；
#   塑料件 metalness=0 保证不发光发亮（不然红车体会像糖纸）。
MAT = {
    "amr_red":      ((0.84, 0.10, 0.09), 0.30, 0.00, 0.00, 0.45),   # 喷漆钣金
    "amr_red_dk":   ((0.56, 0.05, 0.05), 0.25, 0.00, 0.00, 0.55),
    "amr_grey":     ((0.22, 0.23, 0.25), 0.20, 0.00, 0.00, 0.60),   # 工程塑料
    "amr_grey_lt":  ((0.45, 0.46, 0.49), 0.30, 0.00, 0.00, 0.50),
    "amr_black":    ((0.09, 0.09, 0.10), 0.10, 0.00, 0.00, 0.70),   # 橡胶/黑件
    "amr_steel":    ((0.66, 0.68, 0.72), 0.75, 0.00, 0.90, 0.35),   # 轮毂
    "amr_chrome":   ((0.84, 0.86, 0.90), 1.00, 0.00, 1.00, 0.12),   # 镀铬
    # 自发光别给太高：0.85 会把颜色冲成白色（截图里灯变白球就是这么来的），0.40 保留色相
    "lamp_amber":   ((1.00, 0.62, 0.08), 0.50, 0.40, 0.00, 0.25),   # 头灯"亮着"
    "lamp_red":     ((1.00, 0.12, 0.06), 0.50, 0.40, 0.00, 0.25),   # 尾灯 / 示位灯
    "lens_glass":   ((0.05, 0.07, 0.11), 1.00, 0.00, 0.00, 0.05),   # 镜片
    # ---- 演示增强件（同样来自 xacro 的 <material> 定义）
    "hazard_yellow": ((0.93, 0.72, 0.06), 0.35, 0.00, 0.00, 0.55),   # 安全警示黄（铁皮漆）
    "hazard_dark":   ((0.10, 0.10, 0.10), 0.20, 0.00, 0.00, 0.65),   # 警示黑条
    "led_cyan":      ((0.05, 0.70, 0.85), 0.40, 0.55, 0.00, 0.30),   # 状态灯带（自发光）
    "crate_card":    ((0.62, 0.47, 0.30), 0.15, 0.00, 0.00, 0.85),   # 纸箱瓦楞
    "crate_light":   ((0.72, 0.58, 0.40), 0.15, 0.00, 0.00, 0.85),   # 箱盖
    "crate_strap":   ((0.25, 0.22, 0.18), 0.15, 0.00, 0.00, 0.80),   # 打包带
}

# **顺序有讲究**：越具体的放前面（lamp_front 先于 lamp，hub_cap 先于 hub，tower_win 先于 tower，
# deck_stripe 先于 deck，lens_ring 先于 lens）。命中即返回。
RULES: list[tuple[str, str]] = [
    ("tire", "amr_black"),
    ("rim_", "amr_grey"),
    ("hub_core", "amr_steel"),
    ("roller", "amr_grey_lt"),
    ("hub_cap", "amr_chrome"),
    ("hub_out", "amr_steel"),
    ("hub_in", "amr_steel"),
    ("lamp_front", "lamp_amber"),
    ("lamp_rear", "lamp_red"),
    ("beacon_lamp", "lamp_red"),
    ("beacon_pole", "amr_chrome"),
    ("lens_ring", "amr_chrome"),
    ("lens_", "lens_glass"),
    ("tof_window", "lens_glass"),
    ("tower_win", "lens_glass"),
    ("tower_cap", "amr_grey"),
    ("tower_base", "amr_grey"),
    ("tower_post", "amr_grey_lt"),
    ("tower_body", "amr_black"),
    ("antenna", "amr_chrome"),
    ("cam_hood", "amr_grey"),
    ("cam_visor", "amr_black"),
    ("nose_slope", "amr_red"),
    ("tail_slope", "amr_red_dk"),
    ("corner_r", "amr_red_dk"),      # 后四角（corner_rl / corner_rr）
    ("corner_", "amr_red"),          # 前四角（corner_fl / corner_fr）
    ("body_main", "amr_red"),
    ("deck_stripe", "amr_grey_lt"),
    ("deck", "amr_grey"),
    ("skirt", "amr_grey"),
    ("bumper", "amr_black"),
    ("waist", "amr_chrome"),
    ("slot_", "amr_black"),
    ("fender", "amr_grey"),
    # ---- 演示增强件（**顺序**：crate_strap 必须排在 crate_ 之前，否则会被箱体色吃掉）
    ("crate_strap", "crate_strap"),
    ("crate_lid", "crate_light"),
    ("crate_", "crate_card"),
    ("led_strip", "led_cyan"),
    ("hazard_k", "hazard_dark"),
    ("hazard_y", "hazard_yellow"),
    ("cam_gusset", "amr_grey_lt"),
    ("cam_rail", "amr_grey"),
    ("cam_foot", "amr_grey"),
]
FALLBACK = "amr_grey_lt"

VIS_RE = re.compile(r"(<visual\s+name=['\"][^'\"]+['\"]>)(.*?)(</visual>)", re.S)
GEOM_END = "</geometry>"


def pick(name: str) -> str:
    low = name.lower()
    for pat, mat in RULES:
        if pat in low:
            return mat
    return FALLBACK


def mat_xml(mat: str, indent: str) -> str:
    (r, g, b), spec, emis, metal, rough = MAT[mat]
    return (
        f"{indent}<material>\n"
        f"{indent}  <ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient>\n"
        f"{indent}  <diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>\n"
        f"{indent}  <specular>{spec:.3f} {spec:.3f} {spec:.3f} 1</specular>\n"
        f"{indent}  <emissive>{emis:.3f} {emis:.3f} {emis:.3f} 1</emissive>\n"
        f"{indent}  <pbr>\n"
        f"{indent}    <metal>\n"
        f"{indent}      <metalness>{metal:.2f}</metalness>\n"
        f"{indent}      <roughness>{rough:.2f}</roughness>\n"
        f"{indent}    </metal>\n"
        f"{indent}  </pbr>\n"
        f"{indent}</material>\n"
    )


def process(text: str) -> tuple[str, list[tuple[str, str]], list[str]]:
    """返回 (新文本, [(visual名, 材质)], 没配上的名称)"""
    done: list[tuple[str, str]] = []
    unmatched: list[str] = []

    def repl(m: re.Match) -> str:
        head, body, tail = m.group(1), m.group(2), m.group(3)
        name = re.search(r"name=['\"]([^'\"]+)", head).group(1)  # type: ignore[union-attr]
        if "<material>" in body:          # 幂等：已经补过就跳过
            return m.group(0)
        if GEOM_END not in body:
            unmatched.append(name)
            return m.group(0)
        mat = pick(name)
        if all(pat not in name.lower() for pat, _ in RULES):
            unmatched.append(name)
        # 缩进对齐 </geometry> 那一行，插在它后面（SDF visual 的顺序是 pose → geometry → material）
        line_start = body.rfind("\n", 0, body.index(GEOM_END)) + 1
        indent = body[line_start:body.index(GEOM_END)]
        indent = indent if indent.strip() == "" else "        "
        new_body = body.replace(GEOM_END, GEOM_END + "\n" + mat_xml(mat, indent), 1)
        done.append((name, mat))
        return head + new_body + tail

    return VIS_RE.sub(repl, text), done, unmatched


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("sdf")
    ap.add_argument("--check", action="store_true", help="只检查：每个 visual 是否都有材质")
    ap.add_argument("--show", action="store_true", help="打印 名称→材质 对照")
    args = ap.parse_args()

    p = Path(args.sdf)
    if not p.exists():
        print(f"找不到 {p}（先跑 make robot 生成）", file=sys.stderr)
        return 2
    text = p.read_text(encoding="utf-8")
    new, done, unmatched = process(text)

    if args.show:
        for name, mat in done:
            print(f"  {mat:12s} ← {name}")

    if args.check:
        total = len(VIS_RE.findall(text))
        with_mat = text.count("<material>")
        print(f"visual {total} 个，已带材质 {with_mat} 个")
        if unmatched:
            print(f"⚠️  没有配色规则的 visual（用 {FALLBACK} 兜底）：{unmatched}")
        if total == 0:
            print("✗ 一个 visual 都没有 —— SDF 生成有问题")
            return 1
        if with_mat < total:
            print(f"✗ 还差 {total - with_mat} 个未补材质（跑一遍不带 --check 的版本）")
            return 1
        print("✓ 每个 visual 都有材质")
        return 0

    if new == text:
        print(f"已经全部带材质，无需改动（{len(done)} 处）")
        return 0
    p.write_text(new, encoding="utf-8")
    print(f"已补材质：{len(done)} 个 visual → {p}")
    if unmatched:
        print(f"⚠️  兜底配色 {FALLBACK}：{unmatched}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
