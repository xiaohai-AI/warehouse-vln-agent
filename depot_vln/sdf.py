"""SDF 发射器：把 layout + truth 变成 world/warehouse.sdf。

视觉真实感原则（Dan 明确要求"不要积木风"）
  * 货架是**真货架**：立柱 + 每层两根橙色横梁 + X 形斜撑 + 层板 + 底部黄色防撞护栏 + 顶横撑
  * 货物按类别有各自形体：纸箱带封箱胶带与面单、托盘是垫木+铺板、周转箱是四壁+把手、
    油桶有箍圈与桶盖
  * 叉车是叉车：门架双立柱+横撑、竖挡板+两根叉齿、护顶架+顶棚、座椅+方向盘、四轮、警示灯
  * 纯装饰件一律 `collision=false`，不参与物理/代价地图/自检量测；
    真正参与碰撞的只有：层板、立柱、横梁、墙、柱、路障、叉车车体+门架、周转箱

命名约定（自检脚本靠前缀分类）
  wall_ / rack_ / column_ / barrier_ / obstacle_  -> 有碰撞的实体
  cargo_ / trim_ / marking_ / line_ / hazard_ / bc_ / sign_ -> 纯视觉

货位编号牌（sign_slot_*）为什么必须有：`verify` 只能答"这一格有货/空"，
画面里没有编号就**无法回答"这是哪一格"**（LIMITATIONS L1）—— 于是"开到错的格口"
这类错误物理上不可观测。尺寸由相机可读性反推，见 depot_vln/slot_signs.py。
"""
from __future__ import annotations

import math
from typing import Any

from . import slot_signs as signs

HEADER = """<?xml version="1.0" ?>
<!-- 由 scripts/build_world.py 生成，禁止手改。改 config/world.yaml 后重新 make world。 -->
<sdf version="1.8">
  <world name="depot">
"""

WORLD_PLUGINS_HEAD = """    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
"""

WORLD_PLUGINS_TAIL = """    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>
    <plugin filename="ignition-gazebo-contact-system" name="ignition::gazebo::systems::Contact"/>
"""


def world_plugins(render_engine: str = "ogre2") -> str:
    """Sensors 系统的渲染引擎可换：ogre2 画质好但对 EGL 挑剔，ogre(1.x) 更皮实。

    注意顺序：SceneBroadcaster 之类的系统只能在 world 里各出现一次，
    之前把 Sensors 插在中间时复制了一份，会出现两个 SceneBroadcaster（互相打架）。
    """
    sensors = (
        '    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">\n'
        f"      <render_engine>{render_engine}</render_engine>\n"
        "    </plugin>\n"
    )
    return WORLD_PLUGINS_HEAD + sensors + WORLD_PLUGINS_TAIL


# ---------------------------------------------------------------- 材质
FLOOR_COLOR = [0.40, 0.41, 0.43]
WALL_COLOR = [0.60, 0.60, 0.62]
WALL_TRIM = [0.34, 0.36, 0.40]
UPRIGHT_COLOR = [0.18, 0.30, 0.52]      # 货架立柱：蓝
BEAM_ORANGE = [0.85, 0.42, 0.08]        # 横梁：橙（真实货架就是橙/蓝配色）
DECK_COLOR = [0.52, 0.53, 0.55]         # 镀锌层板
DECK_EDGE = [0.80, 0.81, 0.82]
BRACE_COLOR = [0.24, 0.34, 0.52]
GUARD_YELLOW = [0.88, 0.72, 0.10]
COLUMN_COLOR = [0.68, 0.67, 0.65]
BARRIER_COLOR = [0.78, 0.24, 0.16]
RUBBER = [0.10, 0.10, 0.11]
STEEL = [0.62, 0.63, 0.66]
WOOD = [0.72, 0.55, 0.33]
WOOD_DARK = [0.56, 0.41, 0.24]
TAPE = [0.86, 0.80, 0.66]
LABEL = [0.94, 0.94, 0.92]


def _num(v: float, nd: int = 4) -> str:
    s = f"{v:.{nd}f}"
    return "0.0000" if s == "-0.0000" else s


def _pose(center: list[float], rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> str:
    return (
        f"{_num(center[0])} {_num(center[1])} {_num(center[2])} "
        f"{_num(rpy[0])} {_num(rpy[1])} {_num(rpy[2])}"
    )


# 材质必须**极简且高度复用**：
# 原来每个视觉件都带自己的 specular + roughness，2832 个视觉件 -> 2832 个互不相同的
# PBR 材质 -> ogre2 逐个编译着色器 -> Sensors 渲染线程初始化直接卡死（实测 120 s 起不来）。
# 现在只发 ambient + diffuse，并且颜色量化到 1/16，让 gz 能把相同定义合并成少量材质。
_QUANT = 16


def _q(v: float) -> float:
    return round(min(1.0, max(0.0, v)) * _QUANT) / _QUANT


def _material(color: list[float], alpha: float = 1.0) -> str:
    r, g, b = _q(color[0]), _q(color[1]), _q(color[2])
    return (
        "<material>"
        f"<ambient>{_num(r, 3)} {_num(g, 3)} {_num(b, 3)} {_num(alpha, 2)}</ambient>"
        f"<diffuse>{_num(r, 3)} {_num(g, 3)} {_num(b, 3)} {_num(alpha, 2)}</diffuse>"
        "</material>"
    )


# ---------------------------------------------------------------- 基本体
def _box_link(
    name: str,
    center: list[float],
    size: list[float],
    color: list[float],
    collision: bool = True,
    mu: float = 0.9,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    visual: bool = True,
) -> str:
    """visual=False 时只发碰撞体：碰撞体不产生渲染节点，可以用来"物理按 bay 拆、渲染按整条铺"。"""
    geo = (
        "<geometry><box>"
        f"<size>{_num(size[0])} {_num(size[1])} {_num(size[2])}</size>"
        "</box></geometry>"
    )
    parts = [f'<link name="{name}"><pose>{_pose(center, rpy)}</pose>']
    if collision:
        parts.append(
            f'<collision name="{name}_col">{geo}'
            "<surface><friction><ode>"
            f"<mu>{_num(mu)}</mu><mu2>{_num(mu)}</mu2>"
            "</ode></friction></surface></collision>"
        )
    if visual:
        parts.append(f'<visual name="{name}_vis">{geo}{_material(color)}</visual>')
    parts.append("</link>")
    return "".join(parts)


def _cyl_link(
    name: str,
    center: list[float],
    radius: float,
    length: float,
    color: list[float],
    collision: bool = True,
    mu: float = 0.9,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> str:
    geo = (
        "<geometry><cylinder>"
        f"<radius>{_num(radius)}</radius><length>{_num(length)}</length>"
        "</cylinder></geometry>"
    )
    parts = [f'<link name="{name}"><pose>{_pose(center, rpy)}</pose>']
    if collision:
        parts.append(
            f'<collision name="{name}_col">{geo}'
            "<surface><friction><ode>"
            f"<mu>{_num(mu)}</mu><mu2>{_num(mu)}</mu2>"
            "</ode></friction></surface></collision>"
        )
    parts.append(f'<visual name="{name}_vis">{geo}{_material(color)}</visual>')
    parts.append("</link>")
    return "".join(parts)


def _model(name: str, links: list[str]) -> str:
    if not links:
        return ""
    return f'    <model name="{name}"><static>true</static>' + "".join(links) + "</model>"


# ---------------------------------------------------------------- 地面 / 灯
def _floor_and_lights(cfg: dict[str, Any], layout: dict[str, Any]) -> list[str]:
    fp = layout["footprint"]
    sx = fp["x_max"] - fp["x_min"]
    sy = fp["y_max"] - fp["y_min"]
    out: list[str] = [
        _model("floor", [_box_link("floor", [0.0, 0.0, -0.05], [sx, sy, 0.1], FLOOR_COLOR, mu=0.95)])
    ]
    amb = cfg["lights"]["ambient"]
    bg = cfg["lights"]["background"]
    shadows = "true" if cfg.get("shadows", True) else "false"
    out.append(
        "    <scene><ambient>%s 1</ambient><background>%s 1</background>"
        f"<shadows>{shadows}</shadows><grid>false</grid></scene>"
        % (" ".join(_num(c) for c in amb), " ".join(_num(c) for c in bg))
    )
    out.append(
        '    <light type="directional" name="sun">'
        f"<cast_shadows>{'true' if cfg.get('shadows', True) else 'false'}</cast_shadows>"
        "<pose>0 0 %s 0 0.9 0</pose>" % _num(fp["z_max"] + 2)
        + "<diffuse>1.0 1.0 0.98 1</diffuse><specular>0.15 0.15 0.15 1</specular>"
        f"<intensity>{_num(cfg['lights']['directional_intensity'], 3)}</intensity>"
        "<direction>0.25 0.1 -0.96</direction>"
        "</light>"
    )
    for lamp in layout["lights"]:
        # cast_shadows=false 是性能关键：ogre2 给每盏投影 spot 灯建一张阴影贴图，
        # 18 盏 = 18 张，初始化直接把 Sensors 渲染线程卡死（实测 120 s 起不来）。
        # 仓库照明本来就该是均匀泛光，阴影交给方向光就够了。
        out.append(
            f'    <light type="spot" name="{lamp["name"]}">'
            f'<pose>{_pose(lamp["pose"])}</pose>'
            "<cast_shadows>false</cast_shadows>"
            "<diffuse>1.0 0.98 0.94 1</diffuse><specular>0.1 0.1 0.1 1</specular>"
            f'<intensity>{_num(lamp["intensity"], 3)}</intensity>'
            f'<attenuation><range>{_num(lamp["range"], 2)}</range>'
            "<constant>0.9</constant><linear>0.01</linear><quadratic>0.0005</quadratic></attenuation>"
            f'<direction>{_num(lamp["direction"][0], 2)} {_num(lamp["direction"][1], 2)} '
            f'{_num(lamp["direction"][2], 2)}</direction>'
            "<spot><inner_angle>0.9</inner_angle><outer_angle>1.35</outer_angle><falloff>0.6</falloff></spot>"
            "</light>"
        )
    return out


# ---------------------------------------------------------------- 墙 / 柱
def _walls_and_columns(layout: dict[str, Any]) -> list[str]:
    fp = layout["footprint"]
    links = [_box_link(w["name"], w["center"], w["size"], WALL_COLOR, mu=0.7) for w in layout["walls"]]
    span_x = fp["x_max"] * 2 - 0.2
    span_y = fp["y_max"] * 2 - 0.2
    # 墙不是一块纯色板：加踢脚线、顶部收边
    for name, cy in (("trim_north", fp["y_max"] - 0.30), ("trim_south", fp["y_min"] + 0.30)):
        links.append(_box_link(name, [0.0, cy, 0.45], [span_x, 0.05, 0.9], WALL_TRIM, collision=False))
    links.append(_box_link("trim_wall_top", [0.0, 0.0, fp["z_max"] - 0.12], [span_x, span_y, 0.2],
                           WALL_TRIM, collision=False))
    out = [_model("walls", links)]
    if layout["columns"]:
        cols: list[str] = []
        for c in layout["columns"]:
            cols.append(_box_link(c["name"], c["center"], c["size"], COLUMN_COLOR, mu=0.8))
            cols.append(
                _box_link(f"{c['name']}_guard", [c["center"][0], c["center"][1], 0.55],
                          [c["size"][0] + 0.06, c["size"][1] + 0.06, 1.1], GUARD_YELLOW, collision=False)
            )
        out.append(_model("columns", cols))
    return out


# ---------------------------------------------------------------- 货架
def _racks(cfg: dict[str, Any], layout: dict[str, Any], detail: str = "high") -> list[str]:
    racks = cfg["racks"]
    pitch = float(racks["bay_pitch"])
    deck_t = float(racks["deck_thickness"])
    post = float(racks["post_size"])
    every = max(1, int(racks.get("post_every_n_bays", 2)))
    top_z = float(racks["top_z"])
    rack_d = float(racks["rack_depth"])

    first_aisle = layout["aisles"][0]["name"]
    stations = [b for b in layout["bays"] if b["aisle"] == first_aisle]
    deck_len = pitch - 2 * post - 2 * 0.02
    face_off = rack_d / 2          # 货架面到 block 中心 = 0.55
    beam_h, beam_t = 0.10, 0.05
    run_x0, run_x1 = stations[0]["x0"], stations[-1]["x1"]

    out: list[str] = []
    for block in layout["blocks"]:
        k = block["index"]
        cy = block["center_y"]
        links: list[str] = []

        # --- 层板：碰撞按 bay 拆（保持自检的"每 bay 一片"语义），
        #     视觉每层只铺一条通长板。渲染节点 336 -> 14，是把启动时间从 65 s 拉回十几秒的关键。
        for st in stations:
            for lvl in racks["levels"]:
                z = float(lvl["deck_z"]) - deck_t / 2
                links.append(
                    _box_link(f"rack_deck_{k}_{st['bay']}_L{lvl['level']}",
                              [st["center_x"], cy, z], [deck_len, rack_d, deck_t], DECK_COLOR, mu=0.8,
                              visual=False)
                )
        for lvl in racks["levels"]:
            z = float(lvl["deck_z"]) - deck_t / 2
            links.append(
                _box_link(f"trim_deckrun_{k}_L{lvl['level']}",
                          [0.0, cy, z], [run_x1 - run_x0, rack_d, deck_t], DECK_COLOR,
                          collision=False)
            )

        # --- 横梁：每层、两面各一根通长（带碰撞，就是"货架面"）
        for lvl in racks["levels"]:
            z = float(lvl["deck_z"]) - deck_t - beam_h / 2
            for sy in (-1, 1):
                links.append(
                    _box_link(f"rack_beam_{k}_L{lvl['level']}_{'N' if sy > 0 else 'S'}",
                              [0.0, cy + sy * (face_off - beam_t / 2), z],
                              [run_x1 - run_x0, beam_t, beam_h], BEAM_ORANGE, mu=0.8)
                )

        # --- 立柱（带碰撞）
        post_xs = [st["x0"] for st in stations if (st["bay"] - 1) % every == 0]
        post_xs[0] = run_x0 + post / 2
        post_xs.append(run_x1 - post / 2)
        for i, post_x in enumerate(post_xs):
            for sy in (-1, 1):
                links.append(
                    _box_link(f"rack_post_{k}_{i}_{'N' if sy > 0 else 'S'}",
                              [post_x, cy + sy * (face_off - post / 2), top_z / 2],
                              [post, post, top_z], UPRIGHT_COLOR, mu=0.8)
                )

        if detail != "high":
            out.append(_model(block["name"], links))
            continue

        # --- X 形斜撑（纯视觉）
        span = pitch * 2
        for lvl in racks["levels"]:
            z0 = float(lvl["deck_z"])
            z1 = min(top_z, z0 + float(lvl["clear_height"]))
            dz = z1 - z0
            ang = math.atan2(dz, span)
            blen = math.hypot(dz, span)
            for sy in (-1, 1):
                for b in range(2, len(stations), 4):
                    x = stations[b - 1]["center_x"]
                    for sign, tag in ((1, "a"), (-1, "b")):
                        links.append(
                            _box_link(f"trim_brace_{k}_L{lvl['level']}_{b}{tag}_{'N' if sy > 0 else 'S'}",
                                      [x, cy + sy * (face_off - post / 2), (z0 + z1) / 2],
                                      [blen, 0.022, 0.030], BRACE_COLOR,
                                      collision=False, rpy=(0.0, sign * ang, 0.0))
                        )
        # --- 底部防撞护栏（黄色）+ 顶部横撑
        for sy in (-1, 1):
            links.append(
                _box_link(f"trim_guard_{k}_{'N' if sy > 0 else 'S'}",
                          [0.0, cy + sy * (face_off - 0.05), 0.34],
                          [run_x1 - run_x0, 0.06, 0.08], GUARD_YELLOW, collision=False)
            )
        links.append(
            _box_link(f"trim_topbeam_{k}", [0.0, cy, top_z + 0.04],
                      [run_x1 - run_x0, 0.06, 0.08], UPRIGHT_COLOR, collision=False)
        )
        out.append(_model(block["name"], links))
    return out


# ---------------------------------------------------------------- 货位编号牌
PLATE_COLOR = [0.92, 0.92, 0.90]
HANG_COLOR = [0.42, 0.42, 0.44]


def _visual(name: str, center: list[float], size: list[float],
            color: list[float], yaw: float = 0.0) -> str:
    geo = (
        "<geometry><box>"
        f"<size>{_num(size[0])} {_num(size[1])} {_num(size[2])}</size>"
        "</box></geometry>"
    )
    return (
        f'<visual name="{name}">'
        f"<pose>{_pose(center, (0.0, 0.0, yaw))}</pose>{geo}{_material(color)}</visual>"
    )


def _slot_signs(cfg: dict[str, Any], layout: dict[str, Any], detail: str = "high") -> list[str]:
    """每个货位一块能读出地址的编号牌 + 一条吊带（纯视觉）。

    **整块牌做在一个 link 里**：每个 visual 有自己的贴图（apply_world_textures 按名字注入），
    但物理上只有 1 个无碰撞 link —— 576 块牌不增加任何物理实体，静态图上也不出现，
    因此 M1/M2/SLAM/避障判据全不受影响。渲染上每块牌是 2~3 个矩形，代价可忽略。
    """
    if detail == "low":
        return []
    racks = cfg["racks"]
    levels = list(racks["levels"])
    top_z = float(racks["top_z"])

    vis: list[str] = []
    for slot in layout["slots"]:
        g = signs.sign_geometry(slot, levels, top_z)
        vis.append(_visual(signs.visual_name(slot["address"]),
                           g["plate_center"], g["plate_size"], PLATE_COLOR, g["yaw"]))
        if g["hang_center"] and g["hang_size"]:
            vis.append(_visual(signs.visual_name(slot["address"], "hang"),
                               g["hang_center"], g["hang_size"], HANG_COLOR, g["yaw"]))

    if not vis:
        return []
    return [_model("slot_signs", ['<link name="slot_signs_link">' + "".join(vis) + "</link>"])]


# ---------------------------------------------------------------- 地面标线
def _markings(layout: dict[str, Any]) -> list[str]:
    if not layout["markings"]:
        return []
    links = [_box_link(m["name"], m["center"], m["size"], m["color"], collision=False) for m in layout["markings"]]
    return [_model("markings", links)]


# ---------------------------------------------------------------- 货物（按类别建形体）
def cargo_parts(rec: dict[str, Any], tag: str) -> list[str]:
    cls = rec["class"]
    sx, sy, sz = rec["size"]
    x, y, z, yaw = rec["pose"]
    col = rec["color"]
    out: list[str] = []

    if cls.startswith("carton"):
        out.append(_box_link(f"cargo_{tag}", [x, y, z], [sx, sy, sz], col, False, rpy=(0, 0, yaw)))
        out.append(_box_link(f"cargo_{tag}_tape", [x, y, z + sz / 2 + 0.004],
                             [sx * 1.01, 0.055, 0.006], TAPE, False, rpy=(0, 0, yaw)))
        # 面单：贴在朝向巷道的一侧（L 侧货位朝 -Y，R 侧朝 +Y）
        face_y = y + (-1 if rec.get("side", "L") == "L" else 1) * (sy / 2 + 0.005)
        out.append(_box_link(f"cargo_{tag}_label", [x - sx * 0.18, face_y, z + sz * 0.05],
                             [sx * 0.30, 0.006, sz * 0.30], LABEL, False, rpy=(0, 0, yaw)))
        return out

    if cls == "pallet":
        leg_h = sz * 0.55
        slat_t = sz * 0.16
        for i, dy in enumerate((-sy * 0.40, 0.0, sy * 0.40)):
            out.append(_box_link(f"cargo_{tag}_runner{i}", [x, y + dy, z - sz / 2 + leg_h / 2],
                                 [sx, 0.10, leg_h], WOOD_DARK, False, rpy=(0, 0, yaw)))
        for i in range(5):
            dy = -sy * 0.42 + i * (sy * 0.84 / 4)
            out.append(_box_link(f"cargo_{tag}_slat{i}", [x, y + dy, z + sz / 2 - slat_t / 2],
                                 [sx, 0.11, slat_t], WOOD, False, rpy=(0, 0, yaw)))
        for i in range(3):
            dy = -sy * 0.42 + i * (sy * 0.42)
            out.append(_box_link(f"cargo_{tag}_bot{i}", [x, y + dy, z - sz / 2 + slat_t / 2],
                                 [sx, 0.11, slat_t], WOOD, False, rpy=(0, 0, yaw)))
        return out

    if cls == "tote":
        wall, bot = 0.03, 0.02
        out.append(_box_link(f"cargo_{tag}_bot", [x, y, z - sz / 2 + bot / 2], [sx, sy, bot], col, False,
                             rpy=(0, 0, yaw)))
        out.append(_box_link(f"cargo_{tag}_wx0", [x - sx / 2 + wall / 2, y, z], [wall, sy, sz], col, False,
                             rpy=(0, 0, yaw)))
        out.append(_box_link(f"cargo_{tag}_wx1", [x + sx / 2 - wall / 2, y, z], [wall, sy, sz], col, False,
                             rpy=(0, 0, yaw)))
        out.append(_box_link(f"cargo_{tag}_wy0", [x, y - sy / 2 + wall / 2, z], [sx - 2 * wall, wall, sz],
                             col, False, rpy=(0, 0, yaw)))
        out.append(_box_link(f"cargo_{tag}_wy1", [x, y + sy / 2 - wall / 2, z], [sx - 2 * wall, wall, sz],
                             col, False, rpy=(0, 0, yaw)))
        for s2 in (-1, 1):
            out.append(_box_link(f"cargo_{tag}_handle{s2}", [x, y + s2 * (sy / 2), z + sz * 0.26],
                                 [sx * 0.30, 0.022, 0.05], [0.12, 0.20, 0.38], False, rpy=(0, 0, yaw)))
        return out

    if cls == "drum":
        rad = sx / 2
        out.append(_cyl_link(f"cargo_{tag}", [x, y, z], rad, sz, col, collision=False))
        for dz in (-sz * 0.25, sz * 0.25):
            out.append(_cyl_link(f"cargo_{tag}_rim{int(dz * 100)}", [x, y, z + dz], rad * 1.04, 0.030,
                                 [0.22, 0.34, 0.30], collision=False))
        out.append(_cyl_link(f"cargo_{tag}_lid", [x, y, z + sz / 2 + 0.01], rad * 0.86, 0.020,
                             [0.20, 0.30, 0.26], collision=False))
        return out

    out.append(_box_link(f"cargo_{tag}", [x, y, z], [sx, sy, sz], col, False, rpy=(0, 0, yaw)))
    return out


def cargo_parts_medium(rec: dict[str, Any], tag: str) -> list[str]:
    """每件货 2 个视觉件（不是 1 个方块）：保留"一眼能认出类别"的那一条特征。

    为什么砍到 2：货架细节全留着（1314 视觉件）时仿真 RTF 0.34~0.90 能跑；
    把货物做足细节（+1500 视觉件）会让 ogre2 初始化直接卡死（实测 120 s 起不来）。
    这是"看起来像"与"跑得动"之间的实测平衡点，不是审美妥协 —— 要更多细节应走网格实例化
    （一个 OBJ 被 N 个实例复用），而不是拆成一堆独立 link。
    """
    cls = rec["class"]
    sx, sy, sz = rec["size"]
    x, y, z, yaw = rec["pose"]
    col = rec["color"]
    if cls.startswith("carton"):
        return [
            _box_link(f"cargo_{tag}", [x, y, z], [sx, sy, sz], col, False, rpy=(0, 0, yaw)),
            _box_link(f"cargo_{tag}_tape", [x, y, z + sz / 2 + 0.004], [sx * 1.01, 0.055, 0.006], TAPE,
                      False, rpy=(0, 0, yaw)),
        ]
    if cls == "pallet":
        return [
            _box_link(f"cargo_{tag}", [x, y, z - sz * 0.15], [sx, sy, sz * 0.7], WOOD_DARK, False,
                      rpy=(0, 0, yaw)),
            _box_link(f"cargo_{tag}_deck", [x, y, z + sz * 0.3], [sx, sy * 0.98, sz * 0.28], WOOD,
                      False, rpy=(0, 0, yaw)),
        ]
    if cls == "tote":
        # 上沿比箱体窄一点 + 深色内壁：远看就是"开口箱"
        return [
            _box_link(f"cargo_{tag}", [x, y, z - sz * 0.08], [sx, sy, sz * 0.84], col, False,
                      rpy=(0, 0, yaw)),
            _box_link(f"cargo_{tag}_rim", [x, y, z + sz * 0.40], [sx * 0.94, sy * 0.92, sz * 0.10],
                      [max(0.0, col[0] * 0.55), max(0.0, col[1] * 0.55), max(0.0, col[2] * 0.55)],
                      False, rpy=(0, 0, yaw)),
        ]
    if cls == "drum":
        rad = sx / 2
        return [
            _cyl_link(f"cargo_{tag}", [x, y, z], rad, sz, col, collision=False),
            _cyl_link(f"cargo_{tag}_lid", [x, y, z + sz / 2 + 0.012], rad * 0.9, 0.025,
                      [max(0.0, col[0] * 0.6), max(0.0, col[1] * 0.6), max(0.0, col[2] * 0.6)],
                      collision=False),
        ]
    return [_box_link(f"cargo_{tag}", [x, y, z], [sx, sy, sz], col, False, rpy=(0, 0, yaw))]


def _cargo(truth: dict[str, Any], detail: str = "high") -> list[str]:
    if not truth["cargo"]:
        return []
    links: list[str] = []
    for c in truth["cargo"]:
        tag = c["slot"].replace("-", "_")
        if detail == "low":
            links.append(_box_link(f"cargo_{tag}", c["pose"][:3], c["size"], c["color"], False,
                                   rpy=(0, 0, c["pose"][3])))
        elif detail == "full":
            links.extend(cargo_parts(c, tag))
        else:  # medium（默认）：每件 2 个视觉件，保留类别轮廓
            links.extend(cargo_parts_medium(c, tag))
    return [_model("cargo", links)]


# ---------------------------------------------------------------- 静态障碍
def _forklift(cfg: dict[str, Any], layout: dict[str, Any]) -> str:
    ob = cfg.get("obstacles") or {}
    fork = ob.get("forklift")
    if not fork:
        return ""
    aisles = {a["name"]: a for a in layout["aisles"]}
    bays = {(b["aisle"], b["bay"]): b for b in layout["bays"]}
    a_name = fork.get("at_aisle", fork.get("aisle"))
    aisle = aisles[a_name]
    bay = bays[(a_name, int(fork["bay"]))]
    side = 1 if str(fork.get("side", "L")).upper() == "L" else -1
    y = aisle["center_y"] + side * 0.55
    x = bay["center_x"]

    body_col = [0.86, 0.62, 0.06]
    dark = [0.18, 0.19, 0.21]
    L: list[str] = []
    # 车体 / 配重 / 机罩
    L.append(_box_link("obstacle_fl_body", [x, y, 0.55], [1.05, 0.62, 0.55], body_col, mu=0.9))
    L.append(_box_link("obstacle_fl_counter", [x - 0.48, y, 0.50], [0.30, 0.60, 0.62], dark))
    L.append(_box_link("obstacle_fl_hood", [x - 0.06, y, 0.86], [0.60, 0.52, 0.14], body_col,
                       collision=False))
    # 座椅 + 方向盘
    L.append(_box_link("obstacle_fl_seat", [x - 0.28, y, 0.98], [0.34, 0.40, 0.10], dark, collision=False))
    L.append(_box_link("obstacle_fl_seatback", [x - 0.44, y, 1.16], [0.09, 0.40, 0.34], dark,
                       collision=False))
    L.append(_cyl_link("obstacle_fl_column", [x + 0.06, y, 1.02], 0.035, 0.34, dark,
                       collision=False, rpy=(0.5, 0.0, 0.0)))
    L.append(_cyl_link("obstacle_fl_steer", [x + 0.11, y, 1.17], 0.13, 0.03, [0.12, 0.12, 0.13],
                       collision=False, rpy=(1.15, 0.0, 0.0)))
    # 护顶架 + 顶棚
    for dx in (0.30, -0.34):
        for dy in (-0.30, 0.30):
            L.append(_box_link(f"obstacle_fl_pillar{int(dx * 100)}{int(dy * 100)}",
                               [x + dx, y + dy, 1.62], [0.06, 0.06, 0.72], dark, collision=False))
    L.append(_box_link("obstacle_fl_roof", [x - 0.02, y, 2.00], [0.80, 0.66, 0.05],
                       [0.30, 0.32, 0.34], collision=False))
    # 门架（承重，带碰撞）
    for dy in (-0.26, 0.26):
        L.append(_box_link(f"obstacle_fl_mast{int(dy * 100)}", [x + 0.72, y + dy, 1.30],
                           [0.08, 0.09, 1.70], STEEL, mu=0.9))
    for zz in (0.75, 1.25, 1.75, 2.10):
        L.append(_box_link(f"obstacle_fl_mastcross{int(zz * 100)}", [x + 0.72, y, zz],
                           [0.06, 0.60, 0.06], STEEL, collision=False))
    # 货叉：竖挡板 + 两根叉齿
    L.append(_box_link("obstacle_fl_carriage", [x + 0.66, y, 0.60], [0.06, 0.56, 0.90], STEEL,
                       collision=False))
    for dy in (-0.22, 0.22):
        L.append(_box_link(f"obstacle_fl_fork{int(dy * 100)}", [x + 1.05, y + dy, 0.10],
                           [0.95, 0.12, 0.05], [0.34, 0.34, 0.36], collision=False))
    # 四轮
    for dx, dy in ((0.34, 0.32), (0.34, -0.32), (-0.34, 0.30), (-0.34, -0.30)):
        L.append(_cyl_link(f"obstacle_fl_wheel{int(dx * 100)}{int(dy * 100)}",
                           [x + dx, y + dy, 0.22], 0.22, 0.14, RUBBER,
                           collision=False, rpy=(1.5708, 0.0, 0.0)))
    # 警示灯
    L.append(_box_link("obstacle_fl_lamp", [x + 0.78, y, 2.06], [0.10, 0.10, 0.10],
                       [1.0, 0.55, 0.05], collision=False))
    return _model("forklift", L)


def _tote_bin(name: str, pos: list[float], yaw: float, color: list[float]) -> list[str]:
    sx, sy, sz, wall = 0.60, 0.40, 0.30, 0.035
    z0 = pos[2]
    out = [
        _box_link(f"{name}_bot", [pos[0], pos[1], z0 + 0.011], [sx, sy, 0.022], color, False, rpy=(0, 0, yaw)),
        _box_link(f"{name}_x0", [pos[0] - sx / 2 + wall / 2, pos[1], z0 + sz / 2], [wall, sy, sz], color,
                  False, rpy=(0, 0, yaw)),
        _box_link(f"{name}_x1", [pos[0] + sx / 2 - wall / 2, pos[1], z0 + sz / 2], [wall, sy, sz], color,
                  False, rpy=(0, 0, yaw)),
        _box_link(f"{name}_y0", [pos[0], pos[1] - sy / 2 + wall / 2, z0 + sz / 2], [sx - 2 * wall, wall, sz],
                  color, False, rpy=(0, 0, yaw)),
        _box_link(f"{name}_y1", [pos[0], pos[1] + sy / 2 - wall / 2, z0 + sz / 2], [sx - 2 * wall, wall, sz],
                  color, False, rpy=(0, 0, yaw)),
        # 碰撞体：整体一个箱子（视觉上是空箱，物理上不许穿）
        _box_link(f"{name}_solid", [pos[0], pos[1], z0 + sz / 2], [sx, sy, sz], color, mu=0.8,
                  rpy=(0, 0, yaw)),
    ]
    return out


def _obstacles(cfg: dict[str, Any], layout: dict[str, Any]) -> list[str]:
    ob = cfg.get("obstacles") or {}
    out: list[str] = []
    fl = _forklift(cfg, layout)
    if fl:
        out.append(fl)
    n_totes = int(ob.get("totes") or 0)
    if n_totes:
        links: list[str] = []
        spots = [(-32.0, 17.4, 0.0), (32.0, -17.4, 0.4), (-32.0, -4.0, 0.2)]
        for i in range(n_totes):
            x, y, yaw = spots[i % len(spots)]
            links.extend(_tote_bin(f"obstacle_tote_{i}", [x, y, 0.0], yaw, [0.18, 0.34, 0.62]))
        out.append(_model("totes", links))
    return out


# ---------------------------------------------------------------- 路障
def _barriers(truth: dict[str, Any]) -> list[str]:
    """封堵段的路障：**低矮挡轮护栏**，只挡车、不挡雷达。

    高度是关键（2026-10-29 改）：原来 1.14 m 高，把封堵段后面的货架对雷达（装在 0.474 m）
    也一起挡住了 —— "车进不去的段"顺带变成"看不见的段"，SEG-1 因此永远少 7 个 bay。
    降到 0.32 m 之后：
      * 车过不去（离地 0.076 m 的麦克纳姆不可能爬 0.32 m 的坎）；
      * `/scan`（z 0.10–0.60 切带）仍然把它当成实心障碍 → 建图/Nav2 依旧封路；
      * 3D 雷达在它上方，能看过去 → 封堵段后面的货架面可观测。
    几何语义（封哪一段、哪些货位 unreachable）完全不变，GRAPH-1 / LANG-1 / SLAM 路线都不用改。
    """
    if not truth["blocked"]:
        return []
    links: list[str] = []
    for i, seg in enumerate(truth["blocked"]):
        y = seg["center_y"]
        for j, x in enumerate(seg.get("barrier_xs") or [seg["x0"] - 0.35]):
            width = seg["width"] - 0.02
            links.append(_box_link(f"barrier_{i}{j}_base", [x, y, 0.05], [0.72, width, 0.10],
                                   [0.30, 0.30, 0.32], mu=0.95))
            # 主体从底座顶面 0.10 m 起，到 0.32 m 止（净高 0.22 m）——必须低于雷达 0.474 m，
            # 又必须落在 /scan 的 z 0.10–0.60 切带里；底面贴 0.10 是为了不与底座重叠（C1 闸门）。
            links.append(_box_link(f"barrier_{i}{j}_body", [x, y, 0.21], [0.52, width, 0.22],
                                   BARRIER_COLOR, mu=0.9))
            for zz in (0.10, 0.22):
                links.append(_box_link(f"barrier_{i}{j}_stripe{int(zz * 100)}", [x, y, zz],
                                       [0.54, width * 0.99, 0.05], [0.95, 0.92, 0.88], collision=False))
    return [_model("barriers", links)]


# ---------------------------------------------------------------- 组装
def build_sdf(cfg: dict[str, Any], layout: dict[str, Any], truth: dict[str, Any]) -> str:
    detail = str(cfg.get("visual_detail", "medium")).lower()
    chunks: list[str] = [HEADER]
    chunks.append(
        "    <physics type=\"ode\">"
        "<max_step_size>0.002</max_step_size><real_time_factor>1.0</real_time_factor>"
        "<real_time_update_rate>500</real_time_update_rate>"
        "<ode><solver><type>quick</type><iters>50</iters><sor>1.3</sor></solver></ode>"
        "</physics>"
    )
    chunks.append(world_plugins(str(cfg.get("render_engine", "ogre2"))))
    chunks.extend(_floor_and_lights(cfg, layout))
    chunks.extend(_walls_and_columns(layout))
    chunks.extend(_racks(cfg, layout, detail))
    chunks.extend(_slot_signs(cfg, layout, detail))
    chunks.extend(_markings(layout))
    chunks.extend(_cargo(truth, detail))
    chunks.extend(_obstacles(cfg, layout))
    chunks.extend(_barriers(truth))
    chunks.append("  </world>\n</sdf>\n")
    return "\n".join(chunks)
