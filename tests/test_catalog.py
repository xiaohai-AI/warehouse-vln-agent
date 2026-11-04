"""世界事实必须**从配置派生**，不能再写死在代码里。"""
from __future__ import annotations

import textwrap

import pytest
import yaml

from depot_vln.catalog import Catalog, get_catalog, reset_catalog_cache


def test_matches_world_config():
    cat = get_catalog()
    cfg = yaml.safe_load((cat.world_path and open(cat.world_path, encoding="utf-8")))
    assert list(cat.aisles) == [str(a) for a in cfg["racks"]["aisle_names"]]
    assert cat.bays_per_aisle == int(cfg["racks"]["bays_per_aisle"])
    assert cat.level_numbers() == tuple(int(lv["level"]) for lv in cfg["racks"]["levels"])
    assert set(cat.class_names) == set(cfg["semantics"]["classes"])


def test_spawn_is_derived_not_constant():
    cat = get_catalog()
    der = cat.layout["derived"]
    expected_x = -(der["rack_run"] / 2 + der["cross_aisle_width"] / 2)
    assert cat.spawn["x"] == pytest.approx(expected_x, abs=1e-6)
    assert cat.spawn["y"] == 0.0


def test_address_roundtrip_and_validation():
    cat = get_catalog()
    assert str(cat.parse_address("去 B-L-07-2")) == "B-L-07-2"
    assert str(cat.parse_address("go to b l 7 2")) == "B-L-07-2"
    # 缺层：合法，但 level 为 None（由上层展开成歧义）
    assert cat.parse_address("B-L-07").level is None
    # 非法：巷道/层/bay 任一不合规都返回 None，且能被 loose_address 识别成"地址形状"
    for bad in ("Z-L-01-1", "A-L-01-3", "A-L-30-1"):
        assert cat.parse_address(bad) is None
        assert cat.loose_address(bad) is not None
    assert cat.loose_address("去 A 巷看看") is None


def test_new_world_adapts_without_code_change(tmp_path):
    """换一个世界（3 条巷道 / 10 bay / 3 层 / 新货类）—— 只改配置，Catalog 自动适配。"""
    world = yaml.safe_load(open(get_catalog().world_path, encoding="utf-8"))
    world["racks"] = dict(world["racks"])
    world["racks"].update({
        "aisle_names": ["AA", "BB", "CC"],
        "bays_per_aisle": 10,
        "levels": [
            {"level": 1, "deck_z": 0.15, "clear_height": 1.6},
            {"level": 2, "deck_z": 1.5, "clear_height": 1.6},
            {"level": 3, "deck_z": 2.9, "clear_height": 1.6},
        ],
    })
    world["semantics"] = {
        "classes": {"bag": {"size": [0.3, 0.3, 0.3], "color": [1, 1, 1]}},
        "occupancy_prob": 0.5,
    }
    p = tmp_path / "world.yaml"
    p.write_text(yaml.safe_dump(world, allow_unicode=True), encoding="utf-8")
    sem = tmp_path / "semantics.yaml"
    sem.write_text(textwrap.dedent("""
        classes:
          bag: [编织袋, 袋子, bag]
        lexicon:
          goto: [去]
          empty: [空位]
          farthest: [最里面]
          most: [最多]
          blocked: [封住]
          unknown_objects: [冰箱]
        sides: {L: [左], R: [右]}
        levels: {1: [第一层], 2: [第二层], 3: [第三层]}
        units: {aisle: [巷], bay: [号位]}
    """).strip(), encoding="utf-8")

    reset_catalog_cache()
    cat = Catalog.load(p, sem)
    assert cat.bays_per_aisle == 10
    assert cat.level_numbers() == (1, 2, 3)
    assert cat.is_aisle("AA") and not cat.is_aisle("A")
    addr = cat.parse_address("去 AA-左-3-3")
    assert str(addr) == "AA-L-03-3"
    assert cat.parse_bay("第 7 个位子") == 7
    assert cat.classes_in_text("拿一袋编织袋") == {"bag"}
    assert cat.parse_level("第三层") == 3
    reset_catalog_cache()


def test_class_words_come_from_semantics_config():
    cat = get_catalog()
    # 同一个词可以覆盖多个类（"纸箱" = S/M/L），这是配置里写的，不是代码里的分支
    assert cat.classes_in_text("纸箱") == {"carton_s", "carton_m", "carton_l"}
    assert cat.classes_in_text("托盘") == {"pallet"}
    assert cat.classes_in_text("冰箱") == set()
    assert cat.mentions_unknown_object("冰箱")
