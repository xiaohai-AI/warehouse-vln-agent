"""库存快照的出处必须显式、可追溯。"""
from __future__ import annotations

import pytest

from depot_vln.inventory import Inventory
from depot_vln.sim import load_truth_slots, sim_inventory


def test_sim_inventory_is_labelled_as_stand_in():
    inv = sim_inventory()
    assert inv.provenance["provider"] == "sim-sampled"
    assert "不是感知结果" in inv.provenance["note"]
    assert len(inv) == len(load_truth_slots())


def test_partial_coverage_keeps_slots_unknown():
    inv = sim_inventory(coverage=0.3, seed=1)
    assert 0 < len(inv) < len(load_truth_slots())
    assert len(inv.known()) == len(inv)


def test_perception_requires_artifact(tmp_path):
    with pytest.raises(FileNotFoundError):
        Inventory.from_perception(tmp_path / "nope.json")


def test_roundtrip(tmp_path):
    inv = sim_inventory(coverage=0.5, seed=3)
    p = tmp_path / "inv.json"
    inv.to_json(p)
    back = Inventory.from_perception(p)
    assert set(back.entries) == set(inv.entries)
    assert back.provenance["provider"] == "sim-sampled"
