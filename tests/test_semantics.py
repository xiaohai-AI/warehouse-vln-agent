"""语义地图的三种状态：占用 / 空 / **看过但判不出**（occupied=None）。

第三种状态在 `InventoryEntry` 与 docs/ARCHITECTURE §4 里是一等公民，不能在某一次
类型转换里被折叠成"空"——那是把未知当结论，正是本项目最想避免的错误。
"""
from __future__ import annotations

from depot_vln.inventory import Inventory, InventoryEntry
from depot_vln.semantics import SemanticMap
from depot_vln.world import WorldModel, load_grid


def _inventory(entries: dict[str, bool | None]) -> Inventory:
    return Inventory(
        entries={
            addr: InventoryEntry(addr, occ, None, 0.5, 1.0)
            for addr, occ in entries.items()
        },
        provenance={},
    )


def test_undetermined_occupancy_is_not_empty():
    grid = load_grid()
    m = SemanticMap.from_inventory(grid, _inventory({"A-L-01-1": None}))
    st = m.get("A-L-01-1")
    assert st is not None and st.occupied is None
    assert m.empty_slots() == [], "看过但判不出 ≠ 空位"
    assert m.known_slots() == [], "known 只算判出占用/空的"
    assert "A-L-01-1" not in m.unobserved_slots(), "它被看过，只是没判出"


def test_conflict_only_between_known_values():
    grid = load_grid()
    m = SemanticMap.from_inventory(grid, _inventory({"A-L-01-1": None}))
    st = m.observe("A-L-01-1", False, None, 0.9, 2.0)
    assert st.occupied is False
    assert st.stale is False and st.conflicts == [], "未知 -> 已知是补信息，不是冲突"
    st = m.observe("A-L-01-1", True, "carton_s", 0.9, 3.0)
    assert st.occupied is True and st.stale is True
    assert st.conflicts[0]["occupied"] is False


def test_undetermined_is_not_an_empty_slot_candidate():
    grid = load_grid()
    entries = {f"A-{side}-01-{lv}": None for side in ("L", "R") for lv in (1, 2)}
    world = WorldModel(grid, _inventory(entries))
    cands, reason = world.resolve({"kind": "empty_slot_in_bay", "aisle": "A", "bay": 1})
    assert cands == [] and reason == "UNOBSERVED", "判不出就不能给「这个 bay 有空位」"
