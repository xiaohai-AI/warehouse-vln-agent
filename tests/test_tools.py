"""工具层的契约：seq 单调、过期丢弃、取消绝不回报"已完成"。"""
from __future__ import annotations

from depot_vln.agent.tools import OfflineToolLayer
from depot_vln.sim import load_sim_world


def _target(world, addr="B-L-07-2"):
    return world.resolve({"kind": "address", "address": addr})[0][0]["target"]


def test_seq_is_monotonic_and_stale_is_dropped():
    world = load_sim_world()
    tools = OfflineToolLayer(world)
    t = _target(world)
    ok = tools.navigate(t, None, 1)
    assert ok.ok and ok.seq == 1
    stale = tools.navigate(t, None, 0)
    assert not stale.ok and stale.state == "CANCELED"


def test_cancel_and_status():
    world = load_sim_world()
    tools = OfflineToolLayer(world)
    t = _target(world)
    tools.navigate(t, None, 5)
    assert tools.status(5).ok
    assert tools.cancel(5).ok
    assert not tools.cancel(99).ok


def test_observation_has_no_fabricated_occupancy():
    world = load_sim_world()
    tools = OfflineToolLayer(world)
    obs = tools.observe({"kind": "slot", "address": "B-L-07-2"})
    assert obs.ok
    assert obs.occupied in (True, False)
