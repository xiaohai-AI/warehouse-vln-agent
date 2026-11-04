"""理解层：抽取 → 编译。用例覆盖"同一意图的不同说法"，而不是某条正则的形状。"""
from __future__ import annotations

import pytest

from depot_vln.agent.nlu import RuleNLU
from depot_vln.agent.nlu.compile import parse_clarification


@pytest.fixture(scope="module")
def nlu():
    return RuleNLU()


@pytest.mark.parametrize("utterance,q", [
    ("去 B-L-07-2", {"kind": "address", "address": "B-L-07-2"}),
    ("带我去 c l 7 2 那个货位", {"kind": "address", "address": "C-L-07-2"}),
    ("去 10 号位", {"kind": "bay", "bay": 10}),
    ("去 A 巷放纸箱的那面货架", {"kind": "class_in_aisle", "aisle": "A"}),
    ("去放纸箱最多的那排货架", {"kind": "max_class"}),
    ("去 E 巷哪面托盘最多", {"kind": "max_class", "aisle": "E"}),
    ("去 A 巷最里面那个空位", {"kind": "farthest", "aisle": "A", "empty": True}),
    ("去封住的 F 巷", {"kind": "blocked_aisle", "aisle": "F"}),
    ("去 Z-L-01-1", {"kind": "address", "address": "Z-L-01-1"}),
    ("去 Q 巷", {"kind": "aisle", "aisle": "Q"}),
    ("去 A 巷放冰箱的那面货架", {"kind": "max_class", "classes": ["__unknown__"]}),
    ("去 F 巷 20 号位第一层", {"kind": "bay", "bay": 20, "aisle": "F", "level": 1}),
    ("去 A 巷 5 号位第一层放油桶的那格",
     {"kind": "class_in_bay", "aisle": "A", "bay": 5, "classes": ["drum"]}),
])
def test_compile(nlu, utterance, q):
    got = nlu.understand(utterance, {})["query"]
    for k, v in q.items():
        assert got.get(k) == v, f"{utterance}: {k} 期望 {v!r}，实际 {got.get(k)!r}（{got}）"


def test_underdetermined_queries_stay_underdetermined(nlu):
    """"去 10 号位"里没有巷道/侧/层 —— 编译器**不许**替用户补一个。"""
    q = nlu.understand("去 10 号位", {})["query"]
    assert q == {"kind": "bay", "bay": 10}


def test_clarification_parsing():
    assert parse_clarification("C 巷左边第一层") == {"aisle": "C", "side": "L", "level": 1}
    assert parse_clarification("第二层") == {"level": 2}
    assert parse_clarification("右边") == {"side": "R"}
    assert parse_clarification("B-R-07-2") == {"aisle": "B", "side": "R", "bay": 7, "level": 2}


def test_evidence_is_recorded(nlu):
    out = nlu.understand("去 B 巷 7 号位放油桶的那格", {})
    assert out["slots"]["evidence"]["aisle"] == "B"
    assert out["slots"]["evidence"]["bay"] == "7"
    assert "drum" in out["slots"]["evidence"]["classes"]


def test_llm_backend_falls_back_without_server(monkeypatch):
    """没有 ollama 时必须**显式**回退并记账，不能悄悄消失。"""
    from depot_vln.agent.nlu.llm import LLMNLU

    n = LLMNLU(host="http://127.0.0.1:9", model="nope")
    out = n.understand("去 B-L-07-2", {})
    assert out["backend"].endswith("fallback)")
    assert n.stats()["fallbacks"] == 1
    assert out["query"]["kind"] == "address"
