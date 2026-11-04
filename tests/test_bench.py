"""基准自洽性：人工声明必须与真值一致，否则拒绝运行。"""
from __future__ import annotations

from collections import Counter

from depot_vln.eval.oracle import load_cases, validate_all
from depot_vln.eval.runner import run_bench


def test_all_declarations_match_ground_truth():
    assert validate_all() == {}


def test_benchmark_is_balanced():
    counts = Counter(c["category"] for c in load_cases())
    assert set(counts) == {"direct", "attribute", "ambiguous", "unreachable"}
    assert len(load_cases()) >= 40
    for cat, n in counts.items():
        assert n >= 10, f"{cat} 只有 {n} 条"


def test_rule_backend_has_zero_silent_guesses():
    s = run_bench(backend="rule", inventory="sim")
    assert s["silent_wrong"] == 0, [r for r in s["rows"] if r["silent_wrong"]]
    assert s["rate"] >= 0.95, f"规则后端通过率 {s['rate']:.1%}"
