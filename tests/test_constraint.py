"""P6-1 约束层的离线自检（不依赖 ollama、不跑仿真）。

三层测试：
  1. 单元：结构抽取 / 落地面板 / 编译器，逐条断言；
  2. 对抗：用一个"什么都敢填"的假模型（hallucinating）跑完 69 条基准，
     断言 LANG-1 的两条安全判据（静默猜 0、缺码 0）**在最坏的模型上也成立**；
  3. 回退：用一个"输出垃圾"的假模型，断言它退回规则后端、**被记账**（不静默）。

用法
  .venv/bin/python tests/test_constraint.py          # 直接跑
  .venv/bin/python -m pytest tests/test_constraint.py -q
"""
from __future__ import annotations

import csv
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from depot_vln.agent.constraint import (  # noqa: E402
    ConstrainedNLU, compile_query, extract_structure, ground, validate_payload,
)

FAILS: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(f"  [{'PASS' if ok else 'FAIL'}] {name:<52} {detail}")
    if not ok:
        FAILS.append(name)


# ---------------------------------------------------------------- 1. 结构抽取
def test_structure() -> None:
    print("\n== 1. 结构抽取（正则，必须可复现）")
    s = extract_structure("去 C-L-21-1")
    check("整串地址 C-L-21-1", (s["aisle"], s["side"], s["bay"], s["level"]) == ("C", "L", 21, 1)
          and s["address"] == "C-L-21-1", str(s["address"]))
    s = extract_structure("到 A-R-12-2 那个货位看看")
    check("地址带层 A-R-12-2", s["address"] == "A-R-12-2", str(s["address"]))
    s = extract_structure("go to B-L-08-2")
    check("英文地址 B-L-08-2", s["address"] == "B-L-08-2", str(s["address"]))
    s = extract_structure("去 Z-L-01-1")
    check("越界巷号**原样保留**（要走到 resolve 才变 NO_SLOT）",
          s["address"] == "Z-L-01-1", str(s["address"]))
    s = extract_structure("去 A-L-01-3")
    check("越界层号原样保留", s["address"] == "A-L-01-3", str(s["address"]))
    s = extract_structure("去 10 号位")
    check("只说号位：bay=10、巷为 None", s["bay"] == 10 and s["aisle"] is None, str(s))
    s = extract_structure("去 B 巷最里面那个位")
    check("巷 + 最里面：不含 bay/side/level",
          s["aisle"] == "B" and s["bay"] is None and s["side"] is None and s["level"] is None, str(s))
    s = extract_structure("去 C 巷左边最里面第二层那个空位")
    check("巷 + 左 + 第二层", (s["aisle"], s["side"], s["level"]) == ("C", "L", 2), str(s))


# ---------------------------------------------------------------- 2. 落地面板
def test_ground() -> None:
    print("\n== 2. 落地面板（模型给的每个值都必须在原话里有依据）")
    keep, drop = ground({"aisle": "C", "side": "R", "bay": 10, "level": 1},
                        "去 10 号位")
    check("「去 10 号位」里模型补的巷/面/层全被丢",
          keep == {"bay": 10} and len(drop) == 3, f"keep={keep} drop={drop}")

    keep, drop = ground({"bay": 24}, "去 B 巷最里面那个位")
    check("「最里面」被模型猜成 bay=24 → 丢", keep == {} and drop, f"keep={keep} drop={drop}")

    keep, drop = ground({"bay": 99}, "去 10 号位")
    check("越界 bay=99 → 丢", "bay" not in keep and drop, f"keep={keep}")

    keep, drop = ground({"bay": "10"}, "去 10 号位")
    check("bay 给成字符串 '10' 也要救回来", keep.get("bay") == 10, f"keep={keep}")

    keep, drop = ground({"bay": "十"}, "去 10 号位")
    check("bay 给成中文「十」→ 丢掉而不是崩", "bay" not in keep, f"keep={keep}")

    keep, drop = ground({"classes": ["carton_m"]}, "去 A 巷放油桶的那面货架")
    check("类别张冠李戴（纸箱 vs 油桶）→ 丢",
          not keep.get("classes"), f"keep={keep} drop={drop}")

    keep, drop = ground({"classes": ["drum"]}, "去 A 巷放油桶的那面货架")
    check("类别对得上 → 留", keep.get("classes") == ["drum"], f"keep={keep}")

    keep, drop = ground({"empty": True}, "去 A 巷放油桶的那面货架")
    check("没提空位却填 empty=True → 丢", not keep.get("empty"), f"keep={keep}")

    keep, drop = ground({"rank": "farthest"}, "去 A 巷放油桶的那面货架")
    check("没提最里面却填 rank → 丢", not keep.get("rank"), f"keep={keep}")

    keep, _ = ground({"aisle": "Z"}, "去 Z-L-01-1")
    check("闭集外的巷号不进 ground（走地址通道）", not keep.get("aisle"), f"keep={keep}")


# ---------------------------------------------------------------- 3. 编译器
def test_compile() -> None:
    print("\n== 3. 编译器（确定性；欠定时**故意**产出欠定 query）")
    q = compile_query(extract_structure("去 C-L-21-1"), {})
    check("地址 → kind=address", q["kind"] == "address" and q["address"] == "C-L-21-1", str(q))

    q = compile_query(extract_structure("去 B 巷最里面那个位"), {"rank": "farthest"})
    check("最里面但没说面/层 → 保留 None（图上必多候选 → 发问）",
          q["kind"] == "farthest" and q["side"] is None and q["level"] is None, str(q))

    q = compile_query(extract_structure("去 10 号位"), {})
    check("只说号位 → kind=bay 且**不带巷**", q["kind"] == "bay" and "aisle" not in q
          or q.get("aisle") is None, str(q))

    q = compile_query(extract_structure("去 A 巷放油桶的那面货架"), {"classes": ["drum"]})
    check("巷+类别 → class_in_aisle 且 side=None", q["kind"] == "class_in_aisle"
          and q["side"] is None, str(q))

    q = compile_query(extract_structure("去放纸箱最多的那排货架"),
                      {"classes": ["carton_s", "carton_m", "carton_l"], "rank": "most"})
    check("最多+类别 → max_class", q["kind"] == "max_class", str(q))

    q = compile_query(extract_structure("去放纸箱的货架"), {"classes": ["carton_m"]})
    check("只有类别、没有巷 → unknown（不猜）", q["kind"] == "unknown", str(q))

    q = compile_query(extract_structure("去 D 巷 14 号位第一层放油桶的那格"),
                      {"classes": ["drum"]})
    check("巷+号位+类别 → class_in_bay", q["kind"] == "class_in_bay" and q["bay"] == 14, str(q))

    q = compile_query(extract_structure("那个封住的巷子"), {"blocked": True})
    check("封住 → blocked_aisle", q["kind"] == "blocked_aisle", str(q))


# ---------------------------------------------------------------- 4. 校验
def test_validate() -> None:
    print("\n== 4. schema 校验（不合格要能说清原因，供带错误重问）")
    check("None → 拒绝", validate_payload(None) is not None)
    check("缺 slots → 拒绝", validate_payload({"intent": "goto"}) is not None)
    check("未知字段 → 拒绝", validate_payload({"slots": {"foo": 1}}) is not None)
    check("bay 给字符串 → 拒绝", validate_payload({"slots": {"bay": "10"}}) is not None)
    check("合法载荷 → 通过", validate_payload({"intent": "goto", "slots": {"bay": 10}}) is None)


# ---------------------------------------------------------------- 5. 对抗端到端
class HallucinatingModel(ConstrainedNLU):
    """一个"什么都敢填"的模型：永远补全巷/面/层/bay/类别。最坏情况。"""

    name = "fake-hallucinating"

    def _ask_model(self, utterance: str, context: dict[str, Any]):
        return {
            "intent": "goto",
            "slots": {"aisle": "C", "side": "R", "bay": 10, "level": 1,
                      "classes": ["carton_m"], "empty": False, "rank": None, "blocked": False},
            "confidence": 0.95,
            "ask": None,
        }, ""


class GarbageModel(ConstrainedNLU):
    """输出根本不是 JSON 的模型：必须退规则后端，并且被记账。"""

    name = "fake-garbage"

    def _ask_model(self, utterance: str, context: dict[str, Any]):
        return None, "不是合法 JSON 对象"


def _run_bench(nlu) -> dict[str, Any]:
    import eval_bench  # scripts/eval_bench.py

    rows = list(csv.DictReader(open(ROOT / "results" / "bench_spec.csv", encoding="utf-8")))
    steps = []
    for row in rows:
        steps.append(eval_bench.run_one(row, nlu, 0.0, seed=1000, transitions=[], run_id="test"))
    return {"steps": steps}


def test_adversarial_e2e() -> None:
    print("\n== 5. 对抗端到端：69 条基准，模型故意乱填")
    res = _run_bench(HallucinatingModel())
    steps = res["steps"]
    ok = sum(1 for s in steps if s["behavior_ok"])
    silent = [s for s in steps if s["silent_wrong"]]
    print(f"     正确 {ok}/{len(steps)} = {ok / len(steps):.1%}；静默猜 {len(silent)} 条")
    for s in silent[:6]:
        print(f"       ! {s['id']} {s['utterance']!r} -> {s['actual_target']}")
    check("LANG-1a：乱填的模型也 0 次静默猜", not silent, f"{len(silent)} 条")
    # 缺原因码只看"期望 reject"的条目，与 eval_bench 的口径一致
    no_rc = [s for s in steps if s["expected_behavior"] == "reject" and not s["actual_reason_code"]]
    check("LANG-1b：拒绝必须带原因码", not no_rc, f"{len(no_rc)} 条")


def test_fallback_visible() -> None:
    print("\n== 6. 回退可见：输出垃圾的模型必须退规则并记账")
    nlu = GarbageModel()
    res = _run_bench(nlu)
    steps = res["steps"]
    ok = sum(1 for s in steps if s["behavior_ok"])
    print(f"     正确 {ok}/{len(steps)} = {ok / len(steps):.1%}；"
          f"回退 {nlu.fallbacks} 次、修复重问 {nlu.repairs} 次")
    check("回退次数被记账（=条目数，两轮都失败）",
          nlu.fallbacks == len(steps), f"fallbacks={nlu.fallbacks} n={len(steps)}")
    silent = [s for s in steps if s["silent_wrong"]]
    check("退规则后仍 0 静默猜", not silent, f"{len(silent)} 条")


def main() -> int:
    test_structure()
    test_ground()
    test_compile()
    test_validate()
    test_adversarial_e2e()
    test_fallback_visible()
    print()
    if FAILS:
        print(f"结论：FAIL（{len(FAILS)} 项）—— " + "、".join(FAILS))
        return 1
    print("结论：PASS —— 结构/落地/编译/校验 + 对抗端到端 + 回退可见，全部通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
