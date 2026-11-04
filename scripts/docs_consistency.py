#!/usr/bin/env python3
"""P7-1 一致性核对：文档与产物之间不许有对不上的数字。

**为什么要有这个脚本**：`docs/` 里散着 20+ 处 "PASS N / FAIL M"、"69/69"、"偏出 0.66 m"
之类的断言，靠人眼对不齐。这里的每一条都从**产物**重算，对不上就退出码非零。

用法
  .venv/bin/python scripts/docs_consistency.py            # 只检查
  .venv/bin/python scripts/docs_consistency.py --strict   # 有 FAIL 就非零退出

检查项
  1. ACCEPTANCE.md 的"合计"与表内行数一致
  2. 每条判据都有"关键数字"（不许空着冒充通过）
  3. 判据表格顺序与编号一致（P6-2b→c→d→e→f）
  4. 最新 demo 产物里的 `verify_rule` 等于 `verify_depth.RULE_ID`（判据版本没漂）
  5. 深度存档数、真跑记录数与 ACCEPTANCE 里写的数一致
  6. `docs/LIMITATIONS.md` 存在且被 REPORT 引用（唯一权威清单不许是孤儿）
"""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DOCS = ROOT / "docs"
RES = ROOT / "results"
sys.path.insert(0, str(ROOT))

fails: list[str] = []
oks: list[str] = []


def check(cond: bool, name: str, detail: str = "") -> None:
    (oks if cond else fails).append(f"{name}{('：' + detail) if detail else ''}")


def main() -> int:
    acc = (DOCS / "ACCEPTANCE.md").read_text(encoding="utf-8")
    rows = [ln for ln in acc.splitlines() if ln.startswith("| ") and "|---" not in ln]
    rows = [r for r in rows if not r.startswith("| 判据 ")]
    total = re.search(r"合计：PASS (\d+) / FAIL (\d+) / 未跑 (\d+)", acc)
    check(total is not None, "ACCEPTANCE 有合计行")
    if total:
        n_pass = sum(1 for r in rows if "| PASS |" in r)
        n_fail = sum(1 for r in rows if "| FAIL |" in r)
        n_todo = sum(1 for r in rows if "| 未跑 |" in r)
        check((int(total.group(1)), int(total.group(2)), int(total.group(3)))
              == (n_pass, n_fail, n_todo),
              "合计与表内行数一致",
              f"合计 {total.group(1)}/{total.group(2)}/{total.group(3)} vs "
              f"行数 {n_pass}/{n_fail}/{n_todo}")

    # 2. 每条都要有数字（"PASS" 后面空着 = 无证据）
    empty = [r.split("|")[1].strip() for r in rows if len(r.split("|")) > 4
             and not r.split("|")[4].strip()]
    check(not empty, "每条判据都有关键数字", f"空的：{empty}" if empty else "")

    # 3. 顺序
    names = [r.split("|")[1].strip() for r in rows]
    # 注意排除基判据 "P6-2 真工具层…"（它也是 P6-2 开头，但不是子判据）
    sub = [n for n in names if re.match(r"P6-2[a-z]", n)]
    expect_order = ["P6-2b", "P6-2c", "P6-2d", "P6-2e", "P6-2f"]
    got = [next((e for e in expect_order if n.startswith(e)), "?") for n in sub]
    check(got == expect_order[:len(got)], "P6-2 子判据顺序正确", f"{got}")

    # 4. 判据版本号
    from depot_vln.verify_depth import RULE_ID
    # **只看最新的一次真跑**：判据升版后，历史产物永远带着旧版本号（那是不可变历史），
    # 拿"最近 12 份里有几个旧版本"来判会让这条检查在升版后永久 FAIL —— 检查的名字本身就是
    # "最近的 depth 真跑"，语义是"最新那次用的是不是当前判据"，不是"所有历史都是当前判据"。
    newest = None
    n_old = 0
    for f in reversed(sorted(RES.glob("p6_demo_*.json"))):
        try:
            d = json.loads(f.read_text(encoding="utf-8"))
        except Exception:
            continue
        if d.get("dry") or d.get("verify") != "depth" or not d.get("verify_rule"):
            continue
        if newest is None:
            newest = d["verify_rule"]
        elif d["verify_rule"] != RULE_ID:
            n_old += 1
    check(newest is None or newest == RULE_ID,
          "最近的 depth 真跑用的是当前判据版本",
          f"最新一次用的是 {newest!r}，代码是 {RULE_ID!r}（另有 {n_old} 次旧判据，属不可变历史）"
          if newest and newest != RULE_ID else "")

    # 5. 数出来的数
    dumps = len(list((RES / "depth_dumps").glob("*.npz")))
    check(dumps >= 10, "深度存档 ≥10 份", f"{dumps} 份")
    m = re.search(r"(\d+) 份深度存档复算", acc)
    if m:
        check(int(m.group(1)) == dumps, "ACCEPTANCE 的存档数与磁盘一致",
              f"文档 {m.group(1)} vs 磁盘 {dumps}")

    # 6. 唯一权威清单
    lim = DOCS / "LIMITATIONS.md"
    check(lim.exists(), "LIMITATIONS.md 存在")
    rep = (DOCS / "REPORT.md").read_text(encoding="utf-8")
    check("LIMITATIONS.md" in rep, "REPORT 引用了 LIMITATIONS.md")

    for s in oks:
        print(f"  PASS  {s}")
    for s in fails:
        print(f"  FAIL  {s}")
    print(f"\n一致性：{len(oks)} 项通过 / {len(fails)} 项失败")
    return 1 if (fails and "--strict" in sys.argv) else 0


if __name__ == "__main__":
    raise SystemExit(main())
