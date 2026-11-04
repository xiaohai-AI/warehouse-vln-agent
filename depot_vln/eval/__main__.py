"""`python -m depot_vln.eval` —— 跑基准 / 只校验用例。"""
from __future__ import annotations

import argparse
import json
import sys

from depot_vln.eval.oracle import load_cases, validate_all
from depot_vln.eval.runner import format_table, run_bench


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m depot_vln.eval")
    ap.add_argument("--backend", default="rule", choices=["rule", "llm"])
    ap.add_argument("--inventory", default="sim", choices=["sim", "perception"],
                    help="sim = 显式替身（离线）；perception = 读 results/inventory.json")
    ap.add_argument("--noise", type=float, default=0.0, help="替身库存的占用翻转率")
    ap.add_argument("--coverage", type=float, default=1.0, help="替身库存的观测覆盖率")
    ap.add_argument("--out", default="results/bench.csv")
    ap.add_argument("--check-only", action="store_true", help="只校验用例与真值是否自洽")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)

    if args.check_only:
        bad = validate_all()
        print(json.dumps(bad, ensure_ascii=False, indent=2) if bad else "用例全部自洽")
        return 1 if bad else 0

    s = run_bench(backend=args.backend, inventory=args.inventory, noise=args.noise,
                  coverage=args.coverage, out_csv=args.out)
    if args.json:
        print(json.dumps({k: v for k, v in s.items() if k != "rows"}, ensure_ascii=False, indent=2))
    else:
        print(format_table(s))
        for r in s["rows"]:
            if not r["ok"] or r["silent_wrong"]:
                flag = "SILENT" if r["silent_wrong"] else "FAIL"
                print(f"  [{flag}] {r['id']} {r['utterance'][:28]:30s} {r['actual']:34s} {r['note']}")
    # 铁律：静默猜错必须为 0
    return 0 if s["silent_wrong"] == 0 else 2


if __name__ == "__main__":
    sys.exit(main())
