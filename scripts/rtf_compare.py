#!/usr/bin/env python3
"""对比几组 gpu_observe.py 的 JSON，回答"降显示参数值不值得"。

用法
  # 每组先用 gpu_observe 采 2 分钟（要仿真在跑）
  .venv/bin/python scripts/gpu_observe.py --minutes 2 --period 5 --json results/rtf_A_240.json
  ...换设置后...
  .venv/bin/python scripts/gpu_observe.py --minutes 2 --period 5 --json results/rtf_B_60.json
  # 然后对比
  .venv/bin/python scripts/rtf_compare.py results/rtf_A_240.json results/rtf_B_60.json

  --selftest   用**合成数据**验证解析/统计逻辑（不代表任何实测）

为什么 RTF 用中位数而不是均值：起停阶段（GUI 首帧、OTA 着色器编译）会有几秒极低值，
均值会被拉低，看起来像"设置变差了"，其实只是首帧开销。中位数 + 最小值两个数一起看：
  · 中位数 = 稳态性能
  · 最小值 = 有没有掉到卡顿区
"""
from __future__ import annotations

import json
import statistics
import sys
import tempfile
from pathlib import Path


def med(xs: list[float]) -> float | None:
    xs = [x for x in xs if x is not None]
    return statistics.median(xs) if xs else None


def load(p: Path) -> dict:
    d = json.loads(p.read_text(encoding="utf-8"))
    s = d.get("samples") or []
    rtf = [r.get("rtf") for r in s]
    util = []
    temp = []
    for r in s:
        try:
            util.append(float(r.get("gpu_util")))
        except (TypeError, ValueError):
            pass
        try:
            temp.append(float(r.get("gpu_temp")))
        except (TypeError, ValueError):
            pass
    return {
        "name": p.stem,
        "n": len(s),
        "minutes": d.get("minutes"),
        "rtf_med": med(rtf),
        "rtf_min": min([x for x in rtf if x is not None], default=None),
        "gpu_med": med(util),
        "gpu_max": max(util, default=None),
        "temp_max": max(temp, default=None),
        "errs": max([r.get("kernel_errors") or 0 for r in s], default=0),
        "pass": d.get("pass"),
    }


def fmt(v, nd=2, suffix=""):
    return "—" if v is None else f"{v:.{nd}f}{suffix}"


def table(rows: list[dict]) -> None:
    print(f"{'条件':<24}{'样本':>5}{'RTF中位':>9}{'RTF最小':>9}{'GPU中位':>9}"
          f"{'GPU最大':>9}{'温度':>7}{'内核错误':>9}")
    print("-" * 82)
    for r in rows:
        print(f"{r['name']:<24}{r['n']:>5}{fmt(r['rtf_med']):>9}{fmt(r['rtf_min']):>9}"
              f"{fmt(r['gpu_med'], 0, '%'):>9}{fmt(r['gpu_max'], 0, '%'):>9}"
              f"{fmt(r['temp_max'], 0, '°C'):>7}{r['errs']:>9}")
    print()

    bad = [r for r in rows if r["errs"] > 0]
    if bad:
        print("!! 有组出现 GPU 内核报错：" + "、".join(r["name"] for r in bad)
              + " —— 这种设置不能用于录制（会挂驱动）")
    ok = [r for r in rows if r["rtf_med"] is not None and r["errs"] == 0]
    if not ok:
        print("没有可用样本（缺 rtf 或全组内核报错）")
        return
    best = max(ok, key=lambda r: r["rtf_med"])
    base = ok[0]
    print(f"RTF 最高：{best['name']}（{fmt(best['rtf_med'])}）")
    if len(ok) > 1:
        for r in ok[1:]:
            d = (r["rtf_med"] - base["rtf_med"]) / base["rtf_med"] * 100
            print(f"  {r['name']} 相对 {base['name']}：RTF {d:+.1f}%"
                  f"，GPU 中位 {fmt(r['gpu_med'], 0, '%')} vs {fmt(base['gpu_med'], 0, '%')}")
        print("\n判读：RTF 变化 <5% 视为噪声（单次 2 分钟、10 s 采样，抖动就有这么大）。")
        print("      RTF 差不多但 GPU%/温度明显降 → **仍值得降**：省的是挂驱动的风险，不是速度。")


def selftest() -> int:
    d = Path(tempfile.mkdtemp(prefix="rtf_selftest_"))
    a = d / "rtf_A_240.json"
    b = d / "rtf_B_60.json"
    a.write_text(json.dumps({"minutes": 2, "pass": True, "samples": [
        {"t_wall": i * 5, "rtf": 0.30 + i * 0.01, "gpu_util": "97",
         "gpu_temp": "78", "kernel_errors": 0} for i in range(10)]}), encoding="utf-8")
    b.write_text(json.dumps({"minutes": 2, "pass": True, "samples": [
        {"t_wall": i * 5, "rtf": 0.38 + i * 0.005, "gpu_util": "61",
         "gpu_temp": "66", "kernel_errors": 0} for i in range(10)]}), encoding="utf-8")
    table([load(a), load(b)])
    print("\n（--selftest 用的是合成数据，只验证解析/统计，不代表任何实测）")
    return 0


def main() -> int:
    args = [a for a in sys.argv[1:]]
    if "--selftest" in args:
        return selftest()
    if not args:
        print(__doc__)
        return 1
    paths = [Path(a) for a in args]
    missing = [p for p in paths if not p.exists()]
    if missing:
        print("找不到：" + "、".join(str(p) for p in missing))
        return 1
    table([load(p) for p in paths])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
