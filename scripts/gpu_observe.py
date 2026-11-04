#!/usr/bin/env python3
"""开相机渲染时的 10 分钟安全观察：RTF / GPU 利用率 / 显存 / 内核错误。

为什么需要它：这台机器桌面与仿真共用一块 RTX 4080，headless 只是不画窗口，
相机照旧渲染。上次就是整夜跑仿真把 `nvidia-modeset` 挂死（桌面冻结、只能断电）。
开渲染前后必须有量化记录，不能靠"看着还行"。

用法：
  .venv/bin/python scripts/gpu_observe.py --minutes 10        # 每 10 s 打一行
  .venv/bin/python scripts/gpu_observe.py --minutes 10 --json results/gpu_cam.json
"""
from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import time


def run(cmd: list[str], timeout: float = 5.0) -> str:
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout).stdout
    except Exception:
        return ""


def sim_time() -> float | None:
    txt = run(["ign", "topic", "-e", "-t", "/clock", "-n", "1"])
    m = re.search(r"^sim \{\s*sec: (\d+)\s*nsec: (\d+)", txt, re.S | re.M)
    return None if not m else int(m.group(1)) + int(m.group(2)) * 1e-9


def gpu_kernel_errors() -> int:
    out = run(["journalctl", "-k", "-b", "0"], timeout=10.0)
    return len(re.findall(r"nvidia-modeset: ERROR|Error while waiting for GPU progress|NVRM: Xid", out))


def gpu_stats() -> tuple[str, str, str]:
    q = run(["nvidia-smi", "--query-gpu=utilization.gpu,memory.used,temperature.gpu",
             "--format=csv,noheader,nounits"]).strip()
    if not q:
        return "?", "?", "?"
    return tuple(part.strip() for part in q.split(","))  # type: ignore[return-value]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--minutes", type=float, default=10.0)
    ap.add_argument("--period", type=float, default=10.0)
    ap.add_argument("--json", default="")
    args = ap.parse_args()

    t_end = time.monotonic() + args.minutes * 60
    print(f"{'墙钟 s':>7} {'RTF':>6} {'GPU%':>5} {'显存 MiB':>9} {'温度':>5}  GPU错误  判定")
    print("-" * 62)

    t0 = time.monotonic()
    s0 = sim_time()
    w0 = t0
    rows = []
    bad = 0
    while time.monotonic() < t_end:
        time.sleep(args.period)
        s1 = sim_time()
        w1 = time.monotonic()
        rtf = float("nan") if (s0 is None or s1 is None or w1 == w0) else (s1 - s0) / (w1 - w0)
        util, mem, temp = gpu_stats()
        errs = gpu_kernel_errors()
        verdict = "OK"
        if errs > 0:
            verdict = "!! 内核报错，立即 make down"
            bad += 1
        elif rtf == rtf and rtf < 0.3:
            verdict = "RTF 偏低（负载重，别再加传感器）"
        print(f"{w1 - t0:7.0f} {rtf:6.2f} {util:>5} {mem:>9} {temp:>5}  {errs:8d}  {verdict}", flush=True)
        rows.append({"t_wall": round(w1 - t0, 1),
                     "rtf": None if rtf != rtf else round(rtf, 3),
                     "gpu_util": util, "gpu_mem": mem, "gpu_temp": temp,
                     "kernel_errors": errs})
        s0, w0 = s1, w1

    ok = bad == 0
    print()
    print(f"观察 {args.minutes:.0f} 分钟：内核 GPU 报错 {bad} 次 -> {'PASS 可以继续' if ok else 'FAIL 立刻拆栈'}")
    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump({"minutes": args.minutes, "samples": rows, "pass": ok}, f, ensure_ascii=False, indent=2)
        print(f"-> {args.json}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
