#!/usr/bin/env python3
"""读一次 `/scan`，打印最近回波（m）。看不见/读不到时打印 nan。

用途：静止靶标诊断（`scripts/scan_target_probe.sh`）——判断单线雷达到底能不能看见低矮障碍。
"""
from __future__ import annotations

import re
import subprocess
import sys


def main() -> int:
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
    try:
        p = subprocess.run(["ros2", "topic", "echo", "--once", "/scan", "--field", "ranges"],
                           capture_output=True, text=True, timeout=timeout)
    except Exception:
        print("nan")
        return 0
    vals = [float(m.group(0)) for m in
            re.finditer(r"[-+]?\d*\.?\d+(?:e[-+]?\d+)?", p.stdout or "")]
    vals = [v for v in vals if 0.05 < v < 60.0]
    print(f"{min(vals):.2f}" if vals else "nan")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
