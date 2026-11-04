#!/usr/bin/env python3
"""启动前护栏：已经有一套仿真在跑就拒绝再起一套。

为什么必须有：这台机器桌面和仿真共用一块 RTX 4080。每多一个 gz server 就多一套
GL/EGL 上下文，两个叠在一起时实测把 nvidia-modeset 挂死（内核日志每 5 s 一条
"Error while waiting for GPU progress"），随后整个桌面冻结、只能断电。
本会话里已经出现过 2 个 warehouse.sdf 并存。
"""
from __future__ import annotations

import subprocess
import sys

PATTERN = "warehouse.sdf|depot_sim.launch|parameter_bridge|mecanum_odom"
SKIP = ("stack_guard", "pgrep")


def alive() -> list[str]:
    out = subprocess.run(["pgrep", "-af", PATTERN], capture_output=True, text=True).stdout
    return [ln for ln in out.splitlines() if not any(s in ln for s in SKIP)]


def main() -> int:
    rows = alive()
    if not rows:
        return 0
    print("拒绝启动：已经有一套仿真在跑。先 make down 拆干净，再起新的。")
    print("（这很关键：叠两个 gz server 会把 nvidia-modeset 挂死，桌面跟着冻死）")
    for r in rows[:8]:
        print("  " + r[:120])
    return 1


if __name__ == "__main__":
    sys.exit(main())
