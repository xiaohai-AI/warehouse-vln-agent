#!/usr/bin/env python3
"""GPU 看门狗：发现 NVIDIA 驱动开始报错就立刻拆栈，抢在桌面冻结之前。

实测证据（断电前的内核日志）：
  12:26:29  NVRM: Error in service of callback
  12:26:30  nvidia-modeset: ERROR: GPU:0: Error while waiting for GPU progress  （此后每 5 s 一条）
之后再也没恢复，桌面打不开，只能断电。所以这里一看到这类关键字就立即拆栈并存证。

同时盯显存：超阈值先告警（通常意味着传感器又全开了）。
读日志用 journalctl -k -b 0（普通用户可读）。
"""
from __future__ import annotations

import argparse
import re
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
# 只认真正的挂死信号。**不要**匹配裸的 "NVRM:"：
#   NVIDIA 驱动正常启动时也会打 "NVRM: loading NVIDIA UNIX x86_64 Kernel Module ..."，
#   按裸 NVRM 匹配会把健康的一次运行误判成故障、把整套栈拆掉（本会话踩过）。
BAD = re.compile(r"nvidia-modeset: ERROR|Error while waiting for GPU progress|NVRM: Xid", re.I)


def kernel_hits() -> list[str]:
    try:
        out = subprocess.run(["journalctl", "-k", "-b", "0", "--no-pager"],
                             capture_output=True, text=True, timeout=20).stdout
    except Exception:
        return []
    return [ln for ln in out.splitlines() if BAD.search(ln)]


def gpu_mem() -> tuple[int, int] | None:
    try:
        out = subprocess.run(["nvidia-smi", "--query-gpu=memory.used,memory.total",
                              "--format=csv,noheader,nounits"],
                             capture_output=True, text=True, timeout=10).stdout.strip()
        used, total = (int(x) for x in out.split(","))
        return used, total
    except Exception:
        return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--interval", type=float, default=10.0)
    ap.add_argument("--mem-warn-mb", type=int, default=6000)
    ap.add_argument("--log", default=str(ROOT / "results" / "gpu_watchdog.log"))
    args = ap.parse_args()

    log = Path(args.log)
    log.parent.mkdir(parents=True, exist_ok=True)
    seen = len(kernel_hits())     # 启动时已有的旧记录不当作新事件
    warned = False
    print(f"[watchdog] 启动，基线已有 {seen} 条历史 GPU 报错", flush=True)
    while True:
        hits = kernel_hits()
        if len(hits) > seen:
            new = hits[seen:]
            seen = len(hits)
            with log.open("a") as f:
                for h in new:
                    f.write(time.strftime("[%H:%M:%S] ") + h + "\n")
            print(f"[GPU!] NVIDIA 驱动开始报错（累计 {len(hits)} 条），立即拆栈：", flush=True)
            for h in new[:3]:
                print("   " + h[:140], flush=True)
            subprocess.run(["bash", str(ROOT / "scripts" / "stack_down.sh")])
            with log.open("a") as f:
                f.write("== watchdog 已自动拆栈（抢在桌面冻结前）==\n")
            return 2
        m = gpu_mem()
        if m and m[0] > args.mem_warn_mb and not warned:
            warned = True
            msg = f"显存 {m[0]}/{m[1]} MiB 超过 {args.mem_warn_mb}，检查传感器是否全开"
            print(f"[GPU 告警] {msg}", flush=True)
            with log.open("a") as f:
                f.write(time.strftime("[%H:%M:%S] ") + msg + "\n")
        time.sleep(args.interval)


if __name__ == "__main__":
    sys.exit(main())
