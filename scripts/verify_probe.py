#!/usr/bin/env python3
"""深度门限 verify 的探针：在**当前位置**抓一组左右图，判目标格有没有货。

用法（需要仿真 + 相机桥已在跑）
  python3 scripts/verify_probe.py --addr C-L-08-1     # 真值：空
  python3 scripts/verify_probe.py --addr C-L-21-1     # 真值：有货 carton_m
  python3 scripts/verify_probe.py --addr C-L-21-1 --save   # 存调试图

产物
  results/verify_probe_<时间>.json    判决 + 深度带 + 像素计数
  results/verify_probe_<时间>.png     左图 + 投影四边形 + 深度伪彩（--save 时）

**为什么要门限**：货架通透，空格背后 2.2 m 的货会投影到同一像素（TROUBLESHOOTING §33.3）。
只统计"四边形内且深度落在格子内部"的像素，背景自动剔除。
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def grab_pair(left_topic: str, right_topic: str, timeout: float = 20.0):
    """抓一对左右图（近似同步：各取最新一帧）。返回 (left, right, stamp)。"""
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image

    box: dict[str, object] = {}

    class Grab(Node):
        def __init__(self) -> None:
            super().__init__("verify_probe")
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(Image, left_topic, lambda m: box.setdefault("L", m), qos)
            self.create_subscription(Image, right_topic, lambda m: box.setdefault("R", m), qos)

    own_init = False
    if not rclpy.ok():          # 可能已被调用方（demo 进程）初始化
        rclpy.init()
        own_init = True
    node = Grab()

    def to_np(m):
        enc = (m.encoding or "").lower()
        arr = np.frombuffer(bytes(m.data), dtype="uint8").reshape(m.height, m.width, 3)
        if enc == "rgb8":
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        return arr

    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout and not ("L" in box and "R" in box):
            rclpy.spin_once(node, timeout_sec=0.1)
        if "L" not in box or "R" not in box:
            return None, None, None
        stamp = getattr(box["L"], "header", None)
        return to_np(box["L"]), to_np(box["R"]), (stamp.stamp.sec if stamp else 0)
    finally:
        node.destroy_node()
        _ = own_init


def stereo_depth(left, right):
    """SGBM 视差 → 深度（米）。无效视差置 NaN。"""
    import cv2
    import numpy as np

    from depot_vln.verify_depth import BASELINE, F_PX

    g_l = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
    g_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
    sgbm = cv2.StereoSGBM_create(
        minDisparity=0, numDisparities=32, blockSize=5,
        P1=8 * 5 * 5, P2=32 * 5 * 5, uniquenessRatio=8,
        speckleWindowSize=80, speckleRange=2, disp12MaxDiff=1,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )
    disp = sgbm.compute(g_l, g_r).astype(np.float32) / 16.0
    with np.errstate(divide="ignore", invalid="ignore"):
        depth = np.where(disp > 0.5, F_PX * BASELINE / disp, np.nan)
    depth[depth > 12.0] = np.nan           # 远处视差不可信
    return depth


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--addr", required=True, help="货位地址，例如 C-L-08-1")
    ap.add_argument("--left", default="/stereo_left")
    ap.add_argument("--right", default="/stereo_right")
    ap.add_argument("--grid", default=str(ROOT / "world" / "slot_grid.json"))
    ap.add_argument("--save", action="store_true", help="存调试图（四边形 + 深度伪彩）")
    ap.add_argument("--pose", default="", help="手动给位姿 'x,y,yaw'（不给就取真值）")
    args = ap.parse_args()

    from depot_vln.verify_depth import cam_world, depth_gate_verdict, quad_mask, slot_opening

    # 1) 位姿（运动层/定位给；探针默认取真值，便于和真值对照）
    if args.pose:
        x, y, yaw = (float(v) for v in args.pose.split(","))
        robot = {"x": x, "y": y, "yaw": yaw}
    else:
        sys.path.insert(0, str(ROOT / "scripts"))
        from slam_route import TruthPoller

        pol = TruthPoller()
        p, t0 = None, time.monotonic()
        while time.monotonic() - t0 < 30 and p is None:
            p = pol.get(5.0)
            time.sleep(0.5)
        if p is None:
            print("[probe] 30 s 拿不到真值位姿")
            return 2
        robot = {"x": p[0], "y": p[1], "yaw": p[2]}
    print(f"[probe] 车体位姿 ({robot['x']:+.2f}, {robot['y']:+.2f}) yaw {robot['yaw']:+.3f}")

    # 2) 目标格 + 投影
    grid = json.loads(Path(args.grid).read_text(encoding="utf-8"))
    slot = grid["slots"].get(args.addr)
    if slot is None:
        print(f"[probe] slot_grid 里没有 {args.addr}")
        return 2
    side = "L" if slot["side"] == "L" else "R"
    cam = cam_world(robot, side)
    op = slot_opening(slot, cam)
    if op is None:
        print("[probe] 投影失败（格子不在相机前方）")
        return 2
    print(f"[probe] 格子开口面 d_front {op['d_front']:.2f} m / 背面 {op['d_back']:.2f} m，"
          f"四边形 {[(round(u), round(v)) for u, v in op['quad']]}")

    # 3) 抓图 + 深度
    left, right, stamp = grab_pair(args.left, args.right)
    if left is None:
        print(f"[probe] 抓不到双目（{args.left} / {args.right}）")
        return 2
    depth = stereo_depth(left, right)
    import numpy as np

    valid = int(np.isfinite(depth).sum())
    print(f"[probe] 双目 {left.shape[1]}x{left.shape[0]}，有效深度像素 {valid} / {depth.size}"
          f"（{100.0 * valid / depth.size:.1f}%）")

    # 4) 判决
    res = depth_gate_verdict(depth, op)
    res.update({"addr": args.addr, "robot": robot, "stamp": stamp})
    print(f"[probe] 判决：{res['verdict'].upper()}  —— {res['why']}")

    stamp_s = datetime.now().strftime("%Y%m%dT%H%M%S")
    out_json = ROOT / "results" / f"verify_probe_{stamp_s}.json"
    out_json.parent.mkdir(exist_ok=True)
    out_json.write_text(json.dumps(res, ensure_ascii=False, indent=2, default=float), encoding="utf-8")

    if args.save:
        import cv2

        vis = left.copy()
        pts = np.array(op["quad"], dtype=np.int32)
        cv2.polylines(vis, [pts], True, (0, 255, 0), 1)
        mask = np.array(quad_mask(op["quad"], left.shape[1], left.shape[0]), dtype=bool)
        d_vis = np.zeros_like(left)
        d_clip = np.clip(np.nan_to_num(depth, nan=0.0, posinf=0.0), 0, 4.0)
        d_vis[:, :, 1] = (255 * (1.0 - d_clip / 4.0)).astype("uint8")
        d_vis[~mask] = (d_vis[~mask] * 0.25).astype("uint8")
        combo = np.hstack([vis, d_vis])
        out_png = ROOT / "results" / f"verify_probe_{stamp_s}.png"
        cv2.imwrite(str(out_png), combo)
        print(f"[probe] 调试图 {out_png.relative_to(ROOT)}")

    print(f"[probe] 产物 {out_json.relative_to(ROOT)}")
    print("RESULT:", "PASS" if res["verdict"] in ("occupied", "empty") else "UNSEEN")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
