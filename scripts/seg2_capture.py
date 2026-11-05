#!/usr/bin/env python3
"""SEG-2 抓取：开到每个 bay 的观测位，朝左/右货架面各拍一帧 RGB-D。

为什么按货位观测位拍：SPEC §7.1 的 T2 明确"在每个 bay 的观测位拍帧 → 检测 → 3D 质心 → 归属最近 bay"。
观测位和货位地址来自 `world/slot_grid.json`（世界摘要，运行时输入之一）；
**占用/类别真值在 `ground_truth/slots.json`，本脚本绝不读它**（评测阶段才读）。

用法
  .venv/bin/python scripts/seg2_capture.py --bays 1-3            # 试点：3 个 bay / 6 帧
  .venv/bin/python scripts/seg2_capture.py --bays 1-24           # 全 A 巷：48 帧
  .venv/bin/python scripts/seg2_capture.py --bays 1-24 --aisle C # 换巷

每帧存：rgb(uint8) / depth(float32 m) / K(3x3) / 真值位姿 / 该帧覆盖的货位地址 / 相机外参。
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

ROOT = Path(__file__).resolve().parents[1]
# 让 `import depot_vln` 在任何 cwd 下都能用（脚本以 scripts/ 为 sys.path[0] 运行）
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

sys.path.insert(0, str(ROOT / "scripts"))

from drive_to import align_to, drive_to_point, turn_to, turn_to_precise  # noqa: E402  # noqa: F401
from slam_route import TruthPoller, cmd  # noqa: E402
from depot_vln.catalog import get_catalog

# 实测实时率（2026-11-04 带相机渲染：0.22）。只用于折算"距离 -> 墙钟预算"，
# 不参与任何控制；换机器就改这一个数。
RTF_EST = 0.22

# 相机在 base_link 里的位置（xacro: cam_z=0.35；x 前移 0.10，与雷达同位）
CAM_XYZ = (0.10, 0.0, 0.35 - 0.076)


def parse_range(s: str) -> list[int]:
    out: list[int] = []
    for part in s.split(","):
        part = part.strip()
        if "-" in part[1:]:
            a, b = part.split("-", 1)
            out += list(range(int(a), int(b) + 1))
        else:
            out.append(int(part))
    return out


class Cam(Node):
    def __init__(self) -> None:
        super().__init__("seg2_capture")
        try:
            self.declare_parameter("use_sim_time", True)
        except Exception:
            pass
        qos = QoSProfile(depth=3, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.rgb: Image | None = None
        self.depth: Image | None = None
        self.info: CameraInfo | None = None
        self._last_stamp = -1.0        # 已取过的最新一帧（防重复取缓存帧）
        self.create_subscription(Image, "/rgbd/image", self._rgb, qos)
        self.create_subscription(Image, "/rgbd/depth_image", self._depth, qos)
        self.create_subscription(CameraInfo, "/rgbd/camera_info", self._info, qos)

    def _rgb(self, m: Image) -> None:
        self.rgb = m

    def _depth(self, m: Image) -> None:
        self.depth = m

    def _info(self, m: CameraInfo) -> None:
        self.info = m

    def latest_stamp(self) -> float:
        m = self.rgb or self.depth
        err = 0.0
        if m is None:
            return self._last_stamp
        return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 + err

    def mark(self) -> float:
        """把"当前已知最新帧"设为闸门：之后只接受比它新的帧。

        为什么不用 node.get_clock()：如果 use_sim_time 没真正生效，那是**墙钟**（10⁹ 秒量级），
        而图像戳是仿真秒 —— 比较永远不成立，于是每个观测都"取图失败"（踩过）。
        只用图像戳自身判新旧，与时钟无关。
        """
        self._last_stamp = max(self._last_stamp, self.latest_stamp())
        return self._last_stamp

    def fresh(self, timeout: float = 8.0, after: float | None = None
              ) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
        """取一对时间对齐的 RGB/深度（差 <0.2 s）+ 内参。

        **必须比上一帧新**（`after`/`_last_stamp`）：踩过的坑 —— 只检查 RGB 与深度彼此时间一致，
        于是在相机桥出图变慢时反复读到同一帧缓存：三个不同机位存下来的图像逐像素相同，
        而且画面中心深度 8.9 m（是转向前的旧视角），下游全都白算。
        """
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.rgb and self.depth and self.info:
                ts = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9  # noqa: E731
                newer = (ts(self.rgb) > self._last_stamp + 1e-6) and \
                        (after is None or ts(self.rgb) >= after)
                if newer and abs(ts(self.rgb) - ts(self.depth)) < 0.2:
                    self._last_stamp = ts(self.rgb)
                    rgb = image_to_np(self.rgb)
                    dep = depth_to_np(self.depth)
                    k = np.asarray(self.info.k, np.float64).reshape(3, 3)
                    return rgb, dep, k
        return None


def image_to_np(m: Image) -> np.ndarray:
    if m.encoding in ("rgb8", "bgr8"):
        a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, m.step // m.width)[:, : m.width * 3]
        a = a.reshape(m.height, m.width, 3)
        return a[:, :, ::-1].copy() if m.encoding == "bgr8" else a
    raise RuntimeError(f"未处理的彩图编码 {m.encoding}")


def depth_to_np(m: Image) -> np.ndarray:
    if m.encoding == "32FC1":
        return np.frombuffer(bytes(m.data), np.float32).reshape(m.height, m.width).copy()
    if m.encoding == "16UC1":                       # gz 有时给毫米
        d = np.frombuffer(bytes(m.data), np.uint16).reshape(m.height, m.width).astype(np.float32)
        return d / 1000.0
    raise RuntimeError(f"未处理的深度编码 {m.encoding}")


# ---------------------------------------------------------------- 多巷道支持
def _nav_offsets(kind: str = "capture_lane_offsets") -> dict[str, float]:
    import yaml

    cfg = yaml.safe_load((ROOT / "config" / "nav.yaml").read_text(encoding="utf-8")) or {}
    return {str(k).upper(): float(v) for k, v in (cfg.get(kind) or {}).items()}


def aisle_y(sg: dict, aisle: str, offsets: dict[str, float] | None = None) -> float:
    """采集行驶线 y = 巷道中心 + 配置化偏移（C 巷绕开叉车，见 config/nav.yaml）。"""
    off = _nav_offsets() if offsets is None else offsets
    a = next(a for a in sg["aisles"] if a["name"] == aisle)
    return round(float(a["center_y"]) + off.get(aisle.upper(), 0.0), 3)


def blocked_bays(aisle: str) -> list[tuple[int, int]]:
    """该巷被路障封死的 bay 区间（从 catalog 派生，不再靠人记）。"""
    return [(int(s["bay_from"]), int(s["bay_to"]))
            for s in get_catalog().blocked_segments()
            if s["aisle"] == aisle and s.get("bay_from")]


def reachable_bays(sg: dict, aisle: str, want: list[int]) -> tuple[list[int], list[int]]:
    ranges = blocked_bays(aisle)
    ok, bad = [], []
    for b in want:
        (bad if any(lo <= b <= hi for lo, hi in ranges) else ok).append(b)
    return ok, bad


def build_views(sg: dict, aisle: str, bays: list[int], y_line: float) -> list[dict]:
    by_bay = {b["bay"]: b for b in sg["bays"] if b["aisle"] == aisle}
    views = []
    for b in sorted(bays):
        rec = by_bay[b]
        for side, yaw in (("L", math.pi / 2), ("R", -math.pi / 2)):
            views.append({
                "bay": b, "x": rec["center_x"], "y": y_line, "yaw": yaw,
                "side": side, "addr": [s for s in rec["slots"] if f"-{side}-" in s],
            })
    return views


def save_frames(frames: list[dict], out: Path, pitch: float) -> None:
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        out,
        rgb=np.stack([f["rgb"] for f in frames]),
        depth=np.stack([f["depth"] for f in frames]).astype(np.float32),
        k=np.stack([f["k"] for f in frames]),
        pose=np.stack([f["pose"] for f in frames]),
        bay=np.array([f["bay"] for f in frames], np.int32),
        side=np.array([f["side"] for f in frames]),
        addr=np.array([",".join(f["addr"]) for f in frames]),
        yaw_err_deg=np.array([f["yaw_err_deg"] for f in frames], np.float64),
        cam_xyz=np.array(CAM_XYZ, np.float64),
        cam_pitch_deg=np.array(pitch, np.float64),
    )


def load_partial(path: Path, pitch: float) -> tuple[list[dict], set[tuple[int, str]]]:
    """读断点文件，返回 (已抓帧, 已覆盖的 (bay, side) 集合)。

    长任务必须能断：一条巷 48 个观测按实测 RTF 要 50+ 分钟，中途被中断
    （换参数、GPU 报警、会话结束）时，已经开过去的里程不应该白跑。
    """
    if not path.exists():
        return [], set()
    try:
        d = np.load(path)
        if abs(float(d["cam_pitch_deg"]) - pitch) > 1e-6:
            print(f"[seg2] 断点文件的相机仰角 {float(d['cam_pitch_deg'])}° 与本次 {pitch}° 不符，忽略")
            return [], set()
        frames = []
        for i in range(d["rgb"].shape[0]):
            frames.append({"rgb": d["rgb"][i], "depth": d["depth"][i], "k": d["k"][i],
                           "pose": d["pose"][i], "bay": int(d["bay"][i]),
                           "side": str(d["side"][i]),
                           "addr": str(d["addr"][i]).split(",") if str(d["addr"][i]) else [],
                           "yaw_err_deg": float(d["yaw_err_deg"][i])})
        done = {(f["bay"], f["side"]) for f in frames}
        return frames, done
    except Exception as e:
        print(f"[seg2] 断点文件读不了（{type(e).__name__}: {e}），从头抓")
        return [], set()


def capture_aisle(node: "Cam", poller, sg: dict, aisle: str, bays: list[int],
                  y_line: float, out: Path, args) -> int:
    """在一个巷道里拍完所有观测位，返回帧数。"""
    from seg1_go import safe_route

    views = build_views(sg, aisle, bays, y_line)
    if not views:
        print(f"[seg2] {aisle} 巷没有可观测的 bay，跳过")
        return 0
    print(f"[seg2] 巷道 {aisle}（y={y_line:+.2f}）bay {bays[0]}..{bays[-1]} -> "
          f"{len(views)} 个观测（每 bay 左/右各一帧）")

    entry_x = min(v["x"] for v in views) - 1.2
    p0 = poller.get()
    if p0 is None or abs(p0[1] - y_line) > 0.6 or abs(p0[0] - entry_x) > 3.0:
        # 从任意位置安全转场（横通道 + 目标巷道，绝不斜穿货架）
        # 转场预算 = 距离 × RTF 折算 × 余量。固定 600 s 在"绕横通道 76 m"那条腿上
        # 刚好不够（76 / 0.45 / 0.223 ≈ 758 s 墙钟），实测把整条 B 巷判成"转场失败"。
        p0d = poller.get() or (entry_x, y_line, 0.0)
        dist = math.hypot(entry_x - p0d[0], y_line - p0d[1])
        budget = max(420.0, dist / max(args.speed, 0.20) / max(RTF_EST, 0.12) * 1.8 + 120.0)
        print(f"[seg2] {aisle} 巷转场 {dist:.0f} m，预算 {budget:.0f} s 墙钟")
        if not safe_route(poller, entry_x, y_line, speed=max(args.speed, 0.45),
                          max_seconds=min(budget, max(args.max_seconds, 420.0))):
            print(f"[seg2] {aisle} 巷转场失败，跳过")
            return 0

    partial = out.with_name(out.stem + ".partial.npz")
    frames, done = load_partial(partial, args.pitch)
    if frames:
        print(f"[seg2] {aisle} 巷续跑：断点里已有 {len(frames)} 帧（跳过已覆盖的观测）")
    t_start = time.monotonic()
    if node.fresh(20.0) is None:
        print("[seg2] 拿不到 RGB-D（相机桥没起？）-> 未跑")
        return 0
    for i, v in enumerate(views):
        if (v["bay"], v["side"]) in done:
            continue
        if time.monotonic() - t_start > args.max_seconds:
            print(f"[seg2] {aisle} 巷到时间上限，收在 {i}/{len(views)} 个观测")
            break
        p_now = poller.get()
        dist = math.hypot(v["x"] - p_now[0], v["y"] - p_now[1]) if p_now else 60.0
        budget = 2.2 * dist / max(args.speed, 0.05) + 25.0
        if not drive_to_point(poller, v["x"], v["y"], args.speed, 0.12, budget):
            print(f"[seg2] 到 {aisle}-{v['bay']}-{v['side']} 观测位失败，跳过")
            continue
        ok_turn, yaw_err_deg = turn_to_precise(poller, v["yaw"], tol_deg=2.0)
        cmd(0.0, 0.0, 0.0)
        if yaw_err_deg > 5.0:
            # **宁缺毋错**：朝向错了的帧会把"对面货架"当成这一侧存下来，
            # 下游会把它当真实观测（实测出现过 140° 残差）。这种帧只能丢。
            print(f"[seg2] {aisle}-{v['bay']}-{v['side']} 朝向未对齐"
                  f"（残差 {yaw_err_deg:.1f}°），丢弃这一帧")
            continue
        node.mark()
        got = node.fresh(15.0)
        if got is None:
            print(f"[seg2] {aisle}-{v['bay']}-{v['side']} 取图失败（相机桥没出新帧？）")
            continue
        rgb, dep, k = got
        p = poller.get() or (0.0, 0.0, 0.0)
        h, w = dep.shape
        core = dep[h // 4: 3 * h // 4, w // 4: 3 * w // 4]
        core = core[np.isfinite(core) & (core > 0.1)]
        core_med = float(np.median(core)) if core.size else float("nan")
        if core_med < 0.60:
            # 正前方 <0.6 m 有东西：那是挡在观测位前的障碍（C 巷 bay5 的叉车就是这么来的），
            # 拍下来的不是这一侧货架 —— 存进去会污染 SEG-2 的占用/类别统计。
            print(f"[seg2] {aisle}-{v['bay']}-{v['side']} 前方 {core_med:.2f} m 被遮挡，"
                  f"丢弃这一帧（不是该货架面的观测）")
            continue
        frames.append({"rgb": rgb, "depth": dep, "k": k,
                       "pose": np.array(p, np.float64), "bay": v["bay"],
                       "side": v["side"], "addr": v["addr"],
                       "yaw_err_deg": float(yaw_err_deg)})
        if len(frames) % 8 == 0:
            save_frames(frames, partial, args.pitch)
            print(f"[seg2] {aisle} 巷断点已存 {len(frames)} 帧 -> {partial.name}")
        fin = np.isfinite(dep) & (dep > 0.1)
        warn = "  ⚠️ 中心深度 >4 m，可能透过空货位看到对面" if core_med > 4.0 else ""
        print(f"  [seg2] {i + 1}/{len(views)}  {aisle}-{v['bay']}-{v['side']}  "
              f"位姿 ({p[0]:+.2f},{p[1]:+.2f}) yaw={math.degrees(p[2]):+.1f}°  "
              f"深度有效 {100 * fin.mean():.0f}%  中位 {np.median(dep[fin]):.2f} m  "
              f"转向残差 {yaw_err_deg:4.1f}°{warn}")

    if not frames:
        print(f"[seg2] {aisle} 巷一帧都没抓到")
        return 0
    save_frames(frames, out, args.pitch)
    partial.unlink(missing_ok=True)
    print(f"[seg2] {aisle} 巷 {len(frames)} 帧 -> {out}")
    return len(frames)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--aisle", default="A", help="单巷模式（配合 --bays）")
    ap.add_argument("--bays", default="1-24", help="bay 范围，如 1-24 或 1-3,7,9")
    ap.add_argument("--aisles", default="",
                    help="多巷模式：逗号分隔，如 A,B,C；'all' = 全部。每巷一个 npz")
    ap.add_argument("--out", default="", help="单巷输出路径（多巷模式忽略，按巷命名）")
    ap.add_argument("--force", action="store_true", help="已有产物也重抓")
    ap.add_argument("--speed", type=float, default=0.30)
    ap.add_argument("--settle", type=float, default=0.5, help="转向后等几秒让图像稳定")
    ap.add_argument("--max-seconds", type=float, default=3600.0, help="每条巷的墙钟上限")
    ap.add_argument("--pitch", type=float, default=16.0,
                    help="相机仰角（度）。必须与 make seg2-robot 的 cam_pitch 一致")
    args = ap.parse_args()

    sg = json.loads((ROOT / "world" / "slot_grid.json").read_text(encoding="utf-8"))

    if args.aisles:
        aisles = list(get_catalog().aisles) if args.aisles.lower() == "all" \
            else [a.strip().upper() for a in args.aisles.split(",") if a.strip()]
        want = set(parse_range(args.bays))
        jobs = []
        for aisle in aisles:
            ok, bad = reachable_bays(sg, aisle, sorted(want))
            if bad:
                print(f"[seg2] {aisle} 巷 bay {bad[0]}..{bad[-1]} 被路障封死（catalog blocked），"
                      f"只能采到 bay {ok[-1] if ok else '-'}")
            if not ok:
                continue
            jobs.append((aisle, ok, ROOT / "results" / f"seg2_frames_{aisle}.npz"))
        if not jobs:
            print("[seg2] 没有可采的巷道")
            return 2
    else:
        ok, _bad = reachable_bays(sg, args.aisle, sorted(set(parse_range(args.bays))))
        out = Path(args.out) if args.out else ROOT / "results" / f"seg2_frames_{args.aisle}.npz"
        jobs = [(args.aisle.upper(), ok, out)]

    poller = TruthPoller()
    t0 = time.monotonic()
    while poller.get(10.0) is None and time.monotonic() - t0 < 30:
        time.sleep(0.3)
    if poller.get() is None:
        print("[seg2] 读不到真值位姿 —— 仿真没起？")
        return 2

    rclpy.init()
    node = Cam()
    manifest: dict[str, object] = {}
    total = 0
    try:
        for aisle, bays, out in jobs:
            if out.exists() and out.stat().st_size > 1_000_000 and not args.force:
                print(f"[seg2] 跳过 {aisle}：已有 {out.name}（--force 可重抓）")
                manifest[aisle] = {"status": "skipped", "out": str(out.relative_to(ROOT))}
                continue
            y_line = aisle_y(sg, aisle)
            n = capture_aisle(node, poller, sg, aisle, bays, y_line, out, args)
            total += n
            manifest[aisle] = {"status": "ok" if n else "empty", "frames": n,
                               "bays": [bays[0], bays[-1]], "out": str(out.relative_to(ROOT))}
    finally:
        cmd(0.0, 0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()
        poller.stop()

    man = ROOT / "results" / "seg2_capture_manifest.json"
    prev = {}
    if man.exists():
        try:
            prev = json.loads(man.read_text(encoding="utf-8")).get("aisles") or {}
        except Exception:
            prev = {}
    prev.update(manifest)
    man.parent.mkdir(parents=True, exist_ok=True)
    man.write_text(json.dumps({"schema": "depot-vln/seg2_capture@1",
                               "cam_pitch_deg": args.pitch, "aisles": prev},
                              indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"[seg2] 合计 {total} 帧；manifest -> {man.relative_to(ROOT)}")
    return 0 if total else 2


if __name__ == "__main__":
    sys.exit(main())
