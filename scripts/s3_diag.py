#!/usr/bin/env python3
"""S3 归因诊断：把"真值 / 视觉里程计 odom / 建图后的 map->base"三条轨迹同时记录下来。

为什么必须分开记：整条路线跑完后 ATE 32.4 m，但**这只说明整条链坏了，不说明是谁坏的**。
链上有两段：
    rtabmap stereo_odometry  ->  odom -> base_footprint   （视觉能不能测出运动）
    slam_toolbox             ->  map  -> odom             （激光建图把 odom 掰回来没有）
只看 map->base，无法区分"视觉测不出来"和"激光把视觉掰歪了"。所以这里同时读两条 TF。

路线（复刻正式路线里最关键的几何：先进巷，再沿巷前进 20 m —— 相机朝 +x、运动也朝 +x，
即**沿光轴前进**，正是长路线里退化成 3% 的那一段）：
    wp1  (-28.5, -10.75)   进 A 巷
    wp2  (-8.5,  -10.75)   沿巷前进 20 m

输出：每 5 s 一行（真值/视觉/建图 相对起点的位移与 yaw），最后给三个比值。
判读：
    * 视觉位移/真值位移 ≈ 0.03 -> 视觉沿光轴退化（重复纹理歧义），与 slam_toolbox 无关
    * 视觉比值 ≈1 而建图比值 ≪1 -> slam_toolbox 把好 odom 掰坏了
"""
from __future__ import annotations

import math
import os
import sys
import time
from pathlib import Path

import hashlib
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from drive_to import drive_to_point  # noqa: E402
from slam_route import TruthPoller  # noqa: E402

WAYPOINTS = [(-28.5, -10.75), (-8.5, -10.75)]
# 速度可调（DIAG_SPEED=0.8 用来做"前进还原率 vs 每帧位移"的表征）：
# 每帧位移 = 速度 ÷ 帧率，10 Hz 下 0.35 m/s → 3.1 cm/帧 ≈ 2 m 处 1 px 视差，
# 已接近特征定位噪声。若提高速度后 leg2 还原率明显上升，就说明这是 SNR 问题而不是算法缺陷。
SPEED = float(os.environ.get("DIAG_SPEED", "0.35"))


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class TfReader(Node):
    def __init__(self) -> None:
        super().__init__("s3_diag")
        try:
            self.declare_parameter("use_sim_time", True)
        except Exception:
            pass
        self.buf = Buffer(cache_time=Duration(seconds=15))
        self.listener = TransformListener(self.buf, self)
        # **必须后台常驻 spin**：踩过的坑 —— 在驾驶循环里只靠 spin_once 采样，
        # 订阅队列（depth 3）在 10 Hz 下 0.3 s 就溢出，实测 120 s 只收到 38 帧，
        # 于是"画面更新率 0.05 Hz"全是自己的采样率，不是相机的帧率（假阴性）。
        # 用可停止的 spin 循环：直接 rclpy.spin 在退出时与 destroy_node/shutdown 竞争，
        # 会 "terminate called without an active exception" 并 dump core（实测两次）。
        self._stop = False
        # 图像变化量监测：如果 VO 说"没动"而图像压根没变，那是相机/中继的问题；
        # 图像变化明显而 VO 说"没动"，才是估计器/重复纹理的问题。两种病修法相反。
        self.prev_img = None
        self.diffs: list[float] = []
        self.latest_l = None      # 最新左目（原分辨率，用于存图/算视差）
        self.latest_r = None
        self.lr_diff: list[float] = []
        self.n_lr = 0
        self.lr_stamp_diff: list[float] = []   # |t_left - t_right|，判左右目是否同相
        # **新鲜度**：直接量"内容真的变了"的频率，而不是"收到多少条消息"。
        # gz 的相机即使在画面不更新时也会按 update_rate 重复发同一帧，
        # 所以 hz(消息) 完全不能证明画面在动 —— 这也是 SEG-2 当时踩的"缓存帧"坑。
        self.fresh = {"raw": [0.0, 0.0], "relay": [0.0, 0.0]}   # [上次变化时刻, 变化次数]
        self.prev_raw = None
        self.t_start = time.monotonic()
        # **无歧义的新鲜度判定**：对每帧原始字节做哈希，只有哈希变化才算"新帧"。
        # 之前用"平均灰度差 > 0.5"当阈值，而场景很暗（均值 38/255），真实的小位移
        # 帧差只有 0.1 左右 —— 会被误判成"没更新"，所以改哈希 + 全分辨率统计。
        self.hashes: list[str] = []
        self.uniq = {"raw": [0.0, 0], "all": 0}
        self.full_diff: list[float] = []
        self.prev_full = None
        qos = QoSProfile(depth=3, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, "/stereo_left", self._on_raw, qos)
        self.create_subscription(Image, "/stereo_left_tf", self._on_img, qos)
        self.create_subscription(Image, "/stereo_right_tf", self._on_right, qos)
        self._dump_dir = ROOT / "results" / "s3_diag_frames"
        self._dump_dir.mkdir(parents=True, exist_ok=True)
        self._next_dump = 0.0
        # **spin 线程必须在所有订阅建好之后才启动**（顺序反了不会报错，但早期回调会漏）
        self._spin = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin.start()

    def _spin_loop(self) -> None:
        while rclpy.ok() and not self._stop:
            try:
                rclpy.spin_once(self, timeout_sec=0.05)
            except Exception:
                break

    def stop(self) -> None:
        self._stop = True
        self._spin.join(timeout=1.0)

    def _on_raw(self, m: Image) -> None:
        """桥的原始输出：哈希统计真实新帧率（逐字节相同才算重复）。"""
        try:
            blob = bytes(m.data)
            h = hashlib.blake2b(blob, digest_size=8).hexdigest()
            self.uniq["all"] += 1
            if not self.hashes or h != self.hashes[-1]:
                self.uniq["raw"][1] += 1
                self.uniq["raw"][0] = time.monotonic()
            self.hashes.append(h)
            a_full = np.frombuffer(blob, dtype=np.uint8)
            if self.prev_full is not None and self.prev_full.shape == a_full.shape:
                self.full_diff.append(float(np.mean(np.abs(
                    a_full.astype(np.float32) - self.prev_full.astype(np.float32)))))
            self.prev_full = a_full
        except Exception:
            pass
        try:
            a = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(m.height, m.width, -1)
            a = a[::4, ::4].astype(np.float32)
            if self.prev_raw is not None and self.prev_raw.shape == a.shape:
                if float(np.mean(np.abs(a - self.prev_raw))) > 0.5:
                    self.fresh["raw"][1] += 1
                    self.fresh["raw"][0] = time.monotonic()
            self.prev_raw = a
        except Exception:
            pass

    def uniq_hz(self) -> tuple[float, float]:
        """返回（唯一帧率 Hz，收到总帧数）。"""
        el = max(time.monotonic() - self.t_start, 1e-6)
        return self.uniq["raw"][1] / el, float(self.uniq["all"])

    def fresh_hz(self, which: str) -> tuple[float, float]:
        """返回（内容更新频率 Hz，距上次内容变化多少秒）。"""
        el = max(time.monotonic() - self.t_start, 1e-6)
        cnt = self.fresh[which][1]
        return cnt / el, (time.monotonic() - self.fresh[which][0]) if cnt else float("nan")

    def _on_right(self, m: Image) -> None:
        self.latest_r = m
        self.latest_l = getattr(self, "latest_l", None)
        if self.latest_l is None:
            return
        # **左右目差异**：如果左右图像几乎一样 -> 视差≈0 -> 所有点被当成无穷远
        # -> 沿光轴平移在数学上不可观测（这正是"前进测不出、横向能测"的根因）。
        a = np.frombuffer(bytes(self.latest_l.data), dtype=np.uint8).reshape(
            self.latest_l.height, self.latest_l.width, -1)
        b = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(m.height, m.width, -1)
        if a.shape == b.shape:
            self.lr_diff.append(float(np.mean(np.abs(a[::4, ::4].astype(np.float32)
                                               - b[::4, ::4].astype(np.float32)))))
            tl = self.latest_l.header.stamp
            tr = m.header.stamp
            self.lr_stamp_diff.append(abs((tl.sec + tl.nanosec * 1e-9)
                                          - (tr.sec + tr.nanosec * 1e-9)))
            self.n_lr += 1

    def dump_pair(self, tag: str) -> str | None:
        """存一张 左 | 右 | |左-右|×4 的对比图，供肉眼与数值复核。"""
        if self.latest_l is None or self.latest_r is None:
            return None
        import cv2
        a = np.frombuffer(bytes(self.latest_l.data), dtype=np.uint8).reshape(
            self.latest_l.height, self.latest_l.width, -1)[:, :, :3]
        b = np.frombuffer(bytes(self.latest_r.data), dtype=np.uint8).reshape(
            self.latest_r.height, self.latest_r.width, -1)[:, :, :3]
        d = cv2.absdiff(a, b)
        d = np.clip(d.astype(np.int16) * 4, 0, 255).astype(np.uint8)
        img = np.concatenate([a, b, d], axis=1)
        p = self._dump_dir / f"{tag}.png"
        cv2.imwrite(str(p), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        return str(p)

    def lr_stat(self) -> tuple[float, float]:
        if not self.lr_diff:
            return float("nan"), 0.0
        arr = self.lr_diff[-200:]
        return float(np.median(arr)), float(np.max(arr))

    def _on_img(self, m: Image) -> None:
        try:
            a = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(m.height, m.width, -1)
            self.latest_l = m
            a = a[::4, ::4].astype(np.float32)          # 4x 下采样，够用且便宜
            if self.prev_img is not None and self.prev_img.shape == a.shape:
                dlt = float(np.mean(np.abs(a - self.prev_img)))
                self.diffs.append(dlt)
                if dlt > 0.5:
                    self.fresh["relay"][1] += 1
                    self.fresh["relay"][0] = time.monotonic()
            self.prev_img = a
        except Exception:
            pass

    def img_stat(self) -> tuple[float, float]:
        if not self.diffs:
            return float("nan"), 0.0
        arr = self.diffs[-200:]
        return float(np.median(arr)), float(np.max(arr))

    def pose(self, target: str, source: str = "base_footprint"):
        try:
            t: TransformStamped = self.buf.lookup_transform(
                target, source, Time(), timeout=Duration(seconds=0.2))
            p, q = t.transform.translation, t.transform.rotation
            return p.x, p.y, yaw_of(q)
        except Exception:
            return None


def main() -> int:
    rclpy.init()
    node = TfReader()
    poller = TruthPoller()
    t0 = time.monotonic()
    while poller.get(10.0) is None and time.monotonic() - t0 < 30:
        time.sleep(0.3)
    if poller.get() is None:
        print("[diag] 读不到真值")
        return 2

    # 等两条 TF 都出现（build 起来需要几秒）
    for _ in range(120):
        time.sleep(0.1)
        if node.pose("odom") and node.pose("map"):
            break
    if not node.pose("odom"):
        print("[diag] ❌ 没有 odom->base_footprint（视觉里程计没在发 TF）")
        return 2
    have_map = node.pose("map") is not None
    print(f"[diag] odom TF ✓   map TF {'✓' if have_map else '✗(slam_toolbox 还没出图)'}")

    time.sleep(0.2)
    p = poller.get()
    assert p is not None
    start = {"truth": (p[0], p[1], p[2]), "vo": node.pose("odom"), "slam": node.pose("map")}
    print(f"[diag] 起点 真值({start['truth'][0]:+.2f},{start['truth'][1]:+.2f})  "
          f"VO({start['vo'][0]:+.2f},{start['vo'][1]:+.2f})")

    done = 0.0
    for i, (wx, wy) in enumerate(WAYPOINTS, 1):
        q = poller.get() or (0.0, 0.0, 0.0)
        d = math.hypot(wx - q[0], wy - q[1])
        print(f"[diag] leg{i} -> ({wx:+.1f},{wy:+.1f})  {d:.1f} m")
        ok = drive_to_point(poller, wx, wy, speed=SPEED, tol=0.30,
                           max_seconds=60 + 4 * d, progress=_progress(node, poller, start))
        node.dump_pair(f"leg{i}_end")
        done += d
        print(f"[diag] leg{i} {'到位' if ok else '未完全到位'}")

    time.sleep(0.3)
    p = poller.get()
    assert p is not None
    # 起点没有 map TF 不代表最后也没有：slam_toolbox 要等扫到足够多的点才建图，
    # 所以这里重新判定一次，而不是沿用起点那一刻的结论。
    if not have_map:
        for _ in range(40):
            time.sleep(0.1)
            if node.pose("map") is not None:
                have_map = True
                start["slam"] = node.pose("map")   # 建图起点取它第一次出现的位置
                break
    end = {"truth": (p[0], p[1], p[2]), "vo": node.pose("odom"), "slam": node.pose("map")}

    def disp(k):
        s, e = start[k], end[k]
        if s is None or e is None:
            return float("nan"), float("nan")
        return math.hypot(e[0] - s[0], e[1] - s[1]), math.degrees(e[2] - s[2])

    dt, _ = disp("truth")
    dv, vyaw = disp("vo")
    ds, syaw = disp("slam")
    print()
    print(f"[diag] 真值位移 {dt:.2f} m")
    print(f"[diag] 视觉位移 {dv:.2f} m  比值 {dv / max(dt, 1e-6):.3f}  yaw {vyaw:+.1f}°")
    if have_map:
        print(f"[diag] 建图位移 {ds:.2f} m  比值 {ds / max(dt, 1e-6):.3f}  yaw {syaw:+.1f}°")
    lm, lx = node.lr_stat()
    md, mx = node.img_stat()
    print(f"[diag] 左右目帧差 中位 {lm:.2f} 最大 {lx:.2f}（灰度 0-255；≈0 就是视差没了）")
    if node.lr_stamp_diff:
        sd = sorted(node.lr_stamp_diff)[len(node.lr_stamp_diff) // 2]
        print(f"[diag] 左右目戳差 中位 {sd * 1000:6.1f} ms（≈0 才算同相；>50 ms 视差会错）")
    print(f"[diag] 时间帧差   中位 {md:.2f} 最大 {mx:.2f}（≈0 就是画面没变）")
    for which, label in (("raw", "桥输出 /stereo_left"), ("relay", "中继输出 /stereo_left_tf")):
        hz, since = self_hz = node.fresh_hz(which)
        print(f"[diag] 画面真实更新率 {label:24s} {hz:5.2f} Hz（距上次变化 {since:5.1f} s）")
    uhz, ntot = node.uniq_hz()
    if node.full_diff:
        fd = sorted(node.full_diff)
        fc = fd[len(fd) // 2]
    else:
        fc = float("nan")
    print(f"[diag] **哈希判定**：收到 {int(ntot)} 帧，其中唯一（非重复）{uhz * max(1e-6, (time.monotonic() - node.t_start)):.0f} 帧"
          f" -> 唯一帧率 {uhz:5.2f} Hz；全分辨率时帧差中位 {fc:6.2f}（灰度）")
    print()
    if lm < 1.0 and dt > 10:
        print("[diag] 结论：**左右目图像几乎相同（视差≈0）**")
        print("       -> 所有特征被当成无穷远，沿光轴平移数学上不可观测；")
        print("          这与'横向能测/前进测不出'完全一致。要查双目基线/渲染是否是同一视角。")
    elif dv / max(dt, 1e-6) < 0.5:
        print("[diag] 结论：**视觉本身沿光轴退化**（VO 只测出不到一半的前进位移）")
        print("       -> 与 slam_toolbox 无关；病因在重复纹理导致的对应歧义")
    elif have_map and ds / max(dt, 1e-6) < 0.5:
        print("[diag] 结论：视觉位移正常，是 **slam_toolbox 把好 odom 掰坏了**")
    else:
        print("[diag] 结论：视觉与建图都跟得上 —— 长路线里的崩溃发生在别处（看日志）")
    node.stop()
    node.destroy_node()
    rclpy.shutdown()
    poller.stop()
    return 0


def _progress(node, poller, start):
    state = {"n": 0}

    def cb(elapsed: float) -> None:
        state["n"] += 1
        if state["n"] % 4 == 0:          # 每 20 s 存一张 左|右|左右差 对比图
            p = node.dump_pair(f"t{int(elapsed):04d}")
            if p:
                print(f"    （已存对比图 {p}）")
        time.sleep(0.0)
        p = poller.get()
        if p is None:
            return
        vo = node.pose("odom")
        mp = node.pose("map")
        dv = math.hypot(vo[0] - start["vo"][0], vo[1] - start["vo"][1]) if vo else float("nan")
        dtr = math.hypot(p[0] - start["truth"][0], p[1] - start["truth"][1])
        dm = math.hypot(mp[0] - start["slam"][0], mp[1] - start["slam"][1]) if (mp and start["slam"]) else float("nan")
        md, mx = node.img_stat()
        lm, lx = node.lr_stat()
        hz_raw, _ = node.fresh_hz("raw")
        hz_rly, _ = node.fresh_hz("relay")
        uhz, ntot = node.uniq_hz()
        print(f"    t{elapsed:5.0f}s 真值 {dtr:5.2f} m | 视觉 {dv:5.2f} m"
              f"（{dv / max(dtr, 1e-6) * 100:5.0f}%）| 建图 {dm:5.2f} m"
              f" | 左右目差 {lm:5.2f} | **唯一帧 {uhz:4.2f}Hz**（共收 {int(ntot)}）")
    return cb


if __name__ == "__main__":
    sys.exit(main())
