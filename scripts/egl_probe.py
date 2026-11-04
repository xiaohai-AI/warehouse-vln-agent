#!/usr/bin/env python3
"""EGL / 渲染引擎对照实验（不依赖 ROS，不碰你正在跑的仿真）。

背景：Fortress 的 Sensors 系统在 server 进程里建渲染上下文。本机 Mesa 的 EGL 建不出
dri2 screen，渲染线程卡住 → 整个 world 不步进、所有传感器没数据。

做法：起一个**极小世界**（一个相机 + 一个 32 线雷达 + 一块箱子），
在独立 transport partition 里跑固定帧数，然后逐组"渲染引擎 × 环境变量"比较：
  仿真步进了吗 / 相机出图了吗 / 雷达出点了吗
每组都能独立判定，跑完就杀，不留进程。

  python3.10 scripts/egl_probe.py
  python3.10 scripts/egl_probe.py --only nvidia_egl
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import tempfile
import time
import uuid
from pathlib import Path

WORLD_TMPL = """<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="probe">
    <physics type="ode"><max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor><real_time_update_rate>250</real_time_update_rate></physics>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>{engine}</render_engine>
    </plugin>
    <light type="directional" name="sun"><pose>0 0 5 0 0.9 0</pose>
      <diffuse>1 1 1 1</diffuse><direction>0 0 -1</direction></light>

    <model name="ground" static="true">
      <link name="l"><collision name="c"><geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry></visual></link>
    </model>
    <model name="box" static="true">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="l"><collision name="c"><geometry><box><size>0.6 0.6 1.0</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>0.6 0.6 1.0</size></box></geometry></visual></link>
    </model>

    <model name="rig" static="true">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="cam_link">
        <sensor name="probe_cam" type="camera">
          <update_rate>10</update_rate><always_on>true</always_on><topic>probe_cam</topic>
          <camera><horizontal_fov>1.2</horizontal_fov>
            <image><width>320</width><height>240</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>20</far></clip></camera>
        </sensor>
      </link>
      <link name="lidar_link">
        <sensor name="probe_lidar" type="gpu_lidar">
          <update_rate>10</update_rate><always_on>true</always_on><topic>probe_lidar</topic>
          <lidar><scan><horizontal><samples>180</samples><min_angle>-3.14</min_angle><max_angle>3.14</max_angle></horizontal>
            <vertical><samples>8</samples><min_angle>-0.3</min_angle><max_angle>0.3</max_angle></vertical></scan>
            <range><min>0.2</min><max>20</max></range></lidar>
        </sensor>
      </link>
    </model>
  </world>
</sdf>
"""

VARIANTS = [
    # name,                       engine,  extra env
    ("baseline_ogre2",            "ogre2", {}),
    ("nvidia_egl",                "ogre2", {"__EGL_VENDOR_LIBRARY_FILENAMES": "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"}),
    ("nvidia_egl_surfaceless",    "ogre2", {"__EGL_VENDOR_LIBRARY_FILENAMES": "/usr/share/glvnd/egl_vendor.d/10_nvidia.json",
                                            "EGL_PLATFORM": "surfaceless"}),
    ("mesa_egl",                  "ogre2", {"__EGL_VENDOR_LIBRARY_FILENAMES": "/usr/share/glvnd/egl_vendor.d/50_mesa.json"}),
    ("soft_llvmpipe",             "ogre2", {"LIBGL_ALWAYS_SOFTWARE": "1", "MESA_LOADER_DRIVER_OVERRIDE": "llvmpipe",
                                            "GALLIUM_DRIVER": "llvmpipe", "EGL_PLATFORM": "surfaceless"}),
    ("engine_ogre1",              "ogre",  {}),
    ("engine_ogre1_nvidia",       "ogre",  {"__EGL_VENDOR_LIBRARY_FILENAMES": "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"}),
]


def probe(env_extra: dict[str, str], engine: str, wait_s: float, part: str) -> dict:
    env = dict(os.environ)
    env.update(env_extra)
    env["IGN_PARTITION"] = part
    world = Path(tempfile.mkdtemp(prefix="eglprobe_")) / "probe.sdf"
    world.write_text(WORLD_TMPL.format(engine=engine), encoding="utf-8")
    proc = subprocess.Popen(
        ["ign", "gazebo", "-s", "-r", "-v", "3", str(world)],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env,
    )
    time.sleep(wait_s)
    topics = ""
    data = {}
    try:
        topics = subprocess.run(["ign", "topic", "-l"], capture_output=True, text=True,
                                timeout=6, env=env).stdout
        for t in ("/probe_cam/image", "/probe_lidar/scan", "/probe_lidar"):
            try:
                r = subprocess.run(["ign", "topic", "-e", "-t", t, "-n", "1"],
                                   capture_output=True, text=True, timeout=4, env=env)
                if r.stdout.strip():
                    data[t] = len(r.stdout)
            except subprocess.TimeoutExpired:
                pass
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
    log = ""
    try:
        log = (proc.stdout.read() or "") if proc.stdout else ""
    except Exception:
        pass
    has_egl_err = "failed to create dri2" in log or "EGL" in log and "warning" in log
    sensor_topics = [l for l in topics.splitlines() if "probe" in l]
    return {
        "engine": engine,
        "env": list(env_extra.items()),
        "sensor_topics": sensor_topics,
        "data": data,
        "egl_error": has_egl_err,
        "ok": bool(data),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--wait", type=float, default=9.0)
    ap.add_argument("--only", default=None)
    args = ap.parse_args()

    results = []
    for name, engine, env_extra in VARIANTS:
        if args.only and args.only != name:
            continue
        part = "depot_eglprobe_" + uuid.uuid4().hex[:8]
        print(f"[probe] {name:<24} engine={engine:<5} env={list(env_extra)} ...", flush=True)
        r = probe(env_extra, engine, args.wait, part)
        r["name"] = name
        results.append(r)
        got = "、".join(r["data"].keys()) if r["data"] else "无"
        print(f"         -> 话题 {len(r['sensor_topics'])} 个，出数据: {got}"
              f"{'，有 EGL 报错' if r['egl_error'] else ''}", flush=True)

    print()
    print(f"{'variant':<24}{'engine':<7}{'传感器出数据':<14}{'EGL 报错':<10}可用")
    for r in results:
        got = "是" if r["data"] else "否"
        print(f"{r['name']:<24}{r['engine']:<7}{got:<14}{'是' if r['egl_error'] else '否':<10}"
              f"{'✔' if r['ok'] else ''}")
    ok = [r["name"] for r in results if r["ok"]]
    print()
    if ok:
        print("可用组合：", ", ".join(ok))
        print("把可用组合写进 config/world.yaml 的 render_engine + launch 的对应环境变量。")
        return 0
    print("全部组合都拿不到传感器数据 —— 需要换思路（换 gz 版本 / 用 xvfb 起 server / 软件渲染另装 mesa-utils）。")
    return 1


if __name__ == "__main__":
    sys.exit(main())
