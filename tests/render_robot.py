#!/usr/bin/env python3
"""渲染并检查机器人模型：xacro -> URDF -> SDF（P2 的结构验收）。

不依赖 `source setup.bash`：直接把 ROS 的 python site-packages 加到 sys.path，
所以这个脚本可以在任何 shell 里跑。
"""
from __future__ import annotations

import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
# ROS 的 python 包可能在这几个位置之一（取决于安装方式），全部加上，脚本就不依赖
# 先 `source setup.bash` 再跑 —— 这样"一条命令"在任何 shell 里都成立。
ROS_PY_DIRS = [
    "/opt/ros/humble/local/lib/python3.10/dist-packages",
    "/opt/ros/humble/lib/python3.10/site-packages",
]
for p in (*ROS_PY_DIRS, str(ROOT)):
    if p not in sys.path:
        sys.path.insert(0, p)

XACRO = ROOT / "robot" / "depot_bot.urdf.xacro"
URDF = ROOT / "robot" / "depot_bot.urdf"
SDF = ROOT / "robot" / "depot_bot.sdf"

REQUIRED_JOINTS = ["wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint"]
REQUIRED_SENSORS = {"camera": 2, "rgbd_camera": 1, "gpu_lidar": 1, "imu": 1}


def main() -> int:
    import xacro  # type: ignore

    doc = xacro.process_file(str(XACRO))
    xml = doc.toprettyxml(indent="  ")
    URDF.write_text(xml, encoding="utf-8")
    print(f"[robot] URDF  {URDF.relative_to(ROOT)}  {len(xml) / 1024:.1f} KiB")

    root = ET.parse(URDF).getroot()
    links = root.findall("link")
    joints = {j.get("name") for j in root.findall("joint")}
    sensors: list[tuple[str, str]] = []
    plugins: list[str] = []
    for gz in root.findall("gazebo"):
        for s in gz.findall("sensor"):
            sensors.append((s.get("type"), s.get("name")))
        for p in gz.findall("plugin"):
            plugins.append(p.get("filename", ""))

    ok = True
    print(f"[robot] links={len(links)} joints={len(joints)} sensors={len(sensors)}")
    missing = [j for j in REQUIRED_JOINTS if j not in joints]
    if missing:
        ok = False
        print(f"  FAIL 缺关节: {missing}")
    got = {}
    for t, _n in sensors:
        got[t] = got.get(t, 0) + 1
    for t, n in REQUIRED_SENSORS.items():
        if got.get(t, 0) != n:
            ok = False
            print(f"  FAIL 传感器 {t} 数量 {got.get(t, 0)} != {n}")
    mech = [p for p in plugins if "mecanum" in p.lower()]
    if not mech:
        ok = False
        print("  FAIL 没有 MecanumDrive 插件")
    print(f"  {'PASS' if ok else 'FAIL'} 结构（麦克纳姆插件: {mech}）")

    # MecanumDrive 的参数名写错不会报错，只会被静默忽略（踩过：rear_left_joint /
    # left_wheel_radius / publish_odom 全被丢掉，插件等于没接上关节）。
    # 这里拿 ft 时的 strings 结果当白名单，硬卡住这个坑。
    MECANUM_PARAMS = {
        "front_left_joint", "back_left_joint", "front_right_joint", "back_right_joint",
        "wheel_radius", "wheel_separation", "wheelbase",
        "odom_publish_frequency", "odom_topic", "tf_topic",
    }
    plugin_el = None
    for gz in root.findall("gazebo"):
        for pl in gz.findall("plugin"):
            if "mecanum" in (pl.get("filename") or "").lower():
                plugin_el = pl
    if plugin_el is None:
        return 1
    unknown = [c.tag for c in plugin_el if c.tag not in MECANUM_PARAMS]
    joint_vals = {c.tag: (c.text or "").strip() for c in plugin_el if c.tag.endswith("_joint")}
    bad_joints = [f"{k}={v}" for k, v in joint_vals.items() if v not in joints]
    if unknown:
        ok = False
        print(f"  FAIL MecanumDrive 有 {len(unknown)} 个参数 Fortress 不认（会被静默忽略）: {unknown}")
    if len(joint_vals) != 4 or bad_joints:
        ok = False
        print(f"  FAIL 关节映射不全或指向不存在的关节: {joint_vals} / bad={bad_joints}")
    if not unknown and not bad_joints:
        print(f"  PASS MecanumDrive 参数白名单（4 个关节映射 OK，无被忽略的参数）")

    # URDF -> SDF（ros_gz_sim create 需要 SDF）
    ign = subprocess.run(["ign", "sdf", "-p", str(URDF)], capture_output=True, text=True)
    if ign.returncode != 0:
        print("  FAIL URDF->SDF 转换失败：", (ign.stderr or "").strip()[:400])
        return 1
    SDF.write_text(ign.stdout, encoding="utf-8")
    sdf_root = ET.fromstring(ign.stdout)
    sdf_links = sdf_root.findall(".//link")
    sdf_sensors = sdf_root.findall(".//sensor")
    sdf_plugins = [p.get("filename") for p in sdf_root.findall(".//plugin")]
    print(f"[robot] SDF   {SDF.relative_to(ROOT)}  links={len(sdf_links)} sensors={len(sdf_sensors)}")
    if not any("mecanum" in (f or "") for f in sdf_plugins):
        ok = False
        print("  FAIL SDF 里丢了 MecanumDrive 插件")
    if len(sdf_sensors) < sum(REQUIRED_SENSORS.values()):
        ok = False
        print(f"  FAIL SDF 传感器数 {len(sdf_sensors)} < {sum(REQUIRED_SENSORS.values())}")
    for s in sdf_sensors:
        print(f"    - {s.get('type'):<12} {s.get('name')}")

    # 视觉件包络检查：装饰件写错坐标（比如 z 多打了个 0）在 GUI 里很难第一时间发现，
    # 这里按"整车外形包络"卡住：|x| ≤ 0.36、|y| ≤ 0.30、0.03 ≤ z ≤ 1.10（含示位杆）
    # `ign sdf -p` 会把 fixed joint 的子 link 合并进根 link（base_footprint），
    # 所以不能按名字找 base_link —— 取"装视觉件最多的那个 link"。
    links_all = sdf_root.findall(".//link")
    body = max(links_all, key=lambda l: len(l.findall("visual")), default=None)
    out_of_env: list[str] = []
    n_vis = 0
    if body is not None:
        for vis in body.findall("visual"):
            n_vis += 1
            pose = vis.find("pose")
            if pose is None or not pose.text:
                continue
            vx, vy, vz = [float(v) for v in pose.text.split()[:3]]
            if abs(vx) > 0.36 or abs(vy) > 0.30 or vz < 0.03 or vz > 1.10:
                out_of_env.append(f"{vis.get('name')}@({vx:.2f},{vy:.2f},{vz:.2f})")
    roller_vis = len([v for v in (body.findall("visual") if body is not None else [])
                      if (v.get("name") or "").startswith("roller")])
    where = body.get("name") if body is not None else "?"
    print(f"[robot] {where} 视觉件 {n_vis} 个（其中辊子 {roller_vis} 个）")
    if out_of_env:
        ok = False
        print(f"  FAIL {len(out_of_env)} 个视觉件超出整车包络: {out_of_env[:5]}")
    else:
        print("  PASS 视觉件都在整车包络内")
    print()
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
