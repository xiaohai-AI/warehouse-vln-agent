# depot-vln —— 一条命令一个判据
# 依赖（langgraph 等）装在 .venv 里：venv 存在就用它，否则退回系统 python3.10。
# 之前这里写死 python3.10，`make bench-run` 会 ModuleNotFoundError: langgraph —— 离线判据必须一条命令能复跑。
VENV    := .venv
SENSORS ?= nav
# 世界外观：on = 给表面注入程序化 PBR 纹理（视觉/双目/货位识别需要纹理）；off = 纯色世界（A/B 对照）
TEXTURES ?= on
# 相机分辨率/帧率：第一次开渲染永远用 320x240@5Hz（GPU 风险点）
CAM_ARGS ?= cam_width:=320 cam_height:=240 cam_rate:=5
PY ?= $(shell [ -x $(VENV)/bin/python ] && echo $(VENV)/bin/python || echo python3.10)
ROS     := /opt/ros/humble/setup.bash

.DEFAULT_GOAL := help
.PHONY: help cam-bridge cam-check capture-cloud drive-to nav-to cmd-audit s3-smoke slam1-s3 s4-vo-selftest s4-selftest s4-floorflow s4-smoke slam1-s4 s5-smoke slam1-s5 seg1 seg1-capture-all seg1-capture seg2 seg2-capture seg2-robot gpu-observe s1-depth world map map-check loc scan-check slam-smoke slam1 verify perf bench bench-run graph robot sim probe preflight m1 m2 m2-static accept ci clean

help:
	@echo "depot-vln"
	@echo "  make world        生成世界 + 真值 + 自检（判据 P1：巷道净宽 3.20±0.05）"
	@echo "  make verify       只跑世界自检（读已有 world/warehouse.sdf）"
	@echo "  make perf         世界启动耗时 + RTF（不依赖 ROS，结果进 results/world_perf.log）"
	@echo "  make bench        生成 60+ 条四类指令基准（results/bench_spec.csv）"
	@echo "  make bench-run    在 mock 工具层上跑基准（判据 LANG-1）"
	@echo "  make graph        GRAPH-1：语义图 vs 真值最短路（results/graph1.json）"
	@echo "  make robot        由 xacro 生成 robot/depot_bot.sdf 并做 SDF 语法检查"
	@echo "  make sim          起仿真（世界+机器人+桥接）—— 需要手动确认"
	@echo "  make probe        gz 侧健康探测（不需要 ROS 环境，不会挂住）"
	@echo "  make preflight    起飞前检查（话题/频率/RTF/TF/静止/直行）—— 需先 make sim"
	@echo "  make m1           速度跟踪量测（判据 M1）—— 需先 make sim"
	@echo "  make p2           preflight -> M1 -> 汇总（一条命令，仿真已在跑时用）"
	@echo "  make m2           侧移入库位量测（判据 M2）—— 需先 make sim + Nav2"
	@echo "  make accept       汇总所有已产出的判据数字"
	@echo "  make clean        清掉生成物（保留 config/ 与真值以外的源文件）"

world:
	$(PY) scripts/build_world.py --verify
ifeq ($(TEXTURES),on)
	# 世界本身要有纹理：视觉为主的仓库用纯色平面，双目/视觉 SLAM 物理上无信息可匹配
	$(PY) scripts/make_textures.py
	# 地面单独一张**覆盖全场、处处唯一**的大图（非平铺）。原因见脚本头部：
	# 货架每 2.4 m 重复 -> 沿光轴前进在视觉上不可观测（S3 实测只还原 0~3%）；
	# 地面 box 的面 UV 是 0..1，所以一张铺满的图就能给出全局唯一指纹，且不动碰撞几何。
	$(PY) scripts/make_floor_texture.py
	# 货位编号牌贴图：每个货位一张，字高由相机可读性反推（depot_vln/slot_signs.py）
	$(PY) scripts/make_slot_labels.py
	$(PY) scripts/apply_world_textures.py
	# 编号牌自检：牌在不在 / 字够不够大 / 会不会压 verify 观测窗口
	$(PY) scripts/check_slot_signs.py
else
	$(PY) scripts/apply_world_textures.py --strip
endif
	# 世界一变，静态图必须跟着重建 —— 地图只能从构建产物（SDF 静态碰撞体）派生。
	# 曾经的错误：地图只从 world/layout.json 刷货架，漏掉叉车/托盘/护栏，
	# 导致 M2 的入库点落在叉车体内（docs/TROUBLESHOOTING.md §20）。
	$(PY) scripts/make_static_map.py

# 静态图单独重建（不重建世界）
map:
	$(PY) scripts/make_static_map.py

# 一致性闸门：地图 == 世界（每个静态碰撞体都必须在图上为占据）+ 验收点位可用。
# 换世界 / 换机器人之后第一件事就跑这条，它把“地图缺障碍物”挡在仿真之外。
map-check:
	$(PY) scripts/map_check.py

# 绝对定位（静态图 + AMCL + /scan）。手动跑用于观察 AMCL 收敛情况；
# m2-static 默认已经用它（LOC=static 可退回旧的固定 map->odom 模式）。
# 出生点用 x0/y0/yaw0 给，必须和仿真实际出生点一致，否则 AMCL 从错的位置开始。
LOCARGS ?=
loc:
	bash -lc 'source $(ROS) && ros2 launch launch/localization_amcl.launch.py $(LOCARGS)'

# 扫描链一条命令自检：/lidar3d/points -> relay -> p2l -> /scan（起 slam 后自动收尾）
scan-check:
	bash scripts/scan_chain_check.sh

# S1 冒烟：点云链 + slam_toolbox 建图 12 m + 存图 + 基本统计（一条命令）
slam-smoke:
	bash scripts/slam_smoke.sh

# SLAM-1（P4 · S1 纯激光）：固定 199 m 路线 + 真值轨迹 + ATE/地图一致性（约 20–25 分钟）
slam1:
	bash scripts/slam1_run.sh

verify:
	$(PY) scripts/verify_world.py

# 世界启动耗时与 RTF 是 SPEC §11 的风险项（"60×40 m + 32 线 + 双目把 RTF 压到 0.5 以下"）。
# 每次改世界生成参数都该重跑这条，数字进报告；不依赖 ROS，也不占你的 ROS_DOMAIN_ID。
perf:
	$(PY) scripts/world_perf_probe.py --limit 150 2>&1 | tee results/world_perf.log

bench:
	$(PY) scripts/build_bench.py

# 后端可切：make bench-run（rule 基线，LANG-1 的记账口径）
#           BACKEND=ollama make bench-run              # 裸模型（对照组，需要 ollama 在跑）
#           BACKEND=ollama-constrained make bench-run  # P6-1 约束层（实验组）
BACKEND ?= rule
MODEL   ?= qwen2.5:7b
bench-run: bench
	$(PY) scripts/eval_bench.py --backend $(BACKEND) --model $(MODEL)

# P6-1 约束层离线自检（不依赖 ollama、不跑仿真，约 6 s）
#   结构抽取 / 落地面板 / 编译器 / schema 校验 + 对抗端到端（乱填的模型也 0 静默猜）
p6-selftest:
	$(PY) tests/test_constraint.py

# P6-2 一条指令端到端（真工具层）：起 sim + 相机桥 -> 跑 -> 拆栈
#   UTT="去 10 号位" ANSWER="C 巷右边第一层" make p6-demo
#   DRY=1 make p6-demo          # 只走 resolve/plan，不用仿真
p6-demo:
	# 显式转发所有开关：VERIFY/POSE/NAV/DYN/GUI/RTF/KEEP 之前只靠 make 的
	# "命令行变量自动导出到环境" 生效，写清楚免得哪天规则变了静默失效
	UTT="$(UTT)" ANSWER="$(ANSWER)" DRY="$(DRY)" BLOCK="$(BLOCK)" \
	  SUBSTITUTE="$(SUBSTITUTE)" GUARD="$(GUARD)" \
	  VERIFY="$(if $(VERIFY),$(VERIFY),vlm)" POSE="$(if $(POSE),$(POSE),truth)" \
	  NAV="$(if $(NAV),$(NAV),0)" ACT_BACKEND="$(ACT_BACKEND)" \
	  DYN="$(if $(DYN),$(DYN),0)" DYN_HOLD="$(if $(DYN_HOLD),$(DYN_HOLD),4.0)" \
	  GUI="$(if $(GUI),$(GUI),0)" RTF="$(if $(RTF),$(RTF),0.44)" \
	  KEEP="$(if $(KEEP),$(KEEP),0)" bash scripts/p6_demo.sh

# 前向安全层自检（纯逻辑，不需要 ROS/仿真）
guard-selftest:
	$(PY) scripts/forward_guard.py --selftest

# /scan：从多线激光抽一层并改写 frame_id（Nav2 的 costmap 与 slam 都用它）
scan-relay:
	bash -lc 'source $(ROS) && $(PY) scripts/scan_relay.py --type ring --in-topic /lidar3d --out-topic /scan --frame-id lidar3d_link'

# GRAPH-1：语义图路径长度 vs 真值栅格最短路（真值独立实现，不用被测图自己声明）
graph:
	$(PY) scripts/check_graph.py

# 摩擦 A/B：mu2=0 是教科书近似（辊子轴向零摩擦），但严格 0 会让该方向没有任何恢复力，
# 起停/接触切换时的不一致就变成无界滑移。real 辊子有滚动阻力，实测应取小正值。
#   make robot FRICTION_ARGS='--mu2 0.05'
FRICTION_ARGS ?=
robot:
	# xacro / gz 都在 ROS 环境里，必须 source 后再跑（否则 xacro: not found）
	# SENSORS=nav（默认）：三路相机 always_on=false（没人订阅就不渲染）+ 激光 360x16@5Hz
	# SENSORS=full：原始精度 1024x32@10Hz + 相机常开，只给 S1 这类要原图的判据用
	# 桌面和仿真共用一块 RTX 4080，传感器全开会挂死 nvidia-modeset（见 TROUBLESHOOTING §18）
	# 辊子布局是算出来的（9 辊 / 包络=轮半径 / 轴线 45° / 左右镜像），先生成再 xacro
	$(PY) scripts/gen_rollers.py
	bash -lc 'source $(ROS) && xacro robot/depot_bot.urdf.xacro sensors:=$(SENSORS) $(CAM_ARGS) > robot/depot_bot.urdf && gz sdf -p robot/depot_bot.urdf > robot/depot_bot.sdf && echo "robot/depot_bot.sdf OK ($(SENSORS))"' 
	# URDF 的 <gazebo><mu1>/<mu2> 会被 gz sdf -p 静默丢弃 -> 轮子没有各向异性摩擦 ->
	# 横移指令下轮子空转、车不动（实测 -100%）。这里补回 SPEC §4.2 补救① 的 fdir1。
	$(PY) scripts/patch_robot_friction.py robot/depot_bot.sdf $(FRICTION_ARGS)
	$(PY) scripts/patch_robot_friction.py robot/depot_bot.sdf --check
	# 同一个坑的第二处：`gz sdf -p` 连 URDF 的**命名材质**也一起丢（实测 92 个 visual、
	# <diffuse> 计数 0）—— 不补的话 GUI 里整台车是默认灰，看着像"简化模型"（结构其实一个没少）。
	$(PY) scripts/patch_robot_materials.py robot/depot_bot.sdf
	$(PY) scripts/patch_robot_materials.py robot/depot_bot.sdf --check
	# 几何自检：把"文档里的离地高度"变成断言（车身悬空 0.028 m 那种错不会让任何运动判据 FAIL，
	# 只会在镜头里看得出来 —— 所以必须单独断言：落地贴地 / 裙边 0.09 / 车体 0.15 / 相机 0.35 / 激光 0.55）
	$(PY) scripts/check_robot_geometry.py
	# 轮子几何自检：9 辊 / 无胎筒 / 外沿=轮半径 / 轴线 45° / 左右镜像
	# （上一版：实心胎把辊子全盖住 + 外沿 0.088 超碰撞半径 0.076 + 轴线没有轴向分量）
	$(PY) scripts/check_wheels.py
	# 干涉自检：轮子不许碰车体（裙边/车体/保险杠）、载荷箱不许进激光扇面
	$(PY) scripts/check_robot_clearance.py

# 想换参数：make sim SIM_ARGS='gui:=false software_render:=true'
SIM_ARGS ?=
# SEG-1 的输入：把车开到 A 巷里（两段路：先南移到 y=-10.75，再东进到 x=-20），存 3 帧点云。
# 为什么必须在巷道里：bay 网格是沿 x 每 2.4 m 一个，站横通道只看到货架端面，看不出周期。
seg1-capture:
	# 前提：车在 A 巷内。第 1 条负责把车开进巷（两段：先从横通道南移到 y=-10.75，再东进到巷内）；
	# 第 2 条沿巷道开一遍、边开边拼点云。车已在巷内时第 1 条只会多走 2.6 m。
	bash -lc 'source $(ROS) && $(PY) scripts/drive_to.py --x -28.5 --y -10.75 --yaw 0 --wp=-31,-10.75'
	bash -lc 'source $(ROS) && $(PY) scripts/capture_cloud_path.py --to 28.5,-10.75 --aisle-y -10.75 --out results/seg1_path_A.npz'

# SEG-2 的相机配置：640×480@10 + 16° 仰角。
# 为什么必须仰角：相机在 0.274 m、半垂直视场 36.9°，2.15 m 处只能看到 z≤1.89 m，
# 而第 2 层货位在 1.75–2.6 m —— 平视物理上看不见上层（SPEC §7.1 也写了"观测位只能仰视"）。
# 这不影响 P3/S1 的定版：那个配置是 cam_pitch=0。
seg2-robot:
	$(MAKE) robot SENSORS=nav CAM_ARGS='cam_width:=640 cam_height:=480 cam_rate:=10 cam_pitch:=16'

# SEG-2 抓取：按货位观测位拍左右两侧 RGB-D（BAYS=1-3 试点 / 1-24 全巷）
seg2-capture:
	bash -lc 'source $(ROS) && $(PY) scripts/seg2_capture.py --bays $${BAYS:-1-24} --aisle $${AISLE:-A} --pitch $${PITCH:-16} --out $${OUT:-results/seg2_frames.npz}'

# SEG-2 判据：只评下层（上层被层板遮挡）；可给多个帧文件合并评分
seg2:
	$(PY) scripts/seg2_detect.py --frames $${FRAMES:-results/seg2_frames.npz} --out results/seg2.json --plot

# SEG-1 判据：从点云纯几何建 bay 网格，与 world/slot_grid.json 对账
# 默认吃 results/seg1_path_*.npz 全部（单巷时就是 A 巷）；要指定就 CLOUD="a.npz b.npz"
seg1:
	$(PY) scripts/seg1_bay_grid.py --cloud $${CLOUD:-$$(ls results/seg1_path_*.npz 2>/dev/null || echo results/seg1_path_A.npz)} --out results/seg1.json --plot

# 全 6 条巷道的点云采集，分 3 批（GROUP=1|2|3），见脚本头部说明
seg1-capture-all:
	bash scripts/seg1_capture_all.sh

# S3 冒烟：5 分钟判定双目视觉里程计能不能用（自带仿真/桥/里程计全套）
s3-smoke:
	bash scripts/s3_smoke.sh

# S3 正式判据：双目视觉里程计 + slam_toolbox 跑完 203 m 固定路线（约 40 分钟）
slam1-s3:
	CFG=s3 bash scripts/slam1_run.sh

# S4 第 0 级探针：现有相机（pitch=0）的地面带 2D 光流能不能测出沿轴位移
#   先跑 s4-selftest（纯离线，合成地面验证公式），再跑 s4-floorflow（需仿真，约 8 分钟）
# 新视觉里程计（旋转补偿 + 联合解算）的离线自检：六工况（含边走边转），1 秒
s4-vo-selftest:
	.venv/bin/python scripts/s4_vo_selftest.py

s4-selftest:
	$(PY) scripts/s4_floorflow_selftest.py

s4-floorflow:
	CHECK=scripts/s4_floorflow_probe.py bash scripts/s3_smoke.sh

# S4 里程计冒烟（约 6 分钟）：floorflow_odom 出不出 odom + 沿轴 6 m 尺度
s4-smoke:
	NO_ODOM=1 EXTRA_NODES='.venv/bin/python scripts/floorflow_odom.py' \
	  CHECK=scripts/s4_check.py bash scripts/s3_smoke.sh

# S4 整条路线（约 25 分钟）：地面光流里程计 -> slam_toolbox，判据同 SLAM-1
slam1-s4:
	CFG=s4 bash scripts/slam1_run.sh

# S5 冒烟（约 7 分钟）：floorflow(不发 TF) + EKF 融合，测的是 /odometry/filtered
s5-smoke:
	NO_ODOM=1 ODOM_TOPIC=/odometry/filtered \
	  EXTRA_NODES='bash scripts/s5_nodes.sh' \
	  CHECK=scripts/s4_check.py bash scripts/s3_smoke.sh

# S5 整条路线（约 25 分钟）：地面光流 + 轮速 EKF -> slam_toolbox
slam1-s5:
	CFG=s5 bash scripts/slam1_run.sh

# 引号用 make 的 $(if ...) 展开，别用 shell 的 $${VAR:+...}：
# 后者在 `bash -lc '单引号串'` 里会把引号变成字面字符，argparse 收到一整串
# **必须用等号形式 --wp=x,y**：值以负号开头（-30.0,-1.60），空格形式会被 argparse 当成选项 →
# "argument --wp: expected one argument"（实测踩过两次：先是引号被吞，再是负号歧义）
# 用 Nav2 开过去（**有在线避障**：全局 + 局部 costmap）。没起栈会自动起、跑完自动拆。
# 手动/演示驾驶一律用这个。X/Y 必给；YAW 可选（度）；KEEP=1 保留栈；BUDGET 墙钟预算
nav-to:
	KEEP=$(KEEP) BUDGET=$(BUDGET) bash scripts/nav_to.sh $(if $(X),--x $(X)) $(if $(Y),--y $(Y)) $(if $(YAW),--yaw $(YAW))

# 滑移角审计：量"车是在正着走还是横着走"（SPEC §4.4 / DECISIONS D32）
# 只订阅 /odom，不发控制；另开终端在行驶时跑：make cmd-audit SECONDS=90
cmd-audit:
	.venv/bin/python scripts/cmd_audit.py --seconds $(if $(SECONDS),$(SECONDS),60) $(if $(CSV),--csv $(CSV))

# ⚠️ 开路点对点驱动：**没有在线避障**（只做开车前的 path_guard 几何预检）。
# 2026-11-03 用它沿巷道中线走，正面撞上静态叉车。手动驾驶请用 make nav-to。
# 保留它是因为 SEG-2 采集 / S3 检查 / demo 的末端精修需要"点对点 + 精确停位"。
drive-to:
	bash -lc 'source $(ROS) && $(PY) scripts/drive_to.py --x $(X) --y $(Y) $(if $(YAW),--yaw $(YAW)) $(if $(WP),--wp=$(WP)) $(if $(SPEED),--speed $(SPEED)) $(if $(TOL),--tol $(TOL)) $(if $(MAXSEC),--max-seconds $(MAXSEC))'

# 抓一帧 3D 点云 + 真值位姿落盘，给 SEG-1 离线调算法用（不用反复起仿真）
capture-cloud:
	bash -lc 'source $(ROS) && $(PY) scripts/capture_cloud.py --frames $${FRAMES:-1} --out results/seg1_cloud.npz'

# 相机桥（P3 双目深度 / S3 视觉 SLAM 用）：只在需要图像时起，Ctrl-C 即停渲染
cam-bridge:
	bash scripts/cam_bridge.sh

# 相机链路体检：三路相机的 gz 话题、ROS 话题、6 s 收包数一次全查
cam-check:
	bash scripts/cam_check.sh

# 开渲染时的安全观察：RTF / GPU 利用率 / 显存 / 内核错误，每 10 s 一行
gpu-observe:
	.venv/bin/python scripts/gpu_observe.py --minutes 10 --json results/gpu_cam.json

# P3 的 S1 判据：双目 SGBM 深度 vs RGB-D 标尺（1–6 m，中位<5% / 95分位<12%）
# 前提：仿真在跑 + `make cam-bridge` 在跑（相机 always_on=0，没人订阅就不渲染）
s1-depth:
	bash -lc 'source $(ROS) && $(PY) scripts/s1_depth_eval.py --samples $${SAMPLES:-5} --out results/s1_depth.json'

sim:
	@$(PY) scripts/stack_guard.py
	@nohup $(PY) scripts/gpu_watchdog.py > /dev/null 2>&1 &
	bash -lc 'source $(ROS) && ros2 launch launch/depot_sim.launch.py $(SIM_ARGS)'

# 一条命令拆干净整套栈（重起前必须先跑；防止两个 gz server 并存挂死 GPU）
down:
	bash scripts/stack_down.sh

probe:
	$(PY) scripts/gz_probe.py

# 从世界几何生成静态图（M2 阶段用；SLAM 建图那条路见 TROUBLESHOOTING §12）
map-static:
	$(PY) scripts/make_static_map.py

# 发静态图 + map->odom（Nav2 的 static_layer 吃 /map）
map-pub:
	bash -lc 'source $(ROS) && $(PY) scripts/static_map_pub.py $(MAPARGS)'

# 手动桥测试：换不同的 gz 类型名/话题组合，排查传感器桥接失败（见 TROUBLESHOOTING §12）
#   make bridge-test BRIDGE_ARG='/lidar3d@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'
BRIDGE_ARG ?= /lidar3d@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
bridge-test:
	bash -lc 'source $(ROS) && timeout 40 ros2 run ros_gz_bridge parameter_bridge "$(BRIDGE_ARG)"'

# 完美定位对照：用仿真真值持续修正 map->odom（分离"控制误差"与"定位误差"）。
# 用它跑出来的成绩必须标注"完美定位"，不能当作真实定位能力的证据。
truth-loc:
	bash -lc 'source $(ROS) && $(PY) scripts/truth_localization.py'

# S1 纯激光建图（p2l 切片 + slam_toolbox）。建图期间要开一段路，图才有内容。
slam:
	bash -lc 'source $(ROS) && ros2 launch launch/slam.launch.py $(SIM_ARGS)'

# 存图：slam_toolbox 的 /map 落成 maps/depot_slam1.{pgm,yaml}
map-save:
	bash -lc 'source $(ROS) && ros2 run nav2_map_server map_saver_cli -f maps/depot_slam1 --ros-args -p use_sim_time:=true && ls -l maps/'

# Nav2 全向栈（navigation_launch.py，不含 map_server：
# 全局代价地图的 static_layer 直接吃 slam_toolbox 的 /map，这就是 SLAM 模式）
nav2:
	bash -lc 'source $(ROS) && ros2 launch launch/nav2_omni.launch.py $(SIM_ARGS)'

preflight:
	bash -lc 'source $(ROS) && $(PY) scripts/preflight.py'

# 单话题诊断（桥有没有转发 / QoS 对不对）—— 排查 /clock、/odom 这类"有发布者没数据"
probe-topic:
	bash -lc 'source $(ROS) && $(PY) scripts/clock_probe.py --topic $(TOPIC) --type $(TYPE)'

m1:
	bash -lc 'source $(ROS) && $(PY) scripts/m1_velocity_tracking.py'

# 判据 M1 的真值版：用 Gazebo 真实位姿判，不用 /odom（/odom 是我们自己从轮速积出来的，
# 拿它judge 速度跟踪 = 自己给自己打分，SPEC §4.2 明确禁止）。
m1-truth:
	$(PY) scripts/m1_truth_check.py

# odom 的 yaw 单独验（M2 的闭环吃 odom，yaw 错了一切都错）
odom-yaw:
	bash -lc 'source $(ROS) && $(PY) scripts/odom_yaw_check.py'

# P2 全链（仿真已在跑的前提下）：preflight -> M1 -> 汇总。
# preflight 任何一项 FAIL 就停 —— 传感器全哑时量出来的"速度跟踪误差"量的是空气，不是底盘。
p2:
	bash -lc 'source $(ROS) && $(PY) scripts/preflight.py && $(PY) scripts/m1_velocity_tracking.py'
	$(PY) scripts/accept.py

# 参数位：make m2 M2ARGS='--attempts 2'
M2ARGS ?=
m2:
	bash -lc 'source $(ROS) && $(PY) scripts/m2_side_dock.py $(M2ARGS)'

# 一键 M2（静态世界图 + odom 定位）：自动起地图发布与 Nav2、等 bt_navigator active、
# 跑判据、跑完拆栈。避免手动起两套重名节点互相发零速（实测会凭空多出 1 m 落点误差）。
m2-static:
	bash scripts/m2_static_run.sh

docs-check:  ## 文档与产物一致性核对（P7-1）
	$(PY) scripts/docs_consistency.py --strict

delivery-check:  ## 交付自检（离线）：重算 16 条判据 + 文档一致 + 产物齐（P7-2）
	bash scripts/delivery_check.sh

ci:  ## 离线判据链 4/4（CI 跑的是同一条命令；不需要 ROS/GPU）
	bash scripts/ci_local.sh

accept:
	$(PY) scripts/accept.py

clean:
	rm -rf world/warehouse.sdf world/layout.json results/*.csv transitions/*.jsonl
