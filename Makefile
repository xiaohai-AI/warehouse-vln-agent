# depot-vln —— 一条命令一件事
#
# 三层，互不依赖：
#   离线（不需要 ROS/GPU）：world / map-check / robot / bench / test / ci
#   仿真（需要 ROS Humble + Gazebo Fortress）：sim / nav2 / slam / nav-to
#   判据（需要仿真在跑）：m1 / m2 / s1-depth / slam1 / seg1 / seg2 / p6-demo

VENV    := .venv
PY      ?= $(shell [ -x $(VENV)/bin/python ] && echo $(VENV)/bin/python || echo python3.10)
ROS     := /opt/ros/humble/setup.bash
SENSORS ?= nav
TEXTURES ?= on
CAM_ARGS ?= cam_width:=320 cam_height:=240 cam_rate:=5

.DEFAULT_GOAL := help
.PHONY: help world map map-check robot verify perf inventory bench bench-perception \
        test ci sim down nav2 slam nav-to preflight m1 m1-truth m2 m2-static s1-depth \
        slam1 slam1-s3 slam1-s4 slam1-s5 seg1 seg1-capture-all seg2 seg2-capture seg2-robot \
        p6-demo p6-selftest guard-selftest cam-bridge cam-check gpu-observe drive-to \
        cmd-audit graph clean

help:
	@echo "depot-vln"
	@echo ""
	@echo "  离线（不需要 ROS）"
	@echo "    make world        生成世界 + 真值 + 静态图 + 自检"
	@echo "    make map-check    地图/世界/验收点位一致性闸门"
	@echo "    make robot        由 xacro 生成机器人 SDF（含摩擦/材质回填与几何自检）"
	@echo "    make inventory    把感知产物汇总成 results/inventory.json（语义层数据源）"
	@echo "    make bench        跑指令基准（rule 后端 + 显式替身库存）"
	@echo "    make bench-perception  用真实感知库存跑同一份基准（覆盖率低，如实计入）"
	@echo "    make test         单元测试（45 条，秒级）"
	@echo "    make ci           test + bench + 反硬编码守护"
	@echo ""
	@echo "  仿真（需要 ROS + Gazebo）"
	@echo "    make sim / make down / make nav2 / make nav-to X=-20 Y=-2"
	@echo ""
	@echo "  判据（仿真在跑时）"
	@echo "    make m1-truth / m2-static / s1-depth / slam1 / seg1 / seg2 / p6-demo"
	@echo "    make slam1-s3 / slam1-s4 / slam1-s5   视觉里程计三配置"

# ---------------------------------------------------------------- 世界与机器人
world:
	$(PY) scripts/build_world.py --verify
ifeq ($(TEXTURES),on)
	$(PY) scripts/make_textures.py
	$(PY) scripts/make_floor_texture.py
	$(PY) scripts/make_slot_labels.py
	$(PY) scripts/apply_world_textures.py
	$(PY) scripts/check_slot_signs.py
else
	$(PY) scripts/apply_world_textures.py --strip
endif
	$(PY) scripts/make_static_map.py

map:
	$(PY) scripts/make_static_map.py

map-check:
	$(PY) scripts/map_check.py

verify:
	$(PY) scripts/verify_world.py

perf:
	$(PY) scripts/world_perf_probe.py --limit 150 2>&1 | tee results/world_perf.log

robot:
	$(PY) scripts/gen_rollers.py
	bash -lc 'source $(ROS) && xacro robot/depot_bot.urdf.xacro sensors:=$(SENSORS) $(CAM_ARGS) > robot/depot_bot.urdf && gz sdf -p robot/depot_bot.urdf > robot/depot_bot.sdf && echo "robot/depot_bot.sdf OK ($(SENSORS))"'
	$(PY) scripts/patch_robot_friction.py robot/depot_bot.sdf $(FRICTION_ARGS)
	$(PY) scripts/patch_robot_friction.py robot/depot_bot.sdf --check
	$(PY) scripts/patch_robot_materials.py robot/depot_bot.sdf
	$(PY) scripts/patch_robot_materials.py robot/depot_bot.sdf --check
	$(PY) scripts/check_robot_geometry.py
	$(PY) scripts/check_wheels.py
	$(PY) scripts/check_robot_clearance.py

# ---------------------------------------------------------------- 语义与评测
inventory:
	$(PY) scripts/build_inventory.py

bench:
	$(PY) -m depot_vln.eval --backend rule --inventory sim --out results/bench.csv

bench-perception:
	$(PY) -m depot_vln.eval --backend rule --inventory perception --out results/bench_perception.csv

graph:
	$(PY) scripts/check_graph.py

test:
	$(PY) -m pytest tests -q

# 离线判据链：真正的断言（不是读日志对账）
ci: test bench guard-selftest
	@echo ""
	@echo "离线判据链：全部通过（test / bench / guard-selftest）"

guard-selftest:
	$(PY) scripts/forward_guard.py --selftest

p6-selftest:
	$(PY) -m pytest tests -q

# ---------------------------------------------------------------- 仿真栈
sim:
	@$(PY) scripts/stack_guard.py
	@nohup $(PY) scripts/gpu_watchdog.py > /dev/null 2>&1 &
	bash -lc 'source $(ROS) && ros2 launch launch/depot_sim.launch.py $(SIM_ARGS)'

down:
	bash scripts/stack_down.sh

nav2:
	bash -lc 'source $(ROS) && ros2 launch launch/nav2_omni.launch.py $(SIM_ARGS)'

slam:
	bash -lc 'source $(ROS) && ros2 launch launch/slam.launch.py $(SIM_ARGS)'

# 手动/演示驾驶一律用这个（有在线避障）。X/Y 必给，YAW 可选（度）
nav-to:
	KEEP=$(KEEP) BUDGET=$(BUDGET) bash scripts/nav_to.sh $(if $(X),--x $(X)) $(if $(Y),--y $(Y)) $(if $(YAW),--yaw $(YAW))

# 点对点驱动：**没有在线避障**，只用于采集/精修停位
drive-to:
	bash -lc 'source $(ROS) && $(PY) scripts/drive_to.py --x $(X) --y $(Y) $(if $(YAW),--yaw $(YAW)) $(if $(WP),--wp=$(WP)) $(if $(SPEED),--speed $(SPEED)) $(if $(TOL),--tol $(TOL))'

preflight:
	bash -lc 'source $(ROS) && $(PY) scripts/preflight.py'

cmd-audit:
	$(PY) scripts/cmd_audit.py --seconds $(if $(SECONDS),$(SECONDS),60) $(if $(CSV),--csv $(CSV))

# ---------------------------------------------------------------- 判据
m1:
	bash -lc 'source $(ROS) && $(PY) scripts/m1_velocity_tracking.py'

m1-truth:
	$(PY) scripts/m1_truth_check.py

m2:
	bash -lc 'source $(ROS) && $(PY) scripts/m2_side_dock.py $(M2ARGS)'

m2-static:
	bash scripts/m2_static_run.sh

s1-depth:
	bash -lc 'source $(ROS) && $(PY) scripts/s1_depth_eval.py --samples $${SAMPLES:-5} --out results/s1_depth.json'

slam1:
	CFG=$${CFG:-s1} bash scripts/slam1_run.sh

slam1-s3:
	CFG=s3 bash scripts/slam1_run.sh

slam1-s4:
	CFG=s4 bash scripts/slam1_run.sh

slam1-s5:
	CFG=s5 bash scripts/slam1_run.sh

seg1:
	$(PY) scripts/seg1_bay_grid.py --cloud $${CLOUD:-$$(ls results/seg1_path_*.npz 2>/dev/null || echo results/seg1_path_A.npz)} --out results/seg1.json --plot

seg1-capture-all:
	bash scripts/seg1_capture_all.sh

seg2-robot:
	$(MAKE) robot SENSORS=nav CAM_ARGS='cam_width:=640 cam_height:=480 cam_rate:=10 cam_pitch:=16'

seg2-capture:
	bash -lc 'source $(ROS) && $(PY) scripts/seg2_capture.py --bays $${BAYS:-1-24} --aisle $${AISLE:-A} --pitch $${PITCH:-16} --out $${OUT:-results/seg2_frames.npz}'

seg2:
	$(PY) scripts/seg2_detect.py --frames $${FRAMES:-results/seg2_frames.npz} --out results/seg2.json --plot

# 全 6 条巷道的 SEG-2 采集（按巷落盘、断点续跑）。
# 每条巷 48 帧（24 bay × 左右各一帧），F 巷 18–24 被路障封死 -> 只采到 17。
# 需要先 `make seg2-robot && make sim && make cam-bridge`。速度按实测 RTF≈0.22 定。
seg2-all:
	.venv/bin/python -u scripts/seg2_capture.py --aisles all --bays 1-24 --speed 0.40 \
	  --max-seconds 5400 --pitch 16

# 已经在仿真里、只想补某几条巷：AISLES=C,E make seg2-aisles
seg2-aisles:
	.venv/bin/python -u scripts/seg2_capture.py --aisles $${AISLES:?AISLES=A,B} --bays $${BAYS:-1-24} \
	  --speed 0.40 --max-seconds 5400 --pitch 16

# 全巷检测（吃 results/seg2_frames_*.npz）
seg2-detect-all:
	$(PY) scripts/seg2_detect.py --frames $$(ls results/seg2_frames_*.npz) \
	  --out results/seg2.json --plot

# 一条指令端到端（真工具层）：起 sim + 相机桥 -> 跑 -> 拆栈
p6-demo:
	UTT="$(UTT)" ANSWER="$(ANSWER)" DRY="$(DRY)" BLOCK="$(BLOCK)" \
	  SUBSTITUTE="$(SUBSTITUTE)" GUARD="$(GUARD)" \
	  VERIFY="$(if $(VERIFY),$(VERIFY),vlm)" POSE="$(if $(POSE),$(POSE),truth)" \
	  NAV="$(if $(NAV),$(NAV),0)" ACT_BACKEND="$(ACT_BACKEND)" \
	  DYN="$(if $(DYN),$(DYN),0)" DYN_HOLD="$(if $(DYN_HOLD),$(DYN_HOLD),4.0)" \
	  GUI="$(if $(GUI),$(GUI),0)" RTF="$(if $(RTF),$(RTF),0.44)" \
	  KEEP="$(if $(KEEP),$(KEEP),0)" bash scripts/p6_demo.sh

# ---------------------------------------------------------------- 相机 / GPU
cam-bridge:
	bash scripts/cam_bridge.sh

cam-check:
	bash scripts/cam_check.sh

gpu-observe:
	$(PY) scripts/gpu_observe.py --minutes 10 --json results/gpu_cam.json

clean:
	rm -rf world/warehouse.sdf world/layout.json results/*.csv results/*.npz transitions/*.jsonl
	@echo "已清理生成物（保留世界配置、真值与证据 json）"
