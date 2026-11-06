# ROS 2 打包与工业迁移计划

> **结论先行**：当前仓库是一个**可复现的仿真参考实现 / monorepo 应用**，不是 ROS 2
> workspace。它现在这样组织是有意的（离线可测、非 ROS 读者可直接跑 benchmark），
> 因此**不建议为了“看起来像 ROS 项目”而整体重排目录**。
>
> 如果目标变成“上真机、交付、多人长期维护”，应该迁移到 **ROS 2 + colcon workspace，
> 按子系统拆 package**；不是迁到 [ROS-Academy-for-Beginners](https://github.com/xiaohai-AI/ROS-Academy-for-Beginners)
> 那种 ROS 1/catkin 的课程 demo 结构。本文给出目标结构、文件映射和分阶段迁移步骤。
>
> 这份文档描述的是**迁移计划**，不是当前状态。当前运行方式仍以
> [README.md](../README.md) 的 `make` / `.venv` 为准。

## 1. 先分清两种“ROS 项目”

### 1.1 ROS-Academy 风格：按课程章节组织的 ROS 1 包集合

你给的仓库顶层大致是：

```
topic_demo/  service_demo/  action_demo/  tf_demo/
navigation_sim_demo/  slam_sim_demo/  robot_sim_demo/
orbslam2_demo/  rtabmap_demo/  cartographer_demo/ ...
```

每个目录是一个 catkin package，带 `package.xml` / `CMakeLists.txt` / `launch/` / `scripts/`。
它的目的是**教学**：

- 按知识点拆分，而不是按感知、定位、导航、决策这些产品子系统拆分；
- ROS 1 / catkin / `catkin_make`；
- 没有 CI、没有 launch_testing、没有部署描述、没有接口版本管理；
- 一个 demo 一个包，包之间可以没有稳定接口。

它能教会人 ROS 基础，但**不是工业模板**。把它当作目标，会把 ROS 2 Humble 项目
退回 ROS 1 的组织方式。

### 1.2 工业 ROS 2：按“可构建、可部署、可替换”的单元拆包

工业 ROS 2 项目通常是一个 colcon workspace：

```
ros2_ws/
  src/
    warehouse_vln_interfaces/     # msg/srv/action（需要时才建）
    warehouse_vln_core/           # 纯领域库：catalog/layout/semantics/inventory/world
    warehouse_vln_agent/          # LangGraph、NLU、工具层（可 off-board）
    warehouse_vln_description/    # URDF/xacro、mesh、robot_state_publisher
    warehouse_vln_sim/            # Gazebo world/model、仿真 launch
    warehouse_vln_perception/     # 相机、深度、SEG-2、库存快照
    warehouse_vln_localization/   # odom、map->odom、静态图、AMCL 配置
    warehouse_vln_navigation/     # Nav2 参数、导航封装、costmap 诊断
    warehouse_vln_diagnostics/    # preflight、健康检查、安全层、录制审计
    warehouse_vln_bringup/        # 唯一给用户启动的 launch/参数/编排
    warehouse_vln_worldgen/       # 构建期世界生成（不跑在机器人上）
    warehouse_vln_bench/          # 离线基准/评测（不依赖 ROS）
  build/  install/  log/
```

判断该不该拆成一个 package，看四件事：

1. **能不能独立构建/安装**：`colcon build --packages-select <pkg>`；
2. **有没有稳定的对外契约**：topic、action、service、参数、QoS；
3. **生命周期是否独立**：硬件驱动、感知、决策通常不同频、不同进程；
4. **是否由不同人/不同节奏维护**：驱动和算法通常不该互相绑版本。

工业项目真正在意的是这四条，而不是“目录里有没有 `src/`”。

## 2. 当前仓库是什么

当前结构（节选）：

```
depot_vln/                纯 Python 核心：catalog/layout/semantics/inventory/
                          world/semantic_graph、agent/、eval/
scripts/                  ROS 2 节点、采集、驾驶、判据的入口脚本
launch/                   ROS 2 launch（根目录）
config/                   world/semantics/nav/nav2/slam/amcl/ekf 参数
robot/                    xacro + 生成的 URDF/SDF
world/                    生成的世界 SDF/layout/slot_grid/textures
maps/                     静态图（PGM 生成物 + YAML）
tests/                    61 条 pytest（离线）
examples/                 4 个离线示例（含 LangGraph HITL）
docs/                     架构、限制、演示、本文件
Makefile                  make world/robot/sim/nav-to/... 的总入口
pyproject.toml            Python 包元数据（`depot-vln` / `depot_vln`）
```

### 优点（迁移时必须保留）

- `depot_vln/` 是**纯 Python、可 import、可单测**的核心；61 条测试不依赖 ROS；
- “运行时不许读真值”“未观测是一等状态”“只有 act 调动作接口”等边界有测试；
- 离线 benchmark、oracle、示例都能在没有 ROS/GPU 的机器上跑；
- 文档和限制清单比很多“工业 ROS 仓库”完整。

### 与工业 ROS 2 的差距

| 当前做法 | 工业 ROS 2 期望 |
|---|---|
| 根目录 `launch/`、`config/`、`robot/` | 资源装在 `install/<pkg>/share/<pkg>/` |
| `python scripts/xxx.py` | ament_python entry point / 可执行节点 |
| `make sim` 编排 | `ros2 launch warehouse_vln_bringup ...` |
| 源码树内运行，依赖 cwd / `.venv` | 安装后可运行，不依赖源码树 |
| 没有 `package.xml` | 每个 package 声明依赖、版本、维护者 |
| 没有 launch_testing | 每个关键 launch 有集成测试 |
| 参数散在 `config/*.yaml` + Python 常量 | 参数文件随 package 安装，节点声明 `use_sim_time` 等 |
| 无生命周期/组合/命名空间 | 关键节点用 lifecycle、composition、namespace |
| 无诊断/部署/安全层 | diagnostics、健康检查、Docker/systemd、权限与看门狗 |

## 3. 目标 package 划分

下面是建议的拆分。名字里的 `warehouse_vln` 对应本仓库的 `depot_vln`，迁移时可以保留
`depot_vln` 作为 Python 导入名，只改仓库/包展示名。

| package | 职责 | 是否跑在机器人上 |
|---|---|---|
| `warehouse_vln_core` | `catalog`、`layout`、`semantic_graph`、`semantics`、`inventory`、`world` | 是（纯库） |
| `warehouse_vln_agent` | `agent/`（LangGraph、NLU、compile、离线工具层） | 可选，通常 off-board |
| `warehouse_vln_interfaces` | 自定义 msg/srv/action；没有就**不要**建 | 是 |
| `warehouse_vln_description` | `robot/*.xacro`、mesh、`robot_state_publisher` launch | 是 |
| `warehouse_vln_sim` | Gazebo world、fixtures、仿真 launch、`ros_gz` 参数 | 否（仿真机） |
| `warehouse_vln_perception` | 相机/深度、`seg2_capture`、`seg2_detect`、库存构建 | 是 |
| `warehouse_vln_localization` | `mecanum_odom`、`truth_map_odom`、`static_map_pub`、AMCL/map_server 配置 | 是 |
| `warehouse_vln_navigation` | Nav2 参数、`nav_to`、`drive_to`、costmap 诊断 | 是 |
| `warehouse_vln_diagnostics` | `preflight`、`stack_guard`、`cmd_audit`、`gpu_watchdog`、前向安全层 | 是 |
| `warehouse_vln_bringup` | 用户唯一入口：launch、参数、节点组合、模式切换 | 是 |
| `warehouse_vln_worldgen` | `build_world`、纹理、编号牌、静态图、真值导出（构建期） | 否 |
| `warehouse_vln_bench` | `eval/`、`sim.py`、`truth.py`、`scoring.py`、基准/证据 | 否 |

> 关键原则：**`core`/`bench` 不依赖 ROS**。核心算法、评测、世界生成都可以是普通
> Python 包；只有需要 topic/action/参数/launch 的部分才进 ROS package。

## 4. 当前文件 → 目标 package 映射

| 当前路径 | 目标位置 |
|---|---|
| `depot_vln/catalog.py`、`layout.py`、`semantic_graph.py`、`semantics.py`、`inventory.py`、`world.py` | `warehouse_vln_core/` |
| `depot_vln/agent/**` | `warehouse_vln_agent/` |
| `depot_vln/eval/**` | `warehouse_vln_bench/` |
| `depot_vln/sim.py`、`truth.py`、`scoring.py` | `warehouse_vln_bench/`（**不要**进 runtime） |
| `depot_vln/sdf.py`、`slot_signs.py` | `warehouse_vln_worldgen/`（或 core 的共享几何） |
| `depot_vln/camera.py`、`extrinsics.py`、`verify_depth.py`、`floorflow.py` | `warehouse_vln_perception/` |
| `scripts/static_map_pub.py`、`truth_map_odom.py`、`mecanum_odom.py`、`scan_relay.py`、`truth_localization.py`、`nav_to.py` | `warehouse_vln_localization/` + `navigation/` |
| `scripts/seg2_capture.py`、`seg2_detect.py`、`s1_depth_eval.py`、`verify_probe.py` | `warehouse_vln_perception/` |
| `scripts/drive_to.py`、`nav_to.py`、`path_guard.py`、`costmap_probe.py`、`map_drive.py` | `warehouse_vln_navigation/` |
| `scripts/preflight.py`、`stack_guard.py`、`gpu_watchdog.py`、`cmd_audit.py`、`forward_guard.py` | `warehouse_vln_diagnostics/` |
| `scripts/build_world.py`、`make_textures.py`、`make_floor_texture.py`、`make_slot_labels.py`、`apply_world_textures.py`、`check_slot_signs.py`、`make_static_map.py`、`map_check.py`、`verify_world.py` | `warehouse_vln_worldgen/`（构建期命令） |
| `scripts/p6_demo.py`、`p6_demo.sh` | `warehouse_vln_bringup/`（或 `examples/`，取决于是否作为正式入口） |
| `launch/depot_sim.launch.py` | `warehouse_vln_sim/launch/` |
| `launch/nav2_omni.launch.py`、`slam*.launch.py`、`localization_amcl.launch.py` | `warehouse_vln_navigation/`、`localization/` |
| `config/world.yaml`、`semantics.yaml` | `warehouse_vln_core/config/`（或 `worldgen`） |
| `config/nav.yaml` | `warehouse_vln_navigation/config/` |
| `config/nav2_omni.yaml`、`amcl_omni.yaml`、`slam_toolbox.yaml`、`ekf*.yaml`、`rtabmap_stereo.yaml` | `warehouse_vln_navigation/` 或 `localization/` |
| `robot/*.xacro` | `warehouse_vln_description/urdf/` |
| `robot/depot_bot.sdf`、`depot_bot.urdf` | **生成物**：CI/构建期生成，不作为源码维护（或明确标为 generated） |
| `world/warehouse.sdf`、`layout.json`、`slot_grid.json`、`textures/` | `warehouse_vln_sim/` 的生成资源（`make world` 产出） |
| `maps/*.pgm`、`*.yaml` | `warehouse_vln_localization/maps/`（生成物） |
| `tests/` | 核心/agent/bench 的 pytest 留在 Python 包；ROS 包加 `launch_testing` |
| `examples/` | 仓库根 `examples/` 或 `warehouse_vln_bringup/examples/`；不要塞进 runtime |
| `docs/`、`README`、`Makefile` | 留在 monorepo 根；`Makefile` 只作开发入口 |

## 5. 分阶段迁移

**不要一次性大爆炸。** 每一步都保持现有 `make ci` 和离线示例可跑。

### Phase 0：文档与边界（本次）

- 保留当前仓库结构；
- 记录本文件；
- README/ARCHITECTURE 明确“这是 monorepo 应用，不是 ROS 2 package”；
- 验收：读者知道为什么、以及将来怎么拆。

### Phase 1：抽叶子包（description / sim / worldgen）

- 新建 `ros2_ws/src/warehouse_vln_description`：
  - `urdf/`（xacro + mesh）、`launch/robot_state_publisher.launch.py`、`package.xml`；
  - 用 `xacro` 在构建/安装时生成 URDF；生成的 SDF 不作为源码。
- 新建 `warehouse_vln_sim`：
  - `worlds/`、`models/`、`textures/`、`launch/sim.launch.py`、`config/`；
  - 依赖 `ros_gz_sim`、`ros_gz_bridge`；
- 新建 `warehouse_vln_worldgen`：
  - 把 `build_world.py` 等改成 console_scripts，输出到 `share/warehouse_vln_sim/worlds`（或显式 `--out`）。
- 验收：
  ```bash
  colcon build --packages-select warehouse_vln_description warehouse_vln_sim
  source install/setup.bash
  ros2 launch warehouse_vln_sim sim.launch.py gui:=true
  ```
  不再依赖源码树 cwd。

### Phase 2：抽 localization / navigation / bringup

- `warehouse_vln_localization`：把 `mecanum_odom.py`、`truth_map_odom.py`、`static_map_pub.py`、`scan_relay.py` 改成 ament_python entry points；
- `warehouse_vln_navigation`：Nav2 参数、`nav_to`、`drive_to`、costmap 诊断；
- `warehouse_vln_bringup`：唯一给用户的 launch，组合 sim/localization/navigation；
- 参数全部从 `config/*.yaml` 安装到 `share/<pkg>/config`，节点声明参数、启动传入；
- 移除脚本里的 `sys.path.insert(ROOT)` / `os.path.dirname(__file__)` 相对路径；
- 每个 launch 加一个 `launch_testing` 冒烟测试（起节点、等 topic、退出）。
- 验收：
  ```bash
  ros2 launch warehouse_vln_bringup sim.launch.py
  ros2 launch warehouse_vln_bringup navigation.launch.py
  colcon test --packages-select warehouse_vln_bringup
  ```

### Phase 3：抽 perception / agent / diagnostics

- `warehouse_vln_perception`：`seg2_capture`、`seg2_detect`、`verify_probe`、`s1_depth_eval`；
  - 检测器的纯算法部分可以留在 `depot_vln`/core，ROS 包只做节点和参数；
- `warehouse_vln_agent`：LangGraph/NLU/compile 作为纯 Python 包；ROS 侧只留一个
  action/service 适配器（如果 agent 需要和机器人交互）；
- `warehouse_vln_diagnostics`：preflight、stack_guard、cmd_audit、forward_guard、gpu_watchdog；
- 需要自定义消息/动作时再加 `warehouse_vln_interfaces`；
- 验收：agent 可离线跑（不装 ROS）；ROS 节点只依赖接口包。

### Phase 4：上真机前的工业项

- lifecycle 节点、composition、namespace、QoS 显式化；
- diagnostics/health、watchdog、急停/安全层、参数校验；
- Docker 镜像、systemd/launch 服务、版本 pin、`vcs`/repos 文件；
- `colcon test` + `launch_testing` + `ament_lint` 进 CI；
- 真机驱动包（底盘、相机、雷达）单独 package，与 sim 解耦；
- 发布流程：`package.xml` 版本、tag、changelog、回滚策略。

## 6. 最小 package 示例

### 6.1 `warehouse_vln_bringup/package.xml`

```xml
<?xml version="1.0"?>
<package format="3">
  <name>warehouse_vln_bringup</name>
  <version>0.1.0</version>
  <description>Launch and parameter composition for the warehouse VLN demo.</description>
  <maintainer email="you@example.com">your name</maintainer>
  <license>MIT</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>warehouse_vln_sim</exec_depend>
  <exec_depend>warehouse_vln_localization</exec_depend>
  <exec_depend>warehouse_vln_navigation</exec_depend>

  <test_depend>launch_testing_ament_cmake</test_depend>
  <test_depend>ament_lint_auto</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### 6.2 `setup.py` 里的入口点

```python
entry_points={
    "console_scripts": [
        "static_map_pub = warehouse_vln_localization.static_map_pub:main",
        "truth_map_odom = warehouse_vln_localization.truth_map_odom:main",
        "nav_to = warehouse_vln_navigation.nav_to:main",
    ],
},
data_files=[
    ("share/warehouse_vln_bringup/launch", glob("launch/*.launch.py")),
    ("share/warehouse_vln_bringup/config", glob("config/*.yaml")),
],
```

### 6.3 launch 里不要再算源码路径

```python
from ament_index_python.packages import get_package_share_directory
import os

ROOT = get_package_share_directory("warehouse_vln_bringup")
config = os.path.join(ROOT, "config", "nav.yaml")
```

## 7. 哪些东西不要搬进 ROS package

- **评测/基准**（`eval/`、`sim.py`、`truth.py`、`scoring.py`）：留在 `warehouse_vln_bench`，
  普通 Python；不要 import `rclpy`；
- **世界生成**（`build_world.py`、纹理、编号牌、静态图）：构建期工具；不需要 ROS 运行时；
- **真值**（`ground_truth/`）：只给 worldgen/bench/scoring；runtime 不得 import；
- **文档、示例、媒体、Makefile**：留在 monorepo 根；
- **`depot_vln` 核心库**：继续作为 pip 包 / 纯 Python 包；不要为了“ROS 化”给它加 `rclpy` 依赖。

## 8. 不要做的事

1. **不要改成 catkin / ROS 1**：你已经是 ROS 2 Humble，catkin 是倒退。
2. **不要把所有东西塞进一个 `src/` 大包**：包边界是部署/复用边界，不是目录装饰。
3. **不要让 core 依赖 ROS**：核心算法要能离线测。
4. **不要让 runtime 读 `ground_truth`**：这是本仓库最重要的边界，迁移时不能破。
5. **不要保留 `sys.path.insert(ROOT)` / 相对 cwd / `.venv` 假设**：安装后的节点必须能
   在任意 cwd 运行。
6. **不要把生成物当源码**：`world/warehouse.sdf`、`maps/*.pgm`、`robot/depot_bot.sdf`
   应由构建/CI 生成，或明确标记为 generated。
7. **不要跳过 launch_testing**：只跑 pytest 的 ROS 包，在工业里不能算“可交付”。

## 9. 迁移验收清单

- [ ] `colcon build` 在干净环境通过；
- [ ] `source install/setup.bash` 后，从任意目录 `ros2 launch warehouse_vln_bringup ...` 可跑；
- [ ] 没有任何 `sys.path.insert(ROOT)`、相对 `config/`、`.venv` 路径依赖；
- [ ] 每个 launch 有 `launch_testing` 冒烟测试；
- [ ] `depot_vln`/bench 仍可离线 `pytest` + benchmark，不装 ROS；
- [ ] runtime 的 import 图仍然不包含 `ground_truth`/`truth`/`sim`；
- [ ] 参数在 `share/<pkg>/config`，有默认值与校验；
- [ ] 关键节点有 lifecycle/诊断/看门狗；
- [ ] Docker/CI 可复现；`package.xml` 版本与 tag 一致。

## 10. 参考

- ROS 2 Humble：创建 package
  <https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html>
- ROS 2 launch
  <https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html>
- launch_testing
  <https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Testing-Main.html>
- Nav2（工业级导航栈的包结构参考）
  <https://github.com/ros-navigation/navigation2>
- Autoware（大规模 ROS 2 系统的包划分参考）
  <https://github.com/autowarefoundation/autoware>
- ROS-Industrial
  <https://rosindustrial.org/>

---

**一句话**：当前仓库作为“仿真 + 语义 + LangGraph 的参考实现”是完整的；
上真机前，把 ROS 2 的**构建/部署契约**补上——也就是 colcon workspace + 按子系统拆包，
而不是把目录改成 ROS 1 教程的样子。
