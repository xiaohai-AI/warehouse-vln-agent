# depot-vln

亚马逊式自动化仓库里的**麦克纳姆全向车 + 组合 SLAM + 从感知长出来的语义地图 + LangGraph 指代导航**。
底层 ROS 2 Humble / Nav2 / Gazebo Fortress，顶层 LangGraph。

```
离线（不需要 ROS/GPU）：world → map-check → robot → inventory → bench → test
仿真（需要 ROS + Gazebo）：sim → nav2 → nav → 判据
```

## 30 秒上手（离线，秒级）

```bash
python3.10 -m venv --system-site-packages .venv
.venv/bin/pip install -r requirements.txt

make bench      # 指令基准 54 条：direct/attribute/ambiguous/unreachable 各一类
make test       # 45 条单元测试
make ci         # test + bench + 安全层自检
```

`make bench` 的当前结果（rule 后端、全量库存）：

| 类别 | n | 通过 | 静默猜错 |
|---|---|---|---|
| direct | 14 | 14 | 0 |
| attribute | 14 | 14 | 0 |
| ambiguous | 14 | 14 | 0 |
| unreachable | 12 | 12 | 0 |

用**真实感知库存**（`make inventory` 从 SEG-2 的 46 个观测生成，覆盖率 8%）跑同一份基准：
36/54 通过（direct 14/14、ambiguous 11/14、unreachable 10/12、attribute 1/14）、**静默猜错 0**。差距不是"退步"，而是语义层这时只能诚实地说"我没看过" ——
见 [docs/LIMITATIONS.md](docs/LIMITATIONS.md)。

## 架构（三层，边界是类型不是自觉）

```
                config/world.yaml ──┐
                config/semantics.yaml ├─→ depot_vln/catalog.py   ← 世界事实的唯一入口
                config/nav.yaml ──────┘        │
                                               │  巷道名 / bay 数 / 层数 / 货类别名 / 出生点 / 车道偏移
        ┌──────────────────────────────────────┴───────────────────────────────┐
        │                                                                      │
   depot_vln/layout.py                                                  depot_vln/agent/
   几何唯一来源（生成器与自检共用）                                       nlu/    槽位抽取（rule | llm）
        │                                                                 compile.py 槽位 → query
   world/slot_grid.json                                                    │
        │                                                                 nodes.py   图节点
   depot_vln/semantic_graph.py  ← 拓扑图（Dijkstra）                        │
        │                                                                 graph.py   拓扑即约束
   depot_vln/inventory.py  ← 库存快照（**感知产物**，带 provenance）          │
        │                                                                 tools.py   工具层（唯一动作出口）
   depot_vln/semantics.py  ← 时间衰减/冲突合并                                │
        │                                                                 ros_tools.py 真仿真工具层
   depot_vln/world.py  ← 运行时的世界（**不认识真值**）  ────────────────────┘
```

三条硬边界（都有测试守着）：

1. **运行时不许读真值**：`depot_vln/world.py`、`semantic_*`、`agent/**` 里没有任何
   `ground_truth` 依赖，`tests/test_runtime_isolation.py` 用 AST 检查 import 图。
   真值只出现在 `sim.py`（离线替身）、`scoring.py`（打分）、`eval/`（评测）。
2. **图里只有 `act` 节点碰工具层**，工具层是唯一动作出口 —— 这是拓扑性质，不是纪律。
3. **"没看过"是一等状态**：库存没覆盖的货位不是"空"，是未知。`resolve()` 返回
   `UNOBSERVED`，不编答案；静默猜错在基准里是零容忍项。

### 改东西只需要改一处

| 想改 | 改哪里 |
|---|---|
| 巷道数 / 名字 / bay 数 / 层数 / 几何 | `config/world.yaml`，然后 `make world` |
| 货类、以及用户怎么称呼它 | `config/world.yaml`（尺寸颜色）+ `config/semantics.yaml`（别名） |
| 出生点 | `config/world.yaml:spawn`（`auto` = 由几何推导） |
| 行驶速度 / 车道偏移 / 停位容差 | `config/nav.yaml` |
| 澄清问法、拒绝措辞 | `depot_vln/agent/nlu/rule.py`（只有这里是人话） |

新增一个货类（比如 `barrel`）只做两步：`world.yaml` 加尺寸颜色、`semantics.yaml` 加叫法。
**不用碰 Python** —— 这条由 `tests/test_catalog.py::test_new_world_adapts_without_code_change` 证明
（测试里现场造了一个 3 巷道 / 10 bay / 3 层 / 新货类的世界）。

## 目录

```
config/world.yaml        物理世界唯一配置源（禁止手改 .sdf）
config/semantics.yaml    语言层词表（货类别名、方位/层/关系词、闭集外物体）
config/nav.yaml          导航参数（速度、停位容差、车道偏移、墙钟预算）
config/nav2_omni.yaml    Nav2 全向参数（MPPI Omni，PreferForward=0）
config/slam_toolbox.yaml 建图参数       config/ekf_omni.yaml / ekf_s5.yaml  里程计融合
depot_vln/
  catalog.py             世界事实唯一入口（解析/格式化/词表/出生点）
  layout.py              几何派生（生成器与自检共用）
  sdf.py                 世界发射器          camera.py / extrinsics.py  感知几何
  semantic_graph.py      拓扑图（Dijkstra）  semantics.py  语义地图（衰减/冲突）
  inventory.py           库存快照 + 出处    world.py      运行时世界（不读真值）
  sim.py                 离线替身装载器     scoring.py    真值打分（只给评测）
  truth.py               世界生成器的真值导出（构建期）
  agent/
    graph.py             图装配（拓扑即约束）  nodes.py  节点实现
    state.py             显式状态            tools.py  工具层协议 + 离线实现
    ros_tools.py         真仿真工具层（Nav2 / 深度 / VLM）
    nlu/                 槽位抽取（rule / llm）+ 落地校验 + 编译器
  eval/
    cases.yaml           人工编写的基准（与解析器无关）
    oracle.py            用真值复核每条声明，矛盾就拒绝运行
    runner.py            跑图 + 分类统计 + 静默猜错统计
scripts/                 构建、仿真采集、判据（见 `make help`）
tests/                   单元测试 + 反硬编码守护 + 真值隔离守护
docs/ARCHITECTURE.md     设计细节与数据流
docs/LIMITATIONS.md      唯一权威的降级/未测清单
docs/archive/            重构前的过程文档（DECISIONS/TROUBLESHOOTING/REPORT…，仅供追溯）
```

## 这次重构做了什么

**删**：一次性探针脚本 18 个、重复配置 2 份、过期地图 4 张、`results/` 里 120 MB 中间产物、
9 份约 38 万字的过程文档（移到 `docs/archive/`，未删除）、以及"判据对账账本"脚本
（`accept.py` / `docs_consistency.py` / `delivery_check.sh` —— 它们读日志对账，代码坏了也不会红）。

**修**：

| 问题 | 之前 | 现在 |
|---|---|---|
| 世界事实 | 巷道表/24 bay/2 层/货类写在 6+ 个文件里 | `catalog.py` 一处，来自配置；有回归测试 |
| 出生点 | `-31.0` 抄在 10+ 个脚本里 | `catalog.spawn` 由几何派生 |
| 语义地图数据源 | `mock/world.py` 遍历 `ground_truth/slots.json` 抄进地图 | `inventory.py`：感知产物快照，带 provenance；替身明确标 `sim-sampled` |
| 语言层 | 300 行正则分支，与基准模板同构 | 槽位抽取（数据驱动）→ 编译器决定"要不要发问"，图负责歧义 |
| 基准 | 模板生成、与正则一一对应 → 100% 是恒等式 | 人工编写、真值独立复核；静默猜错单独统计 |
| 判据 | 16 条"读日志对账" | 45 条 pytest 断言 |

## 已知边界

见 [docs/LIMITATIONS.md](docs/LIMITATIONS.md)。最重要的三条：

* 感知库存只有 **8%** 覆盖率（SEG-2 只采了一条巷的下层）—— 属性类问题因此只能回答"没看过"；
* 规则解析器不支持"第三个货架""蓝色的大箱子"这类**相对/外观**表达（LLM 后端可补，但没做端到端验证）；
* 本次重构**没有**重跑 Gazebo 侧判据（M1/M2/SLAM/SEG），只保证离线链路与代码一致性。

## 许可

MIT，见 [LICENSE](LICENSE)。世界、纹理、真值均由 `make world` 程序化生成。
