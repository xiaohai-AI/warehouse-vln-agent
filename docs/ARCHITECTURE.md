# 架构

## 一句话

**世界事实只有一份，噪声（感知/语言）各有出处，决策只有一条路径。**

## 1. 数据流

```
                       ┌──────────────────────── 配置 ────────────────────────┐
                       │ world.yaml（几何/物理）  semantics.yaml（语言）        │
                       │ nav.yaml（速度/偏移/预算）                            │
                       └───────────────────────┬──────────────────────────────┘
                                               ▼
                                    depot_vln/catalog.py
                               （唯一的世界事实：谁叫什么、有多少个、从哪儿出发）
                                               │
        ┌──────────────────────────────────────┼──────────────────────────────────────┐
        ▼                                      ▼                                      ▼
  layout.py → world/slot_grid.json      agent/nlu/                          agent/nodes.py
  （几何、slot 位姿、观测位）            （把一句话变成槽位）                  （图节点，调工具层）
        │                                      │                                      │
        ▼                                      ▼                                      ▼
  semantic_graph.py                     compile_query()                        tools.py
  （拓扑图 + 路障 + Dijkstra）           （槽位 → query.kind）                  （唯一动作出口）
        │                                      │                                      │
        ▼                                      ▼                                      ▼
  world.py  ←──  inventory.py  ←──  results/inventory.json  ←──  感知（SEG-2 / P6 verify）
  （运行时世界：resolve/plan/goto/observe，**不读真值**）
```

评测侧是**另一条边**，由图之外的文件系统连接：

```
  ground_truth/slots.json ──→ eval/oracle.py   （复核基准声明是否成立）
                           └→ scoring.py       （打分：观测 vs 真值）
```

运行时永远看不到这条边 —— `tests/test_runtime_isolation.py` 用 AST 检查 import 图。

## 2. 语言层：为什么不是"让模型直接给答案"

```
   一句话
     │
     ├─→ rule.extract()   ─┐   只回答"原话里出现了什么"（巷道/侧/层/类别/关系）
     │     catalog 驱动    ├─→ Slots ──→ compile_query() ──→ query.kind
     └─→ llm.understand() ─┘         （不够就**故意**欠定）
              │
              └─ 每个值必须在原话里找得到依据（grounding），否则丢弃并记账
```

三条性质，缺一条"静默猜"就会回来：

1. **模型没有字段可以放 `kind`**：`Slots` 里只有事实。模型想越权也没有位置。
2. **落地校验**：`aisle=C` 在原话里找不到 `C` 就丢弃（实测裸模型的幻觉就是这么进来的）。
3. **欠定照样产出 query**：`{"kind":"bay","bay":10}` 没有巷道/侧/层 —— `resolve` 必然得到
   24 个候选 → `AMBIGUOUS` → `clarify`。所以"歧义必须发问"不依赖 if-else 的完备性。

## 3. 图（LangGraph）

```
START → understand → resolve ─┬─(ok)────────→ plan → act → verify ─┬─(ok)→ respond → END
                              │                                    │
                              ├─(ambiguous)→ clarify ⇄ interrupt    └─(fail)→ recover
                              │                  │                          │
                              ├─(blocked)───────→ recover ────────────────→ plan
                              └─(no_slot …)────→ respond(原因码) → END
```

约束（都有测试）：

* `act` 是唯一调用 `tools.navigate` 的节点；
* `clarify` 是唯一使用 `interrupt()` 的地方，且只等一句话；
* 环有上限：`RECURSION_LIMIT=30`、`MAX_RECOVER=2`、`MAX_CLARIFY=2`，超限带原因码返回。

## 4. 语义层：三级可信度

| 状态 | 含义 | 表现 |
|---|---|---|
| `occupied=True/False` | 看过，并且判出来了 | 可以下结论 |
| `occupied=None` | 看过，但判不出 | 说"现场未能判定"，不猜 |
| 不在库存里 | **没看过** | `UNOBSERVED`，不发散成"空" |

这条在代码里就是一个 `None`：`world.resolve()` 遇到没观测的格子在跟"空位/没有这种货"
有关的查询上返回 `UNOBSERVED`；跟"能不能开过去"有关的查询不受影响（导航不需要知道货位里有没有货）。

**部分观测的货架面**也不能被当成"没有"：`class_in_aisle` 会把还有未知格子的面一起算成候选
（score=0，why="还有 N 个格子未观测"），图上就成了歧义 → 问人。
`max_class`（"最多"）要求**整个搜索范围都观测过**，否则直接 `UNOBSERVED` ——
"最多"是对全体的断言，没见过全体就不该下。

## 5. 世界事实的边界

`catalog.py` 负责：巷道名/数量、bay 数/间距、层数与层高、货类与别名、左右侧词、
关系词、单位词、出生点、路障区间（从 bay 区间推坐标）。

它**不**负责：几何细节（`layout.py`）、占用与类别（感知）、路径（`semantic_graph.py`）。

`layout.compute_layout()` 的返回值就是"几何唯一来源"：`build_world.py` 用它发射 SDF，
`verify_world.py` 用它做自检，`catalog.spawn` 也来自它 —— 所以"世界改了、出生点没改"
这种错误在结构上不可能发生。

## 6. 测试即约束

| 测试 | 守的是什么 |
|---|---|
| `test_catalog.py` | 世界事实来自配置；换世界不改代码；出生点派生 |
| `test_no_hardcoded_world.py` | `depot_vln/` 里不许再出现 `-31.0` / `"ABCDEF"` / `bays=24` / `LANE_DY={` |
| `test_runtime_isolation.py` | 运行时模块不许 import 真值 |
| `test_nlu.py` | 同一意图的不同说法；欠定查询保持欠定；LLM 回退要记账 |
| `test_agent_graph.py` | 歧义必问、封堵必拒、不存在必拒、未知不编 |
| `test_tools.py` | seq 单调、过期丢弃、取消不回报完成 |
| `test_bench.py` | 基准声明与真值自洽；静默猜错 = 0 |
| `test_inventory.py` | 替身必须标注；覆盖率下降时未知保持未知 |

## 7. 为什么删掉了 `accept.py` 那一套

原来有 16 条"判据"由一个 33KB 的脚本读 `results/*.json` 与日志，重新拼出
`docs/ACCEPTANCE.md`。它的问题是**不会因为代码坏了而变红**：只要历史产物还在，
表就还是 PASS。真正能守住行为的只有断言 —— 判据搬进 `tests/`，数字留在 `results/`。
