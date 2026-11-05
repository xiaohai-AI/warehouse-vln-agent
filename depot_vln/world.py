"""世界模型（WorldModel）：语义导航的"环境"。

职责（与原 `depot_vln/mock/world.py` 相同，但数据来源换了）
------------------------------------------------------------
  1. 语义库存的数据视图    ← **Inventory 快照**（感知产物或显式替身），不再读 ground_truth
  2. 拓扑图（走廊连通性 + 路障）  ← world/slot_grid.json + blocked 区间
  3. 工具层的执行体（规划/导航/观测的耗时与结果），**不含任何 /cmd_vel**

与旧实现的三个实质差别
----------------------
  * 没有 `truth` 属性。本文件不 import `depot_vln.truth`，因此"运行时读真值"在
    import 图层面就不可能（有测试检查这一点）。
  * "没看过"是一等状态：`resolve()` 在信息不足时返回 `UNOBSERVED`，而不是编一个答案。
  * 巷道名 / bay 数 / 层数 / 左右侧统统一律来自 `depot_vln/catalog.py`。
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Iterable

from depot_vln.catalog import Address, Catalog, get_catalog
from depot_vln.inventory import Inventory
from depot_vln.semantic_graph import SemanticGraph
from depot_vln.semantics import SemanticMap

ROOT = Path(__file__).resolve().parents[1]

NOMINAL_SPEED = 0.4      # m/s（导航常用速度）
DOCK_TIME = 2.0          # s（贴靠/观测的准备时间）

# 查询种类 → 必需的槽位（缺一即 UNKNOWN_INTENT，而不是猜）
_KINDS: dict[str, tuple[str, ...]] = {
    "address": ("address",),
    "bay": ("bay",),
    "aisle": ("aisle",),
    "class_in_aisle": ("aisle",),
    "class_in_bay": ("aisle", "bay"),
    "max_class": (),
    "farthest": ("aisle",),
    "empty_slot_in_bay": ("aisle", "bay"),
}


class WorldModel:
    def __init__(
        self,
        slot_grid: dict[str, Any],
        inventory: Inventory,
        *,
        blocked: Iterable[dict[str, Any]] | None = None,
        start: tuple[float, float] | None = None,
        catalog: Catalog | None = None,
        now: float | None = None,
    ):
        self.grid = slot_grid
        self.catalog = catalog or get_catalog()
        self.inventory = inventory
        self.blocked = list(blocked) if blocked is not None else self.catalog.blocked_segments()
        self.map = SemanticMap.from_inventory(slot_grid, inventory)
        if now is not None:
            self.map.now = float(now)
        self.graph = SemanticGraph(slot_grid, self.blocked)
        spawn = self.catalog.spawn
        self.pose = {
            "x": float(start[0]) if start else float(spawn["x"]),
            "y": float(start[1]) if start else float(spawn["y"]),
            "yaw": float(spawn.get("yaw", 0.0)),
        }
        self.t = float(inventory.provenance.get("now") or 0.0)
        self.seq = 0

    # ------------------------------------------------------------ 观测写回
    def observe(self, address: str, occupied: bool | None, cls: str | None,
                confidence: float, source: str = "vision") -> None:
        """现场观测刷新地图（这就是"再次观测覆盖旧值 + 标 stale"的那一步）。"""
        if occupied is None:
            return
        self.map.observe(address, bool(occupied), cls, float(confidence), self.t, source=source)

    # ------------------------------------------------------------ 解析
    def _sanitize(self, query: dict[str, Any]) -> tuple[dict[str, Any], str | None]:
        """**边界校验**：LLM 的输出不可信，坏值一律转成拒绝码，绝不抛异常。"""
        q = dict(query or {})
        kind = q.get("kind")
        required = _KINDS.get(kind) if isinstance(kind, str) else None
        if required is None:
            return {}, "UNKNOWN_INTENT"
        if any(k not in q for k in required):
            return {}, "UNKNOWN_INTENT"

        cat = self.catalog
        if "address" in q:
            a = q["address"]
            a = a.strip().upper() if isinstance(a, str) else ""
            # 允许"缺层地址"（B-L-07）：它会在 resolve 里展开成多候选 -> 歧义 -> 发问。
            # 只要"在货位表里"或"能被 catalog 解析"就放行，其余才是 NO_SLOT。
            if a not in self.grid["slots"] and cat.parse_address(a) is None:
                return {}, "NO_SLOT"
            q["address"] = a
        if "aisle" in q:
            a = q["aisle"]
            a = a.strip().upper() if isinstance(a, str) else ""
            if not cat.is_aisle(a):
                return {}, "NO_SLOT"
            q["aisle"] = a
        if "side" in q:
            s = q["side"]
            if s is None:
                q.pop("side")
            else:
                s = cat.side_from_word(str(s))
                if s is None:
                    return {}, "NO_SLOT"
                q["side"] = s
        for key, ok, bad in (
            ("bay", cat.valid_bay, "NO_SLOT"),
            ("level", lambda v: v in cat.level_numbers(), "NO_SLOT"),
        ):
            if key in q:
                v = q[key]
                # None = "没指定"，不是非法值
                if v is None:
                    q.pop(key)
                    continue
                n = v if isinstance(v, int) and not isinstance(v, bool) else None
                if n is None and isinstance(v, str) and v.strip().isdigit():
                    n = int(v.strip())
                if n is None or not ok(n):
                    return {}, bad
                q[key] = n
        return q, None

    def _slot_candidate(self, addr: str, score: float, why: str) -> dict[str, Any]:
        slot = self.grid["slots"][addr]
        node = self.graph.node_for_slot(addr)
        reachable = self.graph.reachable((self.pose["x"], self.pose["y"]), node)
        st = self.map.get(addr)
        return {
            "target": {
                "kind": "slot",
                "address": addr,
                "aisle": slot["aisle"],
                "side": slot["side"],
                "bay": slot["bay"],
                "level": slot["level"],
                "obs_pose": slot["obs_pose"],
                "pose": slot["pose"],
            },
            "score": score,
            "why": why,
            "reachable": reachable,
            "observed_occupied": None if st is None else st.occupied,
            "observed_class": None if st is None else st.cls,
        }

    def _faces(self) -> list[tuple[str, str]]:
        return [(a, s) for a in self.catalog.aisles for s in self.catalog.sides]

    def _unknown(self, addr: str) -> bool:
        """没看过（无状态）或看过但判不出占用 —— 都不能当"空"或"没有这种货"。"""
        st = self.map.get(addr)
        return st is None or st.occupied is None

    # ------------------------------------------------------------ 查询
    def resolve(self, query: dict[str, Any]) -> tuple[list[dict[str, Any]], str | None]:
        """返回 (候选列表, 拒绝原因码)。候选里带 score / why / reachable。"""
        query, bad = self._sanitize(query)
        if bad:
            return [], bad
        kind = query.get("kind")
        slots = self.grid["slots"]
        cat = self.catalog

        if kind == "address":
            addr = query["address"]
            if addr in slots:
                return [self._slot_candidate(addr, score=1.0, why="地址直达")], None
            # 地址缺层（"B-L-07"）：两层都存在时这就是**真歧义**，必须发问而不是默认第 1 层
            a = cat.parse_address(addr)
            if a is None:
                return [], "NO_SLOT"
            levels = (a.level,) if a.level else cat.level_numbers()
            cands = []
            for lv in levels:
                full = cat.format_address(a.aisle, a.side, a.bay, lv)
                if full in slots:
                    cands.append(self._slot_candidate(full, score=1.0, why="地址直达（未指明层）"))
            return (cands, None) if cands else ([], "NO_SLOT")

        if kind == "bay":
            bay = int(query["bay"])
            if not cat.valid_bay(bay):
                return [], "NO_SLOT"
            levels = (query["level"],) if query.get("level") else cat.level_numbers()
            sides = (query["side"],) if query.get("side") else cat.sides
            aisles = (query["aisle"],) if query.get("aisle") else cat.aisles
            cands = []
            for aisle in aisles:
                for side in sides:
                    for lv in levels:
                        addr = cat.format_address(aisle, side, bay, lv)
                        if addr in slots:
                            cands.append(self._slot_candidate(
                                addr, score=0.5,
                                why=f"{bay:0{cat.pad}d} 号位（多条巷道/多侧/多层同名）"))
            return (cands, None) if cands else ([], "NO_SLOT")

        if kind == "aisle":
            aisle = query["aisle"]
            return [{
                "target": {"kind": "aisle", "aisle": aisle},
                "score": 1.0,
                "why": f"{aisle} 巷",
                "reachable": True,
            }], None

        if kind == "class_in_aisle":
            aisle = query["aisle"]
            classes = set(query.get("classes") or cat.class_names)
            known_classes = set(cat.class_names)
            if classes and not (classes & known_classes):
                return [], "NO_SLOT"
            sides = (query["side"],) if query.get("side") else cat.sides
            cands = []
            unseen_faces = 0
            for side in sides:
                counts = self.map.rack_face_counts(aisle, side)
                n = sum(v for k, v in counts.items() if k in classes)
                # 这一面还有没看过的格子吗？有就不能断言"没有这种货"
                face_slots = [a for a in self.grid["slots"] if a.startswith(f"{aisle}-{side}-")]
                unknown = sum(1 for a in face_slots if self._unknown(a))
                if n <= 0 and not unknown:
                    continue                     # 整面都看过、确实没有
                if unknown:
                    unseen_faces += 1
                why = f"{aisle}-{side} 面有 {n} 件 {','.join(sorted(classes))}"
                if unknown:
                    why += f"（另有 {unknown} 个格子未观测）"
                cands.append({
                    "target": {"kind": "rack_face", "aisle": aisle, "side": side},
                    "score": float(n),
                    "why": why,
                    "reachable": True,
                })
            cands.sort(key=lambda c: (-c["score"], c["target"]["side"]))
            if cands:
                return cands, None
            return [], ("UNOBSERVED" if unseen_faces else "EMPTY_BAY")

        if kind == "class_in_bay":
            aisle, bay = query["aisle"], int(query["bay"])
            classes = set(query.get("classes") or cat.class_names)
            levels = (query["level"],) if query.get("level") else cat.level_numbers()
            sides = (query["side"],) if query.get("side") else cat.sides
            found: list[str] = []
            any_occ = any_known = any_unknown = False
            for side in sides:
                for lv in levels:
                    addr = cat.format_address(aisle, side, bay, lv)
                    st = self.map.get(addr)
                    if st is None or st.occupied is None:
                        any_unknown = True
                        continue
                    any_known = True
                    if st.occupied:
                        any_occ = True
                        if st.cls in classes:
                            found.append(addr)
            if found:
                return [self._slot_candidate(a, 1.0, "bay 内指定类别") for a in found], None
            if not any_known:
                return [], "UNOBSERVED"
            if any_unknown:
                # 有格子没看过：既不能说"没有这种货"，也不能说"这格是空的"
                return [], "UNOBSERVED"
            return [], ("NO_SLOT" if any_occ else "EMPTY_BAY")

        if kind == "max_class":
            classes = set(query.get("classes") or cat.class_names)
            ranking = self.map.rank_rack_faces_by_class(classes)
            if query.get("aisle"):
                ranking = [r for r in ranking if r["aisle"] == query["aisle"]]
            # "最多"是对**整个搜索范围**的全序遍历才能下的结论：
            # 只要范围内还有没看过的货位，答案就可能是错的 —— 明说不知道，不猜。
            scope_aisles = (query["aisle"],) if query.get("aisle") else cat.aisles
            scope_slots = [a for a in self.grid["slots"] if a.split("-")[0] in scope_aisles]
            unknown = [a for a in scope_slots if self._unknown(a)]
            if unknown:
                return [], "UNOBSERVED"
            if not ranking or ranking[0]["count"] == 0:
                return [], ("EMPTY_BAY" if self.map.known_slots() else "UNOBSERVED")
            top = [r for r in ranking if r["count"] == ranking[0]["count"]]
            return [{
                "target": {"kind": "rack_face", "aisle": r["aisle"], "side": r["side"]},
                "score": float(r["count"]),
                "why": f"{r['rack_face']} 有 {r['count']} 件",
                "reachable": True,
            } for r in top], None

        if kind == "farthest":
            aisle = query["aisle"]
            want_empty = bool(query.get("empty"))
            levels = (query["level"],) if query.get("level") else cat.level_numbers()
            sides = (query["side"],) if query.get("side") else cat.sides
            best: list[tuple[float, str]] = []
            seen_any = False
            unknown_d: list[float] = []
            for node in self.graph.stations(aisle):
                bay = int(node.split("@")[1])
                d = self.graph.path_length((self.pose["x"], self.pose["y"]), node)
                if not math.isfinite(d):
                    continue  # 路障后面：不是"最里面"，是"到不了"
                for side in sides:
                    for lv in levels:
                        addr = cat.format_address(aisle, side, bay, lv)
                        st = self.map.get(addr)
                        if st is None or st.occupied is None:
                            unknown_d.append(d)
                            continue
                        seen_any = True
                        if want_empty and st.occupied:
                            continue
                        best.append((d, addr))
            if not best:
                return [], ("UNOBSERVED" if not seen_any else "EMPTY_BAY")
            best.sort(key=lambda b: -b[0])
            # **更远处还有没看过的格子** -> "最里面的空位"这个结论不成立，老实说不知道
            if unknown_d and max(unknown_d) >= best[0][0] - 1e-9:
                return [], "UNOBSERVED"
            far = [b for b in best if abs(b[0] - best[0][0]) < 0.5]
            return [self._slot_candidate(addr, score=1.0, why=f"{aisle} 巷最里面（{d:.1f} m）")
                    for d, addr in far], None

        if kind == "empty_slot_in_bay":
            aisle, bay = query["aisle"], int(query["bay"])
            levels = (query["level"],) if query.get("level") else cat.level_numbers()
            sides = (query["side"],) if query.get("side") else cat.sides
            empties: list[str] = []
            any_known = False
            for side in sides:
                for lv in levels:
                    addr = cat.format_address(aisle, side, bay, lv)
                    st = self.map.get(addr)
                    if st is None or st.occupied is None:
                        continue          # 没看过 / 判不出：不能当空位
                    any_known = True
                    if st.occupied is False:
                        empties.append(addr)
            if not any_known:
                return [], "UNOBSERVED"
            if not empties:
                return [], "EMPTY_BAY"   # 该 bay 全是有货 —— 没有"空位"这回事
            return [self._slot_candidate(a, 1.0, "bay 内空位") for a in empties], None

        return [], "NO_SLOT"

    # ------------------------------------------------------------ 语义级恢复
    def substitute(
        self,
        target: dict[str, Any],
        *,
        origin_xy: tuple[float, float] | None = None,
        exclude: tuple[str, ...] | list[str] = (),
    ) -> dict[str, Any] | None:
        """原目标不可达时，找**同语义**的替代货位：同巷/同面/同层、bay 最近且可达。

        只用地图几何 + 语义图，不读 truth。找不到返回 None —— 那就老实报 BLOCKED。
        """
        if target.get("kind") != "slot":
            return None
        cat = self.catalog
        aisle, side = target.get("aisle"), target.get("side")
        bay, level = int(target.get("bay") or 0), int(target.get("level") or 0)
        if not (cat.is_aisle(aisle) and cat.is_side(side) and bay and level in cat.level_numbers()):
            return None
        origin = origin_xy or (self.pose["x"], self.pose["y"])
        ex = set(exclude)
        n_bays = cat.bays_per_aisle

        best: list[tuple[int, float, str]] = []
        for d in range(1, n_bays):
            for b in (bay + d, bay - d):
                if not 1 <= b <= n_bays:
                    continue
                addr = cat.format_address(aisle, side, b, level)
                if addr in ex or addr not in self.grid["slots"]:
                    continue
                length = self.graph.path_length(origin, self.graph.node_for_slot(addr))
                if not math.isfinite(length):
                    continue
                best.append((d, length, addr))
            if best:            # 同巷有可达的就够了，不再往远处找
                break

        if not best:            # 同巷全不可达（整条巷封了）：换其它巷的同一 bay/层/面
            for a in cat.aisles:
                if a == aisle:
                    continue
                addr = cat.format_address(a, side, bay, level)
                if addr in ex or addr not in self.grid["slots"]:
                    continue
                length = self.graph.path_length(origin, self.graph.node_for_slot(addr))
                if math.isfinite(length):
                    best.append((n_bays, length, addr))

        if not best:
            return None
        best.sort(key=lambda t: (t[0], t[1]))
        d, length, addr = best[0]
        return self._slot_candidate(
            addr, score=1.0,
            why=f"原目标不可达；同 {aisle} 巷同面同层最近的 bay（Δbay={d}，{length:.1f} m）")

    # ------------------------------------------------------------ 规划
    def plan(self, target: dict[str, Any]) -> dict[str, Any] | None:
        """图上算路 + 算观测位 + 决定进入动作（能否侧移贴靠）。"""
        kind = target.get("kind")
        cat = self.catalog
        if kind == "slot":
            node = self.graph.node_for_slot(target["address"])
            obs = target["obs_pose"]
            entry = self._entry_action(target)
        elif kind == "rack_face":
            aisle, side = target["aisle"], target["side"]
            mid = max(1, cat.bays_per_aisle // 2)
            sample = cat.format_address(aisle, side, mid, cat.level_numbers()[0])
            slot = self.grid["slots"][sample]
            node = self.graph.node_for_slot(sample)
            obs = slot["obs_pose"]
            entry = {"type": "front", "lateral": 0.0, "note": "货架面中点观测"}
            target = dict(target, obs_pose=obs, sample_slot=sample)
        elif kind == "aisle":
            aisle = target["aisle"]
            node = f"W:{aisle}"
            a = next(a for a in self.grid["aisles"] if a["name"] == aisle)
            obs = {"x": self.grid["derived"]["rack_run"] / -2, "y": a["center_y"], "yaw": 0.0, "z": 0.0}
            entry = {"type": "front", "lateral": 0.0}
        else:
            return None

        length = self.graph.path_length((self.pose["x"], self.pose["y"]), node)
        if not math.isfinite(length):
            return None
        return {
            "goal": target,
            "node": node,
            "obs_pose": obs,
            "path_length": round(length, 3),
            "eta": round(length / NOMINAL_SPEED + DOCK_TIME, 2),
            "entry": entry,
        }

    def _entry_action(self, target: dict[str, Any]) -> dict[str, Any]:
        """能横移就别掉头：横向位移 ≥ 0.3 m 时用侧移贴靠（这是选麦克纳姆的理由）。"""
        obs = target["obs_pose"]
        slot_xy = (target["pose"]["x"], target["pose"]["y"])
        lateral = abs(slot_xy[1] - obs["y"])
        return {
            "type": "side_dock" if lateral >= 0.30 else "front",
            "lateral": round(lateral, 3),
            "yaw_hold": obs["yaw"],
        }

    # ------------------------------------------------------------ 执行（模拟）
    def goto(self, target: dict[str, Any], plan: dict[str, Any] | None = None,
             seq: int | None = None) -> dict[str, Any]:
        plan = plan or self.plan(target)
        if plan is None:
            return {
                "ok": False,
                "reason_code": "UNREACHABLE",
                "pose": dict(self.pose),
                "detail": "图上没有到该目标的路径",
                "elapsed": 0.0,
            }
        self.seq = seq if seq is not None else self.seq + 1
        elapsed = plan["path_length"] / NOMINAL_SPEED + DOCK_TIME
        self.t += elapsed
        obs = plan["obs_pose"]
        self.pose = {"x": obs["x"], "y": obs["y"], "yaw": obs["yaw"]}
        return {
            "ok": True,
            "reason_code": None,
            "pose": dict(self.pose),
            "detail": f"到达 {plan['node']}，进入动作 {plan['entry']['type']}",
            "elapsed": round(elapsed, 2),
            "seq": self.seq,
        }

    # ------------------------------------------------------------ 观测
    def observe_slot(self, addr: str) -> dict[str, Any]:
        st = self.map.get(addr)
        if st is None:
            # "没有先验记录"不等于"去了也白去"：真机到现场是**新观测**，只是离线替身
            # 没有真图像可看，所以诚实地回"占用未判定"，而不是把整条指令判失败。
            # 踩过：这里返回 ok=False 会让"去没看过的货位"在 verify 阶段直接判 UNOBSERVED，
            # 于是覆盖率<100% 时所有直达指令都失败 —— 那不是判据严格，是把实现缺口当结论。
            return {"ok": True, "reason_code": None, "occupied": None, "class": None,
                    "confidence": 0.0, "pose": dict(self.pose),
                    "detail": f"{addr}: 现场未能判定占用（离线替身无图像）", "elapsed": 0.8}
        self.t += 0.8  # 拍帧 + 推理的固定开销
        return {
            "ok": True,
            "reason_code": None,
            "occupied": st.occupied,
            "class": st.cls,
            "confidence": round(st.effective_confidence(self.t), 3),
            "pose": dict(self.pose),
            "detail": f"{addr}: occupied={st.occupied} class={st.cls}",
            "elapsed": 0.8,
        }


# ---------------------------------------------------------------- 装载
def load_grid(path: str | Path | None = None) -> dict[str, Any]:
    p = Path(path) if path else ROOT / "world" / "slot_grid.json"
    return json.loads(p.read_text(encoding="utf-8"))


def load_world(
    inventory: Inventory,
    *,
    blocked: Iterable[dict[str, Any]] | None = None,
    start: tuple[float, float] | None = None,
    grid_path: str | Path | None = None,
    catalog: Catalog | None = None,
) -> WorldModel:
    """用一份给定的库存快照装载世界。

    **必须显式给 Inventory**：这个模块不认识"真值"，也不认识"替身"。
    感知产物走 `depot_vln/inventory.py:Inventory.from_perception`，
    离线替身走 `depot_vln/sim.py`。来源永远由调用方写明。
    """
    grid = load_grid(grid_path)
    return WorldModel(grid, inventory, blocked=blocked, start=start,
                      catalog=catalog or get_catalog())
