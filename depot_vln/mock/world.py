"""Mock 世界：让 agent / 基准在没有 Gazebo 的情况下跑起来（SPEC §8.6 最后一条）。

它扮演三件事：
  1. 语义地图的数据源（模拟一次巡检：对每个 slot 产出 {occupied, class, confidence}）
  2. 拓扑图（走廊连通性 + 路障）
  3. 工具层的执行体（导航/观测的耗时与结果），**不含任何 /cmd_vel**

ground_truth 只在 `--score` 场景里被用来和地图对比；agent 侧只看 SemanticMap。
noise=0 时地图与真值一致 —— 基准此时专门衡量"语言 + 澄清 + 拒绝"这三层，
感知误差另有 P5 的 SEG-2 判据单独衡量，两边不混。
"""
from __future__ import annotations

import json
import math
import random
from pathlib import Path
from typing import Any

from depot_vln.semantic_graph import SemanticGraph
from depot_vln.semantics import SemanticMap

ROOT = Path(__file__).resolve().parents[2]  # depot_vln/mock/world.py -> 项目根

CARTON_CLASSES = {"carton_s", "carton_m", "carton_l"}
NOMINAL_SPEED = 0.4      # m/s（导航常用速度）
DOCK_TIME = 2.0          # s（贴靠/观测的准备时间）


class MockWorld:
    def __init__(
        self,
        slot_grid: dict[str, Any],
        truth: dict[str, Any],
        *,
        noise: float = 0.0,
        seed: int = 0,
        start: tuple[float, float] | None = None,
        blocked: list[dict[str, Any]] | None = None,
    ):
        self.grid = slot_grid
        self.truth = truth
        self.rng = random.Random(seed)
        self.noise = noise
        self.map = SemanticMap(slot_grid)
        self.blocked = blocked if blocked is not None else (truth.get("blocked") or [])
        self.graph = SemanticGraph(slot_grid, self.blocked)
        der = slot_grid["derived"]
        self.pose = {
            "x": start[0] if start else -der["rack_run"] / 2 - der["cross_aisle_width"] / 2,
            "y": start[1] if start else 0.0,
            "yaw": 0.0,
        }
        self.t = 0.0
        self.seq = 0
        self._scan()

    # ------------------------------------------------------------ 模拟巡检
    def _scan(self) -> None:
        for addr, rec in self.truth["slots"].items():
            occ = rec["occupied"]
            cls = rec["class"]
            conf = 0.95 if rec["source"] == "generator" else 0.8
            if self.noise > 0:
                if self.rng.random() < self.noise:
                    occ = not occ
                if occ and cls and self.rng.random() < self.noise:
                    alt = [c for c in self.grid.get("classes", []) or [] if c != cls]
                    cls = self.rng.choice(alt) if alt else cls
                conf = max(0.3, conf - self.rng.uniform(0, 0.3))
            self.map.observe(addr, occ, cls if occ else None, conf, self.t, source="vision")

    # ------------------------------------------------------------ 解析
    _KINDS = {
        "address": ("address",),
        "bay": ("bay",),
        "aisle": ("aisle",),
        "class_in_aisle": ("aisle",),
        "class_in_bay": ("aisle", "bay"),
        "max_class": (),
        "farthest": ("aisle",),
        "empty_slot_in_bay": ("aisle", "bay"),
    }

    def _sanitize(self, query: dict[str, Any]) -> tuple[dict[str, Any], str | None]:
        """**边界校验**：LLM 的输出不可信，坏值一律转成拒绝码，绝不抛异常。

        踩过（2026-10-31）：ollama 后端把 bay 解析成 "F"（那是巷道字母），
        `int("F")` 直接 ValueError 把整条 69 条基准跑崩。规则后端永远产不出这种值，
        所以这个洞一直没暴露 —— 换了会"自由发挥"的后端才炸。
        处理原则与 LANG-1b 一致：**拒绝要带原因码**，而不是崩、也不是静默猜。
        """
        q = dict(query or {})
        kind = q.get("kind")
        required = self._KINDS.get(kind) if isinstance(kind, str) else None
        if required is None:
            return {}, "UNKNOWN_INTENT"
        if any(k not in q for k in required):
            return {}, "UNKNOWN_INTENT"

        names = {a["name"] for a in self.grid["aisles"]}
        n_bays = int(self.grid["derived"]["bays_per_aisle"])

        if "address" in q:
            a = q["address"]
            a = a.strip().upper() if isinstance(a, str) else ""
            if a not in self.grid["slots"]:
                return {}, "NO_SLOT"
            q["address"] = a
        if "aisle" in q:
            a = q["aisle"]
            a = a.strip().upper() if isinstance(a, str) else ""
            if a not in names:
                return {}, "NO_SLOT"
            q["aisle"] = a
        for key in ("bay", "level"):
            if key in q:
                v = q[key]
                # **None 表示"没指定"，不是非法值**：规则 NLU 会带 level=None / side=None，
                # 把他们当坏值会直接把正常指令判成 NO_SLOT（实测把 43/69 打成 62.3%）。
                if v is None:
                    q.pop(key)
                    continue
                n = v if isinstance(v, int) and not isinstance(v, bool) else None
                if n is None and isinstance(v, str) and v.strip().isdigit():
                    n = int(v.strip())
                if n is None:
                    return {}, "NO_SLOT"
                if key == "bay" and not 1 <= n <= n_bays:
                    return {}, "NO_SLOT"
                if key == "level" and n not in (1, 2):
                    return {}, "NO_SLOT"
                q[key] = n
        return q, None

    def resolve(self, query: dict[str, Any]) -> tuple[list[dict[str, Any]], str | None]:
        """返回 (候选列表, 拒绝原因码)。候选里带 score / why / reachable。"""
        query, bad = self._sanitize(query)
        if bad:
            return [], bad
        kind = query.get("kind")
        slots = self.grid["slots"]

        if kind == "address":
            addr = query["address"]
            if addr not in slots:
                return [], "NO_SLOT"
            c = self._slot_candidate(addr, score=1.0, why="地址直达")
            return [c], None

        if kind == "bay":
            bay = int(query["bay"])
            if not 1 <= bay <= int(self.grid["derived"]["bays_per_aisle"]):
                return [], "NO_SLOT"
            cands = [
                self._slot_candidate(a, score=0.5, why=f"{bay:02d} 号位（多条巷道/多侧都叫这个名字）")
                for a in sorted(slots)
                if int(a.split("-")[2]) == bay
            ]
            return cands, None

        if kind == "aisle":
            aisle = query["aisle"]
            if aisle not in {a["name"] for a in self.grid["aisles"]}:
                return [], "NO_SLOT"
            return [
                {
                    "target": {"kind": "aisle", "aisle": aisle},
                    "score": 1.0,
                    "why": f"{aisle} 巷",
                    "reachable": True,
                }
            ], None

        if kind == "class_in_aisle":
            aisle = query["aisle"]
            classes = set(query.get("classes") or CARTON_CLASSES)
            if aisle not in {a["name"] for a in self.grid["aisles"]}:
                return [], "NO_SLOT"
            known = set(self.grid.get("classes") or [])
            unknown = classes - known
            if unknown and not (classes & known):
                return [], "NO_SLOT"
            cands = []
            for side in ("L", "R"):
                counts = self.map.rack_face_counts(aisle, side)
                n = sum(v for k, v in counts.items() if k in classes)
                if n > 0:
                    cands.append(
                        {
                            "target": {"kind": "rack_face", "aisle": aisle, "side": side},
                            "score": float(n),
                            "why": f"{aisle}-{side} 面有 {n} 件 {','.join(sorted(classes))}",
                            "reachable": True,
                        }
                    )
            cands.sort(key=lambda c: (-c["score"], c["target"]["side"]))
            if not cands:
                return [], "EMPTY_BAY"
            return cands, None

        if kind == "class_in_bay":
            aisle = query["aisle"]
            bay = int(query["bay"])
            if aisle not in {a["name"] for a in self.grid["aisles"]} or not 1 <= bay <= int(
                self.grid["derived"]["bays_per_aisle"]
            ):
                return [], "NO_SLOT"
            classes = set(query.get("classes") or CARTON_CLASSES)
            level = query.get("level")
            levels = (level,) if level else (1, 2)
            found: list[str] = []
            any_occ = False
            for side in ("L", "R"):
                for lvl in levels:
                    addr = f"{aisle}-{side}-{bay:02d}-{lvl}"
                    st = self.map.get(addr)
                    if st is None:
                        continue
                    if st.occupied:
                        any_occ = True
                        if st.cls in classes:
                            found.append(addr)
            if found:
                return [self._slot_candidate(a, 1.0, "bay 内指定类别") for a in found], None
            return [], ("NO_SLOT" if any_occ else "EMPTY_BAY")

        if kind == "max_class":
            classes = set(query.get("classes") or CARTON_CLASSES)
            ranking = self.map.rank_rack_faces_by_class(classes)
            top = [r for r in ranking if r["count"] == ranking[0]["count"] and r["count"] > 0]
            if not ranking or ranking[0]["count"] == 0:
                return [], "EMPTY_BAY"
            return [
                {
                    "target": {"kind": "rack_face", "aisle": r["aisle"], "side": r["side"]},
                    "score": float(r["count"]),
                    "why": f"{r['rack_face']} 有 {r['count']} 件",
                    "reachable": True,
                }
                for r in top
            ], None

        if kind == "farthest":
            aisle = query["aisle"]
            if aisle not in {a["name"] for a in self.grid["aisles"]}:
                return [], "NO_SLOT"
            want_empty = bool(query.get("empty"))
            best: list[tuple[float, str]] = []
            for node in self.graph.stations(aisle):
                bay = int(node.split("@")[1])
                for side in ("L", "R"):
                    for level in (1, 2):
                        addr = f"{aisle}-{side}-{bay:02d}-{level}"
                        st = self.map.get(addr)
                        if st is None:
                            continue
                        if want_empty and st.occupied:
                            continue
                        d = self.graph.path_length((self.pose["x"], self.pose["y"]), node)
                        if not math.isfinite(d):
                            continue  # 路障后面：不是"最里面"，是"到不了"
                        best.append((d, addr))
            if not best:
                return [], "EMPTY_BAY"
            best.sort(key=lambda b: -b[0])
            far = [b for b in best if abs(b[0] - best[0][0]) < 0.5]
            return [
                self._slot_candidate(addr, score=1.0, why=f"{aisle} 巷最里面（{d:.1f} m）")
                for d, addr in far
            ], None

        if kind == "empty_slot_in_bay":
            aisle = query["aisle"]
            bay = int(query["bay"])
            cands = []
            for side in ("L", "R"):
                for level in (1, 2):
                    addr = f"{aisle}-{side}-{bay:02d}-{level}"
                    st = self.map.get(addr)
                    if st is None:
                        continue
                    if st.occupied:
                        cands.append(self._slot_candidate(addr, score=0.2, why="该格有货（不是空位）"))
            if not cands:
                return [], "EMPTY_BAY"
            return cands, None

        return [], "NO_SLOT"

    # ------------------------------------------------------------ 候选构造
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

    # ------------------------------------------------------------ 语义级恢复
    def substitute(
        self,
        target: dict[str, Any],
        *,
        origin_xy: tuple[float, float] | None = None,
        exclude: tuple[str, ...] | list[str] = (),
    ) -> dict[str, Any] | None:
        """原目标不可达时，找**同语义**的替代货位：同巷 / 同面 / 同层、bay 最近且**可达**。

        只用地图（`grid["slots"]`）+ 语义图（`graph.path_length`），不读 truth 的占用/类别。
        找不到返回 None —— 那就老老实实报 BLOCKED，不许编一个。
        """
        if target.get("kind") != "slot":
            return None
        aisle, side = target.get("aisle"), target.get("side")
        bay, level = int(target.get("bay") or 0), int(target.get("level") or 0)
        if not (aisle and side and bay and level):
            return None
        origin = origin_xy or (self.pose["x"], self.pose["y"])
        ex = set(exclude)
        n_bays = int(self.graph.bays_per_aisle)

        best: list[tuple[int, float, str]] = []
        for d in range(1, n_bays):
            for b in (bay + d, bay - d):
                if not 1 <= b <= n_bays:
                    continue
                addr = f"{aisle}-{side}-{b:02d}-{level}"
                if addr in ex or addr not in self.grid["slots"]:
                    continue
                length = self.graph.path_length(origin, self.graph.node_for_slot(addr))
                if not math.isfinite(length):
                    continue
                best.append((d, length, addr))
            if best:            # 同巷有可达的就够了，不再往远处找
                break

        if not best:            # 同巷全不可达（整条巷封了）：换其它巷的同一 bay/层/面
            for a in (x["name"] for x in self.grid["aisles"] if x["name"] != aisle):
                addr = f"{a}-{side}-{bay:02d}-{level}"
                if addr in ex or addr not in self.grid["slots"]:
                    continue
                length = self.graph.path_length(origin, self.graph.node_for_slot(addr))
                if math.isfinite(length):
                    best.append((n_bays, length, addr))

        if not best:
            return None
        best.sort(key=lambda t: (t[0], t[1]))
        d, length, addr = best[0]
        cand = self._slot_candidate(
            addr, score=1.0,
            why=f"原目标不可达；同{aisle}巷同面同层最近的 bay（Δbay={d}，{length:.1f} m）")
        return cand

    # ------------------------------------------------------------ 规划
    def plan(self, target: dict[str, Any]) -> dict[str, Any] | None:
        """图上算路 + 算观测位 + 决定进入动作（能否侧移贴靠）。"""
        kind = target.get("kind")
        if kind == "slot":
            node = self.graph.node_for_slot(target["address"])
            obs = target["obs_pose"]
            entry = self._entry_action(target)
        elif kind == "rack_face":
            aisle = target["aisle"]
            side = target["side"]
            bays = range(1, int(self.grid["derived"]["bays_per_aisle"]) + 1)
            sample = f"{aisle}-{side}-{list(bays)[len(list(bays)) // 2]:02d}-1"
            slot = self.grid["slots"][sample]
            node = self.graph.node_for_slot(sample)
            obs = slot["obs_pose"]
            entry = {"type": "front", "lateral": 0.0, "note": "货架面中点观测"}
            target = dict(target, **{"obs_pose": obs, "sample_slot": sample})
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
    def goto(self, target: dict[str, Any], plan: dict[str, Any] | None = None, seq: int | None = None) -> dict[str, Any]:
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

    # ------------------------------------------------------------ 观测（VLM 之前的一步：单帧 + 占用）
    def observe_slot(self, addr: str) -> dict[str, Any]:
        st = self.map.get(addr)
        if st is None:
            return {"ok": False, "reason_code": "NO_SLOT", "detail": "该货位没有观测记录"}
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

    # ------------------------------------------------------------ 真值对比（只给打分用）
    def score(self, target: dict[str, Any],
              observation: dict[str, Any] | None = None) -> dict[str, Any]:
        """打分：目标地址是否存在于真值，以及**现场观测与真值的逐项对比**。

        **为什么重写**：原来的 `score_target()` 只返回 `matches_truth = (地址在真值表里存在)`，
        根本没比占用/类别 —— 却叫 matches_truth，实测会给出"真值 carton_m、观测 carton_s"
        还报 true 的**假绿灯**。现在占用、类别分开判，观测缺失就留 `None`（不猜）。
        """
        out: dict[str, Any] = {"slot_in_truth": False, "truth_occupied": None,
                               "truth_class": None, "truth_reachable": None,
                               "obs_occupied": None, "obs_class": None,
                               "occupied_match": None, "class_match": None}
        if target.get("kind") != "slot":
            out["slot_in_truth"] = True          # 巷/面级目标没有单点真值，不算失配
            return out
        gt = self.truth["slots"].get(target["address"])
        if gt is None:
            return out
        out["slot_in_truth"] = True
        out["truth_occupied"] = gt["occupied"]
        out["truth_class"] = gt["class"]
        out["truth_reachable"] = gt["reachable"]
        if not observation:
            return out
        occ = observation.get("occupied")
        cls = observation.get("cls") or observation.get("class")
        out["obs_occupied"] = occ
        out["obs_class"] = cls
        if occ is not None:
            out["occupied_match"] = bool(occ) == bool(gt["occupied"])
        if cls and gt["occupied"]:
            out["class_match"] = (str(cls) == str(gt["class"]))
        return out

    def score_target(self, target: dict[str, Any]) -> dict[str, Any]:
        """兼容旧调用：只比目标本身（不含观测）。**别再拿 matches_truth 当"对上了"**。"""
        return self.score(target)
        if kind == "rack_face":
            key = f"{target['aisle']}-{target['side']}"
            return {"truth_carton": self.truth["carton_totals"].get(key, 0), "matches_truth": True}
        return {"matches_truth": True}


def load_mock_world(slot_grid_path: str | Path | None = None, truth_path: str | Path | None = None, **kw) -> MockWorld:
    import yaml

    grid = json.loads(Path(slot_grid_path or ROOT / "world" / "slot_grid.json").read_text(encoding="utf-8"))
    truth = json.loads(Path(truth_path or ROOT / "ground_truth" / "slots.json").read_text(encoding="utf-8"))
    aisles = json.loads((ROOT / "ground_truth" / "aisles.json").read_text(encoding="utf-8"))
    truth["blocked"] = aisles.get("blocked", [])
    truth["carton_totals"] = aisles.get("carton_totals", {})
    cfg = yaml.safe_load((ROOT / "config" / "world.yaml").read_text(encoding="utf-8"))
    grid["classes"] = list(cfg["semantics"]["classes"])
    return MockWorld(grid, truth, **kw)
