"""语义-拓扑图（SPEC §7.2）：节点 = 巷道站位 / 货架面 / bay，边 = 自由空间连通性。

只用 world/slot_grid.json（运行时允许读的几何）+ blocked 区间（来自现场感知或运营信息），
不读 ground_truth。路径代价用 Dijkstra，权重 = 欧氏距离 × 通行代价系数。
"""
from __future__ import annotations

import heapq
import json
import math
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]

# 通行代价：巷道内直行便宜，横通道穿越略贵（模拟叉车交汇概率）
COST_AISLE = 1.0
COST_CROSS = 1.15


def _dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


class SemanticGraph:
    """仓库拓扑图。节点 id：'W:A' / 'E:A' / 'A@b07'（b 号 bay 站位）。"""

    def __init__(self, slot_grid: dict[str, Any], blocked: list[dict[str, Any]] | None = None):
        self.grid = slot_grid
        self.blocked = blocked or []
        self.pos: dict[str, tuple[float, float]] = {}
        self.adj: dict[str, list[tuple[str, float]]] = {}
        self.aisles = {a["name"]: a for a in slot_grid["aisles"]}
        self.bays_per_aisle = int(slot_grid["derived"]["bays_per_aisle"])
        self.cross_w = float(slot_grid["derived"]["cross_aisle_width"])
        self._build()

    # ------------------------------------------------------------ build
    def _barrier_xs(self, aisle: str) -> list[float]:
        xs: list[float] = []
        for seg in self.blocked:
            if seg.get("aisle") == aisle:
                if seg.get("barrier_xs"):
                    xs.extend(float(v) for v in seg["barrier_xs"])
                else:  # 兼容只有区间的情况
                    xs.append(float(seg["x0"]) - 0.35)
        return xs

    def _add(self, node: str, xy: tuple[float, float]) -> None:
        self.pos[node] = xy
        self.adj.setdefault(node, [])

    def _edge(self, a: str, b: str, cost: float) -> None:
        if cost <= 0:
            return
        self.adj[a].append((b, cost))
        self.adj[b].append((a, cost))

    def _build(self) -> None:
        der = self.grid["derived"]
        x_rack0 = -der["rack_run"] / 2
        x_rack1 = der["rack_run"] / 2
        xw = x_rack0 - self.cross_w / 2
        xe = x_rack1 + self.cross_w / 2

        for name, aisle in self.aisles.items():
            y = aisle["center_y"]
            w, e = f"W:{name}", f"E:{name}"
            self._add(w, (xw, y))
            self._add(e, (xe, y))
            stations = [(f"{name}@{b:02d}", (x_rack0 + (b - 0.5) * 2.4, y)) for b in range(1, self.bays_per_aisle + 1)]
            for node, xy in stations:
                self._add(node, xy)
            chain = [w] + [s[0] for s in stations] + [e]
            barriers = self._barrier_xs(name)
            for a, b in zip(chain, chain[1:]):
                x0 = self.pos[a][0]
                x1 = self.pos[b][0]
                if any(min(x0, x1) < bx < max(x0, x1) for bx in barriers):
                    continue  # 路障横跨这段：边不存在
                self._edge(a, b, _dist(self.pos[a], self.pos[b]) * COST_AISLE)

        # 横通道：同一端的站点两两相连（按 Y 排序成链，避免 O(n²) 边数爆炸）
        for prefix in ("W", "E"):
            nodes = sorted(
                [f"{prefix}:{a}" for a in self.aisles],
                key=lambda n: self.pos[n][1],
            )
            for a, b in zip(nodes, nodes[1:]):
                self._edge(a, b, _dist(self.pos[a], self.pos[b]) * COST_CROSS)

    # ------------------------------------------------------------ query
    def node_for_slot(self, addr: str) -> str:
        aisle, _side, bay, _lvl = addr.split("-")
        return f"{aisle}@{int(bay):02d}"

    def nearest_node(self, x: float, y: float) -> str:
        return min(self.pos, key=lambda n: _dist(self.pos[n], (x, y)))

    def dijkstra(self, start: str) -> dict[str, float]:
        dist = {start: 0.0}
        pq = [(0.0, start)]
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist.get(u, math.inf) + 1e-12:
                continue
            for v, w in self.adj[u]:
                nd = d + w
                if nd < dist.get(v, math.inf) - 1e-12:
                    dist[v] = nd
                    heapq.heappush(pq, (nd, v))
        return dist

    def route(self, start_xy: tuple[float, float], goal_node: str) -> list[str]:
        """最短路**节点序列**（含起点、终点）。空列表 = 不可达。

        为什么需要它：`drive_to_point` 是点对点控制器（直线 + 朝目标修正），
        从 A 巷直奔 C 巷的货位会横穿货架排、顶死在货架上（实测 363 s 超时）。
        图里的节点坐标都在巷道中线/站台上，逐节点走就是"沿巷道走"。
        """
        start = self.nearest_node(*start_xy)
        if start == goal_node:
            return [start]
        dist = {start: 0.0}
        prev: dict[str, str] = {}
        pq = [(0.0, start)]
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist.get(u, math.inf) + 1e-12:
                continue
            if u == goal_node:
                break
            for v, w in self.adj[u]:
                nd = d + w
                if nd < dist.get(v, math.inf) - 1e-12:
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))
        if goal_node not in dist:
            return []
        out = [goal_node]
        while out[-1] != start:
            out.append(prev[out[-1]])
        return list(reversed(out))

    def path_length(self, start_xy: tuple[float, float], goal_node: str) -> float:
        start = self.nearest_node(*start_xy)
        return self.dijkstra(start).get(goal_node, math.inf)

    def reachable(self, start_xy: tuple[float, float], goal_node: str) -> bool:
        return math.isfinite(self.path_length(start_xy, goal_node))

    def stations(self, aisle: str) -> list[str]:
        return [n for n in self.pos if n.startswith(f"{aisle}@")]

    def stats(self) -> dict[str, Any]:
        n_edges = sum(len(v) for v in self.adj.values()) // 2
        # 连通分量
        seen: set[str] = set()
        comps = 0
        for n in self.adj:
            if n in seen:
                continue
            comps += 1
            stack = [n]
            seen.add(n)
            while stack:
                u = stack.pop()
                for v, _ in self.adj[u]:
                    if v not in seen:
                        seen.add(v)
                        stack.append(v)
        return {"nodes": len(self.adj), "edges": n_edges, "components": comps}


def load_graph(slot_grid_path: str | Path | None = None, blocked: list | None = None) -> SemanticGraph:
    p = Path(slot_grid_path) if slot_grid_path else ROOT / "world" / "slot_grid.json"
    return SemanticGraph(json.loads(p.read_text(encoding="utf-8")), blocked)


if __name__ == "__main__":
    g = load_graph()
    print("graph", g.stats())
    for a in g.aisles:
        print(f"  {a}: {len(g.stations(a))} 站位")
