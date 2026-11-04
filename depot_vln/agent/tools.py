"""工具层（SPEC §8.4）：唯一能碰 /cmd_vel 的地方，图里只有 act 节点调它。

契约
  * 每个调用带 `seq`（单调递增），过期结果丢弃
  * 返回结构化 {ok, reason_code, pose, detail, elapsed}（Pydantic 校验，缺字段即失败）
  * 有前置条件、后置条件、超时、可取消
  * 长动作不阻塞：`navigate` 立刻返回 "RUNNING"，用 `status(seq)` 轮询
"""
from __future__ import annotations

import time
from typing import Any, Protocol

from pydantic import BaseModel, Field


class ToolResult(BaseModel):
    ok: bool
    reason_code: str | None = None
    pose: dict[str, float] | None = None
    detail: str = ""
    elapsed: float = 0.0
    seq: int | None = None
    state: str = "SUCCEEDED"  # RUNNING | SUCCEEDED | CANCELED | FAILED


class Observation(BaseModel):
    ok: bool
    reason_code: str | None = None
    occupied: bool | None = None
    cls: str | None = None
    confidence: float = 0.0
    detail: str = ""
    elapsed: float = 0.0


class ToolLayer(Protocol):
    def resolve(self, query: dict[str, Any]) -> tuple[list[dict[str, Any]], str | None]: ...

    def plan(self, target: dict[str, Any]) -> dict[str, Any] | None: ...

    def navigate(self, target: dict[str, Any], plan: dict[str, Any] | None, seq: int) -> ToolResult: ...

    def status(self, seq: int) -> ToolResult: ...

    def cancel(self, seq: int) -> ToolResult: ...

    def observe(self, target: dict[str, Any]) -> Observation: ...


# ---------------------------------------------------------------- mock 实现
class MockToolLayer:
    """把 MockWorld 包成工具层：加 seq 校验、取消语义、"过期结果丢弃"。"""

    def __init__(self, world, *, step_time: float = 0.0):
        self.world = world
        self.step_time = step_time
        self._current: dict[str, Any] | None = None
        self._done: dict[int, ToolResult] = {}

    # --- 图查询
    def resolve(self, query: dict[str, Any]):
        return self.world.resolve(query)

    def plan(self, target: dict[str, Any]):
        return self.world.plan(target)

    # --- 动作
    def navigate(self, target: dict[str, Any], plan: dict[str, Any] | None, seq: int) -> ToolResult:
        if self._current is not None and self._current["seq"] > seq:
            return ToolResult(ok=False, reason_code="TIMEOUT", detail="seq 过期，目标被抢占", seq=seq, state="CANCELED")
        plan = plan or self.world.plan(target)
        if plan is None:
            res = ToolResult(ok=False, reason_code="UNREACHABLE", pose=dict(self.world.pose),
                             detail="图上无路径", seq=seq, state="FAILED")
            self._done[seq] = res
            return res
        self._current = {"seq": seq, "target": target, "plan": plan}
        raw = self.world.goto(target, plan, seq=seq)
        if self.step_time:
            time.sleep(self.step_time)
        res = ToolResult(
            ok=raw["ok"],
            reason_code=raw.get("reason_code"),
            pose=raw.get("pose"),
            detail=raw.get("detail", ""),
            elapsed=float(raw.get("elapsed", 0.0)),
            seq=seq,
            state="SUCCEEDED" if raw["ok"] else "FAILED",
        )
        self._done[seq] = res
        return res

    def status(self, seq: int) -> ToolResult:
        res = self._done.get(seq)
        if res is None:
            return ToolResult(ok=False, reason_code="TIMEOUT", detail=f"没有 seq={seq} 的动作", seq=seq, state="FAILED")
        return res

    def cancel(self, seq: int) -> ToolResult:
        if self._current and self._current["seq"] == seq:
            self._current = None
            return ToolResult(ok=True, detail=f"seq={seq} 已取消（绝不回报已完成）", seq=seq, state="CANCELED")
        return ToolResult(ok=False, reason_code="NO_SLOT", detail=f"seq={seq} 不在执行中", seq=seq, state="CANCELED")

    # --- 观测
    def observe(self, target: dict[str, Any]) -> Observation:
        kind = target.get("kind")
        if kind == "slot":
            raw = self.world.observe_slot(target["address"])
        elif kind == "rack_face":
            addr = target.get("sample_slot") or f"{target['aisle']}-{target['side']}-01-1"
            raw = self.world.observe_slot(addr)
        else:
            return Observation(ok=True, detail="巷道级目标无需观测", confidence=1.0)
        return Observation(
            ok=raw["ok"],
            reason_code=raw.get("reason_code"),
            occupied=raw.get("occupied"),
            cls=raw.get("class"),
            confidence=raw.get("confidence", 0.0),
            detail=raw.get("detail", ""),
            elapsed=raw.get("elapsed", 0.0),
        )


# ---------------------------------------------------------------- ROS 实现（P6 接线）
class RosToolLayer:
    """真机/仿真接线的位置。P6 用 rclpy 实现：NavigateToPose action + 状态轮询。

    现在故意抛错，而不是悄悄 mock —— "哪一层还没接"必须是显式的。
    """

    def __init__(self, *args: Any, **kwargs: Any):
        raise NotImplementedError(
            "RosToolLayer 未接线：P6 阶段实现 rclpy 版 "
            "(nav2 NavigateToPose action + /cmd_vel 独占 + seq 取消)。基准请用 MockToolLayer。"
        )
