"""P6-1 约束层：LLM 只提供"语义槽位"，结构与 `query.kind` 由确定性代码决定。

为什么需要这一层（实测，不是设计偏好）
    裸 LLM 直连 69 条基准（P6 第一轮记录）：qwen2.5:7b 72.5%（静默猜 3）、
    qwen3-vl:8b 75.4%（静默猜 2、34–56 s/条）、qwen3:latest 47.8%（静默猜 8）——
    三条全部 FAIL，因为 LANG-1 的两条安全判据一个都没过。
    **换模型救不了**：这不是模型能力问题，而是**接口问题**。裸 prompt 里模型直接产出
    `query`（甚至 `address`），一旦它把「去 10 号位」补成 `C-R-10-1`，图上就只剩
    一个候选，歧义检测根本不会触发 —— **系统不知道自己被猜了**，这才是"静默猜"的来源。

三条约束（缺一不可）
    1. **结构不交给模型**：`X-L-07-2` / `X 巷` / `N 号位` / `第N层` / 左右 这些**文本里
       有唯一写法**的东西一律正则直接取（可复现、不采样）。模型的槽位只是"补充"，
       而且必须过第 2 条。
    2. **落地面板（grounding）**：每个槽位值必须在原话里找得到文字依据，找不到就**丢弃并记账**。
       「去 10 号位」里模型给的 `aisle=C` 会被丢掉（原话没有 C）—— 幻觉传不下去。
    3. **欠定即发问**：槽位不足时编译器**故意**产出欠定的 query（如 `bay` 不带巷、
       `farthest` 不带面/层），图上必然多候选 → 图走 AMBIGUOUS → 发问。
       **不需要改图**：`nodes.make_resolve()` 已有的"候选取 >1 就 AMBIGUOUS"就是这条闸门。

为什么"结构用正则"不算作弊
    这不是"用规则代替模型"，而是**把可判定的部分从模型手里拿走**。工业上同理：
    结构化字段用解析器、开放语义用模型，两者之间用"落地校验"隔开。
    对比实验里 `--backend ollama` 仍是裸模型（对照组），本文件是实验组。

接口
    ConstrainedNLU 与 RuleNLU / OllamaNLU 同签名（understand / ask / respond），
    图不感知后端差异；`stats` 暴露回退/丢弃/修复次数，供基准打印（静默回退会伪造好成绩）。
"""
from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from typing import Any

from .nlu import CLASS_WORDS, UNKNOWN_CLASS_WORDS, OllamaNLU, RuleNLU

# ---- 与 SPEC / slot_grid 一致的契约
AISLES = "ABCDEF"
BAYS_PER_AISLE = 24
LEVELS = (1, 2)
CLASSES = {"carton_s", "carton_m", "carton_l", "tote", "pallet", "drum"}

_SIDE_WORDS: dict[str, tuple[str, ...]] = {
    "L": ("左", "左边", "左侧", "左手", "left"),
    "R": ("右", "右边", "右侧", "右手", "right"),
}
_EMPTY_WORDS = ("空位", "空格", "空的", "空着", "空货位", "没有货", "empty")
_FAR = ("最里面", "最里头", "最里", "最远", "最后一个", "尽头", "farthest", "deepest", "last")
_MOST = ("最多", "most")
_BLOCKED = ("封住", "封堵", "封了", "封锁", "堵住", "blocked", "sealed")

# 地址：`C-L-21-1`、`C L 21`、`c_l_21_2`、`C-L-21`
_ADDR_RE = re.compile(
    r"(?<![a-z])([a-z])\s*[-_\s]\s*(l|r|左|右|left|right)\s*[-_\s]\s*(\d{1,2})"
    r"(?:\s*[-_\s]\s*(\d{1,2}))?(?![0-9])"
)
_AISLE_RE = re.compile(r"(?<![a-z])([a-z])\s*(?:巷|通道|aisle|row)(?![a-z])")
_BAY_RE = re.compile(r"(?<!\d)(\d{1,2})\s*(?:号位|号)")
_SIDE_CHAR = {"l": "L", "左": "L", "left": "L", "r": "R", "右": "R", "right": "R"}


# ---------------------------------------------------------------- 结构（正则，可复现）
def extract_structure(u: str) -> dict[str, Any]:
    """从原话里取**有唯一写法**的结构字段。含越界值（如 `Z-L-01-1` 的 Z）。

    越界值要**原样保留**：它必须走到 resolve 才能变成 NO_SLOT 原因码；
    在这里就"纠正"成合法巷号，等于把一条 reject 变成了一次成功/发问。
    """
    low = (u or "").strip().lower()
    s: dict[str, Any] = {"aisle": None, "side": None, "bay": None, "level": None,
                         "address": None, "from_address": False}
    m = _ADDR_RE.search(low)
    if m:
        bay = int(m.group(3))
        level = int(m.group(4)) if m.group(4) else 1
        s.update(
            aisle=m.group(1).upper(),
            side=_SIDE_CHAR[m.group(2)],
            bay=bay,
            level=level,
            address=f"{m.group(1).upper()}-{_SIDE_CHAR[m.group(2)]}-{bay:02d}-{level}",
            from_address=True,
        )
        return s
    m = _AISLE_RE.search(low)
    if m:
        s["aisle"] = m.group(1).upper()
    m = _BAY_RE.search(low)
    if m:
        s["bay"] = int(m.group(1))
    s["side"] = next((k for k, ws in _SIDE_WORDS.items() if any(w in low for w in ws)), None)
    s["level"] = level_from_text(low)
    return s


def level_from_text(low: str) -> int | None:
    if re.search(r"(第二层|二层|上层|顶层|second|level\s*2|第?2\s*层)", low):
        return 2
    if re.search(r"(第一层|一层|底层|下层|地面|ground|level\s*1|第?1\s*层)", low):
        return 1
    return None


# ---------------------------------------------------------------- 落地面板
def has_aisle(low: str, a: str) -> bool:
    al = a.lower()
    return bool(re.search(rf"(?<![a-z]){al}(?![a-z])", low)) or f"{al}巷" in low.replace(" ", "")


def has_side(low: str, s: str) -> bool:
    if any(w in low for w in _SIDE_WORDS.get(s, ())):
        return True
    tok = "l" if s == "L" else "r"
    return bool(re.search(rf"(?<![a-z]){tok}(?![a-z])", low))


def has_bay(low: str, n: int) -> bool:
    return bool(re.search(rf"(?<!\d)0*{n}(?!\d)", low))


def has_level(low: str, n: int) -> bool:
    got = level_from_text(low)
    if got is not None:
        return got == n
    # 地址里的 `-2` 尾段也算层（`C-L-21-2`）
    m = _ADDR_RE.search(low)
    return bool(m and m.group(4) and int(m.group(4)) == n)


def classes_in_text(low: str) -> set[str]:
    out: set[str] = set()
    for word, classes in CLASS_WORDS:
        if word in low:
            out |= classes
    return out


def has_unknown_class(low: str) -> bool:
    return any(w in low for w in UNKNOWN_CLASS_WORDS)


def ground(slots: dict[str, Any], utterance: str) -> tuple[dict[str, Any], list[str]]:
    """把模型给的槽位逐个对原话落地；返回 (留下的, 被丢弃的说明)。"""
    low = (utterance or "").strip().lower()
    keep: dict[str, Any] = {}
    dropped: list[str] = []

    a = slots.get("aisle")
    if a:
        a = str(a).strip().upper()[:1]
        if a in AISLES and has_aisle(low, a):
            keep["aisle"] = a
        else:
            dropped.append(f"aisle={a}")

    s = slots.get("side")
    if s:
        s = str(s).strip().upper()[:1]
        if s in ("L", "R") and has_side(low, s):
            keep["side"] = s
        else:
            dropped.append(f"side={s}")

    b = slots.get("bay")
    try:
        bn = int(b) if b is not None else None
    except (TypeError, ValueError):
        bn = None
        dropped.append(f"bay={b!r}(非整数)")
    if bn is not None:
        if 1 <= bn <= BAYS_PER_AISLE and has_bay(low, bn):
            keep["bay"] = bn
        else:
            dropped.append(f"bay={bn}")

    lv = slots.get("level")
    try:
        lvn = int(lv) if lv is not None else None
    except (TypeError, ValueError):
        lvn = None
        dropped.append(f"level={lv!r}(非整数)")
    if lvn is not None:
        if lvn in LEVELS and has_level(low, lvn):
            keep["level"] = lvn
        else:
            dropped.append(f"level={lvn}")

    cls = slots.get("classes") or []
    if not isinstance(cls, (list, tuple, set)):
        cls = [cls]
    want = {str(c).strip().lower() for c in cls}
    have = classes_in_text(low)
    ok_cls = sorted(want & have)
    if want - have:
        dropped.append(f"classes={sorted(want - have)}")
    if ok_cls:
        keep["classes"] = ok_cls

    for key, words in (("empty", _EMPTY_WORDS), ("blocked", _BLOCKED)):
        if slots.get(key):
            if any(w in low for w in words):
                keep[key] = True
            else:
                dropped.append(f"{key}=True")
    rank = slots.get("rank")
    if rank in ("farthest", "most"):
        words = _FAR if rank == "farthest" else _MOST
        if any(w in low for w in words):
            keep["rank"] = rank
        else:
            dropped.append(f"rank={rank}")
    return keep, dropped


# ---------------------------------------------------------------- 编译器（确定性）
def compile_query(st: dict[str, Any], sem: dict[str, Any] | None = None) -> dict[str, Any]:
    """结构 + 语义 → SPEC 的闭集 query。**没有唯一解时故意产出欠定的 query**（图上会发问）。"""
    sem = dict(sem or {})
    merged = {**sem, **{k: v for k, v in st.items() if v is not None and k != "from_address"}}
    address = st.get("address")

    aisle = merged.get("aisle")
    side = merged.get("side")
    bay = merged.get("bay")
    level = merged.get("level")
    classes = list(merged.get("classes") or [])
    rank = merged.get("rank")
    empty = bool(merged.get("empty"))
    blocked = bool(merged.get("blocked"))

    # 0) 整串地址（含越界值）：交给 resolve 去判 NO_SLOT/EMPTY_BAY
    if address:
        return {"kind": "address", "address": address}
    # 1) 明确说"封住/封堵"
    if blocked:
        return {"kind": "blocked_aisle", "aisle": aisle}
    # 2) 巷 + bay + 类别
    if aisle and bay and classes:
        return {"kind": "class_in_bay", "aisle": aisle, "bay": bay,
                "classes": sorted(classes), "side": side, "level": level}
    # 3) "某类最多的那排货架"
    if rank == "most" and classes:
        return {"kind": "max_class", "classes": sorted(classes)}
    # 4) "最里面"（**可能没面/层** —— 欠定就发问，绝不补默认值）
    if rank == "farthest":
        return {"kind": "farthest", "aisle": aisle, "empty": empty, "side": side, "level": level}
    # 5) 巷 + 类别（"X 巷放纸箱的那面货架"，不说左右 → 两面都算候选 → 发问）
    if aisle and classes:
        return {"kind": "class_in_aisle", "aisle": aisle, "classes": sorted(classes),
                "side": side, "level": level}
    # 6) 巷 + 空位
    if aisle and empty:
        return {"kind": "farthest", "aisle": aisle, "empty": True, "side": side, "level": level}
    # 7) 只说巷
    if aisle:
        return {"kind": "aisle", "aisle": aisle, "level": level}
    # 8) 只说 N 号位（**故意不带巷**：每个巷都有 → 多候选 → 发问）
    if bay:
        return {"kind": "bay", "bay": bay, "level": level, "side": side}
    # 9) 只有类别、没有巷也没有"最多"：无法唯一定位 → 交给图去问，不猜
    if classes:
        return {"kind": "unknown", "why": "class_without_aisle"}
    return {"kind": "unknown"}


# ---------------------------------------------------------------- 后端
SCHEMA_HINT = """只输出一个 JSON 对象，不要解释、不要 markdown。字段：
{
  "intent": "goto" | "unknown",
  "slots": {
    "aisle": "A".."F" 或 null,
    "side": "L" | "R" | null,
    "bay": 1..24 的整数或 null,
    "level": 1 | 2 或 null,
    "classes": ["carton_s","carton_m","carton_l","tote","pallet","drum"] 的子集,
    "empty": true 或 false,
    "rank": "farthest" | "most" | null,
    "blocked": true 或 false
  },
  "confidence": 0.0-1.0,
  "ask": "如果不确定、需要向用户确认的一句话；不确定就写出来" 
}
硬规则：
1. **只写原话里明确出现的信息**。原话没说的槽位一律 null / []，**绝对不要推断或补全**。
   「去 10 号位」→ bay=10，aisle=null（原话没说哪个巷）。
2. 「最里面」→ rank="farthest"；「最多」→ rank="most"；「空位」→ empty=true；
   「封住/封堵」→ blocked=true。
3. 不确定就把 confidence 写低、在 ask 里写清楚要问什么，**不要猜**。"""


@dataclass
class ConstrainedNLU:
    """LLM 只抽语义槽位 → 落地 → 确定性编译。结构字段根本不经过模型。"""

    model: str = "qwen2.5:7b"
    host: str = "http://127.0.0.1:11434"
    name: str = "ollama-constrained"
    fallbacks: int = 0          # 两轮都没给出合法 JSON → 退回规则（会打印，不静默）
    repairs: int = 0            # 第一次输出被判不合格 → 带错误重问一次
    dropped_slots: int = 0      # 落地失败的槽位总数（幻觉在这里被拦下）
    _base: OllamaNLU | None = field(default=None, init=False, repr=False)
    _rule: RuleNLU | None = field(default=None, init=False, repr=False)

    def __post_init__(self) -> None:
        self._base = OllamaNLU(model=self.model, host=self.host)
        self._rule = RuleNLU()

    # ------------------------------------------------------------ 模型这一侧
    def _ask_model(self, utterance: str, context: dict[str, Any]) -> tuple[dict[str, Any] | None, str]:
        assert self._base is not None
        system = (
            "你是仓储机器人的**槽位抽取器**，不是规划器。你不知道地图、不知道候选，"
            "所以你不许猜位置。\n" + SCHEMA_HINT +
            f"\n巷道范围 A-{context.get('aisles_last', 'F')}，bay 1-{context.get('bays_per_aisle', 24)}，"
            "层 1-2。"
        )
        raw = self._base._chat(system, utterance)
        obj = _parse_json(raw)
        err = validate_payload(obj)
        if err is None:
            return obj, ""
        return None, err

    # ------------------------------------------------------------ 后端接口
    def understand(self, utterance: str, context: dict[str, Any]) -> dict[str, Any]:
        low = (utterance or "").strip().lower()
        st = extract_structure(utterance)

        sem: dict[str, Any] = {}
        payload, err = self._ask_model(utterance, context)
        if payload is None and err:
            # 带错误重问一次（约束校验失败是**可修复**的，直接退规则会掩盖问题）
            self.repairs += 1
            payload, err = self._ask_model(
                utterance + f"\n\n（上一次输出被校验拒绝：{err}。请严格按 schema 重写。）", context)
        if payload is None:
            self.fallbacks += 1
            assert self._rule is not None
            return self._rule.understand(utterance, context)

        slots = payload.get("slots") or {}
        grounded, dropped = ground(slots, utterance)
        self.dropped_slots += len(dropped)
        # **结构字段优先**（正则可复现），模型只补语义；补进来的也必须已在原话里落地。
        # 语义取**并集**：文本关键词（确定性）∪ 模型落地值（补充同义词）。
        # 并集的两边都"有文字依据"，所以不会凭空造出槽位；只会提高召回。
        text_cls = classes_in_text(low)
        model_cls = set(grounded.get("classes") or [])
        sem = {
            "classes": sorted(text_cls | model_cls),
            "rank": (grounded.get("rank")
                     or ("farthest" if any(w in low for w in _FAR)
                         else "most" if any(w in low for w in _MOST) else None)),
            "empty": bool(grounded.get("empty")) or any(w in low for w in _EMPTY_WORDS),
            "blocked": bool(grounded.get("blocked")) or any(w in low for w in _BLOCKED),
        }
        if not sem["classes"] and has_unknown_class(low):
            # 闭集之外的类别（"冰箱"）：认出来，但不猜成最近的类别
            return {"intent": "goto", "query": {"kind": "max_class", "classes": ["__unknown__"]}}

        q = compile_query(st, sem)
        intent = "unknown" if q.get("kind") == "unknown" else "goto"
        if payload.get("intent") == "unknown" and q.get("kind") == "unknown":
            intent = "unknown"
        return {"intent": intent, "query": q}

    def ask(self, candidates: list[dict[str, Any]], utterance: str) -> str:
        assert self._rule is not None
        return self._rule.ask(candidates, utterance)

    def respond(self, state: dict[str, Any]) -> str:
        assert self._rule is not None
        return self._rule.respond(state)

    @property
    def stats(self) -> dict[str, int]:
        return {"fallbacks": self.fallbacks, "repairs": self.repairs,
                "dropped_slots": self.dropped_slots}


# ---------------------------------------------------------------- 校验
def _parse_json(raw: str) -> dict[str, Any] | None:
    if not raw:
        return None
    m = re.search(r"\{.*\}", raw, re.S)
    if not m:
        return None
    try:
        obj = json.loads(m.group(0))
    except json.JSONDecodeError:
        return None
    return obj if isinstance(obj, dict) else None


def validate_payload(obj: dict[str, Any] | None) -> str | None:
    """返回错误说明（用于带错误重问）；None = 合格。

    **只校验形状，不修正内容**：内容层面的"是否真的有依据"由 `ground()` 做。
    这里放过的越界值（bay=99）也会在 ground 里被丢掉。
    """
    if obj is None:
        return "不是合法 JSON 对象"
    if "slots" not in obj:
        return "缺少 slots 字段"
    slots = obj.get("slots")
    if not isinstance(slots, dict):
        return "slots 必须是对象"
    allowed = {"aisle", "side", "bay", "level", "classes", "empty", "rank", "blocked"}
    extra = set(slots) - allowed
    if extra:
        return f"slots 里有未知字段 {sorted(extra)}"
    for key in ("bay", "level"):
        v = slots.get(key)
        if v is not None and not isinstance(v, int):
            return f"{key} 必须是整数或 null（收到 {v!r}）"
    cls = slots.get("classes")
    if cls is not None and not isinstance(cls, list):
        return "classes 必须是数组"
    if obj.get("intent") not in (None, "goto", "unknown"):
        return f"intent 只能是 goto/unknown（收到 {obj.get('intent')!r}）"
    return None
