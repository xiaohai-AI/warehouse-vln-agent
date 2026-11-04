"""世界事实的唯一入口（World Catalog）。

为什么存在这个模块
------------------
重构前，同一个"世界事实"被写死在至少六处：`nlu.py` 里 `list("ABCDEF")`、
`constraint.py` 里 `AISLES = "ABCDEF"` / `BAYS_PER_AISLE = 24`、`nodes.py` 里
`ctx.setdefault("aisles", list("ABCDEF"))`、`mock/world.py` 里 `("L","R")` 与 `(1,2)`、
`build_bench.py` 里又一遍、以及十几个脚本里硬编码的出生点 `(-31.0, 0.0)`。
后果是：改一个世界参数（多一条巷道、bay 数从 24 改 30）必须全局搜字符串，
漏一处就得到"配置说一套、代码做另一套"，而且没有任何测试会报错。

现在只有这里。任何模块想知道"有哪些巷道 / 多少个 bay / 几层 / 哪些货类 /
车从哪里出发 / 用户会怎么叫这些东西"，都问 `get_catalog()`。

分层
----
    config/world.yaml      物理世界（几何、尺寸、颜色、占用概率）
    config/semantics.yaml  这个世界的语言（怎么称呼货类、方位、层、关系）
    world/slot_grid.json   由 world.yaml 生成的几何（可选，用于交叉校验）
    depot_vln/catalog.py   ← 合并三者，提供解析/格式化/词表

本模块**不读 ground_truth**：占用与类别是感知层的事（depot_vln/semantic/）。
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from functools import cached_property
from pathlib import Path
from typing import Any, Iterable, Mapping

import yaml

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_WORLD = ROOT / "config" / "world.yaml"
DEFAULT_SEMANTICS = ROOT / "config" / "semantics.yaml"


# --------------------------------------------------------------------- 值对象
@dataclass(frozen=True)
class Address:
    """货位地址。<aisle>-<side>-<bay>-<level>，bay 零填充宽度由 catalog 决定。"""

    aisle: str
    side: str
    bay: int
    level: int | None = None
    pad: int = 2

    def __str__(self) -> str:  # noqa: D105
        base = f"{self.aisle}-{self.side}-{self.bay:0{self.pad}d}"
        return base if self.level is None else f"{base}-{self.level}"

    @property
    def rack_face(self) -> str:
        return f"{self.aisle}-{self.side}"

    def with_level(self, level: int) -> "Address":
        return Address(self.aisle, self.side, self.bay, level, self.pad)


@dataclass(frozen=True)
class ClassSpec:
    """一个货类：物理属性来自 world.yaml，叫法来自 semantics.yaml。"""

    name: str
    words: tuple[str, ...] = ()
    size: tuple[float, float, float] | None = None
    color: tuple[float, float, float] | None = None
    shape: str = "box"


@dataclass(frozen=True)
class LevelSpec:
    index: int
    deck_z: float
    clear_height: float


# --------------------------------------------------------------------- Catalog
class Catalog:
    """把 world.yaml + semantics.yaml（+ 可选 slot_grid）合成一份只读世界事实。"""

    def __init__(
        self,
        world_cfg: Mapping[str, Any],
        semantics_cfg: Mapping[str, Any] | None = None,
        *,
        layout: Mapping[str, Any] | None = None,
        world_path: str | Path | None = None,
        semantics_path: str | Path | None = None,
    ):
        self.world_cfg = dict(world_cfg)
        self.semantics_cfg = dict(semantics_cfg or {})
        self.world_path = str(world_path or DEFAULT_WORLD)
        self.semantics_path = str(semantics_path or DEFAULT_SEMANTICS)

        racks = self.world_cfg["racks"]
        self.aisles: tuple[str, ...] = tuple(str(a) for a in racks["aisle_names"])
        self.bays_per_aisle: int = int(racks["bays_per_aisle"])
        self.bay_pitch: float = float(racks["bay_pitch"])
        self.pad: int = max(2, len(str(self.bays_per_aisle)))
        self.levels: tuple[LevelSpec, ...] = tuple(
            LevelSpec(int(lv["level"]), float(lv["deck_z"]), float(lv["clear_height"]))
            for lv in racks["levels"]
        )
        self.sides: tuple[str, ...] = ("L", "R")

        # 布局：出生点等派生量。懒算，避免无谓地依赖 layout。
        self._layout = dict(layout) if layout is not None else None

        # ---- 货类：world.yaml 给物理属性，semantics.yaml 给叫法
        sem_classes = self.semantics_cfg.get("classes") or {}
        classes: dict[str, ClassSpec] = {}
        for name, spec in (self.world_cfg.get("semantics", {}).get("classes") or {}).items():
            words = tuple(str(w) for w in (sem_classes.get(name) or []))
            classes[name] = ClassSpec(
                name=name,
                words=words,
                size=tuple(spec.get("size") or ()) or None,
                color=tuple(spec.get("color") or ()) or None,
                shape=str(spec.get("shape", "box")),
            )
        for name, words in sem_classes.items():  # 词表里有、world 里没有的类别：保留但无物理属性
            classes.setdefault(name, ClassSpec(name=name, words=tuple(str(w) for w in words)))
        self.classes: dict[str, ClassSpec] = classes

        lex = self.semantics_cfg.get("lexicon") or {}
        self.lexicon: dict[str, tuple[str, ...]] = {
            k: tuple(str(w).lower() for w in (v or [])) for k, v in lex.items()
        }
        self.side_words = {
            str(side).upper(): tuple(str(w).lower() for w in (words or []))
            for side, words in (self.semantics_cfg.get("sides") or {}).items()
        }
        self.level_words = {
            int(lv): tuple(str(w).lower() for w in (words or []))
            for lv, words in (self.semantics_cfg.get("levels") or {}).items()
        }
        units = self.semantics_cfg.get("units") or {}
        self.aisle_units = tuple(str(w).lower() for w in (units.get("aisle") or ["巷", "aisle"]))
        self.bay_units = tuple(str(w).lower() for w in (units.get("bay") or ["号位", "号", "bay"]))

        self.blocked: tuple[dict[str, Any], ...] = tuple(self.world_cfg.get("blocked") or ())

    # ------------------------------------------------------------ 构造
    @classmethod
    def load(
        cls,
        world_path: str | Path | None = None,
        semantics_path: str | Path | None = None,
        *,
        with_layout: bool = True,
    ) -> "Catalog":
        wp = Path(world_path) if world_path else DEFAULT_WORLD
        sp = Path(semantics_path) if semantics_path else DEFAULT_SEMANTICS
        world_cfg = yaml.safe_load(wp.read_text(encoding="utf-8")) or {}
        sem_cfg = yaml.safe_load(sp.read_text(encoding="utf-8")) if sp.exists() else {}
        layout = None
        if with_layout:
            from depot_vln.layout import compute_layout

            layout = compute_layout(world_cfg)
        return cls(world_cfg, sem_cfg, layout=layout, world_path=wp, semantics_path=sp)

    @property
    def layout(self) -> dict[str, Any]:
        if self._layout is None:
            from depot_vln.layout import compute_layout

            self._layout = compute_layout(self.world_cfg)
        return self._layout

    # ------------------------------------------------------------ 出生点
    @property
    def spawn(self) -> dict[str, float]:
        """出生点（世界系）。`world.yaml:spawn` 写 auto 时由几何推导。"""
        cfg = self.world_cfg.get("spawn") or {}
        derived = self.layout.get("spawn") or {}
        out: dict[str, float] = {}
        for key in ("x", "y", "yaw"):
            v = cfg.get(key, "auto")
            out[key] = float(derived.get(key, 0.0) if v in (None, "auto", "AUTO") else v)
        return out

    # ------------------------------------------------------------ 路障区间
    def blocked_segments(self) -> list[dict[str, Any]]:
        """把 `world.yaml:blocked` 的 bay 区间换算成 X 区间 + 两道墙的位置。

        配置里只写 `{aisle: F, bay_from: 18, bay_to: 24}` —— 坐标由几何推出，
        不再要求人抄 `x0/x1/barrier_xs`（也无法抄错）。
        """
        der = self.layout["derived"]
        x_rack0 = -float(der["rack_run"]) / 2
        pitch = float(der["bay_pitch"])
        out: list[dict[str, Any]] = []
        for seg in self.blocked:
            aisle = str(seg.get("aisle", "")).upper()
            if aisle not in self.aisles:
                continue
            b0 = int(seg.get("bay_from") or 1)
            b1 = int(seg.get("bay_to") or b0)
            x0 = x_rack0 + (b0 - 1) * pitch
            x1 = x_rack0 + b1 * pitch
            rec = dict(seg)
            rec.update({
                "aisle": aisle,
                "bay_from": b0,
                "bay_to": b1,
                "x0": round(x0, 6),
                "x1": round(x1, 6),
                # 两端各一道墙：只封一端的话，被封段还能从另一头进（几何上就不是"封死"）
                "barrier_xs": [round(x0 - 0.35, 6), round(x1 - 0.35, 6)],
            })
            out.append(rec)
        return out

    # ------------------------------------------------------------ 基础判定
    def is_aisle(self, name: str | None) -> bool:
        return bool(name) and str(name).upper() in self.aisles

    def aisle(self, name: str) -> str:
        u = str(name).upper()
        if u not in self.aisles:
            raise KeyError(f"未知巷道 {name!r}；配置里是 {self.aisles}")
        return u

    def is_side(self, side: str | None) -> bool:
        return bool(side) and str(side).upper() in self.sides

    def valid_bay(self, bay: Any) -> bool:
        return isinstance(bay, int) and not isinstance(bay, bool) and 1 <= bay <= self.bays_per_aisle

    def level_numbers(self) -> tuple[int, ...]:
        return tuple(lv.index for lv in self.levels)

    @property
    def class_names(self) -> tuple[str, ...]:
        return tuple(self.classes)

    # ------------------------------------------------------------ 格式
    def format_address(self, aisle: str, side: str, bay: int, level: int | None = None) -> str:
        return str(Address(str(aisle).upper(), str(side).upper(), int(bay), level, self.pad))

    def address(self, aisle: str, side: str, bay: int, level: int | None = None) -> Address:
        return Address(str(aisle).upper(), str(side).upper(), int(bay), level, self.pad)

    def all_addresses(self) -> list[str]:
        return [
            self.format_address(a, s, b, lv)
            for a in self.aisles
            for s in self.sides
            for b in range(1, self.bays_per_aisle + 1)
            for lv in self.level_numbers()
        ]

    # ------------------------------------------------------------ 正则（按 catalog 生成）
    def _alt(self, values: Iterable[str]) -> str:
        vals = sorted({str(v) for v in values if v}, key=len, reverse=True)
        return "|".join(re.escape(v) for v in vals) if vals else r"(?!)"

    @cached_property
    def address_re(self) -> re.Pattern[str]:
        aisle_alt = self._alt(self.aisles)
        side_alt = self._alt(
            list(self.sides) + [w for words in self.side_words.values() for w in words]
        )
        return re.compile(
            rf"(?<![A-Za-z])(?P<aisle>{aisle_alt})\s*[-_\s]\s*(?P<side>{side_alt})\s*[-_\s]\s*"
            rf"(?P<bay>\d{{1,3}})(?:\s*[-_\s]\s*(?P<level>\d{{1,2}}))?(?!\d)",
            re.IGNORECASE,
        )

    @cached_property
    def _aisle_unit_re(self) -> re.Pattern[str]:
        return re.compile(
            rf"(?<![A-Za-z])(?P<aisle>{self._alt(self.aisles)})\s*(?:{'|'.join(map(re.escape, self.aisle_units))})",
            re.IGNORECASE,
        )

    @cached_property
    def _aisle_bare_re(self) -> re.Pattern[str]:
        # 裸字母巷道名（"去 B"）：只允许单字符巷道名，且两侧不是字母
        singles = [a for a in self.aisles if len(a) == 1]
        if not singles:
            return re.compile(r"(?!)")
        return re.compile(rf"(?<![A-Za-z])(?P<aisle>{self._alt(singles)})(?![A-Za-z0-9])", re.IGNORECASE)

    @cached_property
    def _bay_re(self) -> re.Pattern[str]:
        units = "|".join(map(re.escape, self.bay_units))
        return re.compile(rf"(?<!\d)(?P<bay>\d{{1,3}})\s*(?:{units})(?!\d)", re.IGNORECASE)

    @cached_property
    def _english_bay_re(self) -> re.Pattern[str]:
        return re.compile(r"\b(?:bay|slot)\s*(?P<bay>\d{1,3})\b", re.IGNORECASE)

    @cached_property
    def _ordinal_bay_re(self) -> re.Pattern[str]:
        # "第 5 个位子" / "第 5 格" —— 中文序数说法，配置里不可能穷举，故单列
        return re.compile(r"第\s*(?P<bay>\d{1,3})\s*(?:个)?\s*(?:货位|位子|格子|格|号位)")

    @cached_property
    def _ordinal_level_re(self) -> re.Pattern[str]:
        return re.compile(r"第\s*(?P<level>\d)\s*层")

    # ------------------------------------------------------------ 文本解析
    def parse_address(self, text: str) -> Address | None:
        """从自由文本里抽出完整地址（可缺层）。非法 bay/层/巷道一律返回 None。"""
        m = self.address_re.search(text or "")
        if not m:
            return None
        aisle = m.group("aisle").upper()
        side_raw = m.group("side")
        side = self.side_from_word(side_raw)
        bay = int(m.group("bay"))
        level = int(m.group("level")) if m.group("level") else None
        if not self.is_aisle(aisle) or side is None or not self.valid_bay(bay):
            return None
        if level is not None and level not in self.level_numbers():
            return None
        return Address(aisle, side, bay, level, self.pad)

    @cached_property
    def _loose_address_re(self) -> re.Pattern[str]:
        """不校验合法性的地址外形：`<token>-<side>-<num>[-<num>]`。

        用途：把「去 Z-L-01-1」「去 A-L-01-3」识别成**地址**而不是"某条巷子"，
        这样它能走到 NO_SLOT，而不是被降级成 aisle 查询然后静默给个别的答案。
        """
        side_alt = self._alt(
            list(self.sides) + [w for words in self.side_words.values() for w in words]
        )
        return re.compile(
            rf"(?<![A-Za-z])(?P<aisle>[A-Za-z]{{1,3}})\s*[-_\s]\s*(?P<side>{side_alt})\s*[-_\s]\s*"
            rf"(?P<bay>\d{{1,3}})(?:\s*[-_\s]\s*(?P<level>\d{{1,2}}))?(?!\d)",
            re.IGNORECASE,
        )

    def loose_address(self, text: str) -> str | None:
        """返回规格化后的地址字符串（不保证存在）；不是地址形状则 None。"""
        m = self._loose_address_re.search(text or "")
        if not m:
            return None
        side = self.side_from_word(m.group("side"))
        if side is None:
            return None
        bay = int(m.group("bay"))
        out = f"{m.group('aisle').upper()}-{side}-{bay:0{self.pad}d}"
        if m.group("level"):
            out += f"-{int(m.group('level'))}"
        return out

    def side_from_word(self, word: str | None) -> str | None:
        if not word:
            return None
        w = str(word).strip().upper()
        if w in self.sides:
            return w
        low = str(word).strip().lower()
        for side, words in self.side_words.items():
            if low in words:
                return side
        return None

    def parse_side(self, text: str) -> str | None:
        low = (text or "").lower()
        best: tuple[int, str] | None = None
        for side, words in self.side_words.items():
            for w in words:
                if w and w in low:
                    cand = (len(w), side)
                    if best is None or cand[0] > best[0]:
                        best = cand
        return best[1] if best else None

    def parse_level(self, text: str) -> int | None:
        low = (text or "").lower()
        m = self._ordinal_level_re.search(low)
        if m and int(m.group("level")) in self.level_numbers():
            return int(m.group("level"))
        best: tuple[int, int] | None = None
        for lv, words in self.level_words.items():
            for w in words:
                if w and w in low:
                    cand = (len(w), lv)
                    if best is None or cand[0] > best[0]:
                        best = cand
        return best[1] if best else None

    def parse_aisle(self, text: str) -> str | None:
        m = self._aisle_unit_re.search(text or "")
        if m:
            return m.group("aisle").upper()
        m = self._aisle_bare_re.search(text or "")
        return m.group("aisle").upper() if m else None

    @cached_property
    def _loose_aisle_re(self) -> re.Pattern[str]:
        units = "|".join(map(re.escape, self.aisle_units))
        return re.compile(rf"(?<![A-Za-z])(?P<aisle>[A-Za-z]{{1,3}})\s*(?:{units})", re.IGNORECASE)

    def loose_aisle(self, text: str) -> str | None:
        """`<token> 巷` 里那个 token（不保证存在）。用于把「去 Q 巷」判成 NO_SLOT。"""
        m = self._loose_aisle_re.search(text or "")
        return m.group("aisle").upper() if m else None

    def parse_bay(self, text: str) -> int | None:
        m = (self._bay_re.search(text or "")
             or self._english_bay_re.search(text or "")
             or self._ordinal_bay_re.search(text or ""))
        if not m:
            return None
        bay = int(m.group("bay"))
        return bay if self.valid_bay(bay) else bay  # 超范围也返回，交给上层判 NO_SLOT

    def classes_in_text(self, text: str) -> set[str]:
        low = (text or "").lower()
        out: set[str] = set()
        for spec in self.classes.values():
            if any(w and w in low for w in spec.words):
                out.add(spec.name)
        return out

    def relations_in_text(self, text: str) -> set[str]:
        low = (text or "").lower()
        return {key for key, words in self.lexicon.items() if any(w and w in low for w in words)}

    def mentions_unknown_object(self, text: str) -> bool:
        low = (text or "").lower()
        return any(w and w in low for w in self.lexicon.get("unknown_objects", ()))

    # ------------------------------------------------------------ 给 NLU 的上下文
    def context(self) -> dict[str, Any]:
        return {
            "aisles": list(self.aisles),
            "bays_per_aisle": self.bays_per_aisle,
            "levels": list(self.level_numbers()),
            "classes": list(self.class_names),
            "class_words": {n: list(c.words) for n, c in self.classes.items()},
            "spawn": self.spawn,
        }

    def schema_card(self) -> dict[str, Any]:
        """给 LLM 的紧凑 schema（只含可解析的事实，不含任何数据）。"""
        return {
            "address_format": " ".join(
                [f"<{'|'.join(self.aisles)}>", f"<{'|'.join(self.sides)}>",
                 f"<1..{self.bays_per_aisle}>", f"<{'|'.join(map(str, self.level_numbers()))}>"]
            ),
            "aisles": list(self.aisles),
            "bays": [1, self.bays_per_aisle],
            "levels": list(self.level_numbers()),
            "classes": {name: list(spec.words) for name, spec in self.classes.items()},
        }

    # ------------------------------------------------------------ 杂项
    def as_dict(self) -> dict[str, Any]:
        return {
            "aisles": list(self.aisles),
            "bays_per_aisle": self.bays_per_aisle,
            "bay_pitch": self.bay_pitch,
            "levels": [lv.__dict__ for lv in self.levels],
            "classes": {n: c.__dict__ for n, c in self.classes.items()},
            "spawn": self.spawn,
        }

    def __repr__(self) -> str:  # noqa: D105
        return (
            f"Catalog(aisles={self.aisles}, bays={self.bays_per_aisle}, "
            f"levels={self.level_numbers()}, classes={len(self.classes)})"
        )


# --------------------------------------------------------------------- 单例
_CACHE: dict[tuple[str, str], Catalog] = {}


def get_catalog(world_path: str | Path | None = None, semantics_path: str | Path | None = None) -> Catalog:
    """进程内单例：整个程序只用一份世界事实。"""
    key = (str(world_path or DEFAULT_WORLD), str(semantics_path or DEFAULT_SEMANTICS))
    if key not in _CACHE:
        _CACHE[key] = Catalog.load(world_path, semantics_path)
    return _CACHE[key]


def reset_catalog_cache() -> None:
    _CACHE.clear()


if __name__ == "__main__":  # 快速自检
    c = get_catalog()
    print(c)
    print("spawn:", c.spawn)
    for u in ("去 B-L-07-2", "带我去 C 巷左边第三层 12 号位", "go to a-r-3",
              "去 10 号位", "去封住的 F 巷", "去 A 巷放冰箱的那面货架"):
        print(f"  {u!r:38s} addr={c.parse_address(u)} aisle={c.parse_aisle(u)} "
              f"bay={c.parse_bay(u)} side={c.parse_side(u)} level={c.parse_level(u)} "
              f"cls={sorted(c.classes_in_text(u))} rel={sorted(c.relations_in_text(u))} "
              f"unknown={c.mentions_unknown_object(u)}")
