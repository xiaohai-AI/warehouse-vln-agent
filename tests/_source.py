"""把源码里的**注释与文档字符串**剔掉，只留可执行代码。

反硬编码 / 反真值依赖这两条守护必须看"代码"，不能看"文档里提到的反例" ——
否则像 `# 原写死 (-31.0, 0.0)` 这种解释性注释会把测试判红，逼人删掉有价值的说明。
"""
from __future__ import annotations

import ast
import io
import tokenize
from pathlib import Path


def docstring_spans(path: Path) -> set[int]:
    """(起始行, 结束行) 覆盖模块/类/函数的 docstring。"""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    spans: set[int] = set()
    for node in ast.walk(tree):
        if not isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        body = getattr(node, "body", [])
        if not body:
            continue
        first = body[0]
        if isinstance(first, ast.Expr) and isinstance(first.value, ast.Constant) and \
                isinstance(first.value.value, str):
            for ln in range(first.lineno, (first.end_lineno or first.lineno) + 1):
                spans.add(ln)
    return spans


def code_lines(path: Path) -> list[tuple[int, str]]:
    """返回 (行号, 代码) —— 已去掉注释行与 docstring 覆盖的行。"""
    text = path.read_text(encoding="utf-8")
    drop = docstring_spans(path)
    for tok in tokenize.generate_tokens(io.StringIO(text).readline):
        if tok.type == tokenize.COMMENT:
            drop.add(tok.start[0])
    return [
        (i, line) for i, line in enumerate(text.splitlines(), 1)
        if i not in drop and line.strip()
    ]
