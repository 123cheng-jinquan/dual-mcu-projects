#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""控制台输出辅助模块（tools/ 内脚本共用）。

解决的问题
----------
Windows 中文控制台默认代码页是 GBK（cp936）：

* 打印 ``µ``（微秒）、``≥``、``→`` 等字符会抛 ``UnicodeEncodeError`` 直接崩脚本；
* 中文在部分终端显示为乱码。

本模块在**不改变已有环境设置**的前提下，把 ``stdout``/``stderr`` 重新配置为
UTF-8 编码并对无法编码的字符做替换，从而使所有 tools/ 脚本都有稳定的中文输出。

用法::

    from _console import setup_console, safe_print
    setup_console()          # 在 main() 最开始调用一次
    safe_print("中文输出 µs")  # 兜底：即使 setup 失败也不会崩

注意：本模块不依赖任何第三方库，可被同目录脚本以 ``import _console`` 直接导入
（脚本自身目录总在 ``sys.path[0]``）。
"""

from __future__ import annotations

import sys

_SETUP_DONE = False


def setup_console() -> None:
    """把标准输出/错误切到 UTF-8 并对不可编码字符做替换（幂等，可重复调用）。

    只调用 ``reconfigure``，因此不会丢弃已经重定向到文件/管道的输出。
    若运行环境不支持（例如被替换成无 ``reconfigure`` 的对象），静默跳过。
    """
    global _SETUP_DONE
    if _SETUP_DONE:
        return
    for stream in (sys.stdout, sys.stderr):
        reconfigure = getattr(stream, "reconfigure", None)
        if reconfigure is None:
            continue
        try:
            reconfigure(encoding="utf-8", errors="replace")
        except (ValueError, OSError):
            # 流已被关闭或不可重配置：忽略，safe_print 仍会兜底
            pass
    _SETUP_DONE = True


def safe_print(*args, **kwargs) -> None:
    """打印兜底：遇到 ``UnicodeEncodeError`` 时降级为 ASCII 近似输出。

    与 ``print`` 签名一致，额外吞掉编码类异常，保证工具脚本不会因终端编码崩掉。
    """
    try:
        print(*args, **kwargs)
    except UnicodeEncodeError:
        encoding = getattr(sys.stdout, "encoding", None) or "ascii"
        fallback = " ".join(str(a) for a in args)
        print(fallback.encode(encoding, errors="replace").decode(encoding, errors="replace"), **kwargs)


def format_table(headers, rows, aligns=None) -> str:
    """把二维数据渲染成等宽文本表格（中文按 2 列宽估算，避免错位）。

    :param headers: 表头序列。
    :param rows: 行序列，每行元素数量应与表头一致。
    :param aligns: 每列对齐方式，``"<"`` 左对齐 / ``">"`` 右对齐，默认全部左对齐。
    :return: 多行字符串（不含末尾换行）。
    """
    headers = [str(h) for h in headers]
    rows = [[str(c) for c in row] for row in rows]
    aligns = list(aligns) if aligns else ["<"] * len(headers)

    def width(text: str) -> int:
        """按东亚全角字符算 2 列宽估算显示宽度。"""
        return sum(2 if _is_wide(ch) else 1 for ch in text)

    col_widths = [width(h) for h in headers]
    for row in rows:
        for idx, cell in enumerate(row):
            col_widths[idx] = max(col_widths[idx], width(cell))

    def pad(text: str, target: int, align: str) -> str:
        """按显示宽度补齐空格。"""
        return text + " " * (target - width(text)) if align == "<" else " " * (target - width(text)) + text

    lines = ["  ".join(pad(h, col_widths[i], aligns[i]) for i, h in enumerate(headers))]
    lines.append("  ".join("-" * w for w in col_widths))
    for row in rows:
        lines.append("  ".join(pad(c, col_widths[i], aligns[i]) for i, c in enumerate(row)))
    return "\n".join(lines)


def _is_wide(ch: str) -> bool:
    """粗略判断字符是否为东亚全角（用于表格列宽估算）。"""
    code = ord(ch)
    return (
        0x1100 <= code <= 0x115F
        or 0x2E80 <= code <= 0xA4CF
        or 0xAC00 <= code <= 0xD7A3
        or 0xF900 <= code <= 0xFAFF
        or 0xFE30 <= code <= 0xFE6F
        or 0xFF00 <= code <= 0xFF60
        or 0xFFE0 <= code <= 0xFFE6
        or 0x20000 <= code <= 0x3FFFD
    )
