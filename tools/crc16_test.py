#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Modbus RTU CRC16 单元测试与对拍工具（规格来源：docs/02-modbus-protocol.md §3）。

本脚本做三件事：

1. **测试向量对拍**：把 ``docs/02`` §3.1 表格里的全部测试向量喂进纯 Python 实现的
   ``crc16_modbus()``，逐条断言（报文 → CRC 线上字节序，**先低字节后高字节**）。
2. **整帧自检**：把「报文 + CRC」整帧（docs/02 §6 的实战帧例）送进同一个 CRC 函数，
   结果必须为 ``0x0000``。这是从机接收校验最省事的做法，等价于"重算 CRC 再比较"。
3. **表驱动实现交叉验证**：位运算法（对齐 docs/02 §3 的 C 代码）与查表法互相对拍，
   避免"同一个 bug 写了两遍"这种假通过。

用法::

    python crc16_test.py                 # 跑全部用例，打印通过/失败表格，失败退出码 = 1
    python crc16_test.py --bench 20000   # 额外跑吞吐基准（PC 侧参考值，非 MCU 指标）
    python crc16_test.py -v              # 打印每帧的中间细节
    pytest crc16_test.py -v              # 兼容 pytest（无 pytest 时可脱离它单独运行）
    python crc16_test.py --selftest      # 只做仓库内冒烟自检

退出码：全部通过 = 0；任意一条失败 = 1。
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Iterable, List, Sequence, Tuple

from _console import format_table, safe_print, setup_console

# --------------------------------------------------------------------------------------
# 规格常量（docs/02 §2 / §3）
# --------------------------------------------------------------------------------------

CRC16_MODBUS_POLY = 0xA001  # 反射多项式 x^16 + x^15 + x^2 + 1
CRC16_INIT = 0xFFFF  # 初值，对应 docs/02 §3 的 uint16_t crc = 0xFFFF

#: docs/02 §3.1 测试向量表：报文（不含 CRC） → CRC（线上字节序，先低后高）
#: 每条向量的「线上字节序」字符串均已在独立实现中复核过，可直接当单元测试。
VECTORS: List[Tuple[str, str]] = [
    ("01 03 00 00 00 01", "84 0A"),
    ("01 03 00 00 00 0E", "C4 0E"),
    ("01 03 00 00 00 1C", "44 03"),
    ("01 03 01 04 00 64", "04 1C"),
    ("01 06 00 20 00 01", "49 C0"),
    ("01 06 00 21 03 E8", "D9 7E"),
    ("02 03 00 00 00 02", "C4 38"),
    ("01 03 00 4B 02 00", "34 BC"),  # 数量超限，从机应回异常 03
    ("01 83 02", "C0 F1"),  # 异常响应：非法数据地址
    ("01 83 03", "01 31"),  # 异常响应：非法数据值
]

#: docs/02 §6 实战帧例（**只写"地址+功能码+数据"，CRC 由本脚本按 §3 算法现场追加**）。
#: 之所以不把 CRC 硬编码进字符串：手抄 CRC 极易出错。
#: 本项目实测教训：docs/02 §6 示例② 第 1 行曾把 ``01 03 01 04 00 7D`` 的 CRC 误写为 ``84 17``
#: （正解 ``C5 D6``，错误值其实是 ``01 03 01 00 00 7D`` 即块地址 0x0100 的 CRC —— 抄错了一行）。
#: 该笔误已修正，并已在 F411 侧 ``test/test_modbus.c::test_doc_frames()`` 与 ESP32 侧
#: ``modbus_master.cpp::mb_selftest()`` 两处加了复算，防止文档与代码再次各自漂移。
#: 由 ``append_crc()`` 现场组帧可保证 fixture 永远与算法自洽，同时仍然验证 §3 的向量表。
FRAME_SELFTEST: List[Tuple[str, str]] = [
    ("整帧 · 读单位置 0x0000 ×1", "01 03 00 00 00 01"),
    ("整帧 · 慢车道读 A+B 区 28 个寄存器", "01 03 00 00 00 1C"),
    ("整帧 · 快车道读窗口块 1", "01 03 01 04 00 7D"),
    ("整帧 · 快车道读窗口块 2", "01 03 01 81 00 7D"),
    ("整帧 · 快车道读窗口块 3", "01 03 01 FE 00 7D"),
    ("整帧 · 快车道读窗口块 4", "01 03 02 7B 00 7D"),
    ("整帧 · 快车道读窗口块 5", "01 03 02 F8 00 64"),
    ("整帧 · 写控制字开蜂鸣器 FC06", "01 06 00 20 00 01"),
    ("整帧 · 写风扇 PWM=1000 FC06", "01 06 00 21 03 E8"),
    ("整帧 · 连写控制字+PWM FC10", "01 10 00 20 00 02 04 00 01 03 E8"),
    ("整帧 · 读窗口头 0x0100~0x0103 ×4", "01 03 01 00 00 04"),
    ("整帧 · 读窗口序号 0x0101 ×1", "01 03 01 01 00 01"),
    ("整帧 · 读特征向量 0x0400 ×36", "01 03 04 00 00 24"),
    ("整帧 · 正常读响应（30.0℃ / 60.0%RH）", "01 03 04 01 2C 02 58"),
    ("整帧 · 异常响应 01 83 02（非法数据地址）", "01 83 02"),
    ("整帧 · 异常响应 01 83 03（非法数据值）", "01 83 03"),
    ("整帧 · 异常响应 01 86 02（写只读区）", "01 86 02"),
    ("整帧 · 异常响应 01 85 01（非法功能码）", "01 85 01"),
]

# --------------------------------------------------------------------------------------
# 报文辅助
# --------------------------------------------------------------------------------------


def hex_to_bytes(hex_text: str) -> bytes:
    """把 ``"01 03 00 00"`` 这类空格分隔的十六进制文本转成 ``bytes``。

    :param hex_text: 十六进制报文，允许任意空白分隔，也允许 ``01030000`` 连写。
    :return: 报文对应的字节串。
    :raises ValueError: 文本含非十六进制字符或长度为奇数时。
    """
    cleaned = "".join(hex_text.split()).replace(",", "").replace("0x", "").replace("0X", "")
    if len(cleaned) % 2:
        raise ValueError(f"十六进制文本长度必须为偶数：{hex_text!r}")
    return bytes(int(cleaned[i : i + 2], 16) for i in range(0, len(cleaned), 2))


def bytes_to_hex(data: Iterable[int]) -> str:
    """把字节序列格式化成 ``"01 03 00 00"`` 形式，便于打印与断言信息。"""
    return " ".join(f"{b:02X}" for b in data)


# --------------------------------------------------------------------------------------
# CRC16 实现
# --------------------------------------------------------------------------------------


def crc16_modbus(data: bytes) -> int:
    """位运算法 CRC16/Modbus，与 docs/02 §3 的 C 实现逐行等价。

    对应 C 代码::

        uint16_t crc = 0xFFFF;
        while (len--) { crc ^= *buf++;
            for (i=0;i<8;i++) crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1); }

    :param data: 待计算报文字节（不含 CRC；若含 CRC，正常帧的结果应为 0x0000）。
    :return: 16 位 CRC 值（整数）。发送时应**先低字节后高字节**。
    """
    crc = CRC16_INIT
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ CRC16_MODBUS_POLY if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def _build_crc_table() -> Tuple[int, ...]:
    """预生成 256 项 CRC 查表（每个表项按位运算 8 次），供 ``crc16_modbus_table`` 使用。"""
    table: List[int] = []
    for index in range(256):
        crc = index
        for _ in range(8):
            crc = (crc >> 1) ^ CRC16_MODBUS_POLY if (crc & 1) else (crc >> 1)
        table.append(crc & 0xFFFF)
    return tuple(table)


CRC_TABLE: Tuple[int, ...] = _build_crc_table()


def crc16_modbus_table(data: bytes) -> int:
    """查表法 CRC16/Modbus（与 ``crc16_modbus`` 结果必须完全一致）。

    交叉验证用：MCU 侧上位机若为了省 Flash 用查表法，两套实现必须对得上。
    注意查表法在这里仍是"逐字节反射"写法，表项为 ``crc >> 8`` 的镜像形式。

    :param data: 待计算报文字节。
    :return: 16 位 CRC 值。
    """
    crc = CRC16_INIT
    for byte in data:
        crc = (crc >> 8) ^ CRC_TABLE[(crc ^ byte) & 0xFF]
    return crc & 0xFFFF


def crc_bytes_wire(crc: int) -> bytes:
    """把 CRC 转成**线上字节序**（Modbus RTU：先低字节，后高字节，docs/02 §2）。"""
    crc &= 0xFFFF
    return bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def append_crc(payload: bytes) -> bytes:
    """给报文（地址+功能码+数据）追加 CRC，返回可直接上总线的整帧。"""
    return payload + crc_bytes_wire(crc16_modbus(payload))


def check_frame(frame: bytes) -> bool:
    """整帧接收校验：整帧（含 CRC）送进 CRC 函数，结果为 0x0000 即通过。

    :param frame: 完整 ADU（含末尾 2 字节 CRC）。
    :return: True 表示 CRC 校验通过。
    """
    return len(frame) >= 4 and crc16_modbus(frame) == 0x0000


# --------------------------------------------------------------------------------------
# 断言主干（pytest 与本脚本共用）
# --------------------------------------------------------------------------------------


def iter_vector_cases() -> Iterable[Tuple[str, str]]:
    """产出 docs/02 §3.1 全部测试向量，供 pytest 参数化。"""
    return list(VECTORS)


def vector_wire_crc(hex_payload: str) -> str:
    """给定不含 CRC 的报文文本，返回线上字节序 CRC 文本（如 ``"84 0A"``）。"""
    return bytes_to_hex(crc_bytes_wire(crc16_modbus(hex_to_bytes(hex_payload))))


def assert_vector(hex_payload: str, expected_wire: str) -> None:
    """断言单条测试向量。

    :param hex_payload: 报文（不含 CRC），如 ``"01 03 00 00 00 01"``。
    :param expected_wire: 期望的 CRC 线上字节序，如 ``"84 0A"``。
    :raises AssertionError: 结果与规格不符。
    """
    got = vector_wire_crc(hex_payload)
    assert got == expected_wire, (
        f"CRC 不符：报文 [{hex_payload}] 期望 [{expected_wire}]，实际 [{got}]"
        "（重点检查是否把高低字节写反：线上必须**先低字节后高字节**）"
    )


def frame_hex(hex_payload: str) -> str:
    """把不含 CRC 的报文组装成完整 ADU 十六进制文本（CRC 现场计算）。"""
    return bytes_to_hex(append_crc(hex_to_bytes(hex_payload)))


def assert_frame_selfcheck(hex_payload: str) -> None:
    """断言整帧自检：整帧（含现场计算的 CRC）送进 CRC 函数必须得到 0x0000。

    :param hex_payload: 不含 CRC 的报文十六进制文本。
    :raises AssertionError: 整帧 CRC 不为零。
    """
    frame = append_crc(hex_to_bytes(hex_payload))
    result = crc16_modbus(frame)
    assert result == 0x0000, (
        f"整帧自检失败：{bytes_to_hex(frame)} → 0x{result:04X}（应为 0x0000）"
    )


# --------------------------------------------------------------------------------------
# 运行器（打印通过/失败表格）
# --------------------------------------------------------------------------------------

_STATUS_OK = "PASS"
_STATUS_FAIL = "FAIL"


def _run_case(name: str, detail: str, fn) -> Tuple[str, str, str]:
    """执行单个用例，捕获异常并返回 ``(状态, 名称, 详情)``。"""
    try:
        fn()
    except AssertionError as exc:
        return _STATUS_FAIL, name, str(exc)
    except Exception as exc:  # noqa: BLE001 - 用例内部任何异常都应算失败而非崩掉整个套件
        return _STATUS_FAIL, name, f"{type(exc).__name__}: {exc}"
    return _STATUS_OK, name, detail


def run_all(verbose: bool = False) -> int:
    """跑完全部用例并打印表格。

    :param verbose: 是否打印每条向量的 CRC 计算过程。
    :return: 失败用例数（0 表示全通过）。
    """
    rows: List[Tuple[str, str, str]] = []

    # --- 第 1 组：docs/02 §3.1 测试向量 ---
    for hex_payload, expected in VECTORS:
        detail = f"CRC = {expected}"
        rows.append(
            _run_case(
                f"[向量] {hex_payload}",
                detail,
                lambda p=hex_payload, e=expected: assert_vector(p, e),
            )
        )

    # --- 第 2 组：整帧自检（含 CRC → 0x0000）---
    for name, hex_payload in FRAME_SELFTEST:
        full = frame_hex(hex_payload)
        rows.append(
            _run_case(
                f"[整帧] {name}",
                f"{full} → 0x0000",
                lambda p=hex_payload: assert_frame_selfcheck(p),
            )
        )

    # --- 第 3 组：位运算法 vs 查表法 交叉验证 ---
    cross_payloads = [hex_to_bytes(p) for p, _ in VECTORS]
    cross_payloads.append(bytes(range(256)))  # 0x00~0xFF 全字节覆盖
    cross_payloads.append(append_crc(hex_to_bytes("01 03 00 00 00 1C")))
    for idx, payload in enumerate(cross_payloads):
        label = "全字节 0x00~0xFF" if len(payload) == 256 else bytes_to_hex(payload)
        rows.append(
            _run_case(
                f"[交叉] 位运算 vs 查表 #{idx} ({label[:40]}{'...' if len(label) > 40 else ''})",
                "两种实现结果一致",
                lambda p=payload: _assert_cross(p),
            )
        )

    # --- 第 4 组：边界与性质 ---
    rows.append(
        _run_case(
            "[性质] 空报文 CRC == 初值 0xFFFF",
            "空报文 = 0xFFFF",
            lambda: _assert_true(crc16_modbus(b"") == CRC16_INIT, "空报文 CRC 应等于初值 0xFFFF"),
        )
    )
    rows.append(
        _run_case(
            "[性质] append_crc 后 check_frame 为真",
            "组帧/校验闭环",
            lambda: _assert_true(
                check_frame(append_crc(hex_to_bytes("01 03 02 F8 00 64"))),
                "append_crc() 组出的帧未能通过 check_frame()",
            ),
        )
    )
    rows.append(
        _run_case(
            "[性质] 单比特翻转必须被检出",
            f"{len(FRAME_SELFTEST)} 条帧全部检出",
            _assert_bitflip_detected,
        )
    )
    rows.append(
        _run_case(
            "[性质] 线上字节序为先低后高",
            "低字节在前",
            _assert_wire_order,
        )
    )

    # --- 打印表格 ---
    table_rows = [[status, name, detail] for status, name, detail in rows]
    print("=" * 96)
    print("Modbus RTU CRC16 测试向量对拍（规格：docs/02-modbus-protocol.md §3 / §6）")
    print("=" * 96)
    print(format_table(["状态", "用例", "说明"], table_rows))
    print("-" * 96)

    failed = [r for r in rows if r[0] == _STATUS_FAIL]
    print(
        f"合计 {len(rows)} 条：通过 {len(rows) - len(failed)}，失败 {len(failed)}"
        f"（测试向量 {len(VECTORS)} 条 / 整帧自检 {len(FRAME_SELFTEST)} 条）"
    )
    if verbose:
        print("\n--- 明细 ---")
        for hex_payload, expected in VECTORS:
            raw = crc16_modbus(hex_to_bytes(hex_payload))
            print(
                f"  {hex_payload:<24} crc=0x{raw:04X}  "
                f"线上={bytes_to_hex(crc_bytes_wire(raw))}  期望={expected}"
            )
        print("\n--- docs/02 §6 实战帧（CRC 现场计算）---")
        for name, hex_payload in FRAME_SELFTEST:
            print(f"  {name:<44} {frame_hex(hex_payload)}")
    if failed:
        print("\n失败明细：")
        for status, name, detail in failed:
            print(f"  [{status}] {name}\n        {detail}")
        print("\n排查提示：CRC 高低字节写反是最常见错误，线上顺序必须是 低字节 → 高字节。")
    else:
        print("全部通过：CRC16 实现可用于 F411 从机与 PC 主机两侧对拍。")
    return len(failed)


def _assert_true(condition: bool, message: str) -> None:
    """轻量断言：条件为假时抛 ``AssertionError``。"""
    if not condition:
        raise AssertionError(message)


def _assert_cross(payload: bytes) -> None:
    """断言位运算法与查表法对同一报文给出相同 CRC。"""
    a = crc16_modbus(payload)
    b = crc16_modbus_table(payload)
    _assert_true(a == b, f"位运算 0x{a:04X} != 查表 0x{b:04X}（报文 {bytes_to_hex(payload)}）")


def _assert_bitflip_detected() -> None:
    """对每条实战帧逐位翻转，确认 CRC 校验都能检出（且不会偶发通过）。"""
    for name, hex_payload in FRAME_SELFTEST:
        frame = bytearray(append_crc(hex_to_bytes(hex_payload)))
        for byte_index in range(len(frame)):
            for bit in range(8):
                frame[byte_index] ^= 1 << bit
                if check_frame(bytes(frame)):
                    raise AssertionError(
                        f"帧 [{name}] 第 {byte_index} 字节 bit{bit} 翻转后仍通过 CRC 校验"
                    )
                frame[byte_index] ^= 1 << bit


def _assert_wire_order() -> None:
    """断言线上字节序：低字节在前。以 01 03 00 00 00 01 → 84 0A 为例。"""
    frame = append_crc(hex_to_bytes("01 03 00 00 00 01"))
    _assert_true(len(frame) == 8, f"帧长应为 8，实际 {len(frame)}")
    _assert_true(
        frame[-2] == 0x84 and frame[-1] == 0x0A,
        f"线上字节序错误：得到 {bytes_to_hex(frame[-2:])}，应为 84 0A（先低字节后高字节）",
    )


def bench(iterations: int, payload: bytes = b"\x01\x03\x01\x04\x00\x7d") -> None:
    """跑一次吞吐基准（仅作 PC 侧参考，不代表 MCU 性能）。

    :param iterations: 重复次数。
    :param payload: 基准用的报文。
    """
    for func, label in ((crc16_modbus, "位运算法"), (crc16_modbus_table, "查表法")):
        start = time.perf_counter()
        for _ in range(iterations):
            func(payload)
        elapsed = time.perf_counter() - start
        per_call = elapsed / iterations * 1e6
        safe_print(
            f"{label}：{iterations} 次耗时 {elapsed*1000:.1f} ms，"
            f"单次 {per_call:.2f} us（{len(payload)} 字节报文）"
        )


# --------------------------------------------------------------------------------------
# pytest 兼容层
# --------------------------------------------------------------------------------------


def test_crc_vectors() -> None:
    """pytest：docs/02 §3.1 全部测试向量。"""
    failures = []
    for hex_payload, expected in VECTORS:
        got = vector_wire_crc(hex_payload)
        if got != expected:
            failures.append(f"{hex_payload}: 期望 {expected}，实际 {got}")
    assert not failures, "CRC 测试向量不符：" + "; ".join(failures)


def test_frame_selfcheck_zero() -> None:
    """pytest：整帧（含 CRC）送进 CRC 函数结果应为 0x0000。"""
    failures = []
    for name, hex_payload in FRAME_SELFTEST:
        result = crc16_modbus(append_crc(hex_to_bytes(hex_payload)))
        if result != 0x0000:
            failures.append(f"{name}: 0x{result:04X}")
    assert not failures, "整帧自检失败：" + "; ".join(failures)


def test_bitflip_detected() -> None:
    """pytest：单比特翻转必须全部被 CRC 检出。"""
    _assert_bitflip_detected()


def test_wire_byte_order() -> None:
    """pytest：CRC 线上字节序为先低字节后高字节。"""
    _assert_wire_order()


def test_table_matches_bitwise() -> None:
    """pytest：查表法与位运算法结果一致。"""
    for payload in (bytes(range(256)), b"", b"\x01\x03\x00\x00\x00\x1C"):
        _assert_cross(payload)


# --------------------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------------------


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(
        prog="crc16_test.py",
        description="Modbus RTU CRC16 测试向量对拍（docs/02 §3.1 全量向量 + 整帧 0x0000 自检）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "示例：\n"
            "  python crc16_test.py                 # 全量对拍，失败退出码 1\n"
            "  python crc16_test.py -v              # 附带每帧中间值\n"
            "  python crc16_test.py --bench 50000   # 吞吐基准\n"
            "  pytest crc16_test.py -v              # pytest 方式运行\n"
        ),
    )
    parser.add_argument("-v", "--verbose", action="store_true", help="打印每条向量的 CRC 中间值")
    parser.add_argument(
        "--bench",
        type=int,
        default=0,
        metavar="N",
        help="额外跑 N 次 CRC 吞吐基准（0 = 不跑）",
    )
    parser.add_argument(
        "--selftest",
        action="store_true",
        help="仓库内冒烟自检（等价于默认行为，便于 CI 统一入口）",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """脚本入口：打印通过/失败表格并返回退出码。"""
    setup_console()
    args = parse_args(argv)
    failed = run_all(verbose=args.verbose)
    if args.bench > 0:
        print()
        bench(args.bench)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
