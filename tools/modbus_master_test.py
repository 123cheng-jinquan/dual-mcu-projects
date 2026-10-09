#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PC 当 Modbus RTU **主机**，用 pymodbus 3.x 测试真实的 STM32F411 从机。

覆盖 ``docs/02-modbus-protocol.md`` §10 的测试用例清单：

======================  ==========================================================
用例                    期望行为（规格依据）
======================  ==========================================================
读 A/B 区               28 个寄存器回读成功，字段可解（§5 A/B 区、§6 示例①）
边界 · 读 125 个        成功（§4 数量上限 125）
边界 · 读 126 个        异常 03（数量越界，§6 示例⑧）
越界 · 起始 0x0360      异常 02（§10）
写只读区（FC06→0x0000） 异常 02（§6 示例⑧）
非法功能码 0x05         异常 01（§4）
非法值 · 波特率代码 9   异常 03（§5 C 区 0x002D 合法范围 0~4）
非法值 · 从机地址 0/248 异常 03（§5 C 区 0x002C 合法范围 1~247）
CRC 错注入              从机**静默**（§8：CRC 失败静默丢弃，不回异常）
错误从机地址            静默（§2 / §8：地址不匹配必须静默，否则总线冲突）
广播地址 0x00           执行但不响应（§2）
连续 N 帧压力测试       成功率统计 + 从机计数器 `0x0013~0x0015` 增量核对（§10）
帧间隔 > 3.5T           应被识别为两帧（§1）→ 用 `--raw-tests` 打开
粘帧（两帧无间隔）       至少不能死机（§10）→ 用 `--raw-tests` 打开
窗口序号一致性          5 块分读期间 `0x0101` 不变化（§7）→ 用 `--raw-tests` 打开
======================  ==========================================================

用法示例::

    python modbus_master_test.py --port COM5                      # 96,地址 1,全部用例
    python modbus_master_test.py --port COM5 --baud 115200 -v
    python modbus_master_test.py --port COM5 --cases basic,error
    python modbus_master_test.py --port COM5 --raw-tests --raw-port COM5
    python modbus_master_test.py --port COM5 --count 10000 --include-risky

**关于危险用例**：`0x002C`（从机地址）与 `0x002D`（波特率）写入后会立即生效，
会把总线"打断"。默认跳过，必须显式加 ``--include-risky`` 才执行，且脚本会在
用例结束后尝试按旧参数重连、并把寄存器写回原值。

pymodbus 版本差异（重点）
-------------------------
* **3.x**：``ModbusSerialClient(port=..., framer=FramerType.RTU, baudrate=..., timeout=...,
  parity=..., stopbits=..., bytesize=...)``；读请求是
  ``client.read_holding_registers(address, count=..., slave=...)``（**slave** 关键字）。
* **3.7 起**：``FramerType`` 从 ``pymodbus.framer`` 导入；
  ``read_holding_registers`` 的从机号关键字允许用 ``device_id``（``slave`` 仍兼容）。
* **2.x（已废弃）**：``client.read_holding_registers(address, count, unit=...)``，
  串口参数直接写在一个 ``method="rtu"`` 的构造里，``framer`` 叫 ``Framer``/``ModbusRtuFramer``。
  本脚本按 3.x 编写，并对 ``unit=`` / ``slave=`` / ``device_id=`` 做了兼容回退。

退出码：全部用例通过（含 SKIP 不算失败）= 0；有 FAIL = 1；串口打不开 = 2。
"""

from __future__ import annotations

import argparse
import importlib
import statistics
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from _console import format_table, safe_print, setup_console

# 允许从任意工作目录执行本脚本：把脚本自身目录补进 sys.path，保证能 import 同目录模块
_TOOLS_DIR = Path(__file__).resolve().parent
if str(_TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(_TOOLS_DIR))

# --------------------------------------------------------------------------------------
# 规格常量：地址一律取自 docs/02 §5 的十六进制映射
# --------------------------------------------------------------------------------------

SLAVE_ADDR_DEFAULT = 0x01

# A 区 环境量（只读）
REG_A_TEMP = 0x0000
REG_A_SAMPLE_SEQ = 0x000F
# B 区 状态与统计（只读）
REG_B_STATUS1 = 0x0010
REG_B_FW_VERSION = 0x0012
REG_B_OK_FRAMES = 0x0013
REG_B_CRC_ERR = 0x0014
REG_B_EXC_FRAMES = 0x0015
REG_B_UPTIME_MIN = 0x0016
REG_B_RESET_REASON = 0x0017
REG_B_WD_TIMEOUTS = 0x0018
# C 区 控制与配置（读写）
REG_C_CTRL1 = 0x0020
REG_C_FAN_PWM = 0x0021
REG_C_SLAVE_ADDR = 0x002C
REG_C_BAUD_CODE = 0x002D
# D 区 六轴窗口（只读）
REG_D_HEAD = 0x0100
REG_D_SEQ = 0x0101
REG_D_RATE = 0x0102
REG_D_FIRST_SAMPLE = 0x0104
REG_D_LAST = 0x035B
# E 区 特征向量（只读）
REG_E_FEATURES = 0x0400
REG_E_LAST = 0x0423

SLOW_LANE_START = 0x0000  # 慢车道：A 区 + B 区
SLOW_LANE_COUNT = 28  # 0x0000~0x001B，docs/02 §6 示例① / §9
MAX_READ_COUNT = 125  # §4：FC03/FC04 单次上限
WINDOW_CHUNKS: List[Tuple[int, int]] = [  # docs/02 §6 示例② 的 5 次分块
    (0x0104, 125),
    (0x0181, 125),
    (0x01FE, 125),
    (0x027B, 125),
    (0x02F8, 100),
]

ILLEGAL_FUNCTION = 0x01
ILLEGAL_ADDRESS = 0x02
ILLEGAL_VALUE = 0x03

STATUS_PASS = "PASS"
STATUS_FAIL = "FAIL"
STATUS_SKIP = "SKIP"
STATUS_WARN = "WARN"


# --------------------------------------------------------------------------------------
# 结果记录
# --------------------------------------------------------------------------------------


@dataclass
class CaseResult:
    """单个测试用例的结果。"""

    name: str
    status: str
    detail: str = ""
    elapsed_ms: float = 0.0


@dataclass
class TestReport:
    """全部用例的结果汇总。"""

    results: List[CaseResult] = field(default_factory=list)

    def add(self, name: str, status: str, detail: str = "", elapsed_ms: float = 0.0) -> CaseResult:
        """记录一条用例结果并返回它。"""
        result = CaseResult(name=name, status=status, detail=detail, elapsed_ms=elapsed_ms)
        self.results.append(result)
        return result

    def run_case(self, name: str, func: Callable[[], Tuple[str, str]]) -> CaseResult:
        """执行一个用例函数，捕获异常并统一样式记录。

        :param name: 用例名。
        :param func: 无参函数，返回 ``(状态, 说明)``；也可只返回说明字符串（视为 PASS）。
        :return: 记录下来的用例结果。
        """
        start = time.perf_counter()
        try:
            outcome = func()
        except KeyboardInterrupt:
            raise
        except Exception as exc:  # noqa: BLE001 - 单个用例异常不应中断整套测试
            elapsed = (time.perf_counter() - start) * 1000
            return self.add(name, STATUS_FAIL, f"{type(exc).__name__}: {exc}", elapsed)
        elapsed = (time.perf_counter() - start) * 1000
        status, detail = outcome if isinstance(outcome, tuple) else (STATUS_PASS, str(outcome))
        return self.add(name, status, detail, elapsed)

    @property
    def failed(self) -> List[CaseResult]:
        """失败的用例列表。"""
        return [r for r in self.results if r.status == STATUS_FAIL]

    def counts(self) -> Dict[str, int]:
        """各状态计数。"""
        counter = {STATUS_PASS: 0, STATUS_FAIL: 0, STATUS_SKIP: 0, STATUS_WARN: 0}
        for result in self.results:
            counter[result.status] = counter.get(result.status, 0) + 1
        return counter

    def dump(self) -> None:
        """打印逐用例结果与汇总表。"""
        safe_print("\n" + "=" * 100)
        safe_print("用例明细（状态 / 用例 / 耗时 ms / 说明）")
        safe_print("=" * 100)
        rows = [
            [r.status, r.name, f"{r.elapsed_ms:.1f}", r.detail]
            for r in self.results
        ]
        safe_print(format_table(["状态", "用例", "耗时ms", "说明"], rows, [">", "<", ">", "<"]))

        counts = self.counts()
        safe_print("\n" + "=" * 100)
        safe_print("汇总表")
        safe_print("=" * 100)
        summary = [[k, str(v)] for k, v in counts.items() if v]
        safe_print(format_table(["状态", "数量"], summary, ["<", ">"]))
        total_elapsed = sum(r.elapsed_ms for r in self.results)
        safe_print(
            f"用例总数 {len(self.results)}，总耗时 {total_elapsed/1000:.2f} s，"
            f"通过 {counts.get(STATUS_PASS, 0)} / 失败 {counts.get(STATUS_FAIL, 0)} / "
            f"跳过 {counts.get(STATUS_SKIP, 0)}"
        )
        if self.failed:
            safe_print("\n失败用例：")
            for result in self.failed:
                safe_print(f"  [FAIL] {result.name}\n         {result.detail}")


# --------------------------------------------------------------------------------------
# 串口 / pymodbus 兼容层
# --------------------------------------------------------------------------------------


def import_pymodbus():
    """导入 pymodbus 3.x 组件，缺失时给出明确的中文安装提示。

    :return: ``(client_cls, framer_type, exceptions_module)``。
    :raises SystemExit: 未安装 pymodbus 时（退出码 2）。
    """
    try:
        from pymodbus.client import ModbusSerialClient  # pymodbus 3.x
    except ImportError as exc:  # pragma: no cover - 取决于运行环境
        safe_print(
            "错误：未安装 pymodbus 3.x。\n"
            "  请执行：pip install -r tools/requirements.txt\n"
            "  （只装 pymodbus 也可以：pip install \"pymodbus>=3.6\"）"
        )
        raise SystemExit(2) from exc

    framer_type = None
    try:
        from pymodbus.framer import FramerType  # pymodbus >= 3.7 的推荐路径

        framer_type = FramerType.RTU
    except ImportError:
        for module_name, attr in (
            ("pymodbus.framer.rtu_framer", "ModbusRtuFramer"),
            ("pymodbus.framer", "ModbusRtuFramer"),
            ("pymodbus.transaction", "ModbusRtuFramer"),
        ):
            try:
                module = __import__(module_name, fromlist=[attr])
                framer_type = getattr(module, attr)
                break
            except (ImportError, AttributeError):
                continue

    try:
        from pymodbus import exceptions as pymodbus_exceptions
    except ImportError:  # pragma: no cover
        pymodbus_exceptions = None
    return ModbusSerialClient, framer_type, pymodbus_exceptions


def pymodbus_version() -> str:
    """返回 pymodbus 版本字符串（取不到时返回 ``unknown``）。"""
    try:
        from pymodbus import __version__ as version

        return str(version)
    except Exception:  # noqa: BLE001
        return "unknown"


def build_client(args: argparse.Namespace):
    """按 3.x 写法创建 RTU 串口客户端；串口打不开时给出中文提示而不是抛栈。

    :param args: 命令行参数（需含 port/baud/parity/stopbits/bytesize/timeout）。
    :return: 已连接的 ``ModbusSerialClient``。
    :raises SystemExit: 串口打开失败（退出码 2）。
    """
    client_cls, framer_type, _ = import_pymodbus()
    kwargs = dict(
        port=args.port,
        baudrate=args.baud,
        bytesize=8,
        parity=args.parity.upper()[0],
        stopbits=args.stopbits,
        timeout=args.timeout,
        retries=0,  # 重试由本脚本显式控制，避免把"静默"用例误判成有响应
    )
    if framer_type is not None:
        kwargs["framer"] = framer_type  # 3.6+ 用 framer=；2.x 无此参数（见模块 docstring）

    try:
        client = client_cls(**kwargs)
    except TypeError:
        # 极老版本不接受 framer=：退回不带 framer 的构造
        kwargs.pop("framer", None)
        client = client_cls(**kwargs)

    if not client.connect():
        safe_print(
            f"\n错误：串口打不开 —— {args.port} @ {args.baud} 8{kwargs['parity']}{args.stopbits}\n"
            "  请依次检查：\n"
            "    1) USB 转 RS485 驱动是否装好（CH340/CP210x/FT232 驱动），设备管理器里有没有黄色感叹号\n"
            "    2) 端口号是否正确（当前传入的是 %s；Windows 常见 COM3~COM15，可用 --list-ports 查看）\n"
            "    3) 端口是否被 **Modbus Poll / Modbus Slave / 串口助手 / 其它 Python 进程** 占用 —— 同一端口同一时刻只能被一个程序打开\n"
            "    4) 485 接线：A→A、B→B、**必须共地**；调试期波特率先用 9600\n"
            "    5) 是否用了板载 USB 直连（那是 USART1 调试口，不是 485 口）\n"
            % args.port
        )
        raise SystemExit(2)
    return client


def list_serial_ports() -> None:
    """列出可用串口（依赖 pyserial；失败时给出中文提示）。"""
    try:
        from serial.tools import list_ports
    except ImportError:
        safe_print("未安装 pyserial，无法枚举串口。请执行：pip install pyserial")
        return
    ports = list(list_ports.comports())
    if not ports:
        safe_print("未发现任何串口设备。请检查 USB 转 RS485 是否插好、驱动是否安装。")
        return
    safe_print(format_table(["端口", "描述", "硬件 ID"], [[p.device, p.description, p.hwid] for p in ports]))


def _call_with_unit(func: Callable, *args, unit: int, **kwargs):
    """兼容 pymodbus 3.x 的从机号关键字：依次尝试 slave / device_id / unit。

    :param func: 例如 ``client.read_holding_registers``。
    :param unit: 从机地址。
    :return: 请求响应对象。
    """
    last_error: Optional[Exception] = None
    for keyword in ("slave", "device_id", "unit"):
        try:
            return func(*args, **kwargs, **{keyword: unit})
        except TypeError as exc:  # 该版本不支持这个关键字
            last_error = exc
            continue
    raise last_error if last_error else RuntimeError("无法确定 pymodbus 从机号关键字")


def _exception_code(response) -> Optional[int]:
    """从 pymodbus 响应里取异常码；不是异常响应则返回 None。

    覆盖 3.x 的 ``response.exception_code`` 与更早的 ``response.code``。
    """
    if response is None:
        return None
    if getattr(response, "isError", None) and response.isError():
        code = getattr(response, "exception_code", None)
        if code is None:
            code = getattr(response, "code", None)
        return int(code) if code is not None else -1
    return None


def _describe(response) -> str:
    """把响应对象描述成"异常码/寄存器列表"形式，便于打印。"""
    code = _exception_code(response)
    if code is not None:
        return f"异常码 0x{code:02X}"
    registers = getattr(response, "registers", None)
    if registers is not None:
        preview = " ".join(f"{r:04X}" for r in registers[:8])
        suffix = " ..." if len(registers) > 8 else ""
        return f"{len(registers)} 个寄存器 [{preview}{suffix}]"
    return f"响应 {response!r}"


class MasterClient:
    """对 pymodbus 客户端的薄封装，统一从机地址关键字与原始串口访问。"""

    def __init__(self, client, unit: int, timeout: float):
        """:param client: 已连接的 pymodbus 客户端。:param unit: 默认从机地址。"""
        self.client = client
        self.unit = unit
        self.timeout = timeout

    # --- 高层 API ---

    def read_holding(self, address: int, count: int, unit: Optional[int] = None):
        """FC03 读保持寄存器。"""
        return _call_with_unit(
            self.client.read_holding_registers, address, count=count, unit=unit or self.unit
        )

    def read_input(self, address: int, count: int, unit: Optional[int] = None):
        """FC04 读输入寄存器（F411 侧可复用 FC03 实现，docs/02 §4）。"""
        return _call_with_unit(
            self.client.read_input_registers, address, count=count, unit=unit or self.unit
        )

    def write_single(self, address: int, value: int, unit: Optional[int] = None):
        """FC06 写单个寄存器。"""
        return _call_with_unit(
            self.client.write_register, address, value, unit=unit or self.unit
        )

    def write_multiple(self, address: int, values: Sequence[int], unit: Optional[int] = None):
        """FC10 写多个寄存器。"""
        return _call_with_unit(
            self.client.write_registers, address, list(values), unit=unit or self.unit
        )

    # --- 原始收发（用于 CRC 错注入 / 非法功能码 / 帧间隔 / 粘帧）---

    def send_raw(self, frame: bytes) -> None:
        """直接向串口写原始字节（绕过 pymodbus 组帧与校验）。

        3.x 的 ``ModbusSerialClient.socket`` 就是内部 pyserial 的 ``Serial`` 对象；
        不同小版本属性名略有差异，这里做多路回退。
        """
        serial_port = (
            getattr(self.client, "socket", None)
            or getattr(self.client, "serial", None)
            or getattr(getattr(self.client, "transport", None), "socket", None)
        )
        if serial_port is None or not hasattr(serial_port, "write"):
            raise RuntimeError(
                "无法取得底层串口对象（pymodbus 版本差异）。"
                "请对本用例改用 --raw-port 走独立 pyserial 通道，例如：--raw-tests --raw-port COM5"
            )
        serial_port.write(frame)
        serial_port.flush()

    def recv_raw(self, expected_len: int = 256, wait_s: Optional[float] = None) -> bytes:
        """读取原始字节，直到超时；返回读到的全部内容（可能为空 = 从机静默）。"""
        serial_port = (
            getattr(self.client, "socket", None)
            or getattr(self.client, "serial", None)
            or getattr(getattr(self.client, "transport", None), "socket", None)
        )
        if serial_port is None or not hasattr(serial_port, "read"):
            raise RuntimeError("无法取得底层串口对象，请改用 --raw-port 走独立 pyserial 通道")
        deadline = time.time() + (wait_s if wait_s is not None else self.timeout)
        buffer = bytearray()
        while time.time() < deadline and len(buffer) < expected_len:
            chunk = serial_port.read(max(1, expected_len - len(buffer)))
            if chunk:
                buffer.extend(chunk)
        return bytes(buffer)


class RawSerial:
    """独立 pyserial 通道：用于注入坏帧 / 粘帧 / 帧间隔测试。

    与 pymodbus 客户端**不能同时打开同一端口**，所以要么让 pymodbus 客户端用完
    后关闭再开本通道，要么另接一个 USB-485。脚本通过 ``--raw-port`` 区分。
    """

    def __init__(self, port: str, baud: int, parity: str, stopbits: int, timeout: float):
        """:param port: 串口号。:param baud: 波特率。:param timeout: 读超时秒。"""
        try:
            import serial  # pyserial
        except ImportError as exc:
            safe_print("错误：未安装 pyserial，原始帧用例不可用。请执行：pip install pyserial")
            raise SystemExit(2) from exc
        self.serial_module = serial
        try:
            self.port = serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=8,
                parity=parity.upper()[0],
                stopbits=stopbits,
                timeout=timeout,
            )
        except Exception as exc:  # noqa: BLE001 - pyserial 抛的异常类型很多
            safe_print(
                f"\n错误：串口打不开 —— {port}（{exc}）\n"
                "  请检查：\n"
                "    1) USB 转 RS485 驱动（CH340/CP210x/FT232）是否安装、设备管理器是否报错\n"
                "    2) 端口号是否正确，可用 --list-ports 查看\n"
                "    3) 端口是否被 **Modbus Poll / 串口助手 / 正在运行的 pymodbus 客户端** 占用\n"
                "    4) 若与主客户端是同一个口，请让主客户端用完释放（本脚本会先关闭再打开）\n"
            )
            raise SystemExit(2) from exc

    def send(self, frame: bytes, gap_before_s: float = 0.0, gap_after_s: float = 0.0) -> None:
        """发送原始帧，可在前后插入自定义静默间隔（用于帧间隔/粘帧用例）。"""
        if gap_before_s > 0:
            time.sleep(gap_before_s)
        self.port.write(frame)
        self.port.flush()
        if gap_after_s > 0:
            time.sleep(gap_after_s)

    def recv(self, wait_s: float, max_len: int = 256) -> bytes:
        """在 ``wait_s`` 内收集字节，返回读到的内容。"""
        deadline = time.time() + wait_s
        buffer = bytearray()
        while time.time() < deadline and len(buffer) < max_len:
            chunk = self.port.read(max(1, max_len - len(buffer)))
            if chunk:
                buffer.extend(chunk)
        return bytes(buffer)

    def close(self) -> None:
        """关闭串口。"""
        try:
            self.port.close()
        except Exception:  # noqa: BLE001
            pass


def rtu_frame(payload: bytes) -> bytes:
    """按 docs/02 §2/§3 给"地址+功能码+数据"追加 CRC，得到可上总线的整帧。

    复用 ``crc16_test.py`` 里已经过 §3.1 全量向量验证的实现，避免两处算法不一致。
    """
    crc_module = importlib.import_module("crc16_test")
    return crc_module.append_crc(payload)


def corrupt_crc(frame: bytes) -> bytes:
    """把整帧的 CRC 两个字节各翻转 1 位，模拟传输误码（CRC 错注入）。"""
    if len(frame) < 4:
        raise ValueError("帧太短，无法翻转 CRC")
    return frame[:-2] + bytes((frame[-2] ^ 0x01, frame[-1] ^ 0x01))


# --------------------------------------------------------------------------------------
# 用例实现
# --------------------------------------------------------------------------------------


def case_read_slow_lane(client: MasterClient, args: argparse.Namespace, report: TestReport) -> None:
    """用例组：正常读 A 区 + B 区。"""

    def _read_ab() -> Tuple[str, str]:
        """一次读 0x0000~0x001B 共 28 个寄存器，并按 docs/02 §5 解码关键字段。"""
        response = client.read_holding(SLOW_LANE_START, SLOW_LANE_COUNT)
        code = _exception_code(response)
        if code is not None:
            return STATUS_FAIL, f"期望读到 {SLOW_LANE_COUNT} 个寄存器，实际 {_describe(response)}"
        regs = list(response.registers)
        if len(regs) != SLOW_LANE_COUNT:
            return STATUS_FAIL, f"返回数量错误：{len(regs)}（应为 {SLOW_LANE_COUNT}）"

        def as_signed(value: int) -> int:
            """uint16 → int16（docs/02 §5 A 区温度为 int16）。"""
            return value - 0x10000 if value & 0x8000 else value

        temperature = as_signed(regs[REG_A_TEMP]) / 10.0
        humidity = regs[0x0001] / 10.0
        pressure = regs[0x0002] / 10.0
        detail = (
            f"温度 {temperature:.1f}℃ 湿度 {humidity:.1f}%RH 气压 {pressure:.1f}hPa "
            f"光照 {regs[0x0003]} 采样序号 {regs[REG_A_SAMPLE_SEQ - REG_A_TEMP]} "
            f"状态字1 0x{regs[REG_B_STATUS1 - REG_A_TEMP]:04X} "
            f"固件 0x{regs[REG_B_FW_VERSION - REG_A_TEMP]:04X} "
            f"正常帧 {regs[REG_B_OK_FRAMES - REG_A_TEMP]} "
            f"CRC错 {regs[REG_B_CRC_ERR - REG_A_TEMP]} "
            f"运行 {regs[REG_B_UPTIME_MIN - REG_A_TEMP]} 分钟"
        )
        return STATUS_PASS, detail

    report.run_case("正常读 A/B 区（0x0000~0x001B，28 个寄存器）", _read_ab)

    def _sample_seq_increments() -> Tuple[str, str]:
        """连续两次读 0x000F 采样序号：应递增（除非刚好回绕）。"""
        first = client.read_holding(REG_A_SAMPLE_SEQ, 1)
        if _exception_code(first) is not None:
            return STATUS_FAIL, f"读 0x000F 失败：{_describe(first)}"
        first_value = first.registers[0]
        # A 区采样周期按 docs/02 §5 C 区 0x002E 配置（典型 500ms），等 1.2s 保证跨过至少一轮
        time.sleep(args.slow_lane_wait)
        second = client.read_holding(REG_A_SAMPLE_SEQ, 1)
        if _exception_code(second) is not None:
            return STATUS_FAIL, f"读 0x000F 失败：{_describe(second)}"
        second_value = second.registers[0]
        if second_value == first_value:
            return STATUS_WARN, f"采样序号未变化（0x{first_value:04X}，等待 {args.slow_lane_wait}s）"
        return STATUS_PASS, f"0x000F: 0x{first_value:04X} → 0x{second_value:04X}"

    report.run_case("A 区采样序号 0x000F 递增", _sample_seq_increments)

    def _read_window_head() -> Tuple[str, str]:
        """读 D 区窗口头 0x0100~0x0103，校验 bit15 = 有效窗口就绪。"""
        response = client.read_holding(REG_D_HEAD, 4)
        if _exception_code(response) is not None:
            return STATUS_FAIL, f"读 0x0100 失败：{_describe(response)}"
        head, seq, rate, stamp = response.registers[:4]
        ready = bool(head & 0x8000)
        samples = head & 0x00FF
        detail = (
            f"窗口头 0x{head:04X}（就绪={ready}，有效样本 {samples}）"
            f" 序号 {seq} 采样率 {rate/10.0:.1f}Hz 时间戳 {stamp}×10ms"
        )
        if not ready:
            return STATUS_WARN, detail + " —— bit15 未置位，F411 可能还没产出窗口"
        if samples != 100:
            return STATUS_WARN, detail + " —— 有效样本数不是 100（规格为 100）"
        return STATUS_PASS, detail

    report.run_case("D 区窗口头 0x0100 就绪位/样本数", _read_window_head)

    def _read_features() -> Tuple[str, str]:
        """读 E 区特征向量 0x0400~0x0423 共 36 个寄存器。"""
        count = REG_E_LAST - REG_E_FEATURES + 1
        response = client.read_holding(REG_E_FEATURES, count)
        if _exception_code(response) is not None:
            return STATUS_FAIL, f"读 0x0400 ×{count} 失败：{_describe(response)}"
        if len(response.registers) != count:
            return STATUS_FAIL, f"返回数量错误：{len(response.registers)}（应为 {count}）"
        regs = response.registers
        return STATUS_PASS, (
            f"36 维特征：ax 均值 {regs[0]} 标准差 {regs[6]} 峰峰值 {regs[12]} "
            f"过零率 {regs[18]/100.0:.2f} 主频 {regs[24]/10.0:.1f}Hz "
            f"主频带能量占比 {regs[30]/1000.0:.3f}"
        )

    report.run_case("读 E 区特征向量 0x0400 ×36", _read_features)


def case_boundaries(client: MasterClient, args: argparse.Namespace, report: TestReport) -> None:
    """用例组：数量与地址边界。"""

    def _read_125() -> Tuple[str, str]:
        """读 125 个（上限）必须成功。起点选 0x0104（窗口数据块）避免跨区。"""
        response = client.read_holding(REG_D_FIRST_SAMPLE, MAX_READ_COUNT)
        code = _exception_code(response)
        if code is not None:
            return STATUS_FAIL, f"读 {MAX_READ_COUNT} 个应成功，实际 {_describe(response)}"
        if len(response.registers) != MAX_READ_COUNT:
            return STATUS_FAIL, f"返回数量错误：{len(response.registers)}"
        return STATUS_PASS, f"0x0104 ×{MAX_READ_COUNT} 成功，首寄存器 0x{response.registers[0]:04X}"

    report.run_case(f"边界：读 {MAX_READ_COUNT} 个（0x0104 起）应成功", _read_125)

    def _read_126() -> Tuple[str, str]:
        """读 126 个（超限）必须回异常 03（docs/02 §6 示例⑧）。"""
        response = client.read_holding(REG_D_FIRST_SAMPLE, MAX_READ_COUNT + 1)
        code = _exception_code(response)
        if code is None:
            return STATUS_FAIL, f"读 {MAX_READ_COUNT + 1} 个应回异常 03，实际 {_describe(response)}"
        if code != ILLEGAL_VALUE:
            return STATUS_FAIL, f"异常码应为 0x03（非法数据值），实际 0x{code:02X}"
        return STATUS_PASS, "异常码 0x03 非法数据值"

    report.run_case(f"边界：读 {MAX_READ_COUNT + 1} 个应回异常 03", _read_126)

    def _read_512() -> Tuple[str, str]:
        """docs/02 §6 示例⑧ 的原始帧：读 0x004B 起 512 个 → 异常 03。"""
        response = client.read_holding(0x004B, 512)
        code = _exception_code(response)
        if code is None:
            return STATUS_FAIL, f"应回异常 03，实际 {_describe(response)}"
        if code != ILLEGAL_VALUE:
            return STATUS_FAIL, f"异常码应为 0x03，实际 0x{code:02X}"
        return STATUS_PASS, "异常码 0x03（对应帧 01 03 00 4B 02 00 34 BC）"

    report.run_case("边界：读 0x004B ×512 应回异常 03", _read_512)

    def _read_out_of_range() -> Tuple[str, str]:
        """越界起始地址 0x0360（D 区末尾 0x035B 之后）必须回异常 02。"""
        response = client.read_holding(REG_D_LAST + 1, 1)
        code = _exception_code(response)
        if code is None:
            return STATUS_FAIL, f"起始 0x{REG_D_LAST + 1:04X} 应回异常 02，实际 {_describe(response)}"
        if code != ILLEGAL_ADDRESS:
            return STATUS_FAIL, f"异常码应为 0x02（非法数据地址），实际 0x{code:02X}"
        return STATUS_PASS, f"起始 0x{REG_D_LAST + 1:04X} → 异常码 0x02"

    report.run_case(f"越界：起始 0x{REG_D_LAST + 1:04X} 应回异常 02", _read_out_of_range)

    def _read_crossing_boundary() -> Tuple[str, str]:
        """跨区读：0x0420 起 8 个会越过 E 区末尾 0x0423 → 异常 02。"""
        response = client.read_holding(0x0420, 8)
        code = _exception_code(response)
        if code is None:
            return STATUS_FAIL, f"跨过 0x0423 应回异常 02，实际 {_describe(response)}"
        if code != ILLEGAL_ADDRESS:
            return STATUS_FAIL, f"异常码应为 0x02，实际 0x{code:02X}"
        return STATUS_PASS, "起始+数量越过映射表末尾 → 异常码 0x02"

    report.run_case("越界：读 0x0420 ×8（越过 0x0423）应回异常 02", _read_crossing_boundary)


def case_write(client: MasterClient, args: argparse.Namespace, report: TestReport) -> None:
    """用例组：写操作（只读区拒绝、C 区正常写回显）。"""

    def _write_readonly() -> Tuple[str, str]:
        """FC06 写 A 区（只读）0x0000 必须回异常 02（docs/02 §6 示例⑧）。"""
        response = client.write_single(REG_A_TEMP, 0x0001)
        code = _exception_code(response)
        if code is None:
            return STATUS_FAIL, f"写只读区应回异常 02，实际 {_describe(response)}"
        if code != ILLEGAL_ADDRESS:
            return STATUS_FAIL, f"异常码应为 0x02，实际 0x{code:02X}"
        return STATUS_PASS, "写 0x0000（A 区只读）→ 异常码 0x02"

    report.run_case("写只读区（FC06 → 0x0000）应回异常 02", _write_readonly)

    def _write_readonly_multiple() -> Tuple[str, str]:
        """FC10 写 B 区（只读）也必须回异常 02。"""
        response = client.write_multiple(REG_B_STATUS1, [0x1234, 0x5678])
        code = _exception_code(response)
        if code is None:
            return STATUS_FAIL, f"FC10 写只读区应回异常 02，实际 {_describe(response)}"
        if code != ILLEGAL_ADDRESS:
            return STATUS_FAIL, f"异常码应为 0x02，实际 0x{code:02X}"
        return STATUS_PASS, "FC10 写 0x0010（B 区只读）→ 异常码 0x02"

    report.run_case("写只读区（FC10 → 0x0010）应回异常 02", _write_readonly_multiple)

    def _write_readback() -> Tuple[str, str]:
        """FC06 写 C 区风扇 PWM 并回读校验（docs/02 §6 示例⑤：写 1000）。"""
        original = client.read_holding(REG_C_FAN_PWM, 1)
        if _exception_code(original) is not None:
            return STATUS_FAIL, f"预读 0x0021 失败：{_describe(original)}"
        before = original.registers[0]
        target = 1000 if before != 1000 else 500  # 避免与当前值相同导致看不出效果
        response = client.write_single(REG_C_FAN_PWM, target)
        code = _exception_code(response)
        if code is not None:
            return STATUS_FAIL, f"写 0x0021={target} 失败：{_describe(response)}"
        echo = getattr(response, "registers", None)
        if echo and echo[0] != target:
            return STATUS_FAIL, f"FC06 应原样回显 {target}，实际 {echo[0]}"
        verify = client.read_holding(REG_C_FAN_PWM, 1)
        if _exception_code(verify) is not None:
            return STATUS_FAIL, f"回读 0x0021 失败：{_describe(verify)}"
        after = verify.registers[0]
        # 还原原值，避免污染后续测试
        client.write_single(REG_C_FAN_PWM, before)
        if after != target:
            return STATUS_FAIL, f"回读值 {after} != 写入值 {target}"
        return STATUS_PASS, f"0x0021: {before} → {target} → 已还原 {before}"

    report.run_case("写 C 区风扇 PWM 0x0021 并回读（FC06 回显）", _write_readback)

    def _write_multiple_readback() -> Tuple[str, str]:
        """FC10 连写 0x0020（控制字）+ 0x0021（风扇 PWM），对应 docs/02 §6 示例⑥。"""
        original = client.read_holding(REG_C_CTRL1, 2)
        if _exception_code(original) is not None:
            return STATUS_FAIL, f"预读 0x0020~0x0021 失败：{_describe(original)}"
        before = list(original.registers)
        # 只动蜂鸣器位（bit0）与风扇 PWM，最后还原，降低副作用
        ctrl = (before[0] & ~0x0001) | 0x0001
        values = [ctrl, 1000]
        response = client.write_multiple(REG_C_CTRL1, values)
        code = _exception_code(response)
        if code is not None:
            return STATUS_FAIL, f"FC10 写 0x0020~0x0021 失败：{_describe(response)}"
        verify = client.read_holding(REG_C_CTRL1, 2)
        if _exception_code(verify) is not None:
            return STATUS_FAIL, f"回读失败：{_describe(verify)}"
        got = list(verify.registers)
        # 还原
        client.write_multiple(REG_C_CTRL1, before)
        mismatch = [f"0x{REG_C_CTRL1 + i:04X}: {got[i]} != {values[i]}" for i in range(2) if got[i] != values[i]]
        if mismatch:
            return STATUS_FAIL, "回读不一致：" + "; ".join(mismatch)
        return STATUS_PASS, f"FC10 写入 {values} 回读一致，已还原 {before}"

    report.run_case("写 C 区（FC10，0x0020+0x0021）并回读", _write_multiple_readback)


def case_error_codes(client: MasterClient, args: argparse.Namespace, report: TestReport) -> None:
    """用例组：异常码与静默行为（需要原始帧通道）。

    这些用例必须绕过 pymodbus 的组帧与校验（要手工注入坏 CRC / 非法功能码），
    因此统一走 ``args._raw`` 这个 pyserial 裸通道。
    """
    if not args.raw_tests:
        for name in (
            "非法功能码 0x05 应回异常 01",
            "CRC 错注入应静默",
            "错误从机地址应静默",
            "广播（地址 0x00）执行但不响应",
        ):
            report.add(name, STATUS_SKIP, "需要 --raw-tests（原始帧通道）")
        return

    raw = getattr(args, "_raw", None)
    if raw is None:
        for name in (
            "非法功能码 0x05 应回异常 01",
            "CRC 错注入应静默",
            "错误从机地址应静默",
            "广播（地址 0x00）执行但不响应",
        ):
            report.add(name, STATUS_SKIP, "原始帧通道打开失败（见上方提示）")
        return

    def _illegal_function() -> Tuple[str, str]:
        """功能码 0x05（F411 不支持）→ 异常 01（docs/02 §4）。"""
        frame = rtu_frame(bytes((args.addr, 0x05, 0x00, 0x00, 0x00, 0x01)))
        raw.send(frame)
        data = raw.recv(args.raw_wait)
        if not data:
            return STATUS_FAIL, f"发 {frame.hex(' ')} 后无响应（应回异常 01）"
        if len(data) < 3:
            return STATUS_FAIL, f"响应过短：{data.hex(' ')}"
        if data[0] != args.addr or data[1] != (0x05 | 0x80):
            return STATUS_FAIL, f"响应头错误：{data.hex(' ')}（应为 {args.addr:02X} 85 ...）"
        if data[2] != ILLEGAL_FUNCTION:
            return STATUS_FAIL, f"异常码应为 0x01，实际 0x{data[2]:02X}"
        return STATUS_PASS, f"响应 {data.hex(' ').upper()} → 异常码 0x01"

    report.run_case("非法功能码 0x05 应回异常 01", _illegal_function)

    def _crc_error_silent() -> Tuple[str, str]:
        """CRC 错注入：从机必须静默，不回异常、不崩（docs/02 §8/§10）。"""
        frame = corrupt_crc(rtu_frame(bytes((args.addr, 0x03, 0x00, 0x00, 0x00, 0x01))))
        raw.send(frame)
        data = raw.recv(args.raw_wait)
        if data:
            return STATUS_FAIL, f"CRC 错帧竟有响应：{data.hex(' ')}（规格要求静默丢弃）"
        return STATUS_PASS, f"坏帧 {frame.hex(' ').upper()} → 静默（{args.raw_wait}s 内无字节）"

    report.run_case("CRC 错注入应静默", _crc_error_silent)

    def _wrong_address_silent() -> Tuple[str, str]:
        """错误从机地址必须静默（否则多从机总线冲突，docs/02 §2/§8）。"""
        wrong = 0x02 if args.addr != 0x02 else 0x03
        frame = rtu_frame(bytes((wrong, 0x03, 0x00, 0x00, 0x00, 0x01)))
        raw.send(frame)
        data = raw.recv(args.raw_wait)
        if data:
            return STATUS_FAIL, f"地址 {wrong} 竟有响应：{data.hex(' ')}（规格要求静默）"
        return STATUS_PASS, f"地址 0x{wrong:02X} → 静默"

    report.run_case("错误从机地址应静默", _wrong_address_silent)

    def _broadcast_write() -> Tuple[str, str]:
        """广播（地址 0x00）写控制区：执行但不响应（docs/02 §2）。"""
        read_before = raw_read_register(raw, args, REG_C_FAN_PWM)
        if read_before is None:
            return STATUS_FAIL, "广播前无法读取 0x0021"
        target = 700 if read_before != 700 else 300
        frame = rtu_frame(bytes((0x00, 0x06, 0x00, 0x21, (target >> 8) & 0xFF, target & 0xFF)))
        raw.send(frame)
        data = raw.recv(args.raw_wait)
        if data:
            return STATUS_FAIL, f"广播竟有响应：{data.hex(' ')}（规格要求不响应）"
        time.sleep(0.3)
        read_after = raw_read_register(raw, args, REG_C_FAN_PWM)
        # 无论结果如何都广播还原原值，避免污染后续用例
        restore = rtu_frame(
            bytes((0x00, 0x06, 0x00, 0x21, (read_before >> 8) & 0xFF, read_before & 0xFF))
        )
        raw.send(restore)
        raw.recv(args.raw_wait)
        if read_after is None:
            return STATUS_FAIL, "广播后无法回读 0x0021"
        if read_after != target:
            return STATUS_FAIL, f"广播未执行：0x0021 = {read_after}（期望 {target}）"
        return STATUS_PASS, f"无响应且执行生效：0x0021 {read_before} → {target}（已广播还原）"

    if args.allow_broadcast_test:
        report.run_case("广播（地址 0x00）执行但不响应", _broadcast_write)
    else:
        report.add(
            "广播（地址 0x00）执行但不响应",
            STATUS_SKIP,
            "会改变风扇 PWM，需显式加 --allow-broadcast-test",
        )


def raw_read_register(raw: RawSerial, args: argparse.Namespace, register: int) -> Optional[int]:
    """用原始通道自己组 FC03 帧并解析响应（不依赖 pymodbus，避免端口争用）。

    :return: 寄存器值；响应不合法时返回 None。
    """
    frame = rtu_frame(bytes((args.addr, 0x03, (register >> 8) & 0xFF, register & 0xFF, 0x00, 0x01)))
    raw.send(frame)
    data = raw.recv(args.raw_wait)
    if len(data) < 5 or data[1] != 0x03 or data[2] != 0x02:
        return None
    return (data[3] << 8) | data[4]


def case_invalid_values(client: MasterClient, args: argparse.Namespace, report: TestReport) -> None:
    """用例组：非法值（应回异常 03）。"""

    def _baud_code_9() -> Tuple[str, str]:
        """波特率代码 9 越界（合法 0~4）→ 异常 03（docs/02 §5 C 区 0x002D）。"""
        response = client.write_single(REG_C_BAUD_CODE, 9)
        code = _exception_code(response)
        if code is None:
            return STATUS_FAIL, f"波特率代码 9 应回异常 03，实际 {_describe(response)}"
        if code != ILLEGAL_VALUE:
            return STATUS_FAIL, f"异常码应为 0x03，实际 0x{code:02X}"
        # 确认没有真的改掉波特率：用当前链路再读一次
        verify = client.read_holding(REG_C_BAUD_CODE, 1)
        if _exception_code(verify) is not None:
            return STATUS_FAIL, "写非法波特率后链路异常，说明从机可能已改波特率（不应发生）"
        return STATUS_PASS, f"异常码 0x03，0x002D 仍为 {verify.registers[0]}（未被改动）"

    report.run_case("非法值：波特率代码 9（0x002D）应回异常 03", _baud_code_9)

    def _slave_addr_invalid() -> Tuple[str, str]:
        """从机地址 248 越界（合法 1~247）→ 异常 03（docs/02 §5 C 区 0x002C）。"""
        response = client.write_single(REG_C_SLAVE_ADDR, 248)
        code = _exception_code(response)
        if code is None:
            return STATUS_FAIL, f"地址 248 应回异常 03，实际 {_describe(response)}"
        if code != ILLEGAL_VALUE:
            return STATUS_FAIL, f"异常码应为 0x03，实际 0x{code:02X}"
        return STATUS_PASS, "异常码 0x03（地址 248 超 1~247）"

    report.run_case("非法值：从机地址 248（0x002C）应回异常 03", _slave_addr_invalid)

    def _slave_addr_zero() -> Tuple[str, str]:
        """从机地址 0 非法（0 是广播地址，不能作为本机地址）→ 异常 03。"""
        response = client.write_single(REG_C_SLAVE_ADDR, 0)
        code = _exception_code(response)
        if code is None:
            return STATUS_FAIL, f"地址 0 应回异常 03，实际 {_describe(response)}"
        if code != ILLEGAL_VALUE:
            return STATUS_FAIL, f"异常码应为 0x03，实际 0x{code:02X}"
        return STATUS_PASS, "异常码 0x03（地址 0 为广播，不能设为从机地址）"

    report.run_case("非法值：从机地址 0（0x002C）应回异常 03", _slave_addr_zero)

    if args.include_risky:
        def _slave_addr_legal_then_restore() -> Tuple[str, str]:
            """危险用例：写合法地址（立即生效）→ 用新地址通信 → 再写回旧地址。"""
            original = args.addr
            new_addr = 0x05 if original != 0x05 else 0x06
            response = client.write_single(REG_C_SLAVE_ADDR, new_addr)
            if _exception_code(response) is not None:
                return STATUS_FAIL, f"写合法新地址 {new_addr} 失败：{_describe(response)}"
            time.sleep(args.reconnect_wait)
            client.unit = new_addr
            verify = client.read_holding(REG_C_SLAVE_ADDR, 1, unit=new_addr)
            if _exception_code(verify) is not None or verify.registers[0] != new_addr:
                client.unit = original
                return STATUS_FAIL, f"新地址 {new_addr} 通信失败，请手动把 0x002C 改回 {original}"
            restore = client.write_single(REG_C_SLAVE_ADDR, original, unit=new_addr)
            client.unit = original
            if _exception_code(restore) is not None:
                return STATUS_FAIL, f"写回旧地址 {original} 失败，请手动恢复 0x002C"
            time.sleep(args.reconnect_wait)
            check = client.read_holding(REG_C_SLAVE_ADDR, 1)
            if _exception_code(check) is not None:
                return STATUS_FAIL, f"写回后无法用旧地址 {original} 通信"
            return STATUS_PASS, f"地址 {original} → {new_addr} → {original} 往返成功"

        report.run_case("（危险）从机地址合法变更并还原 0x002C", _slave_addr_legal_then_restore)
    else:
        report.add(
            "（危险）从机地址合法变更并还原 0x002C",
            STATUS_SKIP,
            "会打断总线，需显式加 --include-risky",
        )


def case_consistency(client: MasterClient, args: argparse.Namespace, report: TestReport) -> None:
    """用例组：D 区窗口一致性（docs/02 §7）。"""

    def _window_read_consistent() -> Tuple[str, str]:
        """按 docs/02 §7 的主机填充顺序：读头 → 5 块数据 → 再读 0x0101 校验序号不变。"""
        head = client.read_holding(REG_D_HEAD, 4)
        if _exception_code(head) is not None:
            return STATUS_FAIL, f"读窗口头失败：{_describe(head)}"
        seq0 = head.registers[1]
        payload = bytearray()
        for index, (start, count) in enumerate(WINDOW_CHUNKS, start=1):
            response = client.read_holding(start, count)
            code = _exception_code(response)
            if code is not None:
                return STATUS_FAIL, f"第 {index} 块 0x{start:04X} ×{count} 失败：{_describe(response)}"
            for value in response.registers:
                payload += bytes(((value >> 8) & 0xFF, value & 0xFF))
        tail = client.read_holding(REG_D_SEQ, 1)
        if _exception_code(tail) is not None:
            return STATUS_FAIL, f"读窗口序号失败：{_describe(tail)}"
        seq1 = tail.registers[0]
        if seq0 != seq1:
            return STATUS_WARN, f"窗口序号在读期间变化 {seq0} → {seq1}（应重读，不算协议错误）"
        if len(payload) != 600 * 2:
            return STATUS_FAIL, f"窗口字节数错误：{len(payload)}（应为 1200）"
        return STATUS_PASS, f"5 块共 {len(payload)} 字节，序号稳定为 {seq0}, seq1"

    report.run_case("窗口一致性：5 块分读期间 0x0101 不变", _window_read_consistent)

    if not args.raw_tests:
        report.add("帧间隔 > 3.5T 应识别为两帧", STATUS_SKIP, "需要 --raw-tests")
        report.add("粘帧（两帧无间隔连发）不能死机", STATUS_SKIP, "需要 --raw-tests")
        return

    raw = getattr(args, "_raw", None)
    if raw is None:
        report.add("帧间隔 > 3.5T 应识别为两帧", STATUS_SKIP, "原始帧通道未就绪")
        report.add("粘帧（两帧无间隔连发）不能死机", STATUS_SKIP, "原始帧通道未就绪")
        return

    def _frame_gap() -> Tuple[str, str]:
        """帧中间插入 > 3.5T 间隔 → 从机应视为两帧并丢弃（不崩、不响应半帧）。"""
        char_time = 11.0 / args.baud  # 8N1 一字符 11 bit（docs/02 §1）
        gap = char_time * 3.5 * args.gap_multiplier
        first = rtu_frame(bytes((args.addr, 0x03, 0x00, 0x00, 0x00, 0x01)))
        half = len(first) // 2
        raw.send(first[:half])
        time.sleep(gap)
        raw.send(first[half:])
        data = raw.recv(args.raw_wait)
        # 期望：两段各自被当作不完整帧丢弃 → 静默。若回了响应说明帧间隔判定过于宽松。
        if data:
            return STATUS_WARN, (
                f"插入 {gap*1000:.2f} ms（{args.gap_multiplier}×3.5T）间隔后仍收到响应 "
                f"{data.hex(' ').upper()}；docs/02 §1 期望被判为两帧并丢弃"
            )
        return STATUS_PASS, f"插入 {gap*1000:.2f} ms 间隔（{args.gap_multiplier}×3.5T）后静默丢弃"

    report.run_case("帧间隔 > 3.5T 应识别为两帧", _frame_gap)

    def _stuck_frames() -> Tuple[str, str]:
        """两帧无间隔连发：至少不能死机；能解析出第一帧更好。"""
        frame_a = rtu_frame(bytes((args.addr, 0x03, 0x00, 0x00, 0x00, 0x01)))
        frame_b = rtu_frame(bytes((args.addr, 0x03, 0x00, 0x01, 0x00, 0x01)))
        raw.send(frame_a + frame_b)
        data = raw.recv(args.raw_wait * 2)
        # 之后确认从机还活着
        raw.send(frame_a)
        alive = raw.recv(args.raw_wait)
        if not alive:
            return STATUS_FAIL, "粘帧之后从机不再响应（疑似死机或接收状态未恢复）"
        if data:
            return STATUS_PASS, f"粘帧后收到 {len(data)} 字节响应，且从机仍存活"
        return STATUS_PASS, "粘帧未产生响应（可接受），且从机仍存活"

    report.run_case("粘帧（两帧无间隔连发）不能死机", _stuck_frames)


def case_stress(client: MasterClient, args: argparse.Namespace, report: TestReport) -> None:
    """用例组：连续 N 帧压力测试（docs/02 §10 的 10000 帧要求）。"""

    def _stress() -> Tuple[str, str]:
        """连续读 A/B 区 N 次，统计成功率与耗时分布，并核对从机计数器增量。"""
        frames = args.count
        before = client.read_holding(REG_B_OK_FRAMES, 6)
        if _exception_code(before) is not None:
            return STATUS_FAIL, f"压测前读统计寄存器失败：{_describe(before)}"
        counters_before = list(before.registers)
        ok_before, crc_before, exc_before = counters_before[0], counters_before[1], counters_before[2]

        success = 0
        failures: List[str] = []
        latencies: List[float] = []
        start_all = time.perf_counter()
        for index in range(1, frames + 1):
            began = time.perf_counter()
            try:
                response = client.read_holding(SLOW_LANE_START, SLOW_LANE_COUNT)
            except Exception as exc:  # noqa: BLE001 - 压测中任何异常都计入失败
                failures.append(f"#{index} {type(exc).__name__}: {exc}")
                continue
            latencies.append((time.perf_counter() - began) * 1000)
            code = _exception_code(response)
            if code is not None:
                failures.append(f"#{index} 异常码 0x{code:02X}")
            elif len(response.registers) != SLOW_LANE_COUNT:
                failures.append(f"#{index} 数量 {len(response.registers)}")
            else:
                success += 1
            if args.progress and index % max(1, frames // 20) == 0:
                safe_print(f"  ... 压测进度 {index}/{frames}，成功 {success}，失败 {len(failures)}")
        total = time.perf_counter() - start_all
        rate = success / frames * 100.0

        after = client.read_holding(REG_B_OK_FRAMES, 6)
        counter_detail = ""
        if _exception_code(after) is None:
            counters_after = list(after.registers)
            delta_ok = (counters_after[0] - ok_before) & 0xFFFF
            delta_crc = (counters_after[1] - crc_before) & 0xFFFF
            delta_exc = (counters_after[2] - exc_before) & 0xFFFF
            counter_detail = (
                f"；从机计数增量：正常帧 {delta_ok}，CRC 错 {delta_crc}，异常帧 {delta_exc}"
            )
            if delta_crc != 0 and args.strict_counters:
                return STATUS_FAIL, f"压测期间从机 CRC 错计数增加 {delta_crc}（要求为 0）{counter_detail}"

        latency_detail = ""
        if latencies:
            latency_detail = (
                f"，延迟 最小 {min(latencies):.1f} / 中位 {statistics.median(latencies):.1f} / "
                f"最大 {max(latencies):.1f} ms"
            )
        detail = (
            f"{frames} 帧成功 {success}（{rate:.2f}%），失败 {len(failures)}，"
            f"总耗时 {total:.2f}s（{frames/total:.1f} 帧/s）{latency_detail}{counter_detail}"
        )
        if failures:
            preview = "; ".join(failures[:5])
            suffix = f" ... 另有 {len(failures) - 5} 条" if len(failures) > 5 else ""
            detail += f"\n         失败样例：{preview}{suffix}"
            if rate < args.min_success_rate:
                return STATUS_FAIL, detail
            return STATUS_WARN, detail
        return STATUS_PASS, detail

    report.run_case(f"连续 {args.count} 帧压力测试（读 A/B 区）", _stress)

    def _uptime_growth() -> Tuple[str, str]:
        """docs/02 §10：运行时间 0x0016 应单调增长，复位原因 0x0017 不应变化。"""
        first = client.read_holding(REG_B_UPTIME_MIN, 2)
        if _exception_code(first) is not None:
            return STATUS_FAIL, f"读 0x0016~0x0017 失败：{_describe(first)}"
        uptime0, reason0 = first.registers[0], first.registers[1]
        time.sleep(args.uptime_wait)
        second = client.read_holding(REG_B_UPTIME_MIN, 2)
        if _exception_code(second) is not None:
            return STATUS_FAIL, f"读 0x0016~0x0017 失败：{_describe(second)}"
        uptime1, reason1 = second.registers[0], second.registers[1]
        if reason1 != reason0:
            return STATUS_FAIL, f"复位原因变化 0x{reason0:04X} → 0x{reason1:04X}（疑似中途复位）"
        if args.uptime_wait >= 60 and uptime1 == uptime0:
            return STATUS_WARN, f"运行时间未增长（仍为 {uptime0} 分钟），请延长观察"
        return STATUS_PASS, (
            f"运行时间 {uptime0} → {uptime1} 分钟（等待 {args.uptime_wait}s），"
            f"复位原因 0x{reason0:04X} 未变"
        )

    report.run_case("运行时间 0x0016 增长 / 复位原因 0x0017 不变", _uptime_growth)


# --------------------------------------------------------------------------------------
# 主流程
# --------------------------------------------------------------------------------------

CASE_GROUPS: Dict[str, Callable[[MasterClient, argparse.Namespace, TestReport], None]] = {
    "basic": case_read_slow_lane,
    "boundary": case_boundaries,
    "write": case_write,
    "error": case_error_codes,
    "value": case_invalid_values,
    "consistency": case_consistency,
    "stress": case_stress,
}

#: 需要"原始帧裸通道"的用例组：这些组执行时不能同时持有 pymodbus 客户端
RAW_CASE_GROUPS = {"error"}


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """解析命令行参数（``--port --baud --addr --count`` 为主，其余为可选增强）。"""
    parser = argparse.ArgumentParser(
        prog="modbus_master_test.py",
        description=(
            "PC 当 Modbus RTU 主机，用 pymodbus 3.x 测试 STM32F411 从机"
            "（覆盖 docs/02 §10 测试用例清单）"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "示例：\n"
            "  python modbus_master_test.py --port COM5\n"
            "  python modbus_master_test.py --port COM5 --baud 115200 --addr 1 --count 10000\n"
            "  python modbus_master_test.py --port COM5 --cases basic,error --raw-tests --raw-port COM5\n"
            "  python modbus_master_test.py --list-ports\n"
        ),
    )
    parser.add_argument("--port", default="COM3", help="USB 转 RS485 的串口（默认 COM3）")
    parser.add_argument("--baud", type=int, default=115200,
                        help="波特率（默认 115200，与 firmware-esp32 的 MODBUS_BAUD 一致；"
                             "9600 只用于单点链路自检，见 tools/README.md『为什么默认 115200』）")
    parser.add_argument("--addr", type=lambda v: int(v, 0), default=SLAVE_ADDR_DEFAULT,
                        help="从机地址，默认 1（支持 0x01 写法）")
    parser.add_argument("--count", type=int, default=1000,
                        help="压力测试帧数，默认 1000；长跑可用 10000")
    # 串口参数
    parser.add_argument("--parity", default="N", choices=["N", "n", "E", "e", "O", "o"], help="校验位（8N1 → N）")
    parser.add_argument("--stopbits", type=int, default=1, choices=[1, 2], help="停止位（默认 1）")
    parser.add_argument("--timeout", type=float, default=0.2, help="单帧超时秒（docs/02 §9 建 0.05，PC 端留余量）")
    parser.add_argument("--raw-wait", type=float, default=0.15,
                        help="原始帧用例等待静默判定的时间秒（默认 0.15）")
    parser.add_argument("--reconnect-wait", type=float, default=0.5, help="地址/波特率切换后的等待秒")
    parser.add_argument("--slow-lane-wait", type=float, default=1.2, help="采样序号递增用例的等待秒")
    parser.add_argument("--uptime-wait", type=float, default=5.0, help="运行时间增长用例的等待秒")
    # 用例选择
    parser.add_argument("--cases", default=",".join(CASE_GROUPS.keys()),
                        help="要跑的用例组，逗号分隔：" + ",".join(CASE_GROUPS.keys()))
    parser.add_argument("--raw-tests", action="store_true",
                        help="启用需要原始帧的用例（CRC 注入/非法功能码/静默/帧间隔/粘帧）")
    parser.add_argument("--raw-port", default=None,
                        help="原始帧通道用的串口；留空表示复用 --port（先关闭主客户端再打开）")
    parser.add_argument("--gap-multiplier", type=float, default=2.0,
                        help="帧间隔用例插入的间隔倍数（×3.5T，默认 2 倍以保证可靠超时）")
    parser.add_argument("--include-risky", action="store_true",
                        help="执行会打断总线的危险用例（改从机地址并还原）")
    parser.add_argument("--allow-broadcast-test", action="store_true",
                        help="执行广播写用例（会改风扇 PWM，脚本会自动还原）")
    parser.add_argument("--minutes", type=float, default=0.0,
                        help=">0 时忽略 --count，改为按分钟数循环压测（长跑稳定性用）")
    parser.add_argument("--strict-counters", action="store_true", default=True,
                        help="压测后要求从机 CRC 错计数增量为 0（默认开启，docs/02 §10）")
    parser.add_argument("--no-strict-counters", dest="strict_counters", action="store_false",
                        help="关闭 CRC 计数增量检查")
    parser.add_argument("--min-success-rate", type=float, default=100.0,
                        help="压测最低成功率百分比，低于此值判 FAIL（默认 100）")
    parser.add_argument("--progress", action="store_true", default=True, help="打印压测进度")
    parser.add_argument("--no-progress", dest="progress", action="store_false", help="关闭压测进度")
    parser.add_argument("--list-ports", action="store_true", help="列出所有串口后退出")
    parser.add_argument("-v", "--verbose", action="store_true", help="保留位：打印更多细节")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    """脚本入口：连接从机 → 依次跑选用例组 → 打印汇总表 → 返回退出码。"""
    setup_console()
    args = parse_args(argv)

    if args.list_ports:
        list_serial_ports()
        return 0

    safe_print(f"pymodbus 版本：{pymodbus_version()}")
    safe_print(
        f"目标从机：{args.port} @ {args.baud} 8{args.parity.upper()}{args.stopbits}，"
        f"从机地址 {args.addr}，用例组 [{args.cases}]"
    )

    report = TestReport()
    selected = [name.strip() for name in args.cases.split(",") if name.strip()]
    unknown = [name for name in selected if name not in CASE_GROUPS]
    if unknown:
        safe_print(f"错误：未知用例组 {unknown}；可用：{list(CASE_GROUPS)}")
        return 2

    if args.minutes > 0:
        # 长跑模式：按时间换算帧数（用当前波特率下的经验帧时估算，压测内部仍按帧计数）
        frames = max(1, int(args.minutes * 60 * (args.baud / 9600) * 20))
        safe_print(f"长跑模式：{args.minutes} 分钟 → 约 {frames} 帧（估算值，实际按耗时为准）")
        args.count = frames

    client = build_client(args)
    master = MasterClient(client, args.addr, args.timeout)

    # 原始帧通道：与主客户端是同一个口时二者不能同时打开，需要按用例组切换持有权
    raw: Optional[RawSerial] = None
    raw_port = args.raw_port or args.port
    shares_port = raw_port == args.port
    need_raw = args.raw_tests and bool(RAW_CASE_GROUPS & set(selected))
    if need_raw:
        if shares_port:
            safe_print(f"原始帧通道与主客户端共用 {raw_port}：按用例组切换端口持有权。")
            client.close()
            time.sleep(0.2)
        try:
            raw = RawSerial(raw_port, args.baud, args.parity, args.stopbits, args.timeout)
        except SystemExit:
            safe_print("原始帧通道打开失败，相关用例将标记 SKIP。")
            raw = None
    args._raw = raw  # noqa: SLF001 - 传给用例的运行时句柄

    def activate(group: str) -> None:
        """按用例组切换端口持有权：需要裸通道的组关闭 pymodbus 客户端，反之亦然。"""
        if not need_raw or not shares_port:
            return
        if group in RAW_CASE_GROUPS:
            client.close()
            if raw is None:
                return
            try:  # 其它用例组可能已把裸通道关掉，这里重新打开
                if not getattr(raw.port, "is_open", False):
                    raw.port.open()
            except Exception:  # noqa: BLE001
                pass
        else:
            if raw is not None:
                raw.close()
            if not getattr(client, "connected", True):  # 3.x 用 connected 属性表示串口已打开
                try:
                    if not client.connect():
                        safe_print(f"警告：用例组 {group} 重新打开 {args.port} 失败。")
                except Exception as exc:  # noqa: BLE001
                    safe_print(f"警告：用例组 {group} 重新打开 {args.port} 失败：{exc}")

    try:
        for group in selected:
            safe_print(f"\n>>> 用例组：{group}")
            activate(group)
            CASE_GROUPS[group](master, args, report)
    except KeyboardInterrupt:
        safe_print("\n已被用户中断（Ctrl+C）。以下为已完成用例的结果。")
    finally:
        if raw is not None:
            raw.close()
        try:
            client.close()
        except Exception:  # noqa: BLE001
            pass

    report.dump()
    counts = report.counts()
    if counts.get(STATUS_FAIL, 0) > 0:
        safe_print("\n结论：存在失败用例，退出码 1。")
        return 1
    safe_print("\n结论：全部用例通过（SKIP 不算失败，请确认跳过项是否真的不需要测）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
