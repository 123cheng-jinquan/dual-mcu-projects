#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PC 当 Modbus RTU **从机**模拟器：让 ESP32-S3 主站逻辑先脱离 F411 硬件跑通。

用途（对应 docs/03-bringup.md **阶段 4** 第 2 步："先用 tools/modbus_slave_sim.py
把 S3 主机逻辑调通 → 再换真 F411"）：S3 侧的轮询调度、窗口分块读、序号校验、控制下发
都可以对着本模拟器先验证，不必等 F411 的从机协议栈写完。

实现的寄存器区块（地址严格取自 docs/02-modbus-protocol.md §5）
--------------------------------------------------------------
* **A 区 0x0000~0x000F** 环境量，只读：用正弦游走 + 噪声伪造，采样序号每轮 +1
* **B 区 0x0010~0x001F** 状态与统计，只读：传感器在线位图、固件版本、帧计数、运行时间、
  复位原因、模式、本地报警位
* **C 区 0x0020~0x002F** 控制与配置，**可读写**：写入保存在内存（进程内持久），
  并对越界值返回异常 03
* **D 区 0x0100~0x035B** 六轴采样窗口，只读：100 样本 × 6 轴 int16，
  `sin + 噪声` 伪造；窗口序号随时间自动 +1；`0x0100` 头 bit15 置 1、低 8 位 = 100
* **E 区 0x0400~0x0423** 特征向量，只读：由当前窗口的真实统计量算出（均值/标准差/
  峰峰值/过零率/主频/主频带能量占比），保证与 D 区自洽

用法::

    python modbus_slave_sim.py --port COM7 --baud 9600 --addr 1 --rate 2
    python modbus_slave_sim.py --port COM7 --selftest        # 不占串口，只自检寄存器逻辑
    python modbus_slave_sim.py --port COM7 --verbose         # 每次窗口更新打印一行

Windows 上没有物理串口时，可用 com0com 之类的虚拟串口对把 S3 的 USB-485 口与
本模拟器的口对接；Linux 用 ``socat -d -d pty,raw,echo=0 pty,raw,echo=0`` 生成一对 pty。

实现说明 / 已知取舍
-------------------
1. **写入校验**：按 docs/02 §5 的范围表校验，只读区返回异常 02、越界值返回异常 03
   （与真 F411 行为一致）。可用 ``--no-strict-writes`` 关闭校验，单纯看链路是否通。
2. **pymodbus 地址偏移的坑**：``ModbusSlaveContext`` 各小版本对传给数据块的地址做 ``+1``
   （3.6/3.7 可用 ``zero_mode=True`` 关掉，3.8+ 删除该参数、固定 ``+1``）。
   本脚本用 **启动时运行时探测**（:func:`probe_address_mode`）自适应，并在日志里打印结果，
   避免"寄存器整体偏移一位"这种隐蔽错误。
3. **数量边界与异常码需要自定义 PDU**：pymodbus 只在**组帧**时校验数量，
   且写路径的返回码会被 ``ModbusSlaveContext.setValues`` 丢掉。因此本脚本按官方扩展点
   ``custom_pdu`` 注册了 FC03/04/06/10 的校验子类，并把 pymodbus 原生支持但本项目
   不支持的功能码（FC01/02/05/0F）改成回**异常 01**——否则模拟器会比真 F411 "更能干",
   让 S3 侧把错误实现当成对的。
4. **广播**：通过服务端 ``broadcast_enable=True`` 支持"地址 0x00 执行但不响应"（docs/02 §2）。
5. **地址过滤**：``ModbusServerContext`` 只注册本机地址，请求其它地址会抛
   ``NoSuchSlaveException`` 并由服务端静默丢弃（docs/02 §2/§8）。
6. **pymodbus 版本要求**：需要 ``3.7 <= pymodbus < 3.10``。3.10 起 pymodbus 删除了
   ``ModbusSlaveContext``/``ModbusSequentialDataBlock``（改用全新的 ``SimData``/``SimDevice``），
   本脚本的数据块接口在 3.10+ 不可用；3.6.x 的 ``ModbusSerialServer`` 不支持 ``custom_pdu``，
   功能码覆盖（异常 01 与数量边界）会弱化。
"""

from __future__ import annotations

import argparse
import asyncio
import inspect
import math
import random
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

from _console import format_table, safe_print, setup_console

_TOOLS_DIR = Path(__file__).resolve().parent
if str(_TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(_TOOLS_DIR))

# --------------------------------------------------------------------------------------
# 规格常量（docs/02 §5，地址一律用十六进制）
# --------------------------------------------------------------------------------------

REG_A_FIRST, REG_A_LAST = 0x0000, 0x000F
REG_B_FIRST, REG_B_LAST = 0x0010, 0x001F
REG_C_FIRST, REG_C_LAST = 0x0020, 0x002F
REG_D_FIRST, REG_D_LAST = 0x0100, 0x035B
REG_E_FIRST, REG_E_LAST = 0x0400, 0x0423
REG_TABLE_SIZE = REG_E_LAST + 1  # 覆盖全部区块，供 ModbusSequentialDataBlock 使用

# A 区
REG_A_TEMP, REG_A_HUMI, REG_A_PRESS, REG_A_LIGHT = 0x0000, 0x0001, 0x0002, 0x0003
REG_A_MQ2, REG_A_MQ135, REG_A_FLAME, REG_A_SOIL = 0x0004, 0x0005, 0x0006, 0x0007
REG_A_RAIN, REG_A_DIST, REG_A_PIR, REG_A_RADAR = 0x0008, 0x0009, 0x000A, 0x000B
REG_A_HR, REG_A_SPO2, REG_A_POT, REG_A_SAMPLE_SEQ = 0x000C, 0x000D, 0x000E, 0x000F

# B 区
REG_B_STATUS1, REG_B_STATUS2, REG_B_FW_VERSION = 0x0010, 0x0011, 0x0012
REG_B_OK_FRAMES, REG_B_CRC_ERR, REG_B_EXC_FRAMES = 0x0013, 0x0014, 0x0015
REG_B_UPTIME_MIN, REG_B_RESET_REASON, REG_B_WD_TIMEOUTS = 0x0016, 0x0017, 0x0018
REG_B_MODE, REG_B_LOCAL_ALARM, REG_B_WINDOW_STATUS = 0x0019, 0x001A, 0x001B

# C 区
REG_C_CTRL1, REG_C_FAN_PWM, REG_C_MOTOR_PWM, REG_C_SERVO = 0x0020, 0x0021, 0x0022, 0x0023
REG_C_RGB_R, REG_C_RGB_G, REG_C_RGB_B = 0x0024, 0x0025, 0x0026
REG_C_ALARM_TEMP, REG_C_ALARM_SMOKE, REG_C_ALARM_HR = 0x0027, 0x0028, 0x0029
REG_C_WD_TIMEOUT, REG_C_SAFE_BITMAP, REG_C_SLAVE_ADDR = 0x002A, 0x002B, 0x002C
REG_C_BAUD_CODE, REG_C_ACQ_PERIOD, REG_C_COMMAND = 0x002D, 0x002E, 0x002F

# D 区
REG_D_HEAD, REG_D_SEQ, REG_D_RATE, REG_D_STAMP = 0x0100, 0x0101, 0x0102, 0x0103
REG_D_DATA_FIRST = 0x0104
WINDOW_SAMPLES = 100
WINDOW_AXES = 6
WINDOW_WORDS = WINDOW_SAMPLES * WINDOW_AXES  # 600 个寄存器
REG_D_DATA_LAST = REG_D_DATA_FIRST + WINDOW_WORDS - 1  # = 0x035B

# E 区：6 轴 × 6 类特征，按"每轴一组"排列（docs/02 §5 E 区地址段顺序）
FEATURE_NAMES = ("均值", "标准差", "峰峰值", "过零率", "主频", "主频带能量占比")
AXIS_NAMES = ("ax", "ay", "az", "gx", "gy", "gz")
REG_E_LAYOUT_PER_AXIS = "per-axis"  # 每轴连续 6 个特征（与地址段顺序一致）
REG_E_LAYOUT_PER_FEATURE = "per-feature"  # 每个特征连续 6 轴（另一种可能布局）

# 异常码（docs/02 §4）
ILLEGAL_FUNCTION, ILLEGAL_ADDRESS, ILLEGAL_VALUE = 0x01, 0x02, 0x03

# 数量边界（docs/02 §4）：FC03/FC04 单次 1~125，FC10 单次 1~123
MAX_READ_COUNT = 125
MAX_WRITE_COUNT = 123

#: C 区数值范围表（docs/02 §5）。未列出的 C 区寄存器不校验范围。
#: 元组含义：(最小值, 最大值, 是否 int16, 说明)
C_REG_RANGES: Dict[int, Tuple[int, int, bool, str]] = {
    REG_C_CTRL1: (0, 0xFFFF, False, "控制字 1 位图"),
    REG_C_FAN_PWM: (0, 1000, False, "风扇 PWM"),
    REG_C_MOTOR_PWM: (-1000, 1000, True, "电机 PWM（符号=转向）"),
    REG_C_SERVO: (0, 180, False, "舵机角度"),
    REG_C_RGB_R: (0, 255, False, "RGB R"),
    REG_C_RGB_G: (0, 255, False, "RGB G"),
    REG_C_RGB_B: (0, 255, False, "RGB B"),
    REG_C_ALARM_TEMP: (-550, 1250, True, "温度报警上限 ×10℃"),
    REG_C_ALARM_SMOKE: (0, 4095, False, "烟雾报警上限"),
    REG_C_ALARM_HR: (0, 300, False, "心率报警下限 bpm"),
    REG_C_WD_TIMEOUT: (0, 65535, False, "通信看门狗超时 ×100ms"),
    REG_C_SAFE_BITMAP: (0, 0xFFFF, False, "安全状态位图"),
    REG_C_SLAVE_ADDR: (1, 247, False, "从机地址（0 是广播，非法）"),
    REG_C_BAUD_CODE: (0, 4, False, "波特率代码 0=9600…4=115200"),
    REG_C_ACQ_PERIOD: (1, 60000, False, "采集周期 ms"),
    REG_C_COMMAND: (0, 0xFFFF, False, "命令寄存器 0xA5A5/0x5A5A"),
}

#: 控制字 1（0x0020）位定义（docs/02 §5 C 区）
CTRL1_BITS = {
    0: "蜂鸣器",
    1: "风扇使能",
    2: "电机使能",
    3: "RGB 使能",
    4: "舵机使能",
    5: "允许本地自治",
    6: "清统计（自清）",
    7: "保存配置到 Flash",
    8: "蜂鸣器长鸣",
    9: "全部输出安全关断",
    10: "强制窗口重采",
}


# --------------------------------------------------------------------------------------
# 通用小工具
# --------------------------------------------------------------------------------------


def to_int16(value: int) -> int:
    """把 0~65535 的寄存器原始值解释成 int16（Modbus 寄存器本身就是大端 16 位）。"""
    value &= 0xFFFF
    return value - 0x10000 if value & 0x8000 else value


def from_int16(value: int) -> int:
    """把 int16 编码成寄存器原始值（0~65535）。"""
    return value & 0xFFFF


def clamp_int16(value: float) -> int:
    """把浮点裁进 int16 并取整，避免溢出后回绕成相反符号。"""
    return int(max(-32768, min(32767, round(value))))


# --------------------------------------------------------------------------------------
# 寄存器表（纯逻辑，不依赖 pymodbus，便于 --selftest）
# --------------------------------------------------------------------------------------


class RegisterTable:
    """A/B/C/D/E 五区寄存器表的内存实现（线程安全）。

    这是模拟器的"唯一数据源"：后台线程按 ``--rate`` 周期更新 A 区与 D 区窗口，
    所有读请求都先从本表取快照，因此不会出现"读一半被改写"的撕裂。
    """

    def __init__(self, addr: int = 0x01, rate_s: float = 2.0, sample_rate_hz: float = 50.0,
                 seed: Optional[int] = None, strict_writes: bool = True) -> None:
        """:param addr: 本机从机地址。:param rate_s: 窗口更新周期（秒）。
        :param sample_rate_hz: 伪造的采样率（写入 0x0102 = ×10Hz）。
        :param seed: 随机种子（便于复现）。:param strict_writes: 是否做写入校验。
        """
        self.lock = threading.RLock()
        self.strict_writes = strict_writes
        self.sample_rate_hz = sample_rate_hz
        self.rate_s = rate_s
        self.rng = random.Random(seed)

        self.regs: List[int] = [0] * REG_TABLE_SIZE
        self.start_time = time.time()
        self.window_seq = 0
        self.window_stamp_10ms = 0
        self.ok_frames = 0
        self.crc_err = 0
        self.exc_frames = 0
        self.wd_timeouts = 0
        self.acq_round = 0
        self.last_window: Optional[List[List[int]]] = None
        self.last_features: List[int] = [0] * 36
        self.write_log: List[Tuple[float, int, int]] = []

        self.set_value(REG_C_SLAVE_ADDR, addr, validate=False)
        self.set_value(REG_C_BAUD_CODE, 0, validate=False)
        self.set_value(REG_C_ACQ_PERIOD, 500, validate=False)
        self.set_value(REG_C_WD_TIMEOUT, 30, validate=False)  # 30×100ms = 3s
        self.set_value(REG_C_FAN_PWM, 0, validate=False)
        self.set_value(REG_B_RESET_REASON, 0, validate=False)  # 0=上电
        self.set_value(REG_B_FW_VERSION, 0x0100, validate=False)
        self.set_value(REG_B_STATUS2, 0, validate=False)
        self.update_acquisition()
        self.update_window()

    # --- 低层读写 ---

    def get_value(self, address: int) -> int:
        """取单个寄存器的原始值（越界抛 ``IndexError``）。"""
        with self.lock:
            return self.regs[address]

    def set_value(self, address: int, value: int, validate: bool = True) -> None:
        """写单个寄存器；``validate=True`` 时按 C 区范围表校验。

        :raises IndexError: 地址越界。
        :raises ValueError: 地址属于只读区，或值超出合法范围。
        """
        with self.lock:
            if not 0 <= address < REG_TABLE_SIZE:
                raise IndexError(f"地址 0x{address:04X} 超出寄存器表")
            if validate and self.strict_writes:
                self.validate_write(address, [value & 0xFFFF])
            self.regs[address] = value & 0xFFFF
            if REG_C_FIRST <= address <= REG_C_LAST:
                self.write_log.append((time.time(), address, value & 0xFFFF))
                if len(self.write_log) > 256:
                    del self.write_log[:-256]
                self._apply_write_side_effect(address)

    def read_block(self, address: int, count: int) -> List[int]:
        """按地址连续性读一批寄存器（快照，越界抛 ``IndexError``）。

        :param address: 起始地址。
        :param count: 数量。
        :return: 原始值列表。
        """
        if count < 0:
            raise IndexError("数量为负")
        with self.lock:
            if not 0 <= address < REG_TABLE_SIZE or address + count > REG_E_LAST + 1:
                raise IndexError(f"读范围 0x{address:04X}×{count} 越界")
            return list(self.regs[address : address + count])

    def write_block(self, address: int, values: Sequence[int]) -> None:
        """连续写一批寄存器，先整体校验再落盘（避免"写一半失败"）。"""
        with self.lock:
            if not 0 <= address < REG_TABLE_SIZE or address + len(values) > REG_TABLE_SIZE:
                raise IndexError(f"写范围 0x{address:04X}×{len(values)} 越界")
            if self.strict_writes:
                self.validate_write(address, values)
            for offset, value in enumerate(values):
                self.set_value(address + offset, value, validate=False)

    def validate_write(self, address: int, values: Sequence[int]) -> None:
        """写入前的整体校验（与 PDU 层共用 :func:`check_write_request`，规则不会漂移）。

        :raises ValueError: 只读区或值越界（README/自检据此断言异常码 02/03）。
        """
        code = check_write_request(address, list(values))
        if code is None:
            return
        if code == ILLEGAL_VALUE:
            raise ValueError(f"写 0x{address:04X}×{len(values)} 的值不合法 → 异常 03")
        raise ValueError(f"写 0x{address:04X}×{len(values)} 越出 C 区（只读区）→ 异常 02")

    def _apply_write_side_effect(self, address: int) -> None:
        """C 区写入后的副作用（自清位、命令处理），与 F411 侧行为对齐。"""
        if address == REG_C_CTRL1:
            ctrl = self.regs[REG_C_CTRL1]
            if ctrl & (1 << 6):  # 清统计：自清
                self.ok_frames = self.crc_err = self.exc_frames = self.wd_timeouts = 0
                ctrl &= ~(1 << 6)
                self.regs[REG_C_CTRL1] = ctrl
            if ctrl & (1 << 10):  # 强制窗口重采：自清并立刻刷新一次窗口
                ctrl &= ~(1 << 10)
                self.regs[REG_C_CTRL1] = ctrl
                self.update_window(force=True)
            if ctrl & (1 << 7):  # 保存配置到 Flash：模拟器无 Flash，自清并打印
                ctrl &= ~(1 << 7)
                self.regs[REG_C_CTRL1] = ctrl
        elif address == REG_C_COMMAND:
            command = self.regs[REG_C_COMMAND]
            if command == 0xA5A5:  # 恢复默认
                self.set_value(REG_C_FAN_PWM, 0, validate=False)
                self.set_value(REG_C_ACQ_PERIOD, 500, validate=False)
            # 0x5A5A 软重启：模拟器不真的重启，仅保留值供读回
        elif address == REG_C_SLAVE_ADDR:
            # 真 F411 写入后立即生效；模拟器不需要（服务端过滤用启动参数）
            pass

    # --- 动态数据生成 ---

    def update_acquisition(self) -> None:
        """更新 A/B 区：正弦游走 + 噪声伪造环境量（docs/02 §5 A/B 区）。"""
        with self.lock:
            self.acq_round += 1
            t = self.acq_round * 0.5
            noise = lambda amplitude: self.rng.uniform(-amplitude, amplitude)  # noqa: E731

            def adc(phase: float, base: float, swing: float) -> int:
                """伪造一路 12 位 ADC（0~4095）。"""
                value = base + swing * math.sin(t / 7.0 + phase) + noise(30)
                return int(max(0, min(4095, value)))

            self.set_value(REG_A_TEMP, from_int16(int(250 + 50 * math.sin(t / 30.0) + noise(2))), False)
            self.set_value(REG_A_HUMI, int(600 + 100 * math.sin(t / 25.0 + 1.0) + noise(5)), False)
            self.set_value(REG_A_PRESS, int(10132 + 30 * math.sin(t / 60.0) + noise(2)), False)
            self.set_value(REG_A_LIGHT, adc(0.0, 2000, 1500), False)
            self.set_value(REG_A_MQ2, adc(1.2, 800, 300), False)
            self.set_value(REG_A_MQ135, adc(2.1, 1200, 400), False)
            self.set_value(REG_A_FLAME, adc(0.7, 300, 100), False)
            self.set_value(REG_A_SOIL, int(45 + 15 * math.sin(t / 40.0) + noise(2)), False)
            self.set_value(REG_A_RAIN, adc(3.0, 1000, 500), False)
            self.set_value(REG_A_DIST, int(500 + 300 * math.sin(t / 9.0) + noise(10)), False)
            self.set_value(REG_A_PIR, 1 if math.sin(t / 5.0) > 0 else 0, False)
            self.set_value(REG_A_RADAR, 1 if math.sin(t / 6.0) > 0.3 else 0, False)
            self.set_value(REG_A_HR, int(72 + 8 * math.sin(t / 3.0) + noise(1)), False)
            self.set_value(REG_A_SPO2, int(97 + 1.5 * math.sin(t / 11.0) + noise(0.3)), False)
            self.set_value(REG_A_POT, adc(1.7, 2048, 2000), False)
            self.set_value(REG_A_SAMPLE_SEQ, self.acq_round & 0xFFFF, False)

            # B 区状态与统计
            self.set_value(REG_B_STATUS1, 0x03FF, False)  # 前 10 个传感器全在线
            self.set_value(REG_B_OK_FRAMES, self.ok_frames & 0xFFFF, False)
            self.set_value(REG_B_CRC_ERR, self.crc_err & 0xFFFF, False)
            self.set_value(REG_B_EXC_FRAMES, self.exc_frames & 0xFFFF, False)
            self.set_value(REG_B_UPTIME_MIN, int((time.time() - self.start_time) / 60.0) & 0xFFFF, False)
            self.set_value(REG_B_WD_TIMEOUTS, self.wd_timeouts & 0xFFFF, False)
            self.set_value(REG_B_MODE, 1, False)  # 1=远程受控
            self.set_value(REG_B_LOCAL_ALARM, 0, False)
            self.set_value(REG_B_WINDOW_STATUS, self.regs[REG_D_HEAD], False)

    def update_window(self, force: bool = False) -> None:
        """生成一个新的六轴窗口（100 样本 × 6 轴 int16）并刷新 D/E 区。

        波形 = 每轴独立频率/相位的正弦 + 高斯噪声，再按 int16 裁幅。
        """
        with self.lock:
            if not force and self.last_window is not None and time.time() < self.next_window_time:
                return
            self.window_seq = (self.window_seq + 1) & 0xFFFF
            self.window_stamp_10ms = int((time.time() - self.start_time) * 100) & 0xFFFF
            frequencies = (1.5, 1.5, 3.0, 2.0, 2.5, 4.0)  # Hz，加速度轴低频、陀螺轴高频
            amplitudes = (8000, 6000, 10000, 1500, 1200, 2000)
            phases = (0.0, 1.0, 2.0, 0.5, 1.5, 2.5)
            window: List[List[int]] = []
            for sample in range(WINDOW_SAMPLES):
                row: List[int] = []
                for axis in range(WINDOW_AXES):
                    phase = 2 * math.pi * frequencies[axis] * sample / self.sample_rate_hz + phases[axis]
                    value = amplitudes[axis] * math.sin(phase) + self.rng.gauss(0, amplitudes[axis] * 0.05)
                    row.append(clamp_int16(value))
                window.append(row)
            self.last_window = window

            # D 区头 + 数据（行优先：样本 0 的 6 轴在前）
            head = 0x8000 | (WINDOW_SAMPLES & 0x00FF)  # bit15 = 有效窗口就绪
            self.regs[REG_D_HEAD] = head
            self.regs[REG_D_SEQ] = self.window_seq
            self.regs[REG_D_RATE] = int(self.sample_rate_hz * 10) & 0xFFFF
            self.regs[REG_D_STAMP] = self.window_stamp_10ms
            for sample, row in enumerate(window):
                base = REG_D_DATA_FIRST + sample * WINDOW_AXES
                for axis, value in enumerate(row):
                    self.regs[base + axis] = from_int16(value)

            # E 区特征由本窗口真实算出，保证与 D 区自洽
            self.last_features = compute_features(window, self.sample_rate_hz)
            for offset, value in enumerate(self.last_features):
                self.regs[REG_E_FIRST + offset] = from_int16(value)

            self.set_value(REG_B_WINDOW_STATUS, head, False)
            self.next_window_time = time.time() + self.rate_s

    def age_seconds(self) -> float:
        """从机已运行秒数。"""
        return time.time() - self.start_time

    # --- 供 pymodbus 数据块使用的接口 ---

    def read_for_modbus(self, address: int, count: int) -> List[int]:
        """pymodbus 数据块读接口（地址语义由 ``AddressModeProbe`` 校正后调用）。"""
        return self.read_block(address, count)

    def write_for_modbus(self, address: int, values: Sequence[int]) -> None:
        """pymodbus 数据块写接口（可能抛 ``ValueError``，由调用方转异常 02/03）。"""
        self.write_block(address, values)


def compute_features(window: Sequence[Sequence[int]], sample_rate_hz: float) -> List[int]:
    """由六轴窗口算 36 维特征向量（对应 docs/02 §5 E 区）。

    每轴 6 个特征，顺序与 E 区地址段一致：
    均值 / 标准差 / 峰峰值 / 过零率(×100) / 主频(×10Hz) / 主频带能量占比(×1000)。

    :param window: ``100×6`` 的 int16 窗口。
    :param sample_rate_hz: 采样率，用于把频谱 bin 换算成 Hz。
    :return: 36 个 int16 值（未做 0xFFFF 掩码）。
    """
    values = list(window)
    samples = len(values)
    features: List[int] = []
    for axis in range(WINDOW_AXES):
        series = [row[axis] for row in values]
        mean = sum(series) / samples
        variance = sum((x - mean) ** 2 for x in series) / samples
        std = math.sqrt(variance)
        peak_to_peak = max(series) - min(series)
        # 过零率：去均值后符号变化次数占比 ×100
        centered = [x - mean for x in series]
        crossings = sum(
            1 for i in range(1, samples) if (centered[i - 1] < 0) != (centered[i] < 0)
        )
        zcr_x100 = int(crossings * 100 / max(1, samples - 1))
        # 主频与主频带能量占比：用朴素 DFT 在 0.5~25Hz 范围扫峰值（100 点足够快）
        best_freq, best_power, total_power = 0.0, 0.0, 0.0
        for bin_index in range(1, samples // 2):
            omega = 2 * math.pi * bin_index / samples
            real = sum(centered[n] * math.cos(omega * n) for n in range(samples))
            imag = sum(centered[n] * math.sin(omega * n) for n in range(samples))
            power = real * real + imag * imag
            frequency = bin_index * sample_rate_hz / samples
            total_power += power
            if 0.5 <= frequency <= 25.0 and power > best_power:
                best_power, best_freq = power, frequency
        ratio_x1000 = int(best_power * 1000 / total_power) if total_power > 0 else 0
        features.extend(
            [
                clamp_int16(mean),
                clamp_int16(std),
                clamp_int16(peak_to_peak),
                zcr_x100,
                int(round(best_freq * 10)),
                ratio_x1000,
            ]
        )
    return features


# --------------------------------------------------------------------------------------
# pymodbus 数据块适配层
# --------------------------------------------------------------------------------------


def import_pymodbus_slave_parts():
    """导入 pymodbus 3.x 从机所需组件，缺失时给出中文安装提示。

    :return: ``(ModbusSlaveContext, ModbusSequentialDataBlock, ModbusSerialServer,
    FramerType, ModbusDeviceIdentity, exceptions_dict)``。
    """
    try:
        from pymodbus.datastore import ModbusSequentialDataBlock, ModbusSlaveContext
    except ImportError as exc:  # pragma: no cover
        safe_print(
            "错误：未安装 pymodbus 3.x（datastore 组件缺失）。\n"
            "  请执行：pip install -r tools/requirements.txt"
        )
        raise SystemExit(2) from exc

    server_cls = None
    for module_name in ("pymodbus.server", "pymodbus.server.sync", "pymodbus.server.async_io"):
        try:
            module = __import__(module_name, fromlist=["ModbusSerialServer"])
            server_cls = getattr(module, "ModbusSerialServer")
            break
        except (ImportError, AttributeError):
            continue
    if server_cls is None:
        safe_print("错误：当前 pymodbus 版本找不到 ModbusSerialServer（需要 3.x）。")
        raise SystemExit(2)

    framer_type = None
    try:
        from pymodbus.framer import FramerType

        framer_type = FramerType.RTU
    except ImportError:
        for module_name, attr in (
            ("pymodbus.framer.rtu_framer", "ModbusRtuFramer"),
            ("pymodbus.framer", "ModbusRtuFramer"),
        ):
            try:
                framer_type = getattr(__import__(module_name, fromlist=[attr]), attr)
                break
            except (ImportError, AttributeError):
                continue

    identity = None
    try:
        from pymodbus.server import ModbusDeviceIdentity

        identity = ModbusDeviceIdentity(VendorName="MyProject", ProductCode="sim", MajorMinorRevision="v0.2")
    except Exception:  # noqa: BLE001 - 身份对象可选，构造失败不影响功能
        identity = None

    pymodbus_exceptions = {}
    try:
        from pymodbus import exceptions as exc_module

        # 注意：不同版本里异常类的名字并不一致——3.9 的 exceptions 模块里**没有**
        # IllegalAddressException / IllegalValueException，只有 ModbusException 一族。
        # 这里按"是 BaseException 的子类"来收集，别再靠名字后缀过滤（会漏掉全部）。
        pymodbus_exceptions = {
            name: getattr(exc_module, name)
            for name in dir(exc_module)
            if not name.startswith("_")
            and isinstance(getattr(exc_module, name), type)
            and issubclass(getattr(exc_module, name), BaseException)
        }
    except ImportError:  # pragma: no cover
        pass
    return ModbusSlaveContext, ModbusSequentialDataBlock, server_cls, framer_type, identity, pymodbus_exceptions


def build_spec_pdu_classes():
    """构造符合 docs/02 §4/§5 边界的 PDU 子类（FC03 / FC04 / FC06 / FC10）。

    **为什么必须在这一层校验**（两条都已在 pymodbus 3.9.2 源码上核实）：

    1. **数量越界只在组帧时校验**：``register_message.py`` 里 ``verifyCount(125)``
       只在 ``encode()`` 里调用；**收到请求时不校验**，于是"读 126 个"会一路走到
       数据块并被当成"读 126 个跨区块寄存器"成功返回。
    2. **写路径的返回码会被丢掉**：``ModbusSlaveContext.setValues`` 内部是
       ``self.store[...].setValues(address, values)``——**没有 return**，
       所以 PDU 里 ``rc = await context.async_setValues(...)`` 永远是 ``None``，
       "只读区写入 → 异常 02"永远发不出去（数据块返回的异常码被丢弃）。
       若改成抛异常，服务端只会回 ``SLAVE_FAILURE``(0x04)，也不是规格要求的 02。

    因此这里按 pymodbus 官方扩展点 ``custom_pdu`` 注册子类：``DecodePDU.register()``
    会覆盖同名 ``function_code`` 的默认类。子类只做"**先按规格校验，不合法就直接回异常
    响应**"，合法则完全交给父类（即原本的实现），不重复实现 Modbus 逻辑。

    :return: 可注册的 PDU 类列表；拿不到基类时返回 ``[]``（此时退回数据块兜底）。
    """
    try:
        from pymodbus.pdu import register_message as rm
        from pymodbus.pdu.pdu import ExceptionResponse
    except ImportError:
        return []

    def make_read(base_cls):
        """生成读请求子类（FC03/FC04）。"""
        if base_cls is None:
            return None

        class _SpecRead(base_cls):  # type: ignore[misc, valid-type]
            """数量 1~125 且不得跨区块（docs/02 §4）。"""

            async def update_datastore(self, context):  # noqa: ANN001, ANN201
                """校验通过后走父类实现。"""
                code = check_read_request(self.address, self.count)
                if code is not None:
                    return ExceptionResponse(self.function_code, code)
                return await super().update_datastore(context)

        _SpecRead.__name__ = f"Spec{base_cls.__name__}"
        return _SpecRead

    def make_write_single(base_cls):
        """生成 FC06 子类。"""
        if base_cls is None:
            return None

        class _SpecWriteSingle(base_cls):  # type: ignore[misc, valid-type]
            """只读区 → 异常 02；值越界 → 异常 03（docs/02 §4/§5）。"""

            async def update_datastore(self, context):  # noqa: ANN001, ANN201
                """校验通过后走父类实现。"""
                code = check_write_request(self.address, list(self.registers))
                if code is not None:
                    return ExceptionResponse(self.function_code, code)
                return await super().update_datastore(context)

        _SpecWriteSingle.__name__ = f"Spec{base_cls.__name__}"
        return _SpecWriteSingle

    def make_write_multiple(base_cls):
        """生成 FC10 子类。"""
        if base_cls is None:
            return None

        class _SpecWriteMultiple(base_cls):  # type: ignore[misc, valid-type]
            """数量 1~123；只读区 → 异常 02；值越界 → 异常 03（docs/02 §4/§5）。"""

            async def update_datastore(self, context):  # noqa: ANN001, ANN201
                """校验通过后走父类实现。"""
                code = check_write_request(self.address, list(self.registers))
                if code is not None:
                    return ExceptionResponse(self.function_code, code)
                return await super().update_datastore(context)

        _SpecWriteMultiple.__name__ = f"Spec{base_cls.__name__}"
        return _SpecWriteMultiple

    classes = [
        make_read(getattr(rm, "ReadHoldingRegistersRequest", None)),
        make_read(getattr(rm, "ReadInputRegistersRequest", None)),
        make_write_single(getattr(rm, "WriteSingleRegisterRequest", None)),
        make_write_multiple(getattr(rm, "WriteMultipleRegistersRequest", None)),
    ]
    classes.extend(build_unsupported_pdu_classes())
    return [cls for cls in classes if cls is not None]


#: docs/02 §4 未列出、必须回异常 01（非法功能）的功能码 → pymodbus 里的类名
UNSUPPORTED_PDU_SOURCES: Tuple[Tuple[int, str], ...] = (
    (0x01, "ReadCoilsRequest"),
    (0x02, "ReadDiscreteInputsRequest"),
    (0x05, "WriteSingleCoilRequest"),
    (0x0F, "WriteMultipleCoilsRequest"),
)


def build_unsupported_pdu_classes():
    """把 pymodbus 原生支持的线圈类功能码改成"回异常 01"（docs/02 §4）。

    **为什么需要**：pymodbus 默认注册了 FC01/02/05/0F（线圈读写），而本项目
    ``docs/02`` §4 只支持 0x03/0x04/0x06/0x10，其它功能码必须回异常码 01。
    不改掉的话，模拟器会"接受"线圈写入并回正常响应，与真 F411 行为不一致，
    会让 S3 侧把错误的实现当成对的。

    :return: 覆盖用 PDU 类列表（取不到基类时为空）。
    """
    try:
        from pymodbus.pdu import bit_message as bm
        from pymodbus.pdu.pdu import ExceptionResponse
    except ImportError:
        return []

    classes = []
    for code, attr in UNSUPPORTED_PDU_SOURCES:
        base_cls = getattr(bm, attr, None)
        if base_cls is None:
            continue

        class _Unsupported(base_cls):  # type: ignore[misc, valid-type]
            """本项目不支持的功能码：一律回异常 01（非法功能）。"""

            function_code = code

            async def update_datastore(self, context):  # noqa: ANN001, ANN201
                """不碰数据区，直接回异常 01。"""
                _ = context
                return ExceptionResponse(self.function_code, ILLEGAL_FUNCTION)

        _Unsupported.__name__ = f"Unsupported_{attr}"
        classes.append(_Unsupported)
    return classes


class SimDataBlock:
    """把 :class:`RegisterTable` 包装成 pymodbus 数据块（读时生成，写时校验）。

    为什么不用现成的 ``ModbusSequentialDataBlock`` 直接暴露 ``regs``：

    1. **地址换算**：``ModbusSlaveContext`` 在各小版本里对传给数据块的地址做了 ``+1``
       （3.6/3.7 可用 ``zero_mode=True`` 关掉；3.8+ 删除该参数、固定 ``+1``，
       见上游 ``datastore/context.py`` 的 ``address += 1``）。本类通过构造时的
       ``address_offset``（由 :func:`probe_address_mode` 实测得出，正常为 ``-1``）
       换算回"寄存器的 0 基地址"，保证请求 0x0104 就读到 0x0104。
    2. **写入校验**：现成数据块接受任意范围写入，而 docs/02 要求只读区回异常 02、
       非法值回异常 03。

    异常如何返回（**pymodbus 的真实契约**，已对 3.9.2 源码核实）：
    数据块**不抛异常**，而是返回整数异常码 —— ``pdu/register_message.py`` 里
    ``if isinstance(values, int): return ExceptionResponse(self.function_code, values)``，
    写入则是 ``if rc:`` 走异常分支。因此这里返回 ``0x02`` / ``0x03``；
    同时兼容老版本里存在的 ``IllegalAddressException`` / ``IllegalValueException``
    （若运行时能拿到这两个类，则抛出它们，两条路径服务端都能正确处理）。
    """

    def __init__(self, table: RegisterTable, modbus_exceptions: Dict[str, type],
                 address_offset: int = 0) -> None:
        """:param table: 寄存器表。:param modbus_exceptions: pymodbus 异常类字典。
        :param address_offset: 地址校正偏移（0 或 -1，由启动探测决定）。
        """
        self.table = table
        self.address_offset = address_offset
        self.exceptions = modbus_exceptions

    # pymodbus 数据块协议
    def validate(self, *args, **kwargs) -> bool:  # noqa: ANN002, ANN003
        """pymodbus 要求的范围检查接口（兼容 3.6~3.9 的两种签名）。"""
        try:
            address, count = _pick_address_count(args, kwargs)
        except TypeError:
            return True  # 签名不认识时不做拦截，交给 getValues/setValues 判
        try:
            self.table.read_block(address + self.address_offset, count)
            return True
        except IndexError:
            return False

    def getValues(self, *args, **kwargs):  # noqa: ANN003
        """读寄存器；数量越界 / 跨区 / 地址越界时返回异常码（03 / 02）。

        数量检查是"双保险"：正常路径由 :func:`build_spec_pdu_classes` 注册的 PDU 子类
        在 ``update_datastore`` 里拦下（能正确回异常 03）；若当前 pymodbus 版本不支持
        自定义 PDU，则在这里兜底，保证不会出现"一次读 126 个还成功返回"。
        """
        address, count = _pick_address_count(args, kwargs)
        real = address + self.address_offset
        if not 1 <= count <= MAX_READ_COUNT:
            return self._raise_or_code(
                "IllegalValueException", ILLEGAL_VALUE,
                f"读数量 {count} 越界（docs/02 §4 限制 1~{MAX_READ_COUNT}）",
            )
        if not check_span(real, count):
            return self._raise_or_code(
                "IllegalAddressException", ILLEGAL_ADDRESS,
                f"读范围 0x{real:04X}×{count} 越出合法区块（docs/02 §4）",
            )
        try:
            return self.table.read_for_modbus(real, count)
        except IndexError as exc:
            return self._raise_or_code("IllegalAddressException", ILLEGAL_ADDRESS, str(exc))

    def setValues(self, *args, **kwargs):  # noqa: ANN003
        """写寄存器；只读区 → 异常 02，值越界 → 异常 03（与 F411 行为一致）。

        :return: ``None`` 表示成功；返回整数表示异常码（pymodbus 会组异常响应）。
        """
        address, values = _pick_address_values(args, kwargs)
        real = address + self.address_offset
        try:
            self.table.write_for_modbus(real, values)
            return None
        except ValueError as exc:
            message = str(exc)
            if "异常 03" in message:
                return self._raise_or_code("IllegalValueException", ILLEGAL_VALUE, message)
            return self._raise_or_code("IllegalAddressException", ILLEGAL_ADDRESS, message)
        except IndexError as exc:
            return self._raise_or_code("IllegalAddressException", ILLEGAL_ADDRESS, str(exc))

    def _raise_or_code(self, name: str, code: int, message: str):
        """统一异常出口：有对应 pymodbus 异常类就抛，否则返回整数异常码。

        pymodbus 3.9 的 ``exceptions`` 模块里**没有** ``IllegalAddressException`` /
        ``IllegalValueException``（它们只存在于更早的版本），所以整数返回才是主路径。
        """
        cls = self.exceptions.get(name)
        if cls is not None:
            try:
                raise cls(message)
            except TypeError:
                raise cls() from None
        return code


#: 探测用的"魔法值"与地址（取只读区，避免污染 C 区配置）
PROBE_MAGIC = 0x5A5B
PROBE_REQ_ADDRESS = REG_A_PIR  # 0x000A，A 区只读


class _ProbeBlock:
    """探测用数据块：只做"记录写入地址"，读接口返回安全值（不做范围校验、不抛异常）。"""

    def __init__(self) -> None:
        """初始化探测块。"""
        self.writes: List[Tuple[int, int]] = []
        self.last_write_delta: Optional[int] = None

    def validate(self, *args, **kwargs) -> bool:  # noqa: ANN002, ANN003
        """兼容 3.6~3.9 的两种 validate 签名（``fc_as_hex, address`` 与 ``address``）。"""
        return True

    def getValues(self, *args, **kwargs):  # noqa: ANN002, ANN003
        """返回零值：探测只关心写入落点，读值不参与判断。"""
        address, count = _pick_address_count(args, kwargs)
        return [0] * max(1, int(count))

    def setValues(self, *args, **kwargs) -> None:  # noqa: ANN002, ANN003
        """记录 pymodbus 传给数据块的地址，用于推算地址偏移。"""
        address, values = _pick_address_values(args, kwargs)
        self.writes.append((int(address), len(values)))
        if self.last_write_delta is None:
            self.last_write_delta = int(address)
        del self.writes[1:]  # 只保留首次写入


def _pick_address_count(args: Sequence[object], kwargs: Dict[str, object]) -> Tuple[int, int]:
    """从 ``getValues`` 的位置参数/关键字参数里取出 ``(address, count)``。

    pymodbus 不同版本的调用形式有 ``getValues(address, count)``、
    ``getValues(fc_as_hex, address, count)`` 两种，这里统一兼容。
    """
    if "address" in kwargs:
        return int(kwargs["address"]), int(kwargs.get("count", 1) or 1)  # type: ignore[arg-type]
    numbers = [int(v) for v in args if isinstance(v, int)]
    if len(numbers) >= 2:
        return numbers[-2], numbers[-1]
    if len(numbers) == 1:
        return numbers[0], 1
    raise TypeError(f"无法从 {args!r}/{kwargs!r} 解析 address/count")


def _pick_address_values(args: Sequence[object], kwargs: Dict[str, object]) -> Tuple[int, List[int]]:
    """从 ``setValues`` 的位置参数/关键字参数里取出 ``(address, values)``。"""
    values: object = kwargs.get("values")
    if values is None:
        for arg in reversed(args):
            if isinstance(arg, (list, tuple)):
                values = arg
                break
    value_list = [int(v) for v in (values or [])]  # type: ignore[union-attr]
    if "address" in kwargs:
        return int(kwargs["address"]), value_list  # type: ignore[arg-type]
    numbers = [int(v) for v in args if isinstance(v, int)]
    if not numbers:
        raise TypeError(f"无法从 {args!r}/{kwargs!r} 解析 address")
    return numbers[0], value_list


def probe_address_mode(context, table: RegisterTable, verbose: bool = True) -> int:
    """运行时探测 pymodbus 传给数据块的地址偏移，返回需要施加的偏移（0 或 -1）。

    **为什么必须探测**：``ModbusSlaveContext`` 在各小版本里的行为不同——

    * 3.6/3.7 有 ``zero_mode``：为 True 时请求地址原样传给数据块（偏移 0）；
      为 False（默认）时 ``+1``；
    * 3.8/3.9 **删除了 ``zero_mode``**，一律 ``+1``（实测源码 ``address += 1``）。

    数据块收到的地址再减去数据块自身 ``address`` 才是索引。本模拟器的数据块是
    自己的实现（索引 = 请求地址），所以要抵消掉这个 ``+1``，即偏移 ``-1``。

    探测方法：放一个"只记录写入地址"的临时数据块，写一个魔法值并读回，
    找出魔法值实际落在哪个表地址上，直接算出偏移。**先探测再挂真块**，
    避免用错误的偏移做一次真实写入（可能误改 C 区配置）。

    :return: 0 表示请求地址与寄存器表地址一一对应；-1 表示需要 ``-1`` 校正。
    """
    store = getattr(context, "store", None)
    if not isinstance(store, dict):
        if verbose:
            safe_print("提示：当前 pymodbus 的 context 结构不支持数据块替换，偏移按 0 处理。")
        return 0

    probe = _ProbeBlock()
    probe_address = REG_A_PIR & 0xFFFE  # 取偶数地址，便于同时观察 ±1 的落点
    original_block = store["h"]
    original_values = table.read_block(probe_address - 1, 3)
    store["h"] = probe
    mode = 0
    try:
        store["c"] = probe  # 一并覆盖，避免不同版本走不同键
        context.setValues(3, probe_address, [PROBE_MAGIC])
        delta = probe.last_write_delta
        if delta is not None:
            # delta 是数据块收到的地址；请求地址是 probe_address
            shift = int(delta) - probe_address  # 正常应为 0 或 +1
            mode = -shift
    except Exception as exc:  # noqa: BLE001 - 探测失败时退回保守假设
        if verbose:
            safe_print(f"地址偏移探测异常（{type(exc).__name__}: {exc}），保守按 0 处理。")
        mode = 0
    finally:
        store["h"] = original_block
        try:
            store["c"] = original_block
        except Exception:  # noqa: BLE001
            pass
        for index, value in enumerate(original_values):
            table.set_value(probe_address - 1 + index, value, validate=False)

    if verbose:
        safe_print(
            f"pymodbus 地址偏移探测：请求 0x{probe_address:04X} → 数据块收到 "
            f"0x{(probe_address + (-mode)):04X}，施加偏移 {mode} "
            f"（{'请求地址与寄存器表一一对应' if mode == 0 else '已抵消 pymodbus 的内部 +1'}）"
        )
    return mode


# --------------------------------------------------------------------------------------
# 服务端启动
# --------------------------------------------------------------------------------------


def _build_slave_context(ModbusSlaveContext, ModbusSequentialDataBlock):
    """构造 ``ModbusSlaveContext``，兼容 3.6~3.9 的签名差异。

    * 3.6/3.7 支持且推荐 ``zero_mode=True``（请求地址不加 1）；
    * 3.8/3.9 删掉了 ``zero_mode`` 参数，且内部固定 ``address += 1``。

    两种情况都由 :func:`probe_address_mode` 在运行时探测并抵消，这里只负责"能构造出来"。
    """
    kwargs = dict(
        di=ModbusSequentialDataBlock(0x01, [0] * REG_TABLE_SIZE),
        co=ModbusSequentialDataBlock(0x01, [0] * REG_TABLE_SIZE),
        hr=ModbusSequentialDataBlock(0x01, [0] * REG_TABLE_SIZE),
        ir=ModbusSequentialDataBlock(0x01, [0] * REG_TABLE_SIZE),
    )
    try:
        return ModbusSlaveContext(zero_mode=True, **kwargs)
    except TypeError:
        # 3.8+ 没有 zero_mode 参数
        return ModbusSlaveContext(**kwargs)


def _wrap_server_context(store, address: int, verbose: bool = True):
    """把 ``ModbusSlaveContext`` 包成 ``ModbusServerContext``（3.7+ 服务端要求）。

    3.7 以后的 ``ModbusSerialServer`` 签名是 ``(context: ModbusServerContext)``，
    直接传 ``ModbusSlaveContext`` 会在处理请求时才炸；更早的版本则直接接受
    ``ModbusSlaveContext``，此时回退。

    **只注册本机地址这一个 device_id**：这样请求别的从机地址时
    ``ModbusServerContext.__get_device`` / ``__getitem__`` 会抛 ``NoSuchSlaveException``，
    而 ``server/requesthandler.py`` 会捕获它并**返回不响应**——
    正好落实 docs/02 §2/§8"地址不匹配必须静默"的要求。
    （用 ``single=True`` 会让任何地址都命中同一份数据，无法实现静默。）

    参数名在各版本也不同：3.9 及更早是 ``slaves=``，3.15 是 ``devices=``，两种都试。
    """
    try:
        from pymodbus.datastore.context import ModbusServerContext
    except ImportError as exc:  # pragma: no cover
        if verbose:
            safe_print(f"提示：无法导入 ModbusServerContext（{exc}），使用裸 context。")
        return store

    for kwargs in (dict(slaves={address: store}, single=False),
                   dict(devices={address: store}, single=False)):
        try:
            return ModbusServerContext(**kwargs)
        except TypeError:
            continue
        except Exception as exc:  # noqa: BLE001
            if verbose:
                safe_print(f"提示：包装 ModbusServerContext 失败（{type(exc).__name__}: {exc}）。")
            break
    if verbose:
        safe_print("提示：未能包装 ModbusServerContext，使用裸 context（地址过滤由数据块承担）。")
    return store


def build_server(args: argparse.Namespace, table: RegisterTable, verbose: bool = True):
    """创建并返回 pymodbus 串口从机服务端（兼容 3.6~3.9 的构造签名）。

    已按 pymodbus 源码实测对齐：

    * ``ModbusSerialServer(context, *, framer=..., identity=..., **kwargs)``，
      串口参数（``port``/``baudrate``/``timeout``…）通过 ``**kwargs`` 传给底层 transport；
    * ``context`` 需要 ``ModbusServerContext`` 包装（3.7+），且只注册本机地址；
    * ``broadcast_enable=True`` 打开广播：地址 0x00 的写请求会执行但**不响应**
      （docs/02 §2），这是 pymodbus 服务端唯一支持广播的开关。

    :return: ``(server, store)``；``store`` 是 ``ModbusSlaveContext``（便于自检）。
    """
    (
        ModbusSlaveContext,
        ModbusSequentialDataBlock,
        server_cls,
        framer_type,
        identity,
        pymodbus_exceptions,
    ) = import_pymodbus_slave_parts()

    store = _build_slave_context(ModbusSlaveContext, ModbusSequentialDataBlock)

    # 用自带探测替换四个数据块（FC03/FC06/FC10 走 hr；FC04 走 ir；di/co 一并替换保持一致）
    offset = probe_address_mode(store, table, verbose=verbose)
    store.store["d"] = SimDataBlock(table, pymodbus_exceptions, offset)
    store.store["c"] = SimDataBlock(table, pymodbus_exceptions, offset)
    store.store["h"] = SimDataBlock(table, pymodbus_exceptions, offset)
    store.store["i"] = SimDataBlock(table, pymodbus_exceptions, offset)

    server_context = _wrap_server_context(store, args.addr, verbose=verbose)
    spec_pdu_classes = build_spec_pdu_classes()
    base_kwargs = dict(
        port=args.port, baudrate=args.baud, bytesize=8, parity="N",
        stopbits=1, timeout=args.timeout,
        broadcast_enable=True,       # 广播执行但不响应（docs/02 §2）
        ignore_missing_slaves=True,  # 地址不匹配时静默（docs/02 §8）
    )
    if framer_type is not None:
        base_kwargs["framer"] = framer_type
    if identity is not None:
        base_kwargs["identity"] = identity
    if spec_pdu_classes:
        base_kwargs["custom_pdu"] = spec_pdu_classes

    server = None
    last_error: Optional[Exception] = None
    # 依次尝试：(包装后的 context) → (裸 slave context) → (完全按关键字 context= 的旧式构造)
    for mode, context_candidate in (("pos", server_context), ("pos", store), ("kw", server_context)):
        attempt = dict(base_kwargs)
        if mode == "pos":
            positional = (context_candidate,)
        else:
            attempt["context"] = context_candidate
            positional = ()
        try:
            server = server_cls(*positional, **attempt)
            break
        except TypeError as exc:
            last_error = exc
            # custom_pdu 是较新版本才有的参数：去掉后重试（数量校验会退回数据块兜底）
            if "custom_pdu" in attempt:
                attempt.pop("custom_pdu")
                try:
                    server = server_cls(*positional, **attempt)
                    break
                except TypeError as inner:
                    last_error = inner
                    continue
                except Exception as inner:  # noqa: BLE001
                    last_error = inner
                    break
            continue
        except Exception as exc:  # noqa: BLE001 - 串口打开失败等
            last_error = exc
            break

    if server is not None and spec_pdu_classes:
        registered = getattr(getattr(server, "decoder", None), "lookup", {}) or {}
        if all(getattr(cls, "function_code", None) in registered for cls in spec_pdu_classes):
            if verbose:
                safe_print(
                    "已注册符合 docs/02 §4 数量边界的 PDU 子类："
                    + ", ".join(f"FC{cls.function_code:02X}" for cls in spec_pdu_classes)
                )
        elif verbose:
            safe_print(
                "提示：当前 pymodbus 版本未能注册自定义 PDU，"
                "数量越界改由数据块兜底检查（异常码仍是 03）。"
            )
    if server is None:
        safe_print(
            f"\n错误：无法创建 Modbus 串口从机服务端（{type(last_error).__name__}: {last_error}）\n"
            "  请依次检查：\n"
            "    1) USB 转 RS485 驱动是否装好（CH340/CP210x/FT232），设备管理器是否有黄色感叹号\n"
            "    2) 端口号是否正确（--port COM7；Windows 上没有物理口时可用 com0com 建虚拟串口对）\n"
            "    3) 端口是否被 **Modbus Poll / Modbus Slave / 串口助手 / 其它 Python 进程** 占用\n"
            "    4) 是否忘了插板子/接线：A→A、B→B、共地\n"
            "    5) pymodbus 版本是否为 3.6~3.9（3.10 起删除了本脚本使用的 ModbusSlaveContext 数据块接口，"
            "需换用新的 SimData/SimDevice API）\n"
        )
        raise SystemExit(2)
    return server, store


#: 合法的一次性连续读区间（docs/02 §4"起始+数量不得越界" + §6 实战帧划分）。
#: Modbus 只给"起始地址 + 数量"，没有区块概念；这里按文档把**允许跨越的区块边界**
#: 显式列出来，其它跨区读一律回异常 02。这样既允许 §6 示例①的 A+B 一次读 28 个、
#: 示例②的 5 块窗口分读，又能拒绝"从 A 区读进 B 区"这类未定义请求。
ALLOWED_SPANS: Tuple[Tuple[int, int], ...] = (
    (REG_A_FIRST, REG_A_LAST),                 # A 区单独
    (REG_B_FIRST, REG_B_LAST),                 # B 区单独
    (REG_A_FIRST, REG_B_LAST),                 # A+B 整体（含 0x0008 起跨 A/B 的读法）
    (REG_C_FIRST, REG_C_LAST),                 # C 区单独
    (REG_D_FIRST, REG_D_LAST),                 # D 区整窗（5 块分读的并集）
    (REG_E_FIRST, REG_E_LAST),                 # E 区 36 个特征
)


def check_span(address: int, count: int) -> bool:
    """判断 ``[address, address+count)`` 是否落在某个允许的连续区间内。

    :param address: 起始地址。
    :param count: 寄存器数量（可为 0）。
    :return: True 表示合法。
    """
    if count <= 0:
        return False
    end = address + count - 1
    return any(address >= low and end <= high for low, high in ALLOWED_SPANS)


def check_read_request(address: int, count: int) -> Optional[int]:
    """校验 FC03/FC04 读请求，返回 ``None``（合法）或异常码。

    :param address: 起始地址（Modbus 请求里的 0 基地址）。
    :param count: 数量。
    :return: ``None`` / :data:`ILLEGAL_VALUE`(0x03) / :data:`ILLEGAL_ADDRESS`(0x02)。
    """
    if not 1 <= count <= MAX_READ_COUNT:
        return ILLEGAL_VALUE
    if not check_span(address, count):
        return ILLEGAL_ADDRESS
    return None


def check_write_request(address: int, values: Sequence[int]) -> Optional[int]:
    """校验 FC06/FC10 写请求，返回 ``None``（合法）或异常码。

    规则（docs/02 §4/§5）：非 C 区（只读区）→ 异常 02；C 区值越界 → 异常 03；
    FC10 数量越界 → 异常 03。

    :param address: 起始地址。
    :param values: 待写入的寄存器值（0~65535）。
    :return: ``None`` / :data:`ILLEGAL_VALUE` / :data:`ILLEGAL_ADDRESS`。
    """
    if not values:
        return ILLEGAL_VALUE
    if len(values) > MAX_WRITE_COUNT:
        return ILLEGAL_VALUE
    if not check_span(address, len(values)):
        return ILLEGAL_ADDRESS
    if not (REG_C_FIRST <= address and address + len(values) - 1 <= REG_C_LAST):
        return ILLEGAL_ADDRESS  # 只读区写入
    for offset, raw in enumerate(values):
        current = address + offset
        rule = C_REG_RANGES.get(current)
        if rule is None:
            continue
        low, high, signed, _describe = rule
        value = to_int16(raw) if signed else raw & 0xFFFF
        # 舵机 0xFFFF 是"保持"的合法特殊值（docs/02 §5 C 区）
        if current == REG_C_SERVO and (raw & 0xFFFF) == 0xFFFF:
            continue
        if not low <= value <= high:
            return ILLEGAL_VALUE
    return None


def print_register_map(args: argparse.Namespace) -> None:
    """启动时打印寄存器区块表（docs/02 §5）。"""
    rows = [
        ["A 环境量", f"0x{REG_A_FIRST:04X}~0x{REG_A_LAST:04X}", "只读", "温度/湿度/气压/光照/烟雾/…/采样序号"],
        ["B 状态统计", f"0x{REG_B_FIRST:04X}~0x{REG_B_LAST:04X}", "只读", "在线位图/固件/帧计数/运行时间/复位原因"],
        ["C 控制配置", f"0x{REG_C_FIRST:04X}~0x{REG_C_LAST:04X}", "读写", "控制字/风扇/电机/舵机/RGB/阈值/地址/波特率"],
        ["D 六轴窗口", f"0x{REG_D_FIRST:04X}~0x{REG_D_LAST:04X}", "只读", "头+序号+100 样本×6 轴（600 寄存器）"],
        ["E 特征向量", f"0x{REG_E_FIRST:04X}~0x{REG_E_LAST:04X}", "只读", "6 轴 × 均值/标准差/峰峰值/过零率/主频/能量占比"],
    ]
    safe_print("寄存器区块表（地址口径同 docs/02 §5，Modbus 请求里直接用这些 0 基地址）：")
    safe_print(format_table(["区块", "地址范围", "访问", "内容"], rows))
    safe_print(
        f"服务参数：端口 {args.port} @ {args.baud} 8N1，从机地址 {args.addr}，"
        f"窗口更新周期 {args.rate}s，采样率 {args.sample_rate}Hz"
    )
    safe_print("C 区写入副作用：0x0020 bit6=清统计、bit7=保存配置（自清）、bit10=强制重采；0x002F 0xA5A5=恢复默认。")


def pymodbus_version() -> str:
    """返回 pymodbus 版本字符串（取不到时返回 ``unknown``）。"""
    try:
        from pymodbus import __version__ as version

        return str(version)
    except Exception:  # noqa: BLE001
        return "unknown"


def warn_on_unsupported_version(verbose: bool = True) -> None:
    """检查 pymodbus 版本是否落在本脚本支持的区间（3.7 ~ 3.9）。

    3.10 起 pymodbus 删除了 ``ModbusSlaveContext`` 数据块接口，本脚本的数据块注入无效；
    3.6.x 的服务端不接受 ``custom_pdu``，异常码覆盖会弱化。这里只做提示，不阻断启动
    （能构造出服务端就继续跑，构造失败时 :func:`build_server` 会给出中文排查清单）。
    """
    version = pymodbus_version()
    if verbose:
        safe_print(f"pymodbus 版本：{version}（本脚本按 3.7~3.9 的数据块接口实现）")
    parts = []
    for token in version.split(".")[:2]:
        digits = "".join(ch for ch in token if ch.isdigit())
        parts.append(int(digits) if digits else 0)
    while len(parts) < 2:
        parts.append(0)
    major, minor = parts[0], parts[1]
    if major == 3 and minor >= 10:
        safe_print(
            f"警告：pymodbus {version} 已删除本脚本使用的 ModbusSlaveContext 数据块接口"
            "（改用新的 SimData/SimDevice API），请降级到 3.9.x：\n"
            '  pip install "pymodbus>=3.7,<3.10"'
        )
    elif major == 3 and minor < 7:
        safe_print(
            f"警告：pymodbus {version} 的服务端不支持 custom_pdu，"
            "异常码/功能码覆盖会弱化，建议升级到 3.7~3.9。"
        )


def run_server(args: argparse.Namespace) -> int:
    """启动从机模拟器并常驻运行（Ctrl+C 退出）。

    **pymodbus 3.x 的串口服务端是 asyncio 实现**（已对 3.6.9 / 3.9.2 源码核实）：

    * ``ModbusBaseServer.__init__`` 里调用 ``asyncio.get_running_loop()``，
      因此 **必须在事件循环内构造**（在循环外构造会抛 ``RuntimeError: no running event loop``）；
    * ``serve_forever()`` 是协程，要 ``await``（3.x 里没有阻塞版 ``serve_forever``，
      旧文档里的同步写法在 3.x 已不可用）。

    所以这里把"构造 → 起后台更新任务 → await serve_forever"整个放进一个事件循环里。
    """
    table = RegisterTable(
        addr=args.addr,
        rate_s=args.rate,
        sample_rate_hz=args.sample_rate,
        seed=args.seed,
        strict_writes=not args.no_strict_writes,
    )
    print_register_map(args)
    warn_on_unsupported_version(verbose=not args.quiet)
    if args.no_strict_writes:
        safe_print("注意：已用 --no-strict-writes 关闭写入校验（只读区不再回异常 02）。")

    stop_event = threading.Event()

    def updater() -> None:
        """后台线程：按周期更新 A 区（0.5s）与 D 区窗口（--rate）。

        放在独立线程里而不是 asyncio 任务里，是为了让"造数据"与"跑协议栈"解耦：
        update_window 里有纯 Python 的 DFT，放在事件循环里会阻塞串口收发。
        """
        last_acq = 0.0
        while not stop_event.is_set():
            now = time.time()
            if now - last_acq >= 0.5:
                table.update_acquisition()
                last_acq = now
            table.update_window()
            if args.verbose:
                head = table.get_value(REG_D_HEAD)
                safe_print(
                    f"[{table.age_seconds():7.1f}s] 窗口序号 {table.window_seq} "
                    f"头 0x{head:04X} 采样序号 {table.get_value(REG_A_SAMPLE_SEQ)} "
                    f"温度 {to_int16(table.get_value(REG_A_TEMP))/10:.1f}℃ "
                    f"风扇 PWM {table.get_value(REG_C_FAN_PWM)}"
                )
            stop_event.wait(0.25)

    async def async_main() -> None:
        """事件循环主体：构造服务端 → 起更新线程 → await serve_forever。"""
        try:
            server, _store = build_server(args, table, verbose=True)
        except SystemExit:
            stop_event.set()
            raise
        thread = threading.Thread(target=updater, name="sim-updater", daemon=True)
        thread.start()
        safe_print("\n从机模拟器已启动，等待 S3 主站请求…（Ctrl+C 退出）")
        try:
            await server.serve_forever()
        except asyncio.CancelledError:
            pass
        finally:
            stop_event.set()
            thread.join(timeout=1.0)
            try:
                shutdown = server.shutdown()
                if inspect.isawaitable(shutdown):
                    await shutdown
            except Exception:  # noqa: BLE001 - 关闭阶段的异常不应影响退出
                pass

    try:
        asyncio.run(async_main())
    except KeyboardInterrupt:
        safe_print("\n收到 Ctrl+C，正在停止模拟器…")
    except SystemExit:
        return 2
    finally:
        stop_event.set()
    safe_print(
        f"已停止。统计：正常帧 {table.ok_frames}，CRC 错 {table.crc_err}，异常帧 {table.exc_frames}，"
        f"窗口序号最终 {table.window_seq}，运行 {table.age_seconds():.1f}s"
    )
    return 0


def run_selftest(args: argparse.Namespace) -> int:
    """不占串口、不依赖 pymodbus：自检寄存器表生成、写入校验与特征计算。"""
    safe_print("=== 从机模拟器离线自检（不打开串口、不依赖 pymodbus）===")
    table = RegisterTable(addr=args.addr, rate_s=args.rate, sample_rate_hz=args.sample_rate,
                          seed=args.seed, strict_writes=True)
    failures: List[str] = []

    def check(name: str, condition: bool, detail: str = "") -> None:
        """记录一条自检结果。"""
        status = "PASS" if condition else "FAIL"
        if not condition:
            failures.append(name)
        safe_print(f"  [{status}] {name} {detail}")

    # 1) D 区结构与头
    head = table.get_value(REG_D_HEAD)
    check("0x0100 窗口头 bit15=1", bool(head & 0x8000), f"头=0x{head:04X}")
    check("0x0100 低 8 位 = 100 个有效样本", (head & 0x00FF) == WINDOW_SAMPLES,
          f"样本数={head & 0x00FF}")
    check("D 区数据末尾地址 = 0x035B", REG_D_DATA_LAST == REG_D_LAST,
          f"0x{REG_D_DATA_LAST:04X}")
    check("D 区数据寄存器数量 = 600", WINDOW_WORDS == 600)
    data = table.read_block(REG_D_DATA_FIRST, WINDOW_WORDS)
    check("D 区数据非全零（有伪数据）", any(value != 0 for value in data),
          f"首 6 个={[to_int16(v) for v in data[:6]]}")

    # 2) 窗口序号随时间 +1
    seq0 = table.get_value(REG_D_SEQ)
    table.update_window(force=True)
    seq1 = table.get_value(REG_D_SEQ)
    check("0x0101 窗口序号 +1", seq1 == (seq0 + 1) & 0xFFFF, f"{seq0} → {seq1}")

    # 3) E 区与 D 区自洽（重算特征应等于表内值）
    recomputed = compute_features(table.last_window or [], table.sample_rate_hz)
    stored = [to_int16(v) for v in table.read_block(REG_E_FIRST, 36)]
    check("E 区 36 维特征与 D 区窗口自洽", recomputed == stored,
          f"前 6 个={stored[:6]}")
    check("E 区特征寄存器数量 = 36", REG_E_LAST - REG_E_FIRST + 1 == 36)

    # 4) 写入校验
    def expect_error(name: str, func, expected_text: str, expected_type: type = ValueError) -> None:
        """断言某次写入必须被拒绝，且异常类型/理由符合预期。"""
        try:
            func()
        except expected_type as exc:
            check(name, expected_text in str(exc), f"({exc})")
        except Exception as exc:  # noqa: BLE001 - 类型不符也算失败
            check(name, False, f"异常类型 {type(exc).__name__} 不符合预期 {expected_type.__name__}：{exc}")
        else:
            check(name, False, "预期被拒绝，但实际写成功了")

    expect_error("写只读区 A 区 → 异常 02", lambda: table.write_block(REG_A_TEMP, [1]), "异常 02")
    expect_error("写只读区 D 区 → 异常 02", lambda: table.write_block(REG_D_HEAD, [1]), "异常 02")
    expect_error("波特率代码 9 → 异常 03", lambda: table.write_block(REG_C_BAUD_CODE, [9]), "异常 03")
    expect_error("从机地址 0 → 异常 03", lambda: table.write_block(REG_C_SLAVE_ADDR, [0]), "异常 03")
    expect_error("从机地址 248 → 异常 03", lambda: table.write_block(REG_C_SLAVE_ADDR, [248]), "异常 03")
    expect_error("风扇 PWM 1001 → 异常 03", lambda: table.write_block(REG_C_FAN_PWM, [1001]), "异常 03")
    expect_error(
        "写越界地址 0x0500 → IndexError",
        lambda: table.write_block(0x0500, [1]),
        "越界",
        IndexError,
    )

    # 5) 合法写入与副作用
    table.write_block(REG_C_FAN_PWM, [1000])
    check("写 0x0021 = 1000 生效", table.get_value(REG_C_FAN_PWM) == 1000,
          f"回读 {table.get_value(REG_C_FAN_PWM)}")
    table.write_block(REG_C_CTRL1, [1 << 6])  # 清统计（自清位）
    check("0x0020 bit6 清统计后自清", table.get_value(REG_C_CTRL1) & (1 << 6) == 0,
          f"控制字=0x{table.get_value(REG_C_CTRL1):04X}")
    table.write_block(REG_C_SERVO, [0xFFFF])
    check("舵机 0xFFFF（保持）被接受", table.get_value(REG_C_SERVO) == 0xFFFF)
    table.write_block(REG_C_MOTOR_PWM, [from_int16(-1000)])
    check("电机 PWM -1000（int16）被接受且解释正确",
          to_int16(table.get_value(REG_C_MOTOR_PWM)) == -1000,
          f"回读 {to_int16(table.get_value(REG_C_MOTOR_PWM))}")

    # 6) 区块边界与请求校验（与 PDU 层共用 check_read_request / check_write_request）
    check("A+B 区一次块读 28 个（docs/02 §6 示例①）",
          len(table.read_block(REG_A_FIRST, 28)) == 28)
    check("读窗口 5 块合计 600 个寄存器",
          sum(count for _start, count in ((0x0104, 125), (0x0181, 125), (0x01FE, 125),
                                          (0x027B, 125), (0x02F8, 100))) == WINDOW_WORDS)
    read_expectations = [
        ((0x0000, 28), None, "A+B 28 个"),
        ((0x0008, 20), None, "跨 A/B 的 20 个"),
        ((0x0104, 125), None, "D 区 125 个（上限）"),
        ((0x0104, 126), ILLEGAL_VALUE, "126 个 → 异常 03"),
        ((0x004B, 512), ILLEGAL_VALUE, "512 个 → 异常 03"),
        ((0x0360, 1), ILLEGAL_ADDRESS, "起始 0x0360 → 异常 02"),
        ((0x0420, 8), ILLEGAL_ADDRESS, "越过 0x0423 → 异常 02"),
        ((0x0010, 28), ILLEGAL_ADDRESS, "B 区末尾越界 → 异常 02"),
        ((0x0000, 0), ILLEGAL_VALUE, "数量 0 → 异常 03"),
    ]
    for (address, count), expect, label in read_expectations:
        got = check_read_request(address, count)
        check(f"读校验：{label}", got == expect, f"0x{address:04X}×{count} → {got}")
    write_expectations = [
        ((0x0021, [1000]), None, "风扇 PWM=1000 合法"),
        ((0x0023, [0xFFFF]), None, "舵机 0xFFFF（保持）合法"),
        ((0x0000, [1]), ILLEGAL_ADDRESS, "写 A 区只读 → 异常 02"),
        ((0x0100, [1]), ILLEGAL_ADDRESS, "写 D 区只读 → 异常 02"),
        ((0x002D, [9]), ILLEGAL_VALUE, "波特率 9 → 异常 03"),
        ((0x002C, [0]), ILLEGAL_VALUE, "从机地址 0 → 异常 03"),
        ((0x002C, [248]), ILLEGAL_VALUE, "从机地址 248 → 异常 03"),
        ((0x0021, [1001]), ILLEGAL_VALUE, "风扇 PWM 1001 → 异常 03"),
        ((0x0020, [1] * 124), ILLEGAL_VALUE, "FC10 数量 124 → 异常 03"),
    ]
    for (address, values), expect, label in write_expectations:
        got = check_write_request(address, values)
        check(f"写校验：{label}", got == expect, f"0x{address:04X}×{len(values)} → {got}")

    safe_print("")
    if failures:
        safe_print(f"自检失败 {len(failures)} 项：{failures}")
        return 1
    safe_print("自检全部通过：寄存器表生成、写入校验、特征计算均符合 docs/02 §5。")
    safe_print("（提示：--selftest 不验证 pymodbus 集成与串口，请再用真机/虚拟串口跑一次）")
    return 0


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(
        prog="modbus_slave_sim.py",
        description=(
            "pymodbus 3.x 从机模拟器：实现 docs/02 §5 的 A/B/C/D/E 五个区块，"
            "让 ESP32-S3 主站逻辑脱离 F411 硬件先跑通（docs/03 阶段 4）"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "示例：\n"
            "  python modbus_slave_sim.py --port COM7 --baud 9600 --addr 1 --rate 2\n"
            "  python modbus_slave_sim.py --port COM7 --selftest      # 离线自检，不占串口\n"
            "  python modbus_slave_sim.py --port COM7 --verbose --seed 42\n"
        ),
    )
    parser.add_argument("--port", default="COM7", help="USB 转 RS485 串口（默认 COM7）")
    parser.add_argument("--baud", type=int, default=115200,
                        help="波特率（默认 115200，与 firmware-esp32 的 MODBUS_BAUD 一致；"
                             "9600 只用于单点链路自检）")
    parser.add_argument("--addr", type=lambda v: int(v, 0), default=0x01, help="从机地址，默认 0x01")
    parser.add_argument("--rate", type=float, default=2.0, help="窗口更新周期秒（默认 2s，对齐 docs/02 §9 快车道）")
    parser.add_argument("--sample-rate", type=float, default=50.0, help="伪造采样率 Hz（默认 50，写入 0x0102）")
    parser.add_argument("--timeout", type=float, default=1.0, help="串口读超时秒")
    parser.add_argument("--seed", type=int, default=None, help="随机种子（固定后可复现同一批伪数据）")
    parser.add_argument("--no-strict-writes", action="store_true",
                        help="关闭写入校验（默认严格：只读区→异常 02，越界值→异常 03）")
    parser.add_argument("--selftest", action="store_true", help="离线自检寄存器逻辑后退出（不打开串口）")
    parser.add_argument("--quiet", action="store_true", help="不打印 pymodbus 版本提示与警告")
    parser.add_argument("-v", "--verbose", action="store_true", help="每次窗口更新打印一行状态")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    """脚本入口。"""
    setup_console()
    args = parse_args(argv)
    if args.selftest:
        return run_selftest(args)
    return run_server(args)


if __name__ == "__main__":
    sys.exit(main())
