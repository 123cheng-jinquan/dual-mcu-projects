#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PC 端与 MCU 端推理结果对拍（docs/03 阶段 6 第 6 步）。

**为什么必须做这一步**：int8 量化 + 定点标准化 + TFLite Micro 的算子实现差异，
很容易导致"PC 上 96%，部署到 S3 上只有 60%"。把同一份输入分别喂给两侧，
逐类比较输出偏差，能在 5 分钟内定位是"模型问题"还是"部署问题"。

对拍流程
--------
1. PC 侧用 ``tflite-runtime`` / ``ai-edge-litert`` / ``tensorflow.lite`` 加载 ``.tflite``；
2. 从 ``data/`` 取若干测试样本（100×6 窗口或 36 维特征 CSV），按
   ``norm_params.py``（train_mlp.py 导出）标准化并量化成 int8；
3. 通过串口把同样的 int8 输入下发给 MCU，读回 MCU 的输出；
4. 逐类打印最大绝对偏差，判断是否在 ``--tol`` 容差内，给出 PASS/FAIL 与退出码。

MCU 侧串口协议（**需要 S3 固件实现，见下**）
-------------------------------------------
PC → MCU（请求）::

    0xA5 0x01 <count> <input bytes...> <checksum>

* ``0xA5`` 同步字节；``0x01`` 命令 = 推理
* ``count`` = 输入字节数（本项目 = 36）
* ``input bytes`` = 36 个 int8（就是模型输入张量的原始字节）
* ``checksum`` = 前面所有字节的 uint8 累加和（不含自身）

MCU → PC（响应，两种格式，用 ``--mcu-format`` 选择）::

    int8 模式（默认）:  0x5A 0x01 <count=36> <int8 输出 ×36> <checksum>
    class 模式:         0x5A 0x02 0x03 <class_id> <conf_lo> <conf_hi> <checksum>

* 两种响应都是"同步字节 + 命令 + 长度 + 载荷 + 校验和"的统一结构；
* ``class 模式``下 PC 侧把 MCU 的置信度与 PC softmax 的对应类概率比较；
* 固件还没实现这条协议时，可先用 ``--pc-output-json`` 对比"两份离线结果"
  （例如 PC tflite 输出 vs 固件里跑出来的同一份输出），或用 ``--print-input``
  拿到可直接粘进固件测试用例的 36 个 int8 向量。

用法::

    python verify_tflite.py --model model_out/model.tflite --port COM5 --tol 0.05
    python verify_tflite.py --model model_out/model.tflite --data data --samples 10
    python verify_tflite.py --model model_out/model.tflite --selftest    # 无硬件自检
    python verify_tflite.py --model model_out/model.tflite --norm model_out/norm_params.py
"""

from __future__ import annotations

import argparse
import csv
import glob
import importlib.util
import json
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

from _console import format_table, safe_print, setup_console

_TOOLS_DIR = Path(__file__).resolve().parent
if str(_TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(_TOOLS_DIR))

# --------------------------------------------------------------------------------------
# 协议与常量（必须与 S3 固件一致）
# --------------------------------------------------------------------------------------

SYNC_REQUEST = 0xA5
SYNC_RESPONSE = 0x5A
CMD_INFER = 0x01
CMD_RESULT_CLASS = 0x02

FEATURE_VALUES = 36
WINDOW_SAMPLES = 100
WINDOW_AXES = 6
CSV_HEADER = ["ax", "ay", "az", "gx", "gy", "gz"]

STATUS_PASS, STATUS_FAIL, STATUS_WARN = "PASS", "FAIL", "WARN"


# --------------------------------------------------------------------------------------
# 运行时候选（tflite-runtime / ai-edge-litert / tensorflow）
# --------------------------------------------------------------------------------------


def load_interpreter_class():
    """按平台差异依次尝试三种运行时，返回 ``(Interpreter 类, 运行时名)``。

    * **Linux / 树莓派 / 部分 macOS**：``tflite-runtime`` 最轻量（本脚本 PC 侧首选）；
    * **新版替代品**：``ai-edge-litert``（Google 已把 tflite-runtime 改名/迁移）；
    * **Windows**：``tflite-runtime`` **没有官方 wheel**，必须装完整 ``tensorflow``，
      用 ``tensorflow.lite.Interpreter``（见 tools/requirements.txt 注释）。

    :raises SystemExit: 三种都不可用时（退出码 2）。
    """
    candidates = (
        ("tflite_runtime.interpreter", "Interpreter", "tflite-runtime"),
        ("ai_edge_litert.interpreter", "Interpreter", "ai-edge-litert"),
        ("tensorflow.lite", "Interpreter", "tensorflow.lite"),
    )
    errors = []
    for module_name, attr, label in candidates:
        try:
            module = __import__(module_name, fromlist=[attr])
            return getattr(module, attr), label
        except (ImportError, AttributeError) as exc:
            errors.append(f"{label}: {exc}")
    safe_print(
        "错误：找不到任何 TFLite 运行时。请按平台安装其一：\n"
        '  * Windows：pip install tensorflow          # tflite-runtime 在 Windows 无官方 wheel\n'
        '  * Linux/macOS：pip install tflite-runtime  # 最轻量\n'
        '  * 新版替代：pip install ai-edge-litert\n'
        "已尝试：\n  " + "\n  ".join(errors)
    )
    raise SystemExit(2)


class ModelRunner:
    """封装 TFLite 解释器：量化输入 → 推理 → 反量化输出概率。"""

    def __init__(self, model_path: Path, verbose: bool = True) -> None:
        """:param model_path: ``.tflite`` 路径。:param verbose: 是否打印张量信息。"""
        self.model_path = Path(model_path)
        if not self.model_path.is_file():
            safe_print(f"错误：模型文件不存在 —— {self.model_path}")
            raise SystemExit(2)
        interpreter_cls, runtime = load_interpreter_class()
        self.runtime = runtime
        self.interpreter = interpreter_cls(model_path=str(self.model_path))
        self.interpreter.allocate_tensors()
        self.input_detail = self.interpreter.get_input_details()[0]
        self.output_detail = self.interpreter.get_output_details()[0]
        self.input_scale, self.input_zero = self._quant_params(self.input_detail)
        self.output_scale, self.output_zero = self._quant_params(self.output_detail)
        if verbose:
            safe_print(f"运行时：{runtime}")
            safe_print(
                f"输入张量 {self.input_detail['name']} shape={list(self.input_detail['shape'])} "
                f"dtype={self.input_detail['dtype']} scale={self.input_scale} zp={self.input_zero}"
            )
            safe_print(
                f"输出张量 {self.output_detail['name']} shape={list(self.output_detail['shape'])} "
                f"dtype={self.output_detail['dtype']} scale={self.output_scale} zp={self.output_zero}"
            )

    @staticmethod
    def _quant_params(detail: Dict[str, object]) -> Tuple[float, int]:
        """从张量详情里取 ``(scale, zero_point)``（未量化时返回 ``(1.0, 0)``）。"""
        params = detail.get("quantization_parameters") or {}
        scales = params.get("scales")  # type: ignore[union-attr]
        zero_points = params.get("zero_points")  # type: ignore[union-attr]
        scale = float(scales[0]) if scales is not None and len(scales) else float(
            (detail.get("quantization") or (1.0, 0))[0]  # type: ignore[index]
        )
        zero = int(zero_points[0]) if zero_points is not None and len(zero_points) else int(
            (detail.get("quantization") or (1.0, 0))[1]  # type: ignore[index]
        )
        return scale, zero

    @property
    def input_is_int8(self) -> bool:
        """输入张量是否为 int8（决定是否需要量化输入）。"""
        return "int8" in str(self.input_detail["dtype"])

    def quantize_input(self, normalized: Sequence[float]) -> List[int]:
        """把标准化后的 float 特征量化成模型输入的整数（裁进 int8）。"""
        values = []
        for value in normalized:
            quantized = int(round(value / self.input_scale)) + self.input_zero
            values.append(max(-128, min(127, quantized)))
        return values

    def dequantize_output(self, raw: Sequence[int]) -> List[float]:
        """把模型输出反量化成 float（int8 → 真实值）。"""
        dtype = str(self.output_detail["dtype"])
        if "int8" in dtype or "uint8" in dtype:
            return [(int(value) - self.output_zero) * self.output_scale for value in raw]
        return [float(value) for value in raw]

    def predict_quantized(self, quantized_input: Sequence[int]) -> Tuple[List[int], List[float]]:
        """用"已量化的输入"推理，返回 ``(原始输出, 反量化输出)``。"""
        import numpy as np

        shape = tuple(int(v) for v in self.input_detail["shape"])
        dtype = self.input_detail["dtype"]
        array = np.asarray(quantized_input, dtype=dtype).reshape(shape)
        self.interpreter.set_tensor(self.input_detail["index"], array)
        self.interpreter.invoke()
        raw = self.interpreter.get_tensor(self.output_detail["index"])
        flat = [int(v) for v in np.asarray(raw).reshape(-1)]
        return flat, self.dequantize_output(flat)


def softmax(values: Sequence[float]) -> List[float]:
    """数值稳定的 softmax（用于把 logits 变成概率）。"""
    if not values:
        return []
    peak = max(values)
    exps = [math.exp(value - peak) for value in values]
    total = sum(exps)
    return [value / total for value in exps] if total > 0 else [0.0] * len(values)


def looks_like_probabilities(values: Sequence[float]) -> bool:
    """判断输出是否已经是概率（和为 1 且都非负）——用于决定要不要再 softmax。"""
    if not values:
        return False
    if any(value < 0 for value in values):
        return False
    return abs(sum(values) - 1.0) < 1e-3


# --------------------------------------------------------------------------------------
# 标准化参数（train_mlp.py 导出的 norm_params.py）
# --------------------------------------------------------------------------------------


@dataclass
class NormParams:
    """标准化与量化参数（必须与 MCU 完全一致）。"""

    mean: List[float]
    std: List[float]
    input_scale: float = 1.0
    input_zero_point: int = 0
    layout: str = "per-axis"
    loaded_from: Optional[str] = None

    def standardize(self, raw: Sequence[float]) -> List[float]:
        """按 ``(x-mean)/std`` 标准化。"""
        return [
            (float(value) - self.mean[i]) / (self.std[i] if self.std[i] else 1.0)
            for i, value in enumerate(raw)
        ]


def load_norm_params(path: Optional[str], dimension: int) -> NormParams:
    """从 ``norm_params.py`` 读取标准化参数；未提供时用"身份参数"并给出警告。

    :param path: ``norm_params.py`` 路径；为 None 时自动在模型同目录找。
    :param dimension: 特征维数（用于校验）。
    """
    candidate = Path(path) if path else None
    if candidate is None:
        for guess in ("model_out/norm_params.py", "norm_params.py"):
            if Path(guess).is_file():
                candidate = Path(guess)
                break
    if candidate is None or not candidate.is_file():
        safe_print(
            "警告：未找到 norm_params.py，将按『未标准化』处理。\n"
            "      MCU 侧若做了 (x-mean)/std，两边输入就会不一致 → 对拍必然失败。\n"
            "      请用 --norm 指定 train_mlp.py 导出的 norm_params.py。"
        )
        return NormParams(mean=[0.0] * dimension, std=[1.0] * dimension)

    spec = importlib.util.spec_from_file_location("norm_params", candidate)
    if spec is None or spec.loader is None:
        safe_print(f"错误：无法加载 {candidate}")
        raise SystemExit(2)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    mean = [float(v) for v in getattr(module, "FEATURE_MEAN", [])]
    std = [float(v) for v in getattr(module, "FEATURE_STD", [])]
    if len(mean) != dimension or len(std) != dimension:
        safe_print(f"错误：norm_params 维数 {len(mean)}/{len(std)} 与模型输入 {dimension} 不一致。")
        raise SystemExit(2)
    safe_print(f"已加载标准化参数：{candidate}（布局 {getattr(module, 'FEATURE_LAYOUT', '?')}）")
    return NormParams(
        mean=mean,
        std=std,
        input_scale=float(getattr(module, "MODEL_INPUT_SCALE", 1.0)),
        input_zero_point=int(getattr(module, "MODEL_INPUT_ZERO_POINT", 0)),
        layout=str(getattr(module, "FEATURE_LAYOUT", "per-axis")),
        loaded_from=str(candidate),
    )


# --------------------------------------------------------------------------------------
# 测试样本装载（窗口 CSV → 36 维特征）
# --------------------------------------------------------------------------------------


def read_csv_rows(path: Path) -> List[List[str]]:
    """读 CSV，返回所有非空行（不解析数值）。"""
    try:
        with open(path, "r", encoding="utf-8", newline="") as handle:
            return [row for row in csv.reader(handle) if row]
    except (OSError, UnicodeDecodeError):
        return []


def load_sample_vector(path: Path, dimension: int, layout: str = "per-feature") -> Optional[List[float]]:
    """把测试样本文件读成 ``dimension`` 维 float 向量。

    * 36 列 → 直接当特征向量；
    * 100×6 → 用 ``train_mlp.compute_features`` 现场算特征（与训练侧同一实现、同一布局）。

    :param path: CSV 路径。
    :param dimension: 期望维数（36）。
    :param layout: 特征布局，必须与训练和固件一致（默认 per-feature）。
    """
    rows = read_csv_rows(path)
    if not rows:
        return None
    try:
        first = [float(cell) for cell in rows[0]]
        body = rows
    except ValueError:
        body = rows[1:]  # 表头
    if not body:
        return None
    columns = max(len(row) for row in body)
    if columns >= dimension:
        try:
            return [float(cell) for cell in body[0][:dimension]]
        except ValueError:
            return None
    if columns == WINDOW_AXES and len(body) >= WINDOW_SAMPLES:
        try:
            window = [[int(float(cell)) for cell in row] for row in body[:WINDOW_SAMPLES]]
        except ValueError:
            return None
        try:
            from train_mlp import compute_features  # 复用训练侧同一实现

            return [float(v) for v in compute_features(window, 50.0, layout)]
        except ImportError:
            safe_print("警告：无法 import train_mlp，跳过窗口样本（请用 36 列特征文件）。")
            return None
    return None


def collect_samples(spec: Optional[str], data_dir: str, samples: int,
                    dimension: int, layout: str = "per-feature") -> List[Tuple[str, List[float]]]:
    """收集测试样本：``--samples`` 支持文件、glob 或目录。"""
    paths: List[Path] = []
    if spec:
        for pattern in spec.split(","):
            pattern = pattern.strip()
            if not pattern:
                continue
            matches = [Path(p) for p in glob.glob(pattern)]
            if not matches and Path(pattern).is_file():
                matches = [Path(pattern)]
            if not matches and Path(pattern).is_dir():
                matches = sorted(Path(pattern).rglob("*.csv"))
            paths.extend(matches)
    else:
        root = Path(data_dir)
        if root.is_dir():
            for sub in sorted(p for p in root.iterdir() if p.is_dir()):
                if sub.name.startswith("_"):
                    continue
                paths.extend(sorted(sub.glob("*.csv")))
    if samples > 0:
        paths = paths[:samples]

    result: List[Tuple[str, List[float]]] = []
    for path in paths:
        vector = load_sample_vector(path, dimension, layout)
        if vector is None:
            safe_print(f"  跳过无法解析的样本：{path}")
            continue
        result.append((str(path), vector))
    return result


# --------------------------------------------------------------------------------------
# 串口：与 MCU 通信
# --------------------------------------------------------------------------------------


def checksum(data: Sequence[int]) -> int:
    """协议校验：前面所有字节的 uint8 累加和。"""
    return sum(data) & 0xFF


def build_request(quantized_input: Sequence[int]) -> bytes:
    """组请求帧：``A5 01 <count> <input...> <checksum>``。"""
    payload = list(quantized_input)
    body = [SYNC_REQUEST, CMD_INFER, len(payload) & 0xFF] + [value & 0xFF for value in payload]
    return bytes(body + [checksum(body)])


def parse_response(data: bytes, expected_len: Optional[int] = None) -> Tuple[Optional[str], object, str]:
    """解析 MCU 响应帧。

    :param data: 串口读到的原始字节。
    :param expected_len: 期望的输出长度（int8 模式下与 PC 输出长度一致）。
    :return: ``(格式, 内容, 说明)``；格式为 ``'int8'`` / ``'class'`` / None（解析失败）。
    """
    if not data:
        return None, None, "无响应（超时）"
    if len(data) < 5:
        return None, None, f"响应过短：{data.hex(' ')}"
    if data[0] != SYNC_RESPONSE:
        return None, None, f"同步字节错误：0x{data[0]:02X}（应为 0x{SYNC_RESPONSE:02X}）"
    if checksum(data[:-1]) != data[-1]:
        return None, None, f"校验和错误：{data.hex(' ')}"
    command = data[1]
    if command == CMD_INFER:
        count = data[2]
        if len(data) != count + 4:
            return None, None, f"长度不符：声明 {count}，实际 {len(data) - 4}"
        values = [int(v) - 256 if v > 127 else int(v) for v in data[3 : 3 + count]]
        if expected_len is not None and count != expected_len:
            return None, None, f"输出长度 {count} ≠ PC 侧 {expected_len}"
        return "int8", values, f"{count} 个 int8"
    if command == CMD_RESULT_CLASS:
        count = data[2]
        if count != 3 or len(data) != 3 + count + 1:
            return None, None, f"class 响应长度错误：声明 {count}，实际 {len(data)}（应为 3+3+1=7）"
        class_id = data[3]
        confidence = (data[5] << 8) | data[4]  # uint16 低字节在前
        return "class", (class_id, confidence / 1000.0), f"类别 {class_id}，置信度 {confidence/1000:.3f}"
    return None, None, f"未知命令 0x{command:02X}"


class McuLink:
    """串口链路：打开失败给中文提示，收发带超时。"""

    def __init__(self, port: str, baud: int, timeout: float) -> None:
        """:param port: 串口号。:param baud: 波特率。:param timeout: 读超时秒。"""
        try:
            import serial
        except ImportError as exc:
            safe_print("错误：未安装 pyserial。请执行：pip install -r tools/requirements.txt")
            raise SystemExit(2) from exc
        try:
            self.port = serial.Serial(
                port=port, baudrate=baud, bytesize=8, parity="N", stopbits=1, timeout=timeout
            )
        except Exception as exc:  # noqa: BLE001
            safe_print(
                f"\n错误：串口打不开 —— {port}（{exc}）\n"
                "  请依次检查：\n"
                "    1) USB 转串口/CH340 驱动是否装好（设备管理器有无黄色感叹号）\n"
                "    2) 端口号是否正确：S3 的调试/命令口通常是 USB-CDC 或 CH340，别选成 485 口\n"
                "    3) 端口是否被 **串口助手 / idf.py monitor / 另一个 Python 进程** 占用\n"
                "    4) 波特率是否与固件一致（默认 115200）\n"
            )
            raise SystemExit(2) from exc

    def request(self, frame: bytes, wait_s: float, max_len: int = 256) -> bytes:
        """发送请求并收集响应字节。"""
        self.port.reset_input_buffer()
        self.port.write(frame)
        self.port.flush()
        deadline = time.time() + wait_s
        buffer = bytearray()
        while time.time() < deadline and len(buffer) < max_len:
            chunk = self.port.read(max(1, max_len - len(buffer)))
            if chunk:
                buffer.extend(chunk)
                # 收到完整帧就提前返回：3（头）+ count + 1（校验）
                if len(buffer) >= 5 and buffer[0] == SYNC_RESPONSE:
                    if len(buffer) >= 3 + buffer[2] + 1:
                        break
        return bytes(buffer)

    def close(self) -> None:
        """关闭串口。"""
        try:
            self.port.close()
        except Exception:  # noqa: BLE001
            pass


# --------------------------------------------------------------------------------------
# 对拍主流程
# --------------------------------------------------------------------------------------


@dataclass
class CompareResult:
    """单个样本的对拍结果。"""

    name: str
    pc_probs: List[float] = field(default_factory=list)
    mcu_probs: List[float] = field(default_factory=list)
    deviations: List[float] = field(default_factory=list)
    max_deviation: float = 0.0
    status: str = STATUS_PASS
    note: str = ""


def compare_sample(name: str, pc_probs: Sequence[float], mcu_values: Sequence[float],
                   mcu_format: str, class_names: Sequence[str], tol: float) -> CompareResult:
    """比较一个样本的 PC 与 MCU 输出。

    * ``int8`` 模式：MCU 返回与 PC 同长度的输出 → 若二者都是 logits，则各自 softmax 后比较；
    * ``class`` 模式：MCU 只返回 (类别, 置信度) → 比较 argmax 是否一致 + 置信度偏差。
    """
    result = CompareResult(name=name, pc_probs=list(pc_probs))
    if mcu_format == "class":
        class_id, confidence = int(mcu_values[0]), float(mcu_values[1])
        predicted = max(range(len(pc_probs)), key=lambda i: pc_probs[i]) if pc_probs else -1
        result.mcu_probs = list(pc_probs)  # 占位，逐类偏差无意义
        result.deviations = [0.0] * len(pc_probs)
        result.max_deviation = abs(confidence - (pc_probs[class_id] if class_id < len(pc_probs) else 0.0))
        if class_id != predicted:
            result.status = STATUS_FAIL
            result.note = (f"类别不一致：MCU={class_names[class_id] if class_id < len(class_names) else class_id} "
                           f"PC={class_names[predicted] if 0 <= predicted < len(class_names) else predicted}")
        elif result.max_deviation > tol:
            result.status = STATUS_WARN
            result.note = f"类别一致但置信度偏差 {result.max_deviation:.4f} > {tol}"
        else:
            result.note = "类别与置信度一致"
        return result

    # int8 模式：MCU 原始输出先反量化，再决定是否 softmax
    mcu_flat = [float(v) for v in mcu_values]
    pc_flat = list(pc_probs)
    if looks_like_probabilities(mcu_flat) and looks_like_probabilities(pc_flat):
        mcu_probs, pc_probs2 = mcu_flat, pc_flat
    else:
        mcu_probs = softmax(mcu_flat)
        pc_probs2 = softmax(pc_flat)
    result.pc_probs = pc_probs2
    result.mcu_probs = mcu_probs
    result.deviations = [abs(a - b) for a, b in zip(pc_probs2, mcu_probs)]
    result.max_deviation = max(result.deviations) if result.deviations else 0.0
    if len(mcu_probs) != len(pc_probs2):
        result.status = STATUS_FAIL
        result.note = f"输出长度不一致：MCU {len(mcu_probs)} vs PC {len(pc_probs2)}"
    elif result.max_deviation > tol:
        result.status = STATUS_FAIL
        result.note = f"最大绝对偏差 {result.max_deviation:.4f} > 容差 {tol}"
    else:
        result.note = f"最大绝对偏差 {result.max_deviation:.4f} ≤ 容差 {tol}"
    return result


def print_results(results: Sequence[CompareResult], class_names: Sequence[str]) -> int:
    """打印对拍表格与统计，返回失败样本数。"""
    if not results:
        safe_print("没有可对拍的样本。")
        return 1
    rows = []
    for item in results:
        per_class = " ".join(f"{value:.4f}" for value in item.deviations[:len(class_names)]) \
            if item.deviations else "-"
        rows.append([item.status, Path(item.name).name, f"{item.max_deviation:.4f}", per_class, item.note])
    safe_print("\n对拍结果（逐类最大绝对偏差）")
    safe_print(format_table(
        ["状态", "样本", "最大偏差", "逐类偏差", "说明"], rows, ["<", "<", ">", "<", "<"]
    ))
    failures = sum(1 for item in results if item.status == STATUS_FAIL)
    warnings = sum(1 for item in results if item.status == STATUS_WARN)
    worst = max(item.max_deviation for item in results)
    safe_print(
        f"\n样本数 {len(results)}：PASS {len(results) - failures - warnings}，"
        f"WARN {warnings}，FAIL {failures}；全局最大偏差 {worst:.4f}"
    )
    return failures


def print_protocol_reference() -> None:
    """打印 MCU 侧需要实现的串口协议（便于直接交给固件同学）。"""
    safe_print(
        "\nMCU 侧需要实现的串口协议（与 verify_tflite.py 配对）：\n"
        "  请求：0xA5 0x01 <count=36> <36 个 int8 输入> <checksum=前面字节之和&0xFF>\n"
        "  响应A：0x5A 0x01 <count=36> <36 个 int8 输出> <checksum>        (--mcu-format int8)\n"
        "  响应B：0x5A 0x02 0x03 <class_id> <conf_lo> <conf_hi> <checksum>  (--mcu-format class)\n"
        "  说明：int8 输入就是模型输入张量的原始字节（PC 侧已完成 (x-mean)/std 与量化）。\n"
        "        期望 MCU 不做任何额外预处理，直接喂给 TFLite Micro 的 input tensor。"
    )


def selftest(args: argparse.Namespace) -> int:
    """无硬件自检：验证协议组帧/解析、softmax、偏差判定与样本装载逻辑。"""
    safe_print("=== verify_tflite.py 离线自检（不打开串口，不需要模型）===")
    failures: List[str] = []

    def check(name: str, condition: bool, detail: str = "") -> None:
        """记录一条自检结果。"""
        status = "PASS" if condition else "FAIL"
        if not condition:
            failures.append(name)
        safe_print(f"  [{status}] {name} {detail}")

    # 1) 请求组帧
    quantized = [0, 1, -1, 127, -128] + [5] * 31
    frame = build_request(quantized)
    check("请求帧长度 = 3 + 36 + 1", len(frame) == 40, f"{len(frame)}")
    check("请求帧同步字节/命令正确", frame[0] == SYNC_REQUEST and frame[1] == CMD_INFER)
    check("请求帧 count = 36", frame[2] == 36)
    check("请求帧校验和正确", checksum(frame[:-1]) == frame[-1],
          f"0x{frame[-1]:02X}")
    check("负值按补码编码", frame[3 + 2] == 0xFF, f"{frame[3+2]:02X}")

    # 2) 响应解析（int8 模式）
    payload = [7, 200, 3]  # 200 表示 int8 的 -56
    body = [SYNC_RESPONSE, CMD_INFER, len(payload)] + payload
    response = bytes(body + [checksum(body)])
    fmt, content, note = parse_response(response, expected_len=3)
    check("解析 int8 响应", fmt == "int8" and content == [7, -56, 3], f"{content} / {note}")
    fmt_bad, _, note_bad = parse_response(response[:-1] + b"\x00")
    check("校验和错误被检出", fmt_bad is None and "校验和" in note_bad, note_bad)
    fmt_short, _, note_short = parse_response(b"")
    check("无响应被识别为超时", fmt_short is None and "超时" in note_short, note_short)

    # 3) 响应解析（class 模式：5a 02 03 <class> <conf_lo> <conf_hi> <checksum>）
    body = [SYNC_RESPONSE, CMD_RESULT_CLASS, 3, 2, 0xE8, 0x03]  # 类别 2，置信度 1000 → 1.000
    response = bytes(body + [checksum(body)])
    check("class 响应总长 = 3 + 3 + 1", len(response) == 7, f"{len(response)}")
    fmt, content, note = parse_response(response)
    check("解析 class 响应（uint16 低字节在前）",
          fmt == "class" and content == (2, 1.0), f"{content} / {note}")

    # 4) softmax 与偏差判定
    probs = softmax([0.0, 0.0, 0.0])
    check("softmax 均匀输入 → 1/3", all(abs(p - 1 / 3) < 1e-9 for p in probs), f"{probs}")
    check("softmax 数值稳定（大数不溢出）", abs(sum(softmax([1000.0, 1001.0])) - 1.0) < 1e-9)
    check("概率识别：和为 1 判为概率", looks_like_probabilities([0.2, 0.3, 0.5]))
    check("概率识别：logits 判为非概率", not looks_like_probabilities([-3.0, 1.0, 0.5]))

    same = compare_sample("a", [0.7, 0.2, 0.1], [0.7, 0.2, 0.1], "int8", ["x", "y", "z"], 0.01)
    check("完全一致 → PASS", same.status == STATUS_PASS, same.note)
    diff = compare_sample("b", [0.7, 0.2, 0.1], [0.1, 0.2, 0.7], "int8", ["x", "y", "z"], 0.01)
    check("偏差超容差 → FAIL", diff.status == STATUS_FAIL, diff.note)
    logits = compare_sample("c", [2.0, 1.0, 0.0], [2.0, 1.0, 0.0], "int8", ["x", "y", "z"], 0.001)
    check("logits 输入自动 softmax 后比较", logits.status == STATUS_PASS, logits.note)
    class_ok = compare_sample("d", [0.1, 0.8, 0.1], [1, 0.8], "class", ["x", "y", "z"], 0.05)
    check("class 模式类别一致 → PASS", class_ok.status == STATUS_PASS, class_ok.note)
    class_bad = compare_sample("e", [0.1, 0.8, 0.1], [2, 0.9], "class", ["x", "y", "z"], 0.05)
    check("class 模式类别不一致 → FAIL", class_bad.status == STATUS_FAIL, class_bad.note)

    # 5) 脚本自身可被导入（不依赖 numpy/tflite）
    check("模块导入不依赖 TFLite 运行时", callable(load_interpreter_class))

    safe_print("")
    print_protocol_reference()
    safe_print("")
    if failures:
        safe_print(f"自检失败 {len(failures)} 项：{failures}")
        return 1
    safe_print("自检全部通过：协议组帧/解析、softmax、偏差判定逻辑均正常。")
    safe_print("（提示：--selftest 不加载真实模型，模型侧对拍请给 --model 并接真机）")
    return 0


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(
        prog="verify_tflite.py",
        description="PC 端（tflite-runtime）与 MCU 端（TFLite Micro）推理结果对拍（docs/03 阶段 6）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "示例：\n"
            "  python verify_tflite.py --model model_out/model.tflite --port COM5 --tol 0.05\n"
            "  python verify_tflite.py --model model_out/model.tflite --norm model_out/norm_params.py \\\n"
            "      --samples 'data/normal/*.csv,data/loosen/*.csv' --samples-count 10\n"
            "  python verify_tflite.py --model model_out/model.tflite --selftest\n"
        ),
    )
    parser.add_argument("--model", default="model_out/model.tflite", help="int8 tflite 模型路径")
    parser.add_argument("--port", default="COM5", help="与 MCU 通信的串口（默认 COM5）")
    parser.add_argument("--baud", type=int, default=115200, help="波特率（默认 115200）")
    parser.add_argument("--tol", type=float, default=0.05, help="每类输出允许的最大绝对偏差（默认 0.05）")
    parser.add_argument("--norm", default=None,
                        help="train_mlp.py 导出的 norm_params.py（不指定则在模型同目录自动找）")
    parser.add_argument("--feat-layout", default="per-feature",
                        choices=["per-feature", "per-axis"],
                        help="特征布局，必须与训练和 ESP32 固件的 FEATURE_LAYOUT 一致；"
                             "默认 per-feature（docs/02 §5 E 区）；"
                             "per-axis 与固件互为转置，仅在窗口数据且你改了固件顺序时才用")
    parser.add_argument("--data", default="data", help="测试样本根目录（默认 data）")
    parser.add_argument("--samples", default=None,
                        help="样本文件/glob/目录，逗号分隔；默认扫描 --data 下各子目录")
    parser.add_argument("--samples-count", type=int, default=5, dest="samples_count",
                        help="最多对拍多少个样本（默认 5）")
    parser.add_argument("--mcu-format", default="int8", choices=["int8", "class"],
                        help="MCU 响应格式：int8 原始输出 / class+置信度（默认 int8）")
    parser.add_argument("--wait", type=float, default=2.0, help="每个样本等待响应的秒数（默认 2.0）")
    parser.add_argument("--timeout", type=float, default=0.5, help="串口读超时秒")
    parser.add_argument("--print-input", action="store_true",
                        help="打印每个样本的 36 个 int8 输入（可直接粘进固件/MCU 测试用例）")
    parser.add_argument("--selftest", action="store_true", help="无硬件自检协议与判定逻辑后退出")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    """脚本入口。"""
    setup_console()
    args = parse_args(argv)
    if args.selftest:
        return selftest(args)

    runner = ModelRunner(Path(args.model))
    dimension = int(runner.input_detail["shape"][-1])
    if dimension != FEATURE_VALUES:
        safe_print(f"提示：模型输入维度 {dimension}（本项目规格为 {FEATURE_VALUES}）。")

    norm = load_norm_params(args.norm, dimension)
    if norm.loaded_from is None:
        safe_print("提示：未加载标准化参数 → 假设样本已经是标准化后的特征向量。")
    if norm.loaded_from is not None and norm.layout != args.feat_layout:
        safe_print(
            f"⚠ 警告：--feat-layout {args.feat_layout} 与 norm_params.py 里记录的 "
            f"{norm.layout} 不一致；特征含义会错位，对拍必然失败。"
        )
    safe_print(f"特征布局：{args.feat_layout}（必须与 ESP32 固件 FEATURE_LAYOUT 一致）")

    # 收集样本
    samples = collect_samples(args.samples, args.data, args.samples_count, dimension,
                              args.feat_layout)
    if not samples:
        safe_print(
            f"错误：没有找到可用测试样本（--data {args.data} / --samples {args.samples}）。\n"
            "  请先采集并打标：capture_window.py → label_tool.py，或用 --samples 指定文件。"
        )
        return 2
    safe_print(f"已装载 {len(samples)} 个测试样本。")
    print_protocol_reference()

    # PC 侧推理
    prepared: List[Tuple[str, List[int], List[float]]] = []
    for name, raw_vector in samples:
        normalized = norm.standardize(raw_vector)
        quantized = runner.quantize_input(normalized) if runner.input_is_int8 else normalized
        raw_output, dequantized = runner.predict_quantized(quantized)  # type: ignore[arg-type]
        probabilities = (dequantized if looks_like_probabilities(dequantized)
                         else softmax(dequantized))
        prepared.append((name, list(quantized), probabilities))  # type: ignore[arg-type]
        if args.print_input:
            safe_print(f"\n{Path(name).name} 输入（int8 × {len(quantized)}）：")
            safe_print("  " + " ".join(f"{v}" for v in quantized))

    link = McuLink(args.port, args.baud, args.timeout)
    results: List[CompareResult] = []
    try:
        for name, quantized, pc_probs in prepared:
            frame = build_request(quantized)  # type: ignore[arg-type]
            data = link.request(frame, args.wait)
            fmt, content, note = parse_response(
                data, expected_len=len(pc_probs) if args.mcu_format == "int8" else None
            )
            if fmt is None:
                results.append(CompareResult(
                    name=name, pc_probs=pc_probs, status=STATUS_FAIL,
                    note=f"{note}（原始字节：{data.hex(' ') if data else '空'}）",
                ))
                continue
            if fmt != args.mcu_format:
                safe_print(f"提示：MCU 响应格式为 {fmt}，与 --mcu-format {args.mcu_format} 不一致，按实际格式比较。")
            results.append(compare_sample(name, pc_probs, content, fmt, [], args.tol))  # type: ignore[arg-type]
    except KeyboardInterrupt:
        safe_print("\n已被用户中断（Ctrl+C）。")
    finally:
        link.close()

    failures = print_results(results, [])
    if failures:
        safe_print("\n结论：存在超容差样本，退出码 1。排查顺序：")
        safe_print("  1) 标准化参数是否同一套（norm_params.py 是否就是训练时那份）")
        safe_print("  2) 已量化输入是否被 MCU 二次缩放（应直接喂 input tensor）")
        safe_print("  3) 特征布局（per-axis / per-feature）是否与固件一致")
        safe_print("  4) MCU 侧输出反量化（scale/zero_point）是否用对")
        return 1
    safe_print("\n结论：PC 与 MCU 输出在容差内一致，部署没有引入精度损失。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
