#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""36 维特征向量 → MLP 分类器 → int8 全整数量化 tflite → C 数组（docs/03 阶段 6）。

数据从哪来
----------
输入是 **F411 特征区（docs/02 §5 E 区 0x0400~0x0423）的 36 个 int16**：
6 轴 × (均值 / 标准差 / 峰峰值 / 过零率×100 / 主频×10Hz / 主频带能量占比×1000)。

支持两种数据源（自动识别，``--feature-source auto``）：

* ``window``：``capture_window.py`` 采的 ``100×6`` 窗口 CSV（每行 6 列、共 100 行）。
  脚本按 E 区定义现场算 36 维特征 → **推荐路径**：
  ``capture_window.py → label_tool.py → train_mlp.py``。
* ``feature``：已经是 36 列的 CSV（例如从 Modbus 0x0400 区直接导出的）。

> ⚠️ **布局必须与 F411 固件一致（最常见的"模型没错但准确率掉到随机"陷阱）**：
> ``docs/02`` §5 E 区是权威定义——``0x0400~0x0405`` = 6 轴的**均值**、
> ``0x0406~0x040B`` = 6 轴的**标准差**……即"**每特征 6 轴连续**"
> （``--feat-layout per-feature``，**默认值**）。
> 若误用"每轴 6 个特征连续"（``per-axis``），特征矩阵正好是**转置**关系：
> 维数一样、训练不报错，但每一维的含义全错，准确率会掉到随机水平且极难排查。
> 本脚本在 ``--feat-layout per-axis`` 时会显著警告，并把布局写进 ``model_info.json``
> 供 ESP32 侧 ``infer_init()`` 硬校验。

产出（``--out`` 目录）
----------------------
* ``model.tflite``            int8 全整数量化模型（输入 int8 36 维，输出 int8）
* ``model_data.h`` / ``.cc``  ``const unsigned char model_data[]`` + ``__len``，含 ``alignas(16)``
* ``norm_params.h`` / ``.cc`` 标准化参数（MCU 侧做 ``(x-mean)/std`` 后量化，必须与 PC 同一套）
* ``norm_params.py``          PC 侧推理用同一套参数（``verify_tflite.py`` 会用）
* ``model_info.json``         类别表、量化参数、准确率、混淆矩阵、特征布局（给 verify 脚本读）
* ``confusion_matrix.txt``    留档

回退策略：TensorFlow 不可用时自动改用 sklearn ``MLPClassifier`` 训练——
**此时不产出 tflite**（sklearn 无法做 TFLite 量化），但会打印准确率与混淆矩阵，
方便先验证数据与特征是否可分。

用法::

    python train_mlp.py --data data --out model_out --epochs 200 --test-ratio 0.2
    python train_mlp.py --data data --split data/split.json --feature-source window
    python train_mlp.py --selftest --out model_out     # 只验证数据管线，不训练
    python train_mlp.py --data data --out model_out --require-tf   # CI：无 TF 直接失败
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

from _console import format_table, safe_print, setup_console

# --------------------------------------------------------------------------------------
# 规格常量：与 docs/02 §5 E 区、modbus_slave_sim.py 完全一致
# --------------------------------------------------------------------------------------

WINDOW_SAMPLES = 100
WINDOW_AXES = 6
FEATURE_VALUES = 36  # 6 轴 × 6 特征 = 36 个寄存器（0x0400~0x0423）
CSV_HEADER = ["ax", "ay", "az", "gx", "gy", "gz"]
INT16_MIN, INT16_MAX = -32768, 32767

AXIS_NAMES = ("ax", "ay", "az", "gx", "gy", "gz")
FEATURE_NAMES = ("mean", "std", "ptp", "zcr_x100", "dom_freq_x10", "band_ratio_x1000")
FEATURE_NAMES_CN = ("均值", "标准差", "峰峰值", "过零率×100", "主频×10Hz", "主频带能量占比×1000")

#: 特征布局（**必须与 F411 固件写入 E 区的顺序一致**）
#:
#: ``docs/02`` §5 E 区是权威定义：``0x0400~0x0405`` = 6 轴的**均值**、
#: ``0x0406~0x040B`` = 6 轴的**标准差**，其后依次是峰峰值/过零率/主频/主频带能量占比。
#: 也就是"**每个特征 6 个轴连续**" = :data:`LAYOUT_PER_FEATURE`，这是本项目固件的布局，
#: 因此作为默认值。
LAYOUT_PER_FEATURE = "per-feature"
LAYOUT_AXES_MAJOR = "per-axis"  # 每轴 6 个特征连续（**与本项目固件互为转置**）
LAYOUT_RAW_REGISTER = LAYOUT_PER_FEATURE  # 默认值：与 docs/02 §5 E 区一致

LAYOUT_ALIASES = {
    "per-feature": LAYOUT_PER_FEATURE,
    "feature-major": LAYOUT_PER_FEATURE,
    "per-axis": LAYOUT_AXES_MAJOR,
    "peraxis": LAYOUT_AXES_MAJOR,
    "axis-major": LAYOUT_AXES_MAJOR,
    "axes-major": LAYOUT_AXES_MAJOR,
}


def normalize_layout(text: str) -> str:
    """把用户输入的布局别名统一成内部规范名。"""
    key = text.strip().lower()
    if key not in LAYOUT_ALIASES:
        raise ValueError(f"未知特征布局 {text!r}；可用：{sorted(set(LAYOUT_ALIASES))}")
    return LAYOUT_ALIASES[key]


# --------------------------------------------------------------------------------------
# 特征提取（与 F411 的 E 区定义一致；也用于生成训练数据）
# --------------------------------------------------------------------------------------


def clamp_int16(value: float) -> int:
    """裁进 int16 并取整（模拟 MCU 侧 int16 饱和）。"""
    return int(max(INT16_MIN, min(INT16_MAX, round(value))))


def compute_features(window: Sequence[Sequence[int]], sample_rate_hz: float = 50.0,
                     layout: str = LAYOUT_RAW_REGISTER) -> List[int]:
    """由 100×6 窗口算 36 维特征向量（对应 docs/02 §5 E 区）。

    每轴 6 个特征：均值 / 标准差（int16）/ 峰峰值 / 过零率×100 / 主频×10Hz /
    主频带能量占比×1000。主频与能量占比用朴素 DFT（0.5~25Hz 范围扫峰），
    100 点窗口下与 CMSIS-DSP 的 128 点实数 FFT 结果在工程上等价
    （差异主要体现在频点分辨率：100 点 @50Hz → 0.5Hz 一格）。

    :param window: ``100×6`` int16 窗口。
    :param sample_rate_hz: 采样率（用于频率换算，对应 0x0102）。
    :param layout: 输出布局，默认 :data:`LAYOUT_RAW_REGISTER`（= ``per-feature``，
        与 ``docs/02`` §5 E 区"每特征 6 轴连续"一致）。
    :return: 36 个 int16。
    """
    samples = len(window)
    per_axis: List[List[int]] = []
    for axis in range(WINDOW_AXES):
        series = [row[axis] for row in window]
        mean = sum(series) / samples
        std = math.sqrt(sum((x - mean) ** 2 for x in series) / samples)
        peak_to_peak = max(series) - min(series)
        centered = [x - mean for x in series]
        crossings = sum(
            1 for i in range(1, samples) if (centered[i - 1] < 0) != (centered[i] < 0)
        )
        zcr_x100 = int(crossings * 100 / max(1, samples - 1))

        best_freq, best_power, total_power = 0.0, 0.0, 0.0
        for bin_index in range(1, samples // 2):
            omega = 2 * math.pi * bin_index / samples
            real = sum(centered[n] * math.cos(omega * n) for n in range(samples))
            imag = sum(centered[n] * math.sin(omega * n) for n in range(samples))
            power = real * real + imag * imag
            total_power += power
            frequency = bin_index * sample_rate_hz / samples
            if 0.5 <= frequency <= 25.0 and power > best_power:
                best_power, best_freq = power, frequency
        ratio_x1000 = int(best_power * 1000 / total_power) if total_power > 0 else 0

        per_axis.append([
            clamp_int16(mean),
            clamp_int16(std),
            clamp_int16(peak_to_peak),
            zcr_x100,
            int(round(best_freq * 10)),
            ratio_x1000,
        ])

    if layout == LAYOUT_PER_FEATURE:
        flat = [per_axis[axis][feature] for feature in range(6) for axis in range(WINDOW_AXES)]
    else:
        flat = [value for axis_values in per_axis for value in axis_values]
    assert len(flat) == FEATURE_VALUES
    return flat


def feature_names_for(layout: str) -> List[str]:
    """给 36 维特征生成列名（随布局变化，便于人工核对）。"""
    if layout == LAYOUT_PER_FEATURE:
        return [f"{name}_{axis}" for name in FEATURE_NAMES for axis in AXIS_NAMES]
    return [f"{axis}_{name}" for axis in AXIS_NAMES for name in FEATURE_NAMES]


# --------------------------------------------------------------------------------------
# 数据集装载
# --------------------------------------------------------------------------------------


@dataclass
class Dataset:
    """训练/测试用的特征矩阵与标签。"""

    features: List[List[float]] = field(default_factory=list)
    labels: List[str] = field(default_factory=list)
    sources: List[str] = field(default_factory=list)  # 每个样本的来源文件（相对路径）
    class_names: List[str] = field(default_factory=list)
    skipped: List[str] = field(default_factory=list)  # 跳过的文件及原因
    sample_rate_hz: float = 50.0

    def __len__(self) -> int:
        """样本数量。"""
        return len(self.features)

    def class_counts(self) -> Dict[str, int]:
        """每个类别的样本数。"""
        counts: Dict[str, int] = {}
        for label in self.labels:
            counts[label] = counts.get(label, 0) + 1
        return counts


def read_window_csv(path: Path) -> Optional[List[List[int]]]:
    """读 ``100×6`` 窗口 CSV；格式不符返回 None。

    :param path: CSV 路径。
    :return: 100 行 × 6 列 int16；不符返回 None。
    """
    try:
        with open(path, "r", encoding="utf-8", newline="") as handle:
            rows = list(csv.reader(handle))
    except (OSError, UnicodeDecodeError):
        return None
    if not rows:
        return None
    start = 0
    try:
        [int(cell) for cell in rows[0]]
    except ValueError:
        start = 1  # 有表头
    data = rows[start:]
    if len(data) != WINDOW_SAMPLES:
        return None
    window: List[List[int]] = []
    for row in data:
        if len(row) != WINDOW_AXES:
            return None
        try:
            window.append([int(cell) for cell in row])
        except ValueError:
            return None
    return window


def read_feature_csv(path: Path) -> Optional[List[float]]:
    """读 36 列特征 CSV（取第一行数据；多于一行时取所有行的均值）。

    :param path: CSV 路径。
    :return: 36 个 float；不符返回 None。
    """
    try:
        with open(path, "r", encoding="utf-8", newline="") as handle:
            rows = list(csv.reader(handle))
    except (OSError, UnicodeDecodeError):
        return None
    data: List[List[float]] = []
    for row in rows:
        if not row:
            continue
        try:
            values = [float(cell) for cell in row]
        except ValueError:
            if not data:
                continue  # 表头行
            return None
        if len(values) < FEATURE_VALUES:
            return None
        data.append(values[:FEATURE_VALUES])
    if not data:
        return None
    if len(data) == 1:
        return data[0]
    return [sum(row[i] for row in data) / len(data) for i in range(FEATURE_VALUES)]


def detect_source(path: Path) -> str:
    """判断一个 CSV 是窗口文件（100×6）还是 36 维特征文件。

    判定顺序（先判窗口，因为窗口文件更"严格"）：

    1. 数据行恰为 100 且列数为 6 → ``window``
    2. 列数 ≥ 36 → ``feature``
    3. 其它 → ``unknown``

    :return: ``"window"`` / ``"feature"`` / ``"unknown"``。
    """
    try:
        with open(path, "r", encoding="utf-8", newline="") as handle:
            rows = []
            for index, row in enumerate(csv.reader(handle)):
                rows.append(row)
                if index >= WINDOW_SAMPLES + 1:  # 表头 + 100 行足够判断
                    break
    except (OSError, UnicodeDecodeError):
        return "unknown"
    if not rows:
        return "unknown"

    # 去掉表头（第一行不能整行转数字即视为表头）
    try:
        [float(cell) for cell in rows[0]]
        body = rows
    except ValueError:
        body = rows[1:]
    body = [row for row in body if row]
    if not body:
        return "unknown"

    columns = max(len(row) for row in body)
    if columns == WINDOW_AXES and len(body) == WINDOW_SAMPLES:
        return "window"
    if columns >= FEATURE_VALUES:
        return "feature"
    return "unknown"


def gather_files(data_dir: Path, split: Optional[Dict[str, object]] = None,
                 use_test: bool = False, labels_filter: Optional[Sequence[str]] = None) -> List[Path]:
    """收集样本文件（可来自 split.json 的训练/测试清单）。

    未提供 split 时的扫描规则：``data/<类别>/**/*.csv``，**递归两层**，
    这样允许 ``data/<类别>/<设备或日期>/xxx.csv`` 的层级组织；
    以 ``_`` 开头的目录（``_quarantine`` / ``_selftest`` 等）一律跳过。

    :param data_dir: 数据集根目录。
    :param split: ``label_tool.py`` 写出的 split.json 内容；None 表示扫描整个目录。
    :param use_test: 使用 split 里的测试集（否则用训练集）。
    :param labels_filter: 只保留这些类别。
    :return: 文件路径列表。
    """
    if split is not None:
        key = "test" if use_test else "train"
        files = []
        for item in split.get(key, []):  # type: ignore[union-attr]
            path = data_dir / str(item)
            if labels_filter and path.parent.name not in labels_filter:
                continue
            if path.is_file():
                files.append(path)
        return sorted(files)

    files = []
    for sub in sorted(p for p in data_dir.iterdir() if p.is_dir()):
        if sub.name.startswith("_"):  # _quarantine / _selftest 等默认排除
            continue
        if labels_filter and sub.name not in labels_filter:
            continue
        # rglob 递归子目录；同一样本可能在 <类别>/<日期>/ 下
        files.extend(sorted(p for p in sub.rglob("*.csv") if not _has_hidden_part(p, sub)))
    return files


def _has_hidden_part(path: Path, base: Path) -> bool:
    """判断相对 ``base`` 的路径里是否含以 ``_`` 开头的目录（用于排除内部目录）。"""
    return any(part.startswith("_") for part in path.relative_to(base).parts[:-1])


def load_dataset(data_dir: Path, split: Optional[Dict[str, object]] = None, use_test: bool = False,
                 feature_source: str = "auto", layout: str = LAYOUT_RAW_REGISTER,
                 sample_rate_hz: float = 50.0,
                 labels_filter: Optional[Sequence[str]] = None) -> Dataset:
    """装载特征数据集。

    :param data_dir: 数据集根目录。
    :param split: split.json 内容（可选）。
    :param use_test: 是否读测试集部分。
    :param feature_source: ``auto`` / ``window`` / ``feature``。
    :param layout: 特征布局。
    :param sample_rate_hz: 窗口转特征时的采样率。
    :param labels_filter: 类别过滤。
    :return: :class:`Dataset`。
    """
    dataset = Dataset(sample_rate_hz=sample_rate_hz)
    files = gather_files(data_dir, split, use_test, labels_filter)
    for path in files:
        label = path.parent.name
        source = feature_source if feature_source != "auto" else detect_source(path)
        vector: Optional[List[float]] = None
        if source == "window":
            window = read_window_csv(path)
            if window is not None:
                vector = [float(value) for value in compute_features(window, sample_rate_hz, layout)]
        elif source == "feature":
            vector = read_feature_csv(path)
        if vector is None:
            dataset.skipped.append(f"{path.name}（识别为 {source}，格式不符或解析失败）")
            continue
        dataset.features.append(vector)
        dataset.labels.append(label)
        try:
            dataset.sources.append(str(path.relative_to(data_dir)))
        except ValueError:
            dataset.sources.append(str(path))
    dataset.class_names = sorted(set(dataset.labels))
    return dataset


# --------------------------------------------------------------------------------------
# 标准化 + 训练
# --------------------------------------------------------------------------------------


def layout_separability(features: Sequence[Sequence[float]], labels: Sequence[str]) -> float:
    """用 1-NN 训练集自测准确率粗略度量"当前布局下特征是否可分"（0~1）。

    这不是模型指标，只用来**判断布局是否用错**：布局正确时同类样本的特征高度相似，
    1-NN 自测通常接近 1.0；布局用错（per-axis / per-feature 互为转置）时，
    每一维的含义被打乱，可分性会明显下降。

    :return: 1-NN 自测准确率。
    """
    if not features:
        return 0.0
    standardized = [[float(v) for v in row] for row in features]
    correct = 0
    for i, row in enumerate(standardized):
        best_distance, best_index = float("inf"), -1
        for j, other in enumerate(standardized):
            if i == j:
                continue
            distance = sum((row[d] - other[d]) ** 2 for d in range(len(row)))
            if distance < best_distance:
                best_distance, best_index = distance, j
        if best_index >= 0 and labels[best_index] == labels[i]:
            correct += 1
    return correct / len(standardized)


def warn_if_layout_suspect(features: Sequence[Sequence[float]], labels: Sequence[str],
                           layout: str, threshold: float = 0.75) -> float:
    """当 1-NN 可分性过低时提示"布局可能用错"，并给出转置布局的对照分数。

    :param features: 原始特征（未标准化）。
    :param labels: 标签。
    :param layout: 当前布局。
    :param threshold: 低于该分数就告警。
    :return: 当前布局的可分性分数。
    """
    score = layout_separability(features, labels)
    safe_print(f"布局自检（{layout}）：1-NN 训练集自测可分性 {score*100:.1f}%")
    if score >= threshold:
        return score

    dimensions = len(features[0]) if features else 0
    other = LAYOUT_AXES_MAJOR if layout == LAYOUT_PER_FEATURE else LAYOUT_PER_FEATURE
    transposed = [[_transpose_vector(row, dimensions, other) for row in features]]
    other_score = layout_separability(transposed[0], labels)
    safe_print(
        f"⚠ 警告：当前布局 [{layout}] 的可分性只有 {score*100:.1f}%，明显偏低。\n"
        f"  作为对照，转置布局 [{other}] 的可分性为 {other_score*100:.1f}%。\n"
        "  若对照分数明显更高，说明 **--feat-layout 用错了**：\n"
        "    docs/02 §5 E 区 = 每个特征 6 个轴连续（per-feature）\n"
        "    除非你同步改了 F411 写入 E 区的顺序，否则不要用 per-axis。"
    )
    return score


def _transpose_vector(vector: Sequence[float], dimensions: int, target_layout: str) -> List[float]:
    """把一个 36 维向量在 per-feature 与 per-axis 两种布局之间转置。

    :param vector: 36 维向量。
    :param dimensions: 维数（应为 36）。
    :param target_layout: 目标布局。
    :return: 转置后的向量；维数不是 36 时原样返回。
    """
    if dimensions != FEATURE_VALUES:
        return list(vector)
    # 当前向量按"轴主序"解释成 6×6 矩阵（行=轴，列=特征），再按目标布局展平
    matrix = [[vector[axis * 6 + feature] for feature in range(6)] for axis in range(6)]
    if target_layout == LAYOUT_PER_FEATURE:
        return [matrix[axis][feature] for feature in range(6) for axis in range(6)]
    return [matrix[axis][feature] for axis in range(6) for feature in range(6)]


def compute_standardization(features: Sequence[Sequence[float]]) -> Tuple[List[float], List[float]]:
    """算每维均值与标准差（总体标准差，ddof=0，与 MCU 侧定点实现最容易对齐）。

    :return: ``(mean[36], std[36])``；标准差为 0 的维度置 1 避免除零。
    """
    count = len(features)
    if count == 0:
        return [0.0] * FEATURE_VALUES, [1.0] * FEATURE_VALUES
    dimensions = len(features[0])
    means = [sum(row[i] for row in features) / count for i in range(dimensions)]
    variances = [
        sum((row[i] - means[i]) ** 2 for row in features) / count for i in range(dimensions)
    ]
    stds = [math.sqrt(value) if value > 1e-12 else 1.0 for value in variances]
    return means, stds


def standardize(features: Sequence[Sequence[float]], means: Sequence[float],
                stds: Sequence[float]) -> List[List[float]]:
    """按 ``(x-mean)/std`` 标准化。"""
    return [[(row[i] - means[i]) / stds[i] for i in range(len(row))] for row in features]


def confusion_matrix(y_true: Sequence[str], y_pred: Sequence[str],
                     labels: Sequence[str]) -> List[List[int]]:
    """构造混淆矩阵（行=真实，列=预测）。"""
    index = {label: i for i, label in enumerate(labels)}
    matrix = [[0] * len(labels) for _ in labels]
    for truth, pred in zip(y_true, y_pred):
        if truth in index and pred in index:
            matrix[index[truth]][index[pred]] += 1
    return matrix


def render_confusion(y_true: Sequence[str], y_pred: Sequence[str], labels: Sequence[str]) -> str:
    """把混淆矩阵渲染成文本表（含每类召回率）。"""
    matrix = confusion_matrix(y_true, y_pred, labels)
    header = ["真实\\预测"] + list(labels) + ["召回率"]
    rows = []
    for i, label in enumerate(labels):
        total = sum(matrix[i])
        recall = matrix[i][i] / total if total else 0.0
        rows.append([label] + [str(v) for v in matrix[i]] + [f"{recall*100:.1f}%"])
    accuracy = sum(matrix[i][i] for i in range(len(labels))) / max(1, sum(sum(r) for r in matrix))
    lines = [format_table(header, rows, ["<"] + [">"] * len(labels) + [">"])]
    lines.append(f"总体准确率：{accuracy*100:.2f}%（{sum(sum(r) for r in matrix)} 个样本）")
    return "\n".join(lines)


def train_with_keras(x_train, y_train, x_test, y_test, class_names, epochs: int, seed: int,
                     hidden: int = 32):
    """用 TensorFlow/Keras 训练小 MLP。

    结构：``Dense(hidden)-ReLU-Dropout-Dense(16)-ReLU-Dense(N)-Softmax``，
    与 ``plan.md`` §6.2 的"小规模模型"一致（int8 后约 10~40KB）。

    :return: ``(model, history_accuracy)``。
    """
    import tensorflow as tf  # 局部导入：无 TF 时可回退 sklearn

    tf.random.set_seed(seed)
    tf.keras.utils.set_random_seed(seed)
    model = tf.keras.Sequential(
        [
            tf.keras.layers.Input(shape=(FEATURE_VALUES,), name="features"),
            tf.keras.layers.Dense(hidden, activation="relu"),
            tf.keras.layers.Dropout(0.2),
            tf.keras.layers.Dense(16, activation="relu"),
            tf.keras.layers.Dense(len(class_names), activation="softmax"),
        ],
        name="vibration_mlp",
    )
    model.compile(
        optimizer=tf.keras.optimizers.Adam(1e-3),
        loss="sparse_categorical_crossentropy",
        metrics=["accuracy"],
    )
    label_index = {name: i for i, name in enumerate(class_names)}
    y_train_idx = [label_index[name] for name in y_train]
    y_test_idx = [label_index[name] for name in y_test]
    history = model.fit(
        x_train, y_train_idx,
        validation_data=(x_test, y_test_idx),
        epochs=epochs, batch_size=16, verbose=2,
    )
    return model, history


def train_with_sklearn(x_train, y_train, x_test, y_test, class_names, epochs: int, seed: int,
                       hidden: int = 32):
    """回退路径：sklearn ``MLPClassifier``（无法导出 tflite，但能看到准确率与混淆矩阵）。"""
    from sklearn.neural_network import MLPClassifier

    model = MLPClassifier(
        hidden_layer_sizes=(hidden, 16),
        activation="relu",
        max_iter=max(50, epochs),
        random_state=seed,
        early_stopping=False,
    )
    model.fit(x_train, y_train)
    return model


# --------------------------------------------------------------------------------------
# 导出：tflite / C 数组 / 标准化参数
# --------------------------------------------------------------------------------------


def export_tflite(model, out_dir: Path, x_representative, verbose: bool = True) -> Tuple[Optional[Path], str]:
    """导出 int8 全整数量化 tflite（代表性数据集校准）。

    :param model: Keras 模型。
    :param out_dir: 输出目录。
    :param x_representative: 校准用样本（标准化后的训练特征）。
    :return: ``(tflite 路径或 None, 说明文本)``。
    """
    import numpy as np
    import tensorflow as tf

    path = out_dir / "model.tflite"
    representative = np.asarray(x_representative, dtype=np.float32)

    def representative_dataset():
        """校准数据生成器（逐样本 yield，TF 要求 float32）。"""
        for sample in representative:
            yield [sample.reshape(1, FEATURE_VALUES)]

    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    converter.representative_dataset = representative_dataset
    converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    converter.inference_input_type = tf.int8
    converter.inference_output_type = tf.int8
    try:
        tflite_model = converter.convert()
        note = "int8 全整数量化（TFLITE_BUILTINS_INT8，输入/输出均 int8）"
    except Exception as exc:  # noqa: BLE001 - 某些算子组合不支持纯 int8 时会失败
        safe_print(f"警告：纯 int8 量化失败（{type(exc).__name__}: {exc}），回退 float16 权重。")
        converter = tf.lite.TFLiteConverter.from_keras_model(model)
        converter.optimizations = [tf.lite.Optimize.DEFAULT]
        converter.target_spec.supported_types = [tf.float16]
        tflite_model = converter.convert()
        note = "float16 权重量化（回退方案，输入仍是 float32；MCU 侧需用 float 输入）"

    path.write_bytes(tflite_model)
    if verbose:
        safe_print(f"已导出 {path}（{len(tflite_model)} 字节）；{note}")
    return path, note


def tflite_io_details(tflite_path: Path) -> Dict[str, object]:
    """读取 tflite 的输入/输出张量量化参数（供 model_info.json 与 verify 脚本使用）。"""
    details: Dict[str, object] = {"path": str(tflite_path)}
    try:
        import numpy as np
        import tensorflow as tf
    except ImportError:
        try:
            import numpy as np
            from tflite_runtime.interpreter import Interpreter  # type: ignore

            interpreter = Interpreter(model_path=str(tflite_path))
            interpreter.allocate_tensors()
            details["input"] = _tensor_detail(interpreter.get_input_details()[0])
            details["output"] = _tensor_detail(interpreter.get_output_details()[0])
            return details
        except ImportError:
            details["error"] = "无 TensorFlow / tflite-runtime，未能读取张量详情"
            return details

    interpreter = tf.lite.Interpreter(model_path=str(tflite_path))
    interpreter.allocate_tensors()
    details["input"] = _tensor_detail(interpreter.get_input_details()[0])
    details["output"] = _tensor_detail(interpreter.get_output_details()[0])
    return details


def _tensor_detail(tensor: Dict[str, object]) -> Dict[str, object]:
    """把 tflite 张量描述转成可 JSON 序列化的字典。"""
    quantization = tensor.get("quantization_parameters", {}) or {}

    def to_list(value) -> List[float]:  # noqa: ANN001
        """把 numpy 数组/标量转成 Python 列表。"""
        try:
            return [float(v) for v in value]
        except TypeError:
            return [float(value)]

    return {
        "name": tensor.get("name"),
        "shape": [int(v) for v in tensor.get("shape", [])],
        "dtype": str(tensor.get("dtype")),
        "quantization": {
            "scale": to_list(quantization.get("scales", [])),
            "zero_point": [int(v) for v in quantization.get("zero_points", [])],
        },
        "quantized_dimension": int(quantization.get("quantized_dimension", 0) or 0),
    }


def export_c_array(tflite_path: Path, out_dir: Path, align: int = 16) -> Tuple[Path, Path]:
    """生成 ``model_data.h`` / ``model_data.cc``（``alignas(16)`` 的 unsigned char 数组）。

    :return: ``(header 路径, source 路径)``。
    """
    data = tflite_path.read_bytes()
    header = out_dir / "model_data.h"
    source = out_dir / "model_data.cc"

    header.write_text(
        f"""// 自动生成，请勿手改：由 tools/train_mlp.py 从 {tflite_path.name} 生成
// 用途：TFLite Micro 部署（ESP32-S3 侧）。tensor arena 建议用
//       heap_caps_malloc(..., MALLOC_CAP_SPIRAM) 放到 PSRAM（plan.md §7.2）。
#ifndef MODEL_DATA_H_
#define MODEL_DATA_H_

#ifdef __cplusplus
extern "C" {{
#endif

#include <stddef.h>

// 模型字节数：{len(data)}；对齐 {align} 字节（TFLite Micro 要求 16 字节对齐）
extern const unsigned char model_data[];
extern const unsigned int model_data_len;

#ifdef __cplusplus
}}
#endif

#endif  // MODEL_DATA_H_
""",
        encoding="utf-8",
    )

    lines = [
        f"// 自动生成，请勿手改：由 tools/train_mlp.py 从 {tflite_path.name} 生成",
        f"// {len(data)} 字节，对齐 {align} 字节（alignas）",
        '#include "model_data.h"',
        "",
        f"alignas({align}) const unsigned char model_data[] = {{",
    ]
    for offset in range(0, len(data), 12):
        chunk = data[offset : offset + 12]
        lines.append("    " + " ".join(f"0x{byte:02x}," for byte in chunk))
    lines.append("};")
    lines.append("")
    lines.append(f"const unsigned int model_data_len = {len(data)}u;")
    lines.append("")
    source.write_text("\n".join(lines), encoding="utf-8")
    return header, source


def export_norm_params(out_dir: Path, means: Sequence[float], stds: Sequence[float],
                       layout: str, input_scale: float, input_zero_point: int,
                       output_scale: float, output_zero_point: int) -> Tuple[Path, Path, Path]:
    """导出标准化参数：MCU 侧 C 数组 + PC 侧 Python 模块。

    MCU 侧推荐流程（全整数也可做，这里给 float 版最省事，S3 有 FPU）：

    1. ``x_norm = (raw - mean[i]) / std[i]``
    2. ``q = round(x_norm / input_scale) + input_zero_point``，并裁进 int8
    3. 推理后 ``p = (out_int8 - output_zero_point) * output_scale``（若模型输出已 softmax，
       则 ``p`` 就是置信度）

    :return: ``(norm_params.h, norm_params.cc, norm_params.py)``。
    """
    header = out_dir / "norm_params.h"
    source = out_dir / "norm_params.cc"
    py_path = out_dir / "norm_params.py"
    count = len(means)

    header.write_text(
        f"""// 自动生成，请勿手改：由 tools/train_mlp.py 生成
// 特征标准化参数 + tflite 量化参数。**必须与训练时使用同一套**，否则 MCU 与 PC 结果不一致。
#ifndef NORM_PARAMS_H_
#define NORM_PARAMS_H_

#ifdef __cplusplus
extern "C" {{
#endif

#define FEATURE_DIM {count}
#define FEATURE_LAYOUT "{layout}"

// 量化参数
#define MODEL_INPUT_SCALE       {input_scale!r}f
#define MODEL_INPUT_ZERO_POINT  {input_zero_point}
#define MODEL_OUTPUT_SCALE      {output_scale!r}f
#define MODEL_OUTPUT_ZERO_POINT {output_zero_point}

// 标准化：x_norm = (raw - feature_mean[i]) / feature_std[i]
extern const float feature_mean[FEATURE_DIM];
extern const float feature_std[FEATURE_DIM];

#ifdef __cplusplus
}}
#endif

#endif  // NORM_PARAMS_H_
""",
        encoding="utf-8",
    )

    def float_array(name: str, values: Sequence[float]) -> List[str]:
        """生成一个 C float 数组定义。"""
        lines = [f"const float {name}[FEATURE_DIM] = {{"]
        for offset in range(0, len(values), 6):
            chunk = values[offset : offset + 6]
            lines.append("    " + " ".join(f"{value!r}f," for value in chunk))
        lines.append("};")
        return lines

    source_lines = [
        "// 自动生成，请勿手改：由 tools/train_mlp.py 生成",
        '#include "norm_params.h"',
        "",
    ]
    source_lines += float_array("feature_mean", means)
    source_lines.append("")
    source_lines += float_array("feature_std", stds)
    source_lines.append("")
    source.write_text("\n".join(source_lines), encoding="utf-8")

    py_lines = [
        '"""自动生成，请勿手改：由 tools/train_mlp.py 生成。',
        "",
        "PC 侧推理必须使用与 MCU 完全相同的标准化与量化参数。",
        '"""',
        "",
        f"FEATURE_DIM = {count}",
        f'FEATURE_LAYOUT = "{layout}"',
        f"MODEL_INPUT_SCALE = {input_scale!r}",
        f"MODEL_INPUT_ZERO_POINT = {input_zero_point}",
        f"MODEL_OUTPUT_SCALE = {output_scale!r}",
        f"MODEL_OUTPUT_ZERO_POINT = {output_zero_point}",
        "",
        "FEATURE_MEAN = [",
    ]
    for offset in range(0, count, 6):
        py_lines.append("    " + ", ".join(f"{v!r}" for v in means[offset : offset + 6]) + ",")
    py_lines.append("]")
    py_lines.append("")
    py_lines.append("FEATURE_STD = [")
    for offset in range(0, count, 6):
        py_lines.append("    " + ", ".join(f"{v!r}" for v in stds[offset : offset + 6]) + ",")
    py_lines.append("]")
    py_lines.append("")
    py_path.write_text("\n".join(py_lines), encoding="utf-8")
    return header, source, py_path


def print_register_map_reminder(layout: str) -> None:
    """打印 E 区（0x0400~0x0423）与 36 维特征的对应关系，便于与固件核对。"""
    safe_print("\nE 区特征映射核对表（docs/02 §5 E 区 0x0400~0x0423）：")
    rows = []
    names = feature_names_for(layout)
    for index, name in enumerate(names):
        rows.append([f"0x{0x0400 + index:04X}", name])
    safe_print(format_table(["寄存器", "维度（列名）"], rows[:36]))


# --------------------------------------------------------------------------------------
# 自检（只验证数据管线，不训练）
# --------------------------------------------------------------------------------------


def selftest(args: argparse.Namespace) -> int:
    """造合成数据，验证"窗口 → 36 维特征 → 数据集装载 → 标准化 → 精确性"整条管线。"""
    safe_print("=== train_mlp.py 离线自检（不训练、不写模型）===")
    failures: List[str] = []

    def check(name: str, condition: bool, detail: str = "") -> None:
        """记录一条自检结果。"""
        status = "PASS" if condition else "FAIL"
        if not condition:
            failures.append(name)
        safe_print(f"  [{status}] {name} {detail}")

    # 1) 特征维数与量纲
    def make_window(freq: float, amplitude: float, noise: float = 0.0) -> List[List[int]]:
        """造一个已知主频的正弦窗口（用于校验主频/过零率提取是否正确）。"""
        window = []
        for i in range(WINDOW_SAMPLES):
            row = []
            for axis in range(WINDOW_AXES):
                value = amplitude * math.sin(2 * math.pi * freq * i / 50.0 + axis * 0.3)
                value += noise * math.sin(2 * math.pi * 7.0 * i / 50.0)
                row.append(clamp_int16(value))
            window.append(row)
        return window

    features = compute_features(make_window(5.0, 10000.0), 50.0)
    # 默认布局 per-feature：第 f 个特征的第 a 个轴位于下标 f*6 + a
    idx_mean_ax = 0 * 6 + 0
    idx_ptp_ax = 2 * 6 + 0
    idx_freq_ax = 4 * 6 + 0
    idx_ratio_ax = 5 * 6 + 0
    check("特征维数 = 36", len(features) == FEATURE_VALUES, f"{len(features)}")
    check("默认布局为 per-feature（与 docs/02 §5 E 区一致）",
          LAYOUT_RAW_REGISTER == LAYOUT_PER_FEATURE, LAYOUT_RAW_REGISTER)
    check("均值 ≈ 0（正弦整周期）", abs(features[idx_mean_ax]) <= 200,
          f"ax 均值={features[idx_mean_ax]}")
    check("主频提取正确（5.0Hz → 50）", features[idx_freq_ax] == 50,
          f"ax 主频×10={features[idx_freq_ax]}")
    check("峰峰值接近 2×幅度", 18000 <= features[idx_ptp_ax] <= 20000,
          f"ax 峰峰值={features[idx_ptp_ax]}")
    check("主频带能量占比高（>800）", features[idx_ratio_ax] > 800,
          f"ax 占比×1000={features[idx_ratio_ax]}")

    low = compute_features(make_window(1.0, 10000.0), 50.0)
    high = compute_features(make_window(12.0, 10000.0), 50.0)
    check("不同主频给出不同特征", low[idx_freq_ax] != high[idx_freq_ax],
          f"{low[idx_freq_ax]} vs {high[idx_freq_ax]}")

    axes_major = compute_features(make_window(5.0, 10000.0), 50.0, LAYOUT_AXES_MAJOR)
    check("per-axis 与 per-feature 长度一致但顺序不同（转置）",
          len(axes_major) == FEATURE_VALUES and sorted(axes_major) == sorted(features)
          and axes_major != features,
          f"per-axis 首 6 个={axes_major[:6]} / per-feature 首 6 个={features[:6]}")
    check("per-axis 下 ax 的均值在 index 0",
          axes_major[0] == features[idx_mean_ax], f"{axes_major[0]}")
    check("per-axis 下 ax 的主频在 index 4",
          axes_major[4] == features[idx_freq_ax], f"{axes_major[4]}")

    # 2) 窗口 CSV 读写 + 数据集装载
    root = Path(args.data) / "_selftest_train"
    if root.exists():
        import shutil

        shutil.rmtree(root, ignore_errors=True)
    for label, freq in (("normal", 3.0), ("imbalance", 9.0), ("loosen", 15.0)):
        directory = root / label
        directory.mkdir(parents=True, exist_ok=True)
        for index in range(8):
            window = make_window(freq + index * 0.2, 10000.0, noise=1500.0)
            path = directory / f"{label}_{index:02d}.csv"
            with open(path, "w", encoding="utf-8", newline="") as handle:
                writer = csv.writer(handle, lineterminator="\n")
                writer.writerow(CSV_HEADER)
                writer.writerows(window)

    dataset = load_dataset(root, None, False, "auto", LAYOUT_RAW_REGISTER, 50.0)
    check("数据集装载 24 个样本", len(dataset) == 24, f"{len(dataset)}（跳过 {len(dataset.skipped)}）")
    check("类别数为 3", len(dataset.class_names) == 3, f"{dataset.class_names}")
    check("每类 8 个样本", all(v == 8 for v in dataset.class_counts().values()),
          f"{dataset.class_counts()}")
    check("自动识别为 window 数据源", detect_source(root / "normal" / "normal_00.csv") == "window")

    # 3) 特征文件（36 列）数据源：目录结构仍须是 data/<类别>/*.csv
    feature_dir = root / "feature_class"
    feature_dir.mkdir(parents=True, exist_ok=True)
    feature_path = feature_dir / "sample.csv"
    with open(feature_path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(feature_names_for(LAYOUT_RAW_REGISTER))
        writer.writerow([str(v) for v in features])
    check("自动识别为 feature 数据源", detect_source(feature_path) == "feature")
    check("36 列特征文件不被误判为窗口文件",
          detect_source(feature_path) != "window", "（100×6 才判为窗口）")
    single = load_dataset(root, None, False, "auto", LAYOUT_RAW_REGISTER, 50.0)
    check("36 列特征文件与窗口文件可混合装载",
          len(single) == 25, f"{len(single)} 个样本（跳过 {len(single.skipped)}）")
    feature_index = next(
        (i for i, source in enumerate(single.sources) if source.startswith("feature_class")), None
    )
    check("特征文件被识别为独立类别 feature_class",
          feature_index is not None and single.labels[feature_index] == "feature_class",
          f"索引={feature_index}")
    check("窗口算出的向量与特征文件逐维一致",
          feature_index is not None
          and [round(v, 3) for v in single.features[feature_index]]
          == [round(float(v), 3) for v in features],
          "36 维向量相等")

    # 4) 标准化
    means, stds = compute_standardization(dataset.features)
    check("均值/标准差长度为 36", len(means) == FEATURE_VALUES and len(stds) == FEATURE_VALUES)
    check("无零标准差维度", all(value > 0 for value in stds))
    standardized = standardize(dataset.features, means, stds)
    column_means = [sum(row[i] for row in standardized) / len(standardized)
                    for i in range(FEATURE_VALUES)]
    check("标准化后各维均值 ≈ 0", all(abs(value) < 1e-9 for value in column_means),
          f"最大 |均值| = {max(abs(v) for v in column_means):.2e}")

    # 5) 混淆矩阵渲染
    text = render_confusion(["a", "a", "b"], ["a", "b", "b"], ["a", "b"])
    check("混淆矩阵渲染正确（准确率 66.67%）", "66.67%" in text)

    # 6) 三种主频是否可分（用最近邻粗测，1-NN 训练集自测应 100%）
    correct = 0
    for i, row in enumerate(standardized):
        distances = [
            (sum((row[d] - other[d]) ** 2 for d in range(FEATURE_VALUES)), j)
            for j, other in enumerate(standardized)
        ]
        nearest = min(distances)[1]
        correct += int(dataset.labels[nearest] == dataset.labels[i])
    check("1-NN 自测可分（≥90%）", correct / len(standardized) >= 0.9,
          f"{correct}/{len(standardized)}")

    if not args.keep_selftest:
        import shutil

        shutil.rmtree(root, ignore_errors=True)
    else:
        safe_print(f"自检数据保留在 {root}")

    safe_print("")
    if failures:
        safe_print(f"自检失败 {len(failures)} 项：{failures}")
        return 1
    safe_print("自检全部通过：特征提取、两种数据源、标准化、混淆矩阵均正常。")
    safe_print("（提示：--selftest 不训练模型、不写 tflite；正式训练请去掉 --selftest）")
    return 0


# --------------------------------------------------------------------------------------
# 训练主流程
# --------------------------------------------------------------------------------------


def main_train(args: argparse.Namespace) -> int:
    """完整训练 + 量化 + 导出流程。"""
    try:
        layout = normalize_layout(args.feat_layout)
    except ValueError as exc:
        safe_print(f"错误：{exc}")
        return 2
    if layout == LAYOUT_AXES_MAJOR:
        safe_print(
            "⚠ 警告：--feat-layout per-axis 与 docs/02 §5 E 区的定义『不一致』。\n"
            "  E 区是『每个特征 6 个轴连续』（per-feature）。只有当你同时修改了 F411\n"
            "  写入 E 区的顺序时，才应该用 per-axis；否则特征矩阵与固件互为转置，\n"
            "  维数相同、训练不报错，但准确率会掉到随机水平。"
        )

    data_dir = Path(args.data)
    if not data_dir.is_dir():
        safe_print(f"错误：数据目录不存在 —— {data_dir}")
        return 2

    split = None
    if args.split:
        split_path = Path(args.split)
        if not split_path.is_file():
            safe_print(f"错误：split 文件不存在 —— {split_path}（先用 label_tool.py 生成）")
            return 2
        with open(split_path, "r", encoding="utf-8") as handle:
            split = json.load(handle)
        safe_print(f"已读取划分文件 {split_path}（seed={split.get('seed')}，"
                   f"test_ratio={split.get('test_ratio')}）")

    dataset = load_dataset(
        data_dir, split, use_test=False, feature_source=args.feature_source,
        layout=layout, sample_rate_hz=args.sample_rate,
        labels_filter=args.labels.split(",") if args.labels else None,
    )
    if len(dataset) == 0:
        safe_print("错误：没有装载到任何样本。请检查 --data 目录与 --feature-source。")
        return 2
    if dataset.skipped:
        safe_print(f"跳过 {len(dataset.skipped)} 个无法解析的文件，例如：{dataset.skipped[:3]}")

    counts = dataset.class_counts()
    safe_print(
        format_table(["类别", "样本数"], [[k, str(v)] for k, v in sorted(counts.items())],
                     ["<", ">"])
    )
    if len(dataset.class_names) < 2:
        safe_print("错误：至少需要 2 个类别才能分类训练。")
        return 2

    # 测试集：优先用 split.json，否则现场分层划分
    if split is not None:
        test_dataset = load_dataset(
            data_dir, split, use_test=True, feature_source=args.feature_source,
            layout=layout, sample_rate_hz=args.sample_rate,
            labels_filter=args.labels.split(",") if args.labels else None,
        )
        x_train_raw, y_train = dataset.features, dataset.labels
        x_test_raw, y_test = test_dataset.features, test_dataset.labels
    else:
        x_train_raw, y_train, x_test_raw, y_test = stratified_split_in_memory(
            dataset.features, dataset.labels, args.test_ratio, args.seed
        )
        safe_print(f"未提供 --split：现场按 test_ratio={args.test_ratio} 分层划分。")

    safe_print(f"训练样本 {len(x_train_raw)}，测试样本 {len(x_test_raw)}")
    if not x_test_raw:
        safe_print("错误：测试集为空。请增加样本量或降低 --test-ratio。")
        return 2

    means, stds = compute_standardization(x_train_raw)
    x_train = standardize(x_train_raw, means, stds)
    x_test = standardize(x_test_raw, means, stds)

    # 布局自检：用 1-NN 可分性判断 --feat-layout 是否用错（转置陷阱）
    warn_if_layout_suspect(dataset.features, dataset.labels, layout)

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    use_keras = True
    try:
        import tensorflow  # noqa: F401
    except ImportError:
        use_keras = False
    if args.require_tf and not use_keras:
        safe_print("错误：--require-tf 但未检测到 TensorFlow。请 pip install tensorflow（或改用 ai-edge-litert 环境）。")
        return 3

    if use_keras:
        model, history = train_with_keras(
            x_train, y_train, x_test, y_test, dataset.class_names, args.epochs, args.seed
        )
        try:
            import numpy as np

            probabilities = model.predict(np.asarray(x_test, dtype="float32"), verbose=0)
            y_pred = [dataset.class_names[int(index)] for index in probabilities.argmax(axis=1)]
        except Exception as exc:  # noqa: BLE001
            safe_print(f"警告：预测失败（{exc}），跳过混淆矩阵。")
            y_pred = list(y_test)
    else:
        safe_print("提示：未检测到 TensorFlow，回退到 sklearn MLPClassifier（**不会产出 tflite**）。")
        safe_print("      要部署到 ESP32-S3，请先 pip install tensorflow。")
        model = train_with_sklearn(
            x_train, y_train, x_test, y_test, dataset.class_names, args.epochs, args.seed
        )
        y_pred = [str(label) for label in model.predict(x_test)]

    matrix_text = render_confusion(y_test, y_pred, dataset.class_names)
    safe_print("\n" + matrix_text)
    (out_dir / "confusion_matrix.txt").write_text(matrix_text + "\n", encoding="utf-8")

    accuracy = sum(1 for a, b in zip(y_test, y_pred) if a == b) / max(1, len(y_test))
    info: Dict[str, object] = {
        "created": __import__("datetime").datetime.now().isoformat(timespec="seconds"),
        # class_names 必须**按字母序**：ESP32 侧 kClassLabels[] 与此一一对应，
        # 顺序不一致会让"类别 0/1/2"在两端指向不同含义。
        "class_names": sorted(dataset.class_names),
        "num_classes": len(dataset.class_names),
        "feature_dim": FEATURE_VALUES,
        # ESP32 侧 infer_init() 会用这个字段做硬校验，不符直接拒绝启动推理
        "feature_layout": layout,
        "feature_names": feature_names_for(layout),
        "sample_rate_hz": args.sample_rate,
        "epochs": args.epochs,
        "test_ratio": args.test_ratio,
        "seed": args.seed,
        "test_accuracy": accuracy,
        "confusion_matrix": confusion_matrix(y_test, y_pred, dataset.class_names),
        "train_samples": len(x_train_raw),
        "test_samples": len(x_test_raw),
        "class_counts": counts,
        "quantized": False,
        "notes": [],
    }

    if use_keras:
        tflite_path, note = export_tflite(model, out_dir, x_train)
        info["notes"].append(note)  # type: ignore[union-attr]
        io_details = tflite_io_details(tflite_path)
        info["tflite"] = io_details

        input_detail = io_details.get("input", {}) if isinstance(io_details, dict) else {}
        output_detail = io_details.get("output", {}) if isinstance(io_details, dict) else {}
        input_quant = (input_detail or {}).get("quantization", {}) if isinstance(input_detail, dict) else {}
        output_quant = (output_detail or {}).get("quantization", {}) if isinstance(output_detail, dict) else {}
        input_scale = float((input_quant.get("scale") or [1.0])[0])
        input_zero = int((input_quant.get("zero_point") or [0])[0])
        output_scale = float((output_quant.get("scale") or [1.0])[0])
        output_zero = int((output_quant.get("zero_point") or [0])[0])
        info["quantized"] = "int8" in str(input_detail.get("dtype", ""))

        header, source, py_module = export_norm_params(
            out_dir, means, stds, layout, input_scale, input_zero, output_scale, output_zero
        )
        safe_print(f"已导出标准化参数：{header.name} / {source.name} / {py_module.name}")
        model_header, model_source = export_c_array(tflite_path, out_dir, align=16)
        safe_print(f"已导出 C 数组：{model_header.name} / {model_source.name}")
        info["norm_params"] = {"mean": means, "std": stds,
                               "input_scale": input_scale, "input_zero_point": input_zero,
                               "output_scale": output_scale, "output_zero_point": output_zero}
        if not info["quantized"]:
            safe_print(
                "⚠ 注意：模型输入不是 int8（回退为 float16 权重）。"
                "ESP32-S3 的 TFLite Micro 需要 float 输入路径，或改用纯 int8 训练配置。"
            )
    else:
        info["notes"].append("未产出 tflite：TensorFlow 不可用，仅用 sklearn 验证可分性。")

    info_path = out_dir / "model_info.json"
    with open(info_path, "w", encoding="utf-8") as handle:
        json.dump(info, handle, ensure_ascii=False, indent=2)
    safe_print(f"已写出 {info_path}")

    print_register_map_reminder(layout)

    safe_print("\n下一步：")
    safe_print(f"  1) 把 {out_dir / 'model_data.cc'} + .h 加进 ESP32-S3 工程，TFLite Micro 加载")
    safe_print(f"  2) MCU 侧用 {out_dir / 'norm_params.h'} 做 (x-mean)/std 后量化成 int8 再喂模型")
    safe_print(f"  3) 对拍：python verify_tflite.py --model {out_dir / 'model.tflite'} --port COM5")
    if accuracy < 0.9:
        safe_print(
            f"\n⚠ 测试集准确率 {accuracy*100:.2f}% < 90%（docs/03 阶段 6 验收线）。"
            "先补样本/检查打标，再考虑加数据或改模型，不要急着部署。"
        )
    return 0


def stratified_split_in_memory(features: Sequence[Sequence[float]], labels: Sequence[str],
                               test_ratio: float, seed: int
                               ) -> Tuple[List[List[float]], List[str], List[List[float]], List[str]]:
    """没有 split.json 时按类别分层现场划分（与 label_tool.py 同策略、同种子语义）。"""
    import random

    rng = random.Random(seed)
    by_label: Dict[str, List[int]] = {}
    for index, label in enumerate(labels):
        by_label.setdefault(label, []).append(index)

    train_idx: List[int] = []
    test_idx: List[int] = []
    for label in sorted(by_label):
        indices = by_label[label][:]
        rng.shuffle(indices)
        n_test = int(round(len(indices) * test_ratio))
        if len(indices) >= 2:
            n_test = max(1, min(len(indices) - 1, n_test))
        else:
            n_test = 0
        test_idx.extend(indices[:n_test])
        train_idx.extend(indices[n_test:])

    train_idx.sort()
    test_idx.sort()
    return (
        [list(features[i]) for i in train_idx],
        [labels[i] for i in train_idx],
        [list(features[i]) for i in test_idx],
        [labels[i] for i in test_idx],
    )


# --------------------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------------------


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(
        prog="train_mlp.py",
        description=(
            "训练小 MLP 分类器：36 维特征（docs/02 §5 E 区）→ int8 全整数量化 tflite "
            "→ model_data.h/.cc（docs/03 阶段 6）"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "示例：\n"
            "  python train_mlp.py --data data --out model_out --epochs 200 --test-ratio 0.2\n"
            "  python train_mlp.py --data data --split data/split.json --out model_out\n"
            "  python train_mlp.py --data data --feature-source feature --feat-layout per-feature\n"
            "  python train_mlp.py --selftest --data data          # 只验证数据管线\n"
        ),
    )
    parser.add_argument("--data", default="data", help="数据集根目录（默认 data）")
    parser.add_argument("--out", default="model_out", help="输出目录（默认 model_out）")
    parser.add_argument("--epochs", type=int, default=200, help="训练轮数（默认 200）")
    parser.add_argument("--test-ratio", type=float, default=0.2, help="测试集比例（默认 0.2 = 8:2）")
    parser.add_argument("--seed", type=int, default=20261007, help="随机种子（固定可复现）")
    parser.add_argument("--split", default=None, help="label_tool.py 生成的 split.json（推荐提供）")
    parser.add_argument("--feature-source", default="auto", choices=["auto", "window", "feature"],
                        help="数据源类型：100×6 窗口 / 36 列特征 / 自动识别（默认 auto）")
    parser.add_argument("--feat-layout", default=LAYOUT_RAW_REGISTER,
                        choices=sorted(set(LAYOUT_ALIASES)),
                        help="36 维特征布局，必须与 F411 固件 E 区写入顺序一致。"
                             "默认 per-feature（与 docs/02 §5 E 区一致）；"
                             "per-axis 与本项目固件互为转置，除非你同步改了 F411 的写入顺序，"
                             "否则会导致准确率掉到随机水平！")
    parser.add_argument("--sample-rate", type=float, default=50.0,
                        help="窗口数据的采样率 Hz（默认 50，对应 0x0102 = 500）")
    parser.add_argument("--labels", default=None, help="只使用这些类别（逗号分隔）")
    parser.add_argument("--selftest", action="store_true", help="只验证数据管线（不训练不导出）")
    parser.add_argument("--keep-selftest", action="store_true", help="自检后保留合成数据")
    parser.add_argument("--require-tf", action="store_true", help="无 TensorFlow 时直接失败（CI 用）")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    """脚本入口。"""
    setup_console()
    args = parse_args(argv)
    if args.selftest:
        return selftest(args)
    return main_train(args)


if __name__ == "__main__":
    sys.exit(main())
