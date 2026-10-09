#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""交互式打标 + 数据体检 + 分层划分工具（docs/03 阶段 6 第 2~3 步）。

三个职责
--------
1. **数据体检**：扫描 ``data/<类别>/`` 下所有 CSV，检测坏样本——
   行数 ≠ 100、列数 ≠ 6、非数字、非 int16、全零/常量（疑似采集失败）。
2. **键盘打标**：对未打标目录（默认 ``unlabeled`` / ``raw`` / ``_inbox``）里的样本，
   按单个数字键归入目标类别目录（**移动文件**，不复制，避免数据集翻倍）。
3. **分层划分**：按类别分层抽 8:2 训练/测试集，固定随机种子，写出 ``data/split.json``，
   供 ``train_mlp.py`` 直接使用（``--split data/split.json``）。

用法::

    python label_tool.py --data data                      # 体检 + 打印统计
    python label_tool.py --data data --label-keys 0=normal 1=imbalance 2=loosen
    python label_tool.py --data data --split-only         # 只做分层划分
    python label_tool.py --selftest                        # 造临时数据集跑通全流程

打标键位约定（可用 ``--label-keys`` 覆盖）::

    0..9  归入对应类别（按键即移动）
    s     跳过当前样本
    d     移到 _quarantine（丢弃/疑似坏样本）
    b     返回上一个（仅提示，不回滚文件）
    q     退出打标
"""

from __future__ import annotations

import argparse
import csv
import json
import random
import shutil
import sys
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from _console import format_table, safe_print, setup_console

# --------------------------------------------------------------------------------------
# 常量（与 docs/02 §5 D 区、capture_window.py 输出保持一致）
# --------------------------------------------------------------------------------------

WINDOW_SAMPLES = 100
WINDOW_AXES = 6
CSV_HEADER = ["ax", "ay", "az", "gx", "gy", "gz"]
INT16_MIN, INT16_MAX = -32768, 32767

#: 视为"尚未打标"的目录名（打标模式的输入池）
UNLABELED_DIR_NAMES = ("unlabeled", "raw", "_inbox", "todo", "new")

#: 坏样本隔离目录（以 ``_`` 开头，默认不参与训练）
QUARANTINE_DIR = "_quarantine"

#: 默认打标键位 → 类别名
DEFAULT_LABEL_KEYS: Dict[str, str] = {
    "0": "normal",
    "1": "imbalance",
    "2": "loosen",
    "3": "static",
    "4": "circle",
    "5": "shake",
}

#: 默认分层划分参数
DEFAULT_TEST_RATIO = 0.2
DEFAULT_SEED = 20261007  # 固定种子：保证划分可复现（plan.md 里 10/7 是里程碑日）


# --------------------------------------------------------------------------------------
# 数据结构
# --------------------------------------------------------------------------------------


@dataclass
class SampleInfo:
    """一个样本文件体检后的信息。"""

    path: Path
    label: str
    rows: int = 0
    cols: int = 0
    problems: List[str] = field(default_factory=list)
    axis_mean: List[float] = field(default_factory=lambda: [0.0] * WINDOW_AXES)
    axis_range: List[int] = field(default_factory=lambda: [0] * WINDOW_AXES)

    @property
    def ok(self) -> bool:
        """是否通过体检（没有任何问题项）。"""
        return not self.problems

    def summary(self) -> str:
        """一行摘要：行×列 + 各轴峰峰值，便于人工判断打标是否合理。"""
        ranges = " ".join(f"{name}:{value}" for name, value in zip(CSV_HEADER, self.axis_range))
        return f"{self.rows}×{self.cols} 峰峰值[{ranges}]"


# --------------------------------------------------------------------------------------
# 体检
# --------------------------------------------------------------------------------------


def inspect_sample(path: Path, label: str, max_problems: int = 4) -> SampleInfo:
    """体检单个 CSV：行数/列数/数值范围/常量检测。

    :param path: CSV 路径。
    :param label: 所属类别（父目录名）。
    :param max_problems: 每个文件最多记录几条具体问题（其余折叠成计数），
        避免"4800 行全坏"时把报告刷爆。
    :return: :class:`SampleInfo`（问题写进 ``problems``，不抛异常）。
    """
    info = SampleInfo(path=path, label=label)
    extra_problems = 0

    def note(message: str) -> None:
        """记录一条问题（超过 max_problems 后只累加计数）。"""
        nonlocal extra_problems
        if len(info.problems) < max_problems:
            if message not in info.problems:
                info.problems.append(message)
        else:
            extra_problems += 1

    try:
        with open(path, "r", encoding="utf-8", newline="") as handle:
            rows = list(csv.reader(handle))
    except OSError as exc:
        info.problems.append(f"无法读取：{exc}")
        return info
    except UnicodeDecodeError as exc:
        info.problems.append(f"编码错误（应为 UTF-8）：{exc}")
        return info

    if not rows:
        info.problems.append("空文件")
        return info

    # 允许有表头，也允许没有表头（只看第一行能不能整行转成整数）
    start = 0
    try:
        [int(cell) for cell in rows[0]]
        first_is_data = True
    except ValueError:
        first_is_data = False
    if not first_is_data:
        header = [cell.strip().lower() for cell in rows[0]]
        if header[:WINDOW_AXES] != CSV_HEADER:
            note(f"表头异常：{rows[0]}")
        start = 1

    data_rows = rows[start:]
    info.rows = len(data_rows)
    if info.rows != WINDOW_SAMPLES:
        note(f"行数 {info.rows} ≠ {WINDOW_SAMPLES}")

    columns = 0
    values: List[List[int]] = []
    for index, row in enumerate(data_rows):
        if not row:
            note(f"第 {start + index + 1} 行为空")
            continue
        columns = max(columns, len(row))
        if len(row) != WINDOW_AXES:
            note(f"第 {start + index + 1} 行 {len(row)} 列 ≠ {WINDOW_AXES}")
            continue
        try:
            parsed = [int(cell) for cell in row]
        except ValueError:
            note(f"第 {start + index + 1} 行含非数字：{row}")
            continue
        out_of_range = [v for v in parsed if not INT16_MIN <= v <= INT16_MAX]
        if out_of_range:
            note(f"第 {start + index + 1} 行超出 int16：{out_of_range[:3]}")
            continue
        values.append(parsed)
    info.cols = columns

    if values:
        info.axis_mean = [
            sum(row[axis] for row in values) / len(values) for axis in range(WINDOW_AXES)
        ]
        info.axis_range = [
            max(row[axis] for row in values) - min(row[axis] for row in values)
            for axis in range(WINDOW_AXES)
        ]
        if all(value == 0 for row in values for value in row):
            note("全零（疑似采集失败）")
        elif all(max(row[axis] for row in values) == min(row[axis] for row in values)
                 for axis in range(WINDOW_AXES)):
            note("六轴全为常量（疑似采集卡死）")

    if extra_problems:
        info.problems.append(f"… 另有 {extra_problems} 处同类问题")
    return info


def scan_dataset(data_dir: Path, include_quarantine: bool = False) -> Dict[str, List[SampleInfo]]:
    """扫描 ``data_dir`` 下的类别子目录，返回 ``{类别: [样本信息]}``。

    :param data_dir: 数据集根目录。
    :param include_quarantine: 是否把 ``_quarantine`` 也纳入统计。
    """
    result: Dict[str, List[SampleInfo]] = {}
    if not data_dir.is_dir():
        return result
    for sub in sorted(p for p in data_dir.iterdir() if p.is_dir()):
        if sub.name.startswith("_") and not (include_quarantine and sub.name == QUARANTINE_DIR):
            continue
        files = sorted(sub.glob("*.csv"))
        if not files:
            continue
        result[sub.name] = [inspect_sample(path, sub.name) for path in files]
    return result


def print_dataset_report(data_dir: Path, dataset: Dict[str, List[SampleInfo]],
                         max_bad_shown: int = 20) -> Tuple[int, int]:
    """打印体检报告表格。

    :return: ``(样本总数, 坏样本数)``。
    """
    if not dataset:
        safe_print(f"目录 {data_dir} 下没有找到任何 CSV 样本。")
        return 0, 0

    rows = []
    total = bad_total = 0
    for label, samples in dataset.items():
        good = sum(1 for s in samples if s.ok)
        bad = len(samples) - good
        total += len(samples)
        bad_total += bad
        flagged = "❌" if bad else "✅"
        rows.append([label, str(len(samples)), str(good), str(bad), flagged])
    safe_print(f"数据集体检：{data_dir}")
    safe_print(format_table(["类别", "样本数", "合格", "坏样本", ""], rows, ["<", ">", ">", ">", "<"]))

    bad_samples = [s for samples in dataset.values() for s in samples if not s.ok]
    if bad_samples:
        safe_print(f"\n坏样本明细（最多显示 {max_bad_shown} 条；用 --quarantine 移入 {QUARANTINE_DIR}/）：")
        detail = [[str(s.path.relative_to(data_dir)), s.label, "; ".join(s.problems)[:110]]
                  for s in bad_samples[:max_bad_shown]]
        safe_print(format_table(["文件", "类别", "问题（截断显示）"], detail))
        if len(bad_samples) > max_bad_shown:
            safe_print(f"… 另有 {len(bad_samples) - max_bad_shown} 条")

    counts = {label: len(samples) for label, samples in dataset.items()}
    if counts:
        values = list(counts.values())
        imbalance = max(values) / max(1, min(values))
        safe_print(
            f"\n合计 {total} 个样本（坏 {bad_total}），类别数 {len(counts)}，"
            f"最多/最少 = {max(values)}/{min(values)} = {imbalance:.2f}×"
        )
        if imbalance > 2.0:
            safe_print("提示：类别不均衡（>2×）。docs/03 建议每类 50~80 组，否则测试集准确率会虚高。")
        if total and total < 150:
            safe_print("提示：样本总量偏少（docs/03 建议 3~4 类共 200~300 组），先每类采够 50 组。")
    return total, bad_total


# --------------------------------------------------------------------------------------
# 隔离与打标
# --------------------------------------------------------------------------------------


def quarantine(data_dir: Path, dataset: Dict[str, List[SampleInfo]], dry_run: bool = False) -> int:
    """把坏样本移动到 ``_quarantine/<原类别>/``。

    :return: 移动的文件数。
    """
    moved = 0
    for samples in dataset.values():
        for sample in samples:
            if sample.ok:
                continue
            target_dir = data_dir / QUARANTINE_DIR / sample.label
            target = target_dir / sample.path.name
            if dry_run:
                safe_print(f"  [dry-run] {sample.path} → {target}")
                moved += 1
                continue
            target_dir.mkdir(parents=True, exist_ok=True)
            if target.exists():
                target = target_dir / f"{sample.path.stem}_dup{sample.path.suffix}"
            try:
                shutil.move(str(sample.path), str(target))
                moved += 1
            except OSError as exc:
                safe_print(f"  移动失败 {sample.path}: {exc}")
    return moved


def collect_unlabeled(data_dir: Path, dirs: Sequence[str]) -> List[Path]:
    """收集待打标样本（按文件名排序）。"""
    files: List[Path] = []
    for name in dirs:
        sub = data_dir / name
        if sub.is_dir():
            files.extend(sorted(sub.glob("*.csv")))
    return files


def interactive_label(data_dir: Path, label_keys: Dict[str, str], dirs: Sequence[str],
                      assume_yes: bool = False) -> int:
    """键盘打标主循环：把未打标样本移动到目标类别目录。

    :param data_dir: 数据集根目录。
    :param label_keys: ``{'0': 'normal', ...}`` 键位映射。
    :param dirs: 待打标目录名列表。
    :param assume_yes: 为 True 时不读键盘（用于自动化），直接把键位表打印出来。
    :return: 已打标文件数。
    """
    files = collect_unlabeled(data_dir, dirs)
    if not files:
        safe_print(f"没有待打标样本（查找目录：{', '.join(dirs)}）。"
                   f"先用 capture_window.py --label <未打标目录> 采集。")
        return 0

    safe_print(f"待打标样本 {len(files)} 个。键位：")
    key_rows = [[key, value] for key, value in sorted(label_keys.items())]
    key_rows += [["s", "跳过"], ["d", f"移到 {QUARANTINE_DIR}"], ["q", "退出"]]
    safe_print(format_table(["按键", "动作"], key_rows))
    if assume_yes:
        safe_print("（--yes 模式：仅列出待打标文件，不做交互）")
        for path in files:
            safe_print(f"  {path.relative_to(data_dir)}")
        return 0

    labeled = 0
    for index, path in enumerate(files, start=1):
        info = inspect_sample(path, path.parent.name)
        safe_print("\n" + "-" * 76)
        safe_print(f"[{index}/{len(files)}] {path.relative_to(data_dir)}")
        safe_print(f"  {info.summary()}")
        if not info.ok:
            safe_print(f"  ⚠ 体检问题：{'; '.join(info.problems)}")
        safe_print("  各轴均值：" + " ".join(
            f"{name}:{value:+.0f}" for name, value in zip(CSV_HEADER, info.axis_mean)
        ))
        if not info.problems:
            # 无问题时用一个极小的 ASCII 波形帮助判断（每轴用峰峰值当幅度指标）
            safe_print("  提示：峰峰值大→振动剧烈；六轴都接近 0→静止。")
        try:
            choice = input("  按键选择（0-9/s/d/q）> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            safe_print("\n输入中断，退出打标。")
            break

        if choice == "q":
            safe_print("退出打标。")
            break
        if choice in ("s", ""):
            continue
        if choice == "d":
            target_dir = data_dir / QUARANTINE_DIR / info.label
        elif choice in label_keys:
            target_dir = data_dir / label_keys[choice]
        else:
            safe_print(f"  未知按键 {choice!r}，跳过。")
            continue

        target_dir.mkdir(parents=True, exist_ok=True)
        target = target_dir / path.name
        if target.exists():
            target = target_dir / f"{path.stem}_dup{path.suffix}"
        try:
            shutil.move(str(path), str(target))
            labeled += 1
            safe_print(f"  → {target.relative_to(data_dir)}")
        except OSError as exc:
            safe_print(f"  移动失败：{exc}")
    safe_print(f"\n本次打标 {labeled} 个文件。")
    return labeled


# --------------------------------------------------------------------------------------
# 分层划分
# --------------------------------------------------------------------------------------


def stratified_split(data_dir: Path, dataset: Dict[str, List[SampleInfo]],
                     test_ratio: float = DEFAULT_TEST_RATIO, seed: int = DEFAULT_SEED,
                     only_ok: bool = True) -> Dict[str, object]:
    """按类别分层抽训练/测试集（固定随机种子，可复现）。

    :param data_dir: 数据集根目录（用于生成相对路径）。
    :param dataset: :func:`scan_dataset` 的结果。
    :param test_ratio: 测试集比例（默认 0.2）。
    :param seed: 随机种子。
    :param only_ok: 是否只纳入体检合格的样本。
    :return: 可直接 ``json.dump`` 的字典。
    """
    rng = random.Random(seed)
    train: List[str] = []
    test: List[str] = []
    per_class: Dict[str, Dict[str, int]] = {}

    for label in sorted(dataset):
        paths = [s.path for s in dataset[label] if (s.ok or not only_ok)]
        paths.sort()  # 排序保证跨平台一致
        rng.shuffle(paths)
        n_test = int(round(len(paths) * test_ratio))
        # 保证小类别也有测试样本（>=2 个样本时至少留 1 个测试）
        if len(paths) >= 2:
            n_test = max(1, min(len(paths) - 1, n_test))
        else:
            n_test = 0
        test_paths = paths[:n_test]
        train_paths = paths[n_test:]
        train.extend(str(p.relative_to(data_dir)) for p in sorted(train_paths))
        test.extend(str(p.relative_to(data_dir)) for p in sorted(test_paths))
        per_class[label] = {
            "total": len(paths),
            "train": len(train_paths),
            "test": len(test_paths),
            "excluded_bad": sum(1 for s in dataset[label] if not s.ok),
        }

    return {
        "created": datetime.now().isoformat(timespec="seconds"),
        "data_dir": str(data_dir),
        "seed": seed,
        "test_ratio": test_ratio,
        "only_ok": only_ok,
        "labels": sorted(dataset.keys()),
        "per_class": per_class,
        "train": train,
        "test": test,
    }


# --------------------------------------------------------------------------------------
# 自检
# --------------------------------------------------------------------------------------


def selftest(args: argparse.Namespace) -> int:
    """造一个临时数据集，跑通"体检 → 打标 → 分层划分"全流程。"""
    safe_print("=== label_tool.py 离线自检（造临时数据集）===")
    failures: List[str] = []

    def check(name: str, condition: bool, detail: str = "") -> None:
        """记录一条自检结果。"""
        status = "PASS" if condition else "FAIL"
        if not condition:
            failures.append(name)
        safe_print(f"  [{status}] {name} {detail}")

    root = Path(args.data) / "_selftest"
    if root.exists():
        shutil.rmtree(root, ignore_errors=True)

    def write_sample(label: str, name: str, rows: int = WINDOW_SAMPLES, cols: int = WINDOW_AXES,
                     bad_text: bool = False, const: bool = False) -> Path:
        """写一个合成样本，可指定坏样本特征。"""
        directory = root / label
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / name
        with open(path, "w", encoding="utf-8", newline="") as handle:
            writer = csv.writer(handle, lineterminator="\n")
            writer.writerow(CSV_HEADER[:cols])
            for i in range(rows):
                if bad_text:
                    writer.writerow(["nan"] * cols)
                elif const:
                    writer.writerow([0] * cols)
                else:
                    writer.writerow([(i * 7 + axis * 13) % 200 - 100 for axis in range(cols)])
        return path

    # 造数据：unlabeled 6 个（其中 2 个坏）+ normal 10 个 + imbalance 6 个
    for i in range(4):
        write_sample("unlabeled", f"20260101-12000{i}_0000{i}.csv")
    write_sample("unlabeled", "bad_rows.csv", rows=37)
    write_sample("unlabeled", "bad_text.csv", bad_text=True)
    for i in range(10):
        write_sample("normal", f"n{i:02d}.csv")
    for i in range(6):
        write_sample("imbalance", f"i{i:02d}.csv")
    write_sample("static", "allzero.csv", const=True)

    dataset = scan_dataset(root)
    total, bad = print_dataset_report(root, dataset)
    check("扫描到 4 个类别", set(dataset) == {"unlabeled", "normal", "imbalance", "static"},
          f"{sorted(dataset)}")
    check("样本总数正确（23）", total == 23, f"{total}")
    check("坏样本数正确（3：行数/非数字/全零）", bad == 3, f"{bad}")

    # 隔离坏样本
    moved = quarantine(root, dataset)
    check("坏样本已隔离到 _quarantine", moved == 3 and (root / QUARANTINE_DIR).is_dir(),
          f"移动 {moved} 个")
    rescan = scan_dataset(root)
    check("隔离后 unlabeled 只剩 4 个", len(rescan.get("unlabeled", [])) == 4,
          f"{len(rescan.get('unlabeled', []))}")
    check("_quarantine 默认不计入统计（以 _ 开头）", QUARANTINE_DIR not in rescan)

    # 模拟打标：把 unlabeled 挪到 normal
    files = collect_unlabeled(root, ("unlabeled",))
    check("收集到待打标样本 4 个", len(files) == 4, f"{len(files)}")
    if args.yes:
        interactive_label(root, {"0": "normal"}, ("unlabeled",), assume_yes=True)
    else:
        for path in files:  # 非交互路径：直接模拟移动
            (root / "normal").mkdir(exist_ok=True)
            shutil.move(str(path), str(root / "normal" / path.name))
    final = scan_dataset(root)
    check("打标后 normal = 14", len(final.get("normal", [])) == 14, f"{len(final.get('normal', []))}")

    # 分层划分
    split = stratified_split(root, final, test_ratio=0.2, seed=DEFAULT_SEED)
    again = stratified_split(root, final, test_ratio=0.2, seed=DEFAULT_SEED)
    check("同种子划分可复现", split["train"] == again["train"] and split["test"] == again["test"])
    check("训练+测试 = 合格样本总数（20）",
          len(split["train"]) + len(split["test"]) == 20,
          f"{len(split['train'])}+{len(split['test'])}")
    check("每个类别在测试集里都有样本",
          all(split["per_class"][label]["test"] >= 1 for label in split["per_class"]
              if split["per_class"][label]["total"] >= 2),
          str({k: v["test"] for k, v in split["per_class"].items()}))
    check("训练/测试无重叠", not (set(split["train"]) & set(split["test"])))

    out_path = root / "split.json"
    with open(out_path, "w", encoding="utf-8") as handle:
        json.dump(split, handle, ensure_ascii=False, indent=2)
    check("split.json 已写出", out_path.is_file())
    with open(out_path, "r", encoding="utf-8") as handle:
        reloaded = json.load(handle)
    check("split.json 可回读且结构一致", reloaded["train"] == split["train"])

    if args.keep_selftest:
        safe_print(f"\n自检数据保留在：{root}")
    else:
        shutil.rmtree(root, ignore_errors=True)

    safe_print("")
    if failures:
        safe_print(f"自检失败 {len(failures)} 项：{failures}")
        return 1
    safe_print("自检全部通过：体检、坏样本隔离、打标移动、分层划分与 split.json 均正常。")
    return 0


# --------------------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------------------


def parse_label_keys(text: Optional[str]) -> Dict[str, str]:
    """解析 ``--label-keys "0=normal,1=imbalance"``；为空时用默认键位表。"""
    if not text:
        return dict(DEFAULT_LABEL_KEYS)
    mapping: Dict[str, str] = {}
    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        if "=" not in item:
            raise argparse.ArgumentTypeError(f"键位定义应为 key=类别，收到 {item!r}")
        key, value = item.split("=", 1)
        mapping[key.strip()] = value.strip()
    if not mapping:
        raise argparse.ArgumentTypeError("键位表为空")
    return mapping


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(
        prog="label_tool.py",
        description="数据集体检 + 键盘打标 + 分层划分（docs/03 阶段 6 第 2~3 步）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "示例：\n"
            "  python label_tool.py --data data\n"
            "  python label_tool.py --data data --quarantine          # 把坏样本移入 _quarantine\n"
            "  python label_tool.py --data data --label-keys 0=normal,1=imbalance,2=loosen\n"
            "  python label_tool.py --data data --split-only --test-ratio 0.2 --seed 20261007\n"
            "  python label_tool.py --selftest --data data\n"
        ),
    )
    parser.add_argument("--data", default="data", help="数据集根目录（默认 data）")
    parser.add_argument("--label-keys", default=None,
                        help="键位映射，如 0=normal,1=imbalance；默认 " +
                             ",".join(f"{k}={v}" for k, v in DEFAULT_LABEL_KEYS.items()))
    parser.add_argument("--unlabeled-dirs", default=",".join(UNLABELED_DIR_NAMES),
                        help="视为未打标的子目录名（逗号分隔），默认 " + ",".join(UNLABELED_DIR_NAMES))
    parser.add_argument("--test-ratio", type=float, default=DEFAULT_TEST_RATIO,
                        help=f"测试集比例（默认 {DEFAULT_TEST_RATIO} = 8:2）")
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED, help=f"随机种子（默认 {DEFAULT_SEED}）")
    parser.add_argument("--split-out", default=None,
                        help="split.json 输出路径（默认 <data>/split.json）")
    parser.add_argument("--quarantine", action="store_true",
                        help="把体检不合格的样本移入 _quarantine/")
    parser.add_argument("--quarantine-dry-run", action="store_true", help="只打印将要隔离的文件")
    parser.add_argument("--split-only", action="store_true", help="只做分层划分，不打标")
    parser.add_argument("--scan-only", action="store_true", help="只做体检，不打标不划分")
    parser.add_argument("--include-bad", action="store_true", help="划分时把坏样本也纳入")
    parser.add_argument("--yes", "-y", action="store_true", help="非交互模式（只列文件，不读键盘）")
    parser.add_argument("--selftest", action="store_true", help="造临时数据集跑通全流程后退出")
    parser.add_argument("--keep-selftest", action="store_true", help="自检后保留临时数据")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    """脚本入口。"""
    setup_console()
    args = parse_args(argv)
    if args.selftest:
        return selftest(args)

    data_dir = Path(args.data)
    if not data_dir.is_dir():
        safe_print(
            f"错误：数据集目录不存在 —— {data_dir}\n"
            "  请先用 capture_window.py 采集，例如：\n"
            "    python capture_window.py --port COM4 --label unlabeled --out data"
        )
        return 2

    try:
        label_keys = parse_label_keys(args.label_keys)
    except argparse.ArgumentTypeError as exc:
        safe_print(f"错误：{exc}")
        return 2

    dataset = scan_dataset(data_dir)
    print_dataset_report(data_dir, dataset)

    if args.quarantine or args.quarantine_dry_run:
        moved = quarantine(data_dir, dataset, dry_run=args.quarantine_dry_run)
        safe_print(f"\n已隔离（移动）{moved} 个坏样本到 {QUARANTINE_DIR}/。")
        if not args.quarantine_dry_run:
            dataset = scan_dataset(data_dir)

    if args.scan_only:
        return 0

    if not args.split_only:
        interactive_label(
            data_dir,
            label_keys,
            [name.strip() for name in args.unlabeled_dirs.split(",") if name.strip()],
            assume_yes=args.yes,
        )
        dataset = scan_dataset(data_dir)  # 打标后重新体检，保证 split 用的是最新数据
        print_dataset_report(data_dir, dataset)

    if not dataset:
        safe_print("没有可用样本，跳过划分。")
        return 1

    split = stratified_split(
        data_dir,
        dataset,
        test_ratio=args.test_ratio,
        seed=args.seed,
        only_ok=not args.include_bad,
    )
    split_path = Path(args.split_out) if args.split_out else data_dir / "split.json"
    split_path.parent.mkdir(parents=True, exist_ok=True)
    with open(split_path, "w", encoding="utf-8") as handle:
        json.dump(split, handle, ensure_ascii=False, indent=2)

    rows = [
        [label, str(info["total"]), str(info["train"]), str(info["test"]), str(info["excluded_bad"])]
        for label, info in sorted(split["per_class"].items())
    ]
    safe_print(f"\n分层划分结果（test_ratio={args.test_ratio}，seed={args.seed}）")
    safe_print(format_table(["类别", "合格样本", "训练", "测试", "排除(坏)"], rows,
                            ["<", ">", ">", ">", ">"]))
    safe_print(
        f"合计：训练 {len(split['train'])}，测试 {len(split['test'])}，"
        f"已写出 {split_path}"
    )
    safe_print(
        "\n下一步：python train_mlp.py --data "
        f"{data_dir} --split {split_path} --out model_out"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
