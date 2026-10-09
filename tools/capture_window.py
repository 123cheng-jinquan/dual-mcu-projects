#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从串口采集 F411 直接打出的六轴窗口 → 每窗口一个 CSV（docs/03 阶段 6 第 1~2 步）。

为什么走 USART1(CH340) 而不是 Modbus：``plan.md`` §6.2 明确"推荐先做，不依赖 S3"——
数据集采集不该卡在网关联调之后。F411 侧只要加一个"打点模式"把窗口按行 printf 出来即可。

支持的两种输入格式（自动识别，按行解析）
----------------------------------------
1. **``W,`` 前缀行**（推荐，F411 侧最容易实现）::

       W,<seq>,<ax0>,<ay0>,<az0>,<gx0>,<gy0>,<gz0>,<ax1>,...,<gz99>

   共 ``1 + 1 + 600`` 个逗号分隔字段：首字段 ``W``、第二字段窗口序号、随后 600 个
   int16（行优先：样本 0 的 6 轴在前，与 docs/02 §5 D 区一致）。窗口序号允许缺省
   （写成 ``W,...`` 时用自增计数代替）。

2. **裸数据行**：每行正好 600 个整数（可空格/制表符/逗号分隔），无序号。

输出
----
每个窗口保存为 ``<out>/<label>/<timestamp>_<seq>.csv``，6 列 100 行带表头::

    ax,ay,az,gx,gy,gz
    -282,4871,9717,608,1206,1221
    ...

用法::

    python capture_window.py --port COM4 --baud 115200 --label normal
    python capture_window.py --port COM4 --label imbalance --out data --flush-every 10
    python capture_window.py --selftest --label demo     # 不开串口，验证解析+落盘逻辑

Ctrl+C 会优雅退出并打印已保存文件数；不会留下写了一半的 CSV（先写临时文件再改名）。
"""

from __future__ import annotations

import argparse
import csv
import io
import os
import re
import signal
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

from _console import format_table, safe_print, setup_console

# --------------------------------------------------------------------------------------
# 常量（与 docs/02 §5 D 区对齐）
# --------------------------------------------------------------------------------------

WINDOW_SAMPLES = 100
WINDOW_AXES = 6
WINDOW_VALUES = WINDOW_SAMPLES * WINDOW_AXES  # 600 个 int16
CSV_HEADER = ["ax", "ay", "az", "gx", "gy", "gz"]
INT16_MIN, INT16_MAX = -32768, 32767

#: ``W,`` 行的前缀（大小写不敏感；也接受 F411 打印 ``W:`` / ``[W]`` 的写法）
PREFIX_RE = re.compile(r"^\s*[\[\(]?\s*[Ww]\s*[\]\)]?\s*[:,]")

#: 分隔符：逗号 / 空白 / 分号 / 制表符
SPLIT_RE = re.compile(r"[,;\s]+")

TMP_SUFFIX = ".part"


# --------------------------------------------------------------------------------------
# 解析
# --------------------------------------------------------------------------------------


class ParseError(ValueError):
    """窗口行解析失败（内容不符合两种受支持格式）。"""


def parse_window_line(line: str, last_seq: int = 0) -> Tuple[int, List[List[int]]]:
    """解析一行文本为一个窗口。

    :param line: 串口读到的一行（可含 ``\\r\\n`` 与 ``W,`` 前缀）。
    :param last_seq: 上一个窗口序号；当本行没有序号时用它 +1 作为序号。
    :return: ``(窗口序号, 100×6 的 int16 窗口)``。
    :raises ParseError: 字段数不足/非数字/数值超 int16。
    """
    text = line.strip()
    if not text:
        raise ParseError("空行")

    has_prefix = bool(PREFIX_RE.match(text))
    if has_prefix:
        text = PREFIX_RE.sub("", text, count=1)

    tokens = [token for token in SPLIT_RE.split(text.strip()) if token]
    if not tokens:
        raise ParseError("无有效字段")

    seq: Optional[int] = None
    if has_prefix:
        # 带前缀时第二个字段是窗口序号；若 F411 没打序号，则首个字段就是数据
        if len(tokens) == WINDOW_VALUES + 1:
            try:
                seq = int(tokens[0], 0) & 0xFFFF
                tokens = tokens[1:]
            except ValueError:
                seq = None  # 首字段不是数字 → 当成数据，用自增序号
        elif len(tokens) != WINDOW_VALUES:
            raise ParseError(
                f"W 行字段数应为 {WINDOW_VALUES}（无序号）或 {WINDOW_VALUES + 1}（有序号），"
                f"实际 {len(tokens)}"
            )
    elif len(tokens) != WINDOW_VALUES:
        raise ParseError(f"裸数据行应正好 {WINDOW_VALUES} 个整数，实际 {len(tokens)}")

    if len(tokens) != WINDOW_VALUES:
        raise ParseError(f"窗口数据应为 {WINDOW_VALUES} 个整数，实际 {len(tokens)}")

    values: List[int] = []
    for index, token in enumerate(tokens):
        try:
            value = int(token, 0)
        except ValueError as exc:
            raise ParseError(f"第 {index} 个字段不是整数：{token!r}") from exc
        if not INT16_MIN <= value <= INT16_MAX:
            raise ParseError(f"第 {index} 个字段 {value} 超出 int16 范围")
        values.append(value)

    window = [values[i * WINDOW_AXES : (i + 1) * WINDOW_AXES] for i in range(WINDOW_SAMPLES)]
    return (seq if seq is not None else (last_seq + 1) & 0xFFFF), window


def format_window_line(seq: int, window: Sequence[Sequence[int]]) -> str:
    """把一个窗口反向格式化成 ``W,<seq>,...`` 行（自检与回放用）。"""
    flat = [str(value) for row in window for value in row]
    return "W," + str(seq) + "," + ",".join(flat)


# --------------------------------------------------------------------------------------
# 落盘
# --------------------------------------------------------------------------------------


class WindowWriter:
    """把窗口写成一个窗口一个 CSV（先写 ``.part`` 再改名的原子写）。"""

    def __init__(self, out_dir: Path, label: str, flush_every: int = 1, dry_run: bool = False,
                 timestamp_format: str = "%Y%m%d-%H%M%S") -> None:
        """:param out_dir: 数据集根目录。:param label: 类别（子目录名）。
        :param flush_every: 每写多少个窗口做一次 ``os.fsync``（默认 1）。
        :param dry_run: 只解析不落盘。:param timestamp_format: 文件名时间戳格式。
        """
        self.label_dir = Path(out_dir) / label
        self.flush_every = max(1, flush_every)
        self.dry_run = dry_run
        self.timestamp_format = timestamp_format
        self.saved = 0
        self.since_sync = 0
        self.skipped = 0
        self.errors = 0
        self.last_path: Optional[Path] = None
        if not dry_run:
            self.label_dir.mkdir(parents=True, exist_ok=True)

    def save(self, seq: int, window: Sequence[Sequence[int]], when: Optional[datetime] = None) -> Path:
        """保存一个窗口，返回文件路径。

        :param seq: 窗口序号（进文件名，便于与 0x0101 对账）。
        :param window: 100×6 的 int16 窗口。
        :param when: 时间戳；默认当前时间。
        :return: 生成的 CSV 路径（dry_run 时返回拟写入路径，不落盘）。
        """
        stamp = (when or datetime.now()).strftime(self.timestamp_format)
        base = f"{stamp}_{seq:05d}"
        path = self.label_dir / f"{base}.csv"
        suffix = 1
        while path.exists():  # 同一秒内多窗口时避免覆盖
            path = self.label_dir / f"{base}_{suffix}.csv"
            suffix += 1
        self.last_path = path
        if self.dry_run:
            self.saved += 1
            return path

        tmp_path = path.with_suffix(path.suffix + TMP_SUFFIX)
        buffer = io.StringIO()
        writer = csv.writer(buffer, lineterminator="\n")
        writer.writerow(CSV_HEADER)
        writer.writerows(window)
        text = buffer.getvalue()

        with open(tmp_path, "w", encoding="utf-8", newline="") as handle:
            handle.write(text)
            self.since_sync += 1
            if self.since_sync >= self.flush_every:
                handle.flush()
                os.fsync(handle.fileno())
                self.since_sync = 0
        tmp_path.replace(path)  # 原子改名：不会留下半截 CSV
        self.saved += 1
        return path


# --------------------------------------------------------------------------------------
# 串口采集
# --------------------------------------------------------------------------------------


def import_serial():
    """导入 pyserial；缺失时给出中文提示。

    :return: ``serial`` 模块。
    """
    try:
        import serial  # noqa: PLC0415

        return serial
    except ImportError as exc:
        safe_print("错误：未安装 pyserial。请执行：pip install -r tools/requirements.txt")
        raise SystemExit(2) from exc


def open_serial(port: str, baud: int, timeout: float):
    """打开采集串口，失败时给出中文排查清单而不是抛栈。"""
    serial = import_serial()
    try:
        return serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=timeout,
        )
    except Exception as exc:  # noqa: BLE001 - pyserial 抛的异常类型很多
        safe_print(
            f"\n错误：串口打不开 —— {port}（{exc}）\n"
            "  请依次检查：\n"
            "    1) USB 转串口/CH340 驱动是否装好（设备管理器有没有黄色感叹号）\n"
            "    2) 端口号是否正确：F411 的板载 CH340 是 **USART1(PA9/PA10) 调试口**，"
            "不是 485 口，别选错\n"
            "    3) 端口是否被 **串口助手 / Modbus Poll / 另一个 capture_window.py** 占用\n"
            "    4) 波特率是否与 F411 打点模式一致（建议 115200，打点数据量大）\n"
            "    5) 供电与共地：USB 线松、板子没复位都会表现为完全收不到数据\n"
            "  只想验证解析与落盘逻辑可加 --selftest（不占串口）。\n"
        )
        raise SystemExit(2) from exc


class StopFlag:
    """Ctrl+C / SIGBREAK 优雅退出的标志位。"""

    def __init__(self) -> None:
        self.stopped = False

    def install(self) -> None:
        """注册信号处理（Windows 也支持 SIGBREAK，Ctrl+Break 可强制优雅退出）。"""
        def handler(signum, _frame):  # noqa: ANN001
            """收到中断信号：置位后由主循环收尾。"""
            self.stopped = True
            safe_print(f"\n收到信号 {signum}，将在当前行处理完后退出…")

        for name in ("SIGINT", "SIGTERM", "SIGBREAK"):
            sig = getattr(signal, name, None)
            if sig is not None:
                try:
                    signal.signal(sig, handler)
                except (ValueError, OSError):
                    pass


def capture(args: argparse.Namespace) -> int:
    """主采集循环：读串口 → 解析 → 落盘，返回退出码。"""
    writer = WindowWriter(
        out_dir=Path(args.out),
        label=args.label,
        flush_every=args.flush_every,
        dry_run=args.dry_run,
        timestamp_format=args.timestamp_format,
    )
    stop = StopFlag()
    stop.install()

    port = open_serial(args.port, args.baud, args.timeout)
    safe_print(
        f"采集已开始：{args.port} @ {args.baud} 8N1 → "
        f"{'（--dry-run 不落盘）' if args.dry_run else writer.label_dir}"
    )
    safe_print(
        f"目标：{args.target} 个窗口；按 Ctrl+C 结束。"
        f"识别格式：'W,<seq>,600 个整数' 或每行 600 个整数。"
    )

    last_seq = 0
    parse_errors = 0
    error_samples: List[str] = []
    started = time.time()
    blank_streak = 0

    try:
        while not stop.stopped:
            if args.target and writer.saved >= args.target:
                safe_print(f"已达到目标窗口数 {args.target}，正常结束。")
                break
            if args.seconds and time.time() - started >= args.seconds:
                safe_print(f"已达到采集时长 {args.seconds}s，正常结束。")
                break

            raw = port.readline()
            if not raw:
                blank_streak += 1
                if blank_streak == 50:  # 约 5s 没数据
                    safe_print(
                        "提示：已连续 5 秒没收到任何数据。检查 F411 是否处于打点模式、"
                        "波特率是否一致、TX/RX 是否接反。"
                    )
                continue
            blank_streak = 0
            line = raw.decode("utf-8", errors="ignore")
            if args.verbose:
                safe_print(f"RX> {line.strip()[:120]}")

            try:
                seq, window = parse_window_line(line, last_seq)
            except ParseError as exc:
                parse_errors += 1
                if len(error_samples) < 5:
                    error_samples.append(f"{line.strip()[:80]!r} → {exc}")
                continue

            last_seq = seq
            path = writer.save(seq, window)
            if args.verbose or writer.saved % max(1, args.log_every) == 0:
                safe_print(f"[{writer.saved:5d}] 保存 {path.name}（序号 {seq}）")
    except KeyboardInterrupt:
        safe_print("\n收到 Ctrl+C。")
    finally:
        try:
            port.close()
        except Exception:  # noqa: BLE001
            pass

    elapsed = time.time() - started
    safe_print("\n" + "=" * 72)
    safe_print("采集结果")
    safe_print("=" * 72)
    safe_print(
        format_table(
            ["项目", "值"],
            [
                ["已保存窗口", str(writer.saved)],
                ["输出目录", "（dry-run）" if args.dry_run else str(writer.label_dir)],
                ["解析失败行数", str(parse_errors)],
                ["耗时", f"{elapsed:.1f}s"],
                ["平均速率", f"{writer.saved / elapsed:.2f} 窗口/s" if elapsed > 0 else "-"],
                ["最后一个文件", str(writer.last_path.name) if writer.last_path else "-"],
            ],
        )
    )
    if error_samples:
        safe_print("\n前几条解析失败样例（多半是 printf 的启动横幅或调试行，可忽略）：")
        for sample in error_samples:
            safe_print(f"  {sample}")
    if writer.saved == 0:
        safe_print("\n没有保存任何窗口。若确认 F411 在打点：检查波特率、格式（--verbose 看原始行）。")
        return 1
    safe_print(f"\n下一步：python label_tool.py --data {args.out}   # 体检 + 打标 + 分层划分")
    return 0


def selftest(args: argparse.Namespace) -> int:
    """离线自检：造 3 个窗口 → 走完整解析 + 落盘 + 回读校验，不打开串口。"""
    safe_print("=== capture_window.py 离线自检（不打开串口）===")
    failures: List[str] = []

    def check(name: str, condition: bool, detail: str = "") -> None:
        """记录一条自检结果。"""
        status = "PASS" if condition else "FAIL"
        if not condition:
            failures.append(name)
        safe_print(f"  [{status}] {name} {detail}")

    out_root = Path(args.out) / "_selftest"
    writer = WindowWriter(out_root, args.label, flush_every=1)

    def make_window(seed: int) -> List[List[int]]:
        """生成一个可预测的假窗口（样本 i 的 6 轴 = seed+i*6 .. seed+i*6+5）。"""
        return [[(seed + i * WINDOW_AXES + axis) % 3000 - 1500 for axis in range(WINDOW_AXES)]
                for i in range(WINDOW_SAMPLES)]

    # 1) W, 格式（带序号）
    window0 = make_window(0)
    line_prefixed = format_window_line(1234, window0)
    seq, parsed = parse_window_line(line_prefixed, 0)
    check("解析 'W,<seq>,600 整数' 行", seq == 1234 and parsed == window0, f"序号={seq}")

    # 2) 裸 600 整数行
    line_bare = ",".join(str(value) for row in window0 for value in row)
    seq2, parsed2 = parse_window_line(line_bare, 1234)
    check("解析裸 600 整数行（序号自增）", seq2 == 1235 and parsed2 == window0, f"序号={seq2}")

    # 3) 空格分隔 + 末尾换行 + W: 变体
    window1 = make_window(1000)
    line_space = "W: " + " ".join(str(value) for row in window1 for value in row)
    seq3, parsed3 = parse_window_line(line_space + "\r\n", 0)
    check("解析 'W: ' + 空格分隔 + CRLF", parsed3 == window1, f"序号={seq3}")

    # 4) 异常输入必须被拒绝
    for name, bad in (
        ("字段数不足", "W,1,2,3"),
        ("非数字", "W,1," + ",".join(["x"] * WINDOW_VALUES)),
        ("int16 溢出", "W,1," + ",".join(["40000"] + ["0"] * (WINDOW_VALUES - 1))),
        ("空行", "   "),
    ):
        try:
            parse_window_line(bad, 0)
        except ParseError:
            check(f"拒绝非法行：{name}", True)
        else:
            check(f"拒绝非法行：{name}", False, "竟然解析成功了")

    # 5) 落盘 + 回读
    before = datetime(2026, 1, 1, 12, 0, 0)
    path = writer.save(seq, parsed, when=before)
    check("CSV 文件名形态 <时间戳>_<seq>.csv", path.name.startswith("20260101-120000_"),
          f"{path.name}")
    check("CSV 已落盘", path.exists())
    check("未残留 .part 临时文件", not list(writer.label_dir.glob("*" + TMP_SUFFIX)))
    with open(path, "r", encoding="utf-8") as handle:
        rows = [row for row in csv.reader(handle)]
    check("CSV 表头正确", rows[0] == CSV_HEADER, f"{rows[0]}")
    check("CSV 行数 = 100 数据行 + 1 表头", len(rows) == WINDOW_SAMPLES + 1, f"{len(rows)} 行")
    check("CSV 每行 6 列", all(len(row) == WINDOW_AXES for row in rows[1:]))
    check("CSV 内容与源窗口一致",
          [int(value) for row in rows[1:] for value in row]
          == [value for row in window0 for value in row])

    # 6) 同秒不覆盖
    path2 = writer.save(seq, parsed, when=before)
    check("同一秒内第二个窗口不覆盖第一个", path2 != path and path.exists(), f"{path2.name}")

    # 7) dry-run 不落盘
    dry_writer = WindowWriter(out_root, "dry", dry_run=True)
    dry_path = dry_writer.save(1, window0, when=before)
    check("--dry-run 不落盘", not dry_path.exists() and dry_writer.saved == 1)

    # 清理自检产物
    for item in sorted((out_root).rglob("*"), reverse=True):
        try:
            if item.is_file():
                item.unlink()
            elif item.is_dir():
                item.rmdir()
        except OSError:
            pass

    safe_print("")
    if failures:
        safe_print(f"自检失败 {len(failures)} 项：{failures}")
        return 1
    safe_print("自检全部通过：两种输入格式解析、CSV 落盘与命名、异常行拒绝均正常。")
    return 0


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(
        prog="capture_window.py",
        description=(
            "从串口读取 F411 直接打出的六轴窗口并保存为每窗口一个 CSV"
            "（docs/03 阶段 6 数据集采集，不依赖 ESP32-S3）"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "示例：\n"
            "  python capture_window.py --port COM4 --baud 115200 --label normal\n"
            "  python capture_window.py --port COM4 --label imbalance --out data --flush-every 10\n"
            "  python capture_window.py --port COM4 --label loosen --target 80 -v\n"
            "  python capture_window.py --selftest --label demo --out data\n"
        ),
    )
    parser.add_argument("--port", default="COM4", help="F411 的 USART1(CH340) 串口（默认 COM4）")
    parser.add_argument("--baud", type=int, default=115200, help="波特率（默认 115200；打点数据量大）")
    parser.add_argument("--label", default="unlabeled", help="类别标签，作为 data/ 下的子目录名")
    parser.add_argument("--out", default="data", help="数据集根目录（默认 data）")
    parser.add_argument("--flush-every", type=int, default=1, metavar="N",
                        help="每 N 个窗口做一次 fsync（默认 1 = 每个窗口都落稳）")
    parser.add_argument("--timeout", type=float, default=1.0, help="串口读超时秒（默认 1.0）")
    parser.add_argument("--target", type=int, default=0, help="采够 N 个窗口后自动停止（0 = 不限）")
    parser.add_argument("--seconds", type=float, default=0.0, help="采集时长上限秒（0 = 不限）")
    parser.add_argument("--log-every", type=int, default=10, help="每 N 个窗口打印一行进度（默认 10）")
    parser.add_argument("--timestamp-format", default="%Y%m%d-%H%M%S",
                        help="文件名时间戳格式（默认 %%Y%%m%%d-%%H%%M%%S）")
    parser.add_argument("--dry-run", action="store_true", help="只解析不落盘（先验证格式）")
    parser.add_argument("--selftest", action="store_true", help="离线自检解析与落盘逻辑后退出")
    parser.add_argument("-v", "--verbose", action="store_true", help="打印每一行原始数据与保存动作")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    """脚本入口。"""
    setup_console()
    args = parse_args(argv)
    if args.selftest:
        return selftest(args)
    return capture(args)


if __name__ == "__main__":
    sys.exit(main())
