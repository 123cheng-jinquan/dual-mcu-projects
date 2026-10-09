"""从原理图 PDF 里抽取所有可见文本（Tj 字符串），用于找屏幕/触摸/连接器的型号与网络名。

为什么不直接看 PDF：这些原理图是 CAD 导出的，文本以 `(字符) Tj` 形式散在内容流里，
而且字体是子集化的。本脚本把全部 `(...) Tj` 字符串、以及 PDF 的注释/链接文本都抓出来，
再按"像元件位号/网络名/型号"的模式归类输出。

用法：
    python tools/pdf_text_dump.py 路径\\xxx.pdf
    python tools/pdf_text_dump.py 路径\\xxx.pdf --all      # 打印全部文本
"""
import argparse
import re
import sys
import zlib


def read_bytes(path):
    with open(path, "rb") as f:
        return f.read()


def strings_from_stream(data: bytes):
    """抓 (..) Tj 里的字符串，并处理 \\( \\) \\\\ 以及八进制转义。"""
    out = []
    text = data.decode("latin-1")
    for m in re.finditer(r"\(((?:[^()\\]|\\.)*)\)\s*Tj", text):
        raw = m.group(1)
        # 处理转义
        s = re.sub(r"\\([nrtbf()\\])", lambda mm: {
            "n": "\n", "r": "\r", "t": "\t", "b": "\b", "f": "\f",
            "(": "(", ")": ")", "\\": "\\",
        }[mm.group(1)], raw)
        s = re.sub(r"\\([0-7]{1,3})", lambda mm: chr(int(mm.group(1), 8)), s)
        out.append(s)
    return out


def decode_flate_streams(data: bytes):
    """把 FlateDecode 流解开后再抽一次文本（有些 PDF 把内容流压了）。"""
    out = []
    for m in re.finditer(rb"stream\r?\n", data):
        start = m.end()
        end = data.find(b"endstream", start)
        if end < 0:
            continue
        chunk = data[start:end]
        try:
            dec = zlib.decompress(chunk)
        except Exception:                                  # noqa: BLE001
            continue
        out.extend(strings_from_stream(dec))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("pdf")
    ap.add_argument("--all", action="store_true", help="打印全部文本（不筛选）")
    args = ap.parse_args()

    data = read_bytes(args.pdf)
    items = strings_from_stream(data)
    items += decode_flate_streams(data)

    # 去掉纯空白、重复
    seen = set()
    uniq = []
    for s in items:
        s = s.strip()
        if not s or s in seen:
            continue
        seen.add(s)
        uniq.append(s)

    print(f"== {args.pdf}: 共 {len(uniq)} 条不重复文本 ==")

    if args.all:
        for s in uniq:
            print(s)
        return 0

    # 筛选：像元件位号(U?/R?/C?/X?/J?/D?/Q?/Y?/SW?)、型号(含数字+字母)、网络名
    pat_ref = re.compile(r"^[A-Z]{1,3}\d{1,3}$")
    pat_model = re.compile(r"(?i)(gc9|st77|st73|ili9|nv30|ssd13|cst8|ft6|gt9|xpt2|"
                           r"ch34|max3|sp34|tps6|mpu6|em70|spl0|lsm3|kt63|bl24|aht2|"
                           r"w25q|stm32)")
    pat_net = re.compile(r"^[A-Z][A-Z0-9_+\-/]{1,20}$")

    refs, models, nets, others = [], [], [], []
    for s in uniq:
        if pat_model.search(s):
            models.append(s)
        elif pat_ref.match(s):
            refs.append(s)
        elif pat_net.match(s):
            nets.append(s)
        else:
            others.append(s)

    print("\n---- 疑似型号 ----")
    for s in models:
        print(" ", s)
    print("\n---- 元件位号 ----")
    print("  " + "  ".join(refs[:200]))
    print("\n---- 疑似网络名 ----")
    print("  " + "  ".join(nets[:200]))
    print(f"\n---- 其它文本（{len(others)} 条，前 120）----")
    for s in others[:120]:
        print(" ", s)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
