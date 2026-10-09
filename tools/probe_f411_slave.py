"""PC 侧单点探测：用「USB 转 RS485」适配器直接对 F411 的 485 口发一帧 FC03。

⚠️ 先说清楚它**不能**做什么（这一点踩过坑，写下来省得再踩）：
    把 F411 板载的那个 CH340 串口（设备管理器里的 "USB-SERIAL CH340"）当成 Modbus 口
    去发帧，是**没有用的** —— 那一路是 PA9/PA10 = USART1 = 调试 printf 口，
    而从机跑在 PA2/PA3 = USART2 上。往调试口发 Modbus 帧只会得到满屏 [hb]/[imu] 日志。
    这个脚本要的是一个**真的挂在 RS485 A/B 上的 USB-RS485 适配器**。

它解决什么问题：
    当「ESP32 主站 ↔ F411 从机」那条链路只有 ~26% 成功率时，责任可能在
    主站、总线、从机三者之一。把 ESP32 断电、用本脚本（PC 当主站）单独打 F411，
    就能把 ESP32 那一侧整个排除掉：

        PC 当主站 → F411 成功率高   ⇒ 问题在 ESP32 侧或那段线
        PC 当主站 → F411 同样低     ⇒ 问题在 F411 侧或总线本身（终端/共地/双绞）

用法：
    python tools/probe_f411_slave.py COM10 --baud 115200            # USB-RS485 适配器的口
    python tools/probe_f411_slave.py COM10 --start 0x0400 --count 36
    python tools/probe_f411_slave.py COM10 --extra                   # 顺带读 B 区统计

前置条件：
    ① 适配器的 A/B/GND 接到 F411 那对 485 线上，**GND 必须互联**；
    ② **把 ESP32 断电或拔掉它的 485 线** —— 两个主站同时打总线会互相踩，
       现象是"两个都只有一半成功率"，会把你带偏；
    ③ F411 跑的是从机固件（不是打点模式）：`DBG_WINDOW_DUMP` 置 0。
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("缺少 pyserial：pip install pyserial")
    raise SystemExit(2)


def crc16_modbus(buf: bytes) -> int:
    crc = 0xFFFF
    for b in buf:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc


def build_read(addr: int, start: int, count: int) -> bytes:
    body = bytes([addr, 0x03, start >> 8, start & 0xFF, count >> 8, count & 0xFF])
    crc = crc16_modbus(body)
    return body + bytes([crc & 0xFF, crc >> 8])      # 线上序：先低后高


def hexs(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)


def find_frame(buf: bytes, addr: int, func: int = 0x03):
    """在缓冲里找一条合法 Modbus 响应（跳过 [hb]/[imu] 之类的日志字节）。

    为什么要"找"而不是"对齐读"：F411 的调试口和 485 口虽然物理上是两路串口，
    但如果你错误地用了调试口（见文件头），收到的就是日志文本；
    而且真实链路上也可能混入半截帧。所以按 (地址, 功能码, 字节数, CRC) 校验着扫。
    """
    for off in range(0, max(0, len(buf) - 4)):
        cand = buf[off:]
        if cand[0] != addr or cand[1] != func:
            continue
        if len(cand) < 3:
            continue
        exp = 5 + cand[2]
        if len(cand) < exp:
            continue
        if crc16_modbus(cand[:exp]) == 0:
            return cand[:exp]
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("port", help="USB-RS485 适配器的串口，例如 COM10")
    ap.add_argument("--baud", type=int, default=115200,
                    help="必须与 F411 的 0x002D 一致（默认 115200）")
    ap.add_argument("--addr", type=lambda s: int(s, 0), default=1)
    ap.add_argument("--start", type=lambda s: int(s, 0), default=0)
    ap.add_argument("--count", type=int, default=28)
    ap.add_argument("--timeout", type=float, default=0.3)
    ap.add_argument("--tries", type=int, default=10)
    ap.add_argument("--extra", action="store_true", help="顺带读 B 区统计/状态字")
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    except Exception as exc:                                     # noqa: BLE001
        print(f"打不开 {args.port}：{exc}")
        return 1

    # 不要用 DTR/RTS 去复位（很多 CH340/CP210x 板把这两根线接成复位/BOOT），
    # 我们只是借用串口收发，板子该跑什么还跑什么。
    ser.dtr = False
    ser.rts = False
    time.sleep(0.2)
    ser.reset_input_buffer()

    idle = ser.read(256)
    print(f"[{args.port}@{args.baud}] 空闲读 {len(idle)} 字节"
          + ("（线上有别的数据在刷？ESP32 是不是还通电在轮询？）" if idle else "（安静 ✓）"))

    req = build_read(args.addr, args.start, args.count)
    print(f"请求: {hexs(req)}   (FC03 addr={args.addr} start=0x{args.start:04X} count={args.count})")

    hits = 0
    for i in range(1, args.tries + 1):
        ser.reset_input_buffer()
        ser.write(req)
        ser.flush()
        time.sleep(args.timeout)
        resp = ser.read(2048)
        if not resp:
            print(f"  #{i:2d}  无响应（静默）")
            continue
        frame = find_frame(resp, args.addr)
        if frame is None:
            preview = hexs(resp[:24])
            print(f"  #{i:2d}  收到 {len(resp)} 字节但无合法帧：{preview}"
                  + (" ..." if len(resp) > 24 else ""))
            continue
        hits += 1
        vals = [(frame[3 + k * 2] << 8) | frame[4 + k * 2] for k in range(frame[2] // 2)]
        print(f"  #{i:2d}  ✅ {len(frame)} 字节，前 8 个寄存器: {vals[:8]}")

    rate = 100.0 * hits / args.tries
    print(f"\n小结：{args.tries} 次里拿到合法帧 {hits} 次（{rate:.0f}%）")
    if hits == 0:
        print("  → 完全不通：查 A/B、共地、波特率、F411 是否在跑从机固件")
    elif rate < 90:
        print("  → 与 ESP32 主站的成功率对比：若同样低 ⇒ 总线/F411 侧；若明显更高 ⇒ ESP32 那段")

    if args.extra:
        for name, start, cnt in [("B 区状态字 0x0010", 0x0010, 2),
                                 ("帧计数 0x0013", 0x0013, 3),
                                 ("E 区特征 0x0400", 0x0400, 2)]:
            ser.reset_input_buffer()
            ser.write(build_read(args.addr, start, cnt))
            ser.flush()
            time.sleep(args.timeout)
            resp = ser.read(512)
            frame = find_frame(resp, args.addr)
            print(f"{name}: " + (hexs(frame) if frame else "无合法响应"))

    ser.close()
    return 0 if hits else 1


if __name__ == "__main__":
    raise SystemExit(main())
