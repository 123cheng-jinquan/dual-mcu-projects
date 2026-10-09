"""从 PC 通过 USB-RS485 直接写 F411 从机的 C 区寄存器，验证**写方向链路**。

为什么单独写这个脚本（而不是走 ESP32 网关）：
  · 长跑测试正在占用 ESP32 的两个串口，不能改固件（那会打断长跑）
  · 走 ESP32 需要 MQTT broker 或新增串口命令，都依赖改固件/broker
  · 从 PC 直接发 Modbus 帧是**最独立**的验证：PC 是主站、F411 是从机，
    和 ESP32 那条链路完全解耦。它通了，就说明"从机的写处理"没问题；
    再结合 ESP32 的 [CMD] 日志，就能把"网关→从机"这段也确认下来。

⚠ 与长跑的共存：
  ESP32 慢车道 1s 一轮、快车道 5s 一轮，每轮之间总线是**空闲**的。
  本脚本在发送前先检测空闲（静默 > 60ms 才发），冲突概率很低；
  万一撞上，最多是给长跑多记一次"重试"，不影响结论。

用法：
    python tools/probe_slave_write.py                 # 只读 + 写测试（默认 COM8 是调试口，不对）
    python tools/probe_slave_write.py --port COM7     # 指定 USB-RS485 适配器的口

  ⚠ 注意别选错口：
     COM6 = ESP32 的 CH343（调试口，不是 485）
     COM8 = F411 的 CH340（调试口，不是 485）
     需要的是**你那个 USB 转 RS485 适配器**的串口号。

寄存器（见 docs/02 §5 C 区）：
    0x0020 控制字1   bit0=蜂鸣器 bit1=风扇 bit2=电机 bit3=RGB bit4=舵机 ...
    0x0021 风扇PWM 0~1000
    0x0023 舵机角度 0~180
    0x0024~0x0026 RGB R/G/B（0~255）
"""
import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("需要 pyserial：pip install pyserial")
    sys.exit(1)

REG_CTRL = 0x0020
REG_FAN_PWM = 0x0021
REG_SERVO = 0x0023
REG_RGB_R, REG_RGB_G, REG_RGB_B = 0x0024, 0x0025, 0x0026

CTRL_BUZZER = 1 << 0
CTRL_FAN_EN = 1 << 1
CTRL_RGB_EN = 1 << 3
CTRL_SERVO_EN = 1 << 4

SLAVE = 1


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def frame(pdu: bytes) -> bytes:
    body = bytes([SLAVE]) + pdu
    c = crc16(body)
    return body + bytes([c & 0xFF, c >> 8])


def fc03(start: int, qty: int) -> bytes:
    return frame(bytes([0x03, start >> 8, start & 0xFF, qty >> 8, qty & 0xFF]))


def fc06(reg: int, val: int) -> bytes:
    return frame(bytes([0x06, reg >> 8, reg & 0xFF, val >> 8, val & 0xFF]))


class Bus:
    def __init__(self, port, baud):
        self.sp = serial.Serial(port, baud, timeout=0.05)
        time.sleep(0.3)
        self.sp.reset_input_buffer()

    def quiet(self, ms=60) -> bool:
        """线路静默 >= ms 毫秒 才认为总线空闲"""
        t0 = time.time()
        last = self.sp.in_waiting
        while (time.time() - t0) * 1000 < ms:
            if self.sp.in_waiting != last:
                return False
            time.sleep(0.005)
        return True

    def xfer(self, req: bytes, expect_len: int, retries=6):
        """发包并收响应（带重试，避开与 ESP32 的撞车）"""
        for _ in range(retries):
            if not self.quiet():
                time.sleep(0.05)
                continue
            self.sp.reset_input_buffer()
            self.sp.write(req)
            self.sp.flush()
            time.sleep(0.08)
            resp = self.sp.read(expect_len + 16)
            if len(resp) >= expect_len and resp[0] == SLAVE:
                # 校验 CRC
                body, got = resp[:-2], resp[-2] | (resp[-1] << 8)
                if crc16(body) == got:
                    return resp
            time.sleep(0.12)
        return None

    def read_regs(self, start, qty):
        r = self.xfer(fc03(start, qty), 5 + qty * 2)
        if r is None or r[1] != 0x03:
            return None
        n = r[2]
        return [ (r[3 + i*2] << 8) | r[4 + i*2] for i in range(n // 2) ]

    def write_reg(self, reg, val):
        r = self.xfer(fc06(reg, val), 8)
        if r is None:
            return False
        return r[1] == 0x06 and ((r[2] << 8) | r[3]) == reg

    def close(self):
        self.sp.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM7", help="USB-RS485 适配器的串口（不是 COM6/COM8！）")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--keep", action="store_true", help="测试完不关输出（默认会全部关掉）")
    args = ap.parse_args()

    print(f"打开 {args.port} @ {args.baud} ...")
    try:
        b = Bus(args.port, args.baud)
    except Exception as e:
        print(f"✗ 打不开 {args.port}: {e}")
        print("  提示：COM6=ESP32调试口、COM8=F411调试口，都不是 485。")
        print("        需要你那个 USB 转 RS485 适配器的口。")
        return 1

    print("\n═══ 第 1 步：只读，确认链路通 ═══")
    v = b.read_regs(REG_CTRL, 8)
    if v is None:
        print("✗ 读不到 0x0020~0x0027。检查：A/B 接线、波特率(115200)、适配器方向")
        b.close()
        return 1
    print(f"✓ C 区 0x0020~0x0027 = {[hex(x) for x in v]}")
    ctrl0 = v[0]
    print(f"  控制字 = 0x{ctrl0:04X}")

    print("\n═══ 第 2 步：写方向测试 ═══")
    ok_all = True

    # ---- 测试 1：写一个"无害且可读回"的值：阈值寄存器 ----
    # 故意用阈值（0x0027~0x0029）而不是输出寄存器：写它不会有物理动作，
    # 但能验证 FC06 的完整往返（写进去 + 读回来一致）。
    test_reg = 0x0027          # 温度报警上限
    old = b.read_regs(test_reg, 1)
    print(f"  写前 0x0027 = {old}")
    if not b.write_reg(test_reg, 450):
        print("  ✗ FC06 写失败（无响应 / 异常码）")
        ok_all = False
    else:
        back = b.read_regs(test_reg, 1)
        if back and back[0] == 450:
            print(f"  ✓ 写入 450 并读回 {back[0]} —— FC06 往返成功")
        else:
            print(f"  ✗ 写入后读回 {back}（期望 450）")
            ok_all = False
        if old:
            b.write_reg(test_reg, old[0])          # 还原
            print(f"  已还原 0x0027 = {old[0]}")

    # ---- 测试 2：RGB（有物理现象，最直观）----
    print("\n  测试 RGB 输出（红→绿→蓝，各 1 秒）...")
    if not b.write_reg(REG_CTRL, ctrl0 | CTRL_RGB_EN):
        print("  ✗ 使能 RGB 失败")
        ok_all = False
    else:
        for name, (r, g, bl) in (("红", (255, 0, 0)), ("绿", (0, 255, 0)), ("蓝", (0, 0, 255))):
            b.write_reg(REG_RGB_R, r)
            b.write_reg(REG_RGB_G, g)
            b.write_reg(REG_RGB_B, bl)
            print(f"    → {name}")
            time.sleep(1.0)
        # 读回确认
        rgb = b.read_regs(REG_RGB_R, 3)
        print(f"  读回 RGB = {rgb}")

    # ---- 测试 3：蜂鸣器（有声音）----
    print("\n  测试蜂鸣器（响 1 秒）...")
    if b.write_reg(REG_CTRL, ctrl0 | CTRL_BUZZER):
        time.sleep(1.0)
        print("    → 已触发")
    else:
        print("  ✗ 蜂鸣器命令失败")
        ok_all = False

    # ---- 收尾：恢复原控制字 ----
    if not args.keep:
        b.write_reg(REG_CTRL, ctrl0)
        print(f"\n  已恢复控制字 0x{ctrl0:04X}")
    else:
        print("\n  （--keep：保留输出状态，未恢复）")

    b.close()
    print("\n═══ 结论 ═══")
    print("  ✓ 写方向链路可用" if ok_all else "  ✗ 写方向有问题，看上面哪一步失败")
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
