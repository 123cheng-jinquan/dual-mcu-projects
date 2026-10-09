"""采集前「静音/振动」预检 —— 读几个窗口，直接告诉你现在是什么状态。

为什么需要它：打点模式下 [dsp] 日志被静音了（为了保持串口流干净 ✓），
于是采集时**你看不到 std**，很容易在"传感器其实在剧烈振动"的状态下采了一批
`normal` 数据 ✗（实测踩过：采完发现两类特征方向反了，整批数据报废 ✗）。

用法（每轮采集前跑一次，约 12 秒）：
    python tools/precheck_quiet.py            # 默认 COM4
    python tools/precheck_quiet.py COM7       # 指定串口

判读：
    az std ≈ 20~60   → 静止 ✓ 可以采 normal
    az std ≈ 100~500 → 有振动 ✓ 可以采 tap / unbalance
    az std > 800     → 剧烈振动（手持/碰线/马达带着动）✗ 先排除振源再采

注意：跑的期间**串口监视器必须关掉**（串口独占 ✓）。
"""

import math
import sys
import time

try:
    import serial
except ImportError:
    print("缺少 pyserial：D:\\MSYS2\\ucrt64\\bin\\python.exe -m pip install pyserial")
    raise SystemExit(2)

WINDOW_VALUES = 600
AXIS_NAMES = ("ax", "ay", "az", "gx", "gy", "gz")
AZ_INDEX = 2          # 每个样本 6 个轴，az 是第 3 个（下标 2）


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM4"
    try:
        ser = serial.Serial(port, 115200, timeout=2)
    except Exception as exc:                                  # noqa: BLE001
        print(f"打不开 {port}：{exc}")
        print("（是不是串口监视器还开着？或者板子没在跑打点固件？）")
        return 1

    print(f"正在读 {port}，约 12 秒（6 个窗口）…  请保持当前状态不要动")
    az_stds: list[float] = []
    buf = b""
    t0 = time.time()
    while time.time() - t0 < 20 and len(az_stds) < 6:
        chunk = ser.read(8192)
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            if not line.startswith(b"W,"):
                continue
            toks = line.decode("utf-8", "ignore").split(",")
            if len(toks) < WINDOW_VALUES + 2:
                continue                                      # 半行，丢弃
            try:
                nums = [int(x) for x in toks[2:WINDOW_VALUES + 2]]
            except ValueError:
                continue
            az = nums[AZ_INDEX::6]
            mean = sum(az) / len(az)
            az_stds.append(math.sqrt(sum((v - mean) ** 2 for v in az) / len(az)))
    ser.close()

    if not az_stds:
        print("一个完整窗口都没读到 ✗")
        print("  检查：① 板子在跑打点固件吗 ② 串口对吗 ③ 监视器关了吗")
        return 1

    avg = sum(az_stds) / len(az_stds)
    print()
    print("az std 逐窗口：" + ", ".join(f"{v:.0f}" for v in az_stds))
    print(f"平均 az std = {avg:.0f}")
    print()
    if avg < 1.0:
        # ⚠️ 全 0 不是"静止"，而是**传感器没数据** ✗！
        # 固件在传感器离线时会把窗口填 0（这是刻意设计 ✓），
        # 所以 std 恒为 0 恰好说明"I2C 完全读不到 MPU6050" ✗。
        print("❌ 全部为 0 —— 传感器**没有数据（离线）**，这不是'静止' ✗")
        print("   固件串口里应该能看到：'[sensor] mpu6050 未就绪（未接线？），5 秒后重试' ✗")
        print("   逐项检查：")
        print("     ① GY-521 模块上的**电源灯**亮不亮（不亮 = VCC/GND 掉了）")
        print("     ② VCC→3V3、GND→GND **别接反**（接反可能烧模块 ✗）")
        print("     ③ SCL→PB6、SDA→PB7 **别调换**")
        print("     ④ 换一组杜邦线（内部断芯极常见 ✗）")
        return 1
    if avg < 60:
        print("✅ 静止 —— 可以采 normal ✓")
    elif avg < 500:
        print("🔶 有振动 —— 适合采 tap / unbalance ✓（别用它采 normal ✗）")
    else:
        print("❌ 剧烈振动 —— 传感器可能被手持/被线拉/被马达带着动 ✗")
        print("   采 normal 前先把振源关掉、线放松、传感器放稳 ✓")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
