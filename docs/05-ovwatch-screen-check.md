# 硬件资料核对笔记（2026-10）

> 来源：用户提供的 `Core.pdf`（FryPi 核心板原理图）、`Back.pdf`（OV-Watch 背面原理图）、
> `Firmware.zip`（OV-Watch 预编译固件）、`外壳.zip`（表壳 3D 模型）。
> 本文件只记录**从原理图里实际读出来的**结论，推测部分单独标注。

---

## 1. `Core.pdf` = FryPi-STM32F411RET6 核心板（不是 OV-Watch 的主板）

从原理图文本抽出来的关键网络名与器件：

| 项 | 值 | 依据 |
|---|---|---|
| 板名 | `FryPi-STM32F411RET6` | 原理图标题 |
| 主控 | `STM32F411RET6TR` | 原理图 |
| 调试串口 | `CH340N`（USART1 / PA9 / PA10） | 网络 `USART1_TX/RX`、`TXD/RXD` |
| SPI Flash | `W25Q128JVSIM`（16MB） | `SPI FLASH` / `F_CS` / `SPI2_*` |
| 稳压 | `XC6206P332MR`（3.3V LDO） | 原理图 |
| 按键/LED | `KEY1`(PA0)、`LED_T`(PC13 共用) | 网络名 |
| 晶振 | 25MHz（HSE）、32.768kHz（LSE） | 原理图 |

### ★ 屏接口（这一条最关键）★

| 项 | 值 |
|---|---|
| **屏型号** | **`P169H002-CTP`**（`CTP` = Capacitive Touch Panel，电容触摸） |
| **尺寸** | **1.69 英寸** |
| **FPC 连接器** | **`AFC01-S18FCA-00` → 18 pin** |

**屏幕相关网络名（原理图里明确列出）**：

| 功能 | 网络名 | 说明 |
|---|---|---|
| LCD 时钟 | `LCD_CLK` | ← `SPI1_SCK` |
| LCD 数据 | `LCD_SDA` | ← `SPI1_MOSI` |
| LCD 数据/命令 | `LCD_DC` | 四线 SPI 的 D/C |
| LCD 片选 | `LCD_CS` | |
| LCD 复位 | `LCD_RST` | |
| LCD 背光 | `LCD_BL` | |
| 触摸数据 | `TP_SDA` | ← `I2C1_SDA` |
| 触摸时钟 | `TP_SCL` | ← `I2C1_SCL` |
| 触摸中断 | `TP_INT` | |
| 触摸复位 | `TP_RST` | |

> 结论：这是**标准四线 SPI 屏 + I2C 电容触摸**，和 `docs/02`/`docs/04` 选型要求
> （SPI 四线 / 3.3V / 带驱动 IC）完全一致 —— **但它是接在 F411 上的，不是 ESP32-S3**。

---

## 2. `Back.pdf` = OV-Watch **背面** PCB（传感器 + 蓝牙，**没有屏**）

抽出来的型号/网络名：

- 传感器：`MPU6050`(六轴)、`AHT21`(温湿度)、`SPL06-001`(气压)、`LSM303DLHC`(地磁)、`EM7028`(心率 PPG)
- 蓝牙：`KT6368A`（`BLE_EN` / `BLE_RX` / `BLE_TX` / `BT_RF`）
- 其它：`BL1551B`（模拟开关）、`KT_LED`、`LED_EN`、`VBAT`、`3V3`、`IIC BUS`、`Connect`
- 参考图框：`Kingham` / `OV-Watch_V2.4` / `2024-05-19` / `2023-07-01` / `Back`

> **这一页里没有任何 LCD 或触摸相关的网络名/器件** —— 屏不在背面 PCB 上。

---

## 3. `Firmware.zip` = OV-Watch 的**预编译固件**（不含源码）

```
Firmware/BootLoader_F411.hex
Firmware/OV_Watch_V2_4_5.bin
Firmware/OV_Watch_V2_4_5_NoBoot.hex
Firmware/README.md
```

> 没有 BSP / 屏驱动源码。要看驱动实现得去
> [gitee.com/kingham/OV-Watch](https://gitee.com/kingham/OV-Watch) 的
> `Software/OV_Watch/BSP/`（目录里能看到 AHT21 / BL24C02 / EM7028 等子目录）。

---

## 4. 屏能不能复用？—— 结论与理由

**用户手上那块屏（照片：圆角方形、20-pin FPC、型号丝印 `P193001 V6`、排线上带触摸 IC）
与 FryPi 核心板的屏幕接口不匹配：**

| 对比项 | FryPi 核心板设计 | 用户手上的屏 |
|---|---|---|
| 屏型号 | `P169H002-CTP` | `P193001`（丝印） |
| 尺寸 | 1.69" | 约 1.28~1.3" |
| 分辨率 | 240×280（1.69" 该型号的常见规格） | 240×240（照片为方形） |
| 连接器 | **18-pin** `AFC01-S18FCA-00` | **20-pin** FPC |
| 驱动 IC | 未知（1.69" 常见 ST7789V2） | **未知**（排线丝印只有 `P193001 V6`） |

**所以：**

1. **物理上就插不上** —— 18-pin 座 vs 20-pin 排线，且排线宽度/间距通常也不同。
2. **FryPi 核心板上没有任何 ESP32-S3 的屏接口** —— 屏是接 F411 的（`SPI1` + `I2C1`）。
   要把这块屏接到 ESP32-S3，只能从 FPC 座子旁边的引脚**飞线**，而且必须先搞清 20 个脚的定义。
3. **驱动 IC 型号未确认** —— 这决定 `User_Setup.h` 选哪个驱动，是点亮的前提。
4. 反过来说，**这块屏本来是为 OV-Watch 那块 F411 `Back` 板 + 表壳设计的**，
   它的正确归宿是装回手表（配合 `Firmware.zip` 的固件）。

## 5. 待确认（需要用户操作）

- [ ] 用户手上的屏：**驱动 IC 型号**（看排线背面丝印 / 屏的柔性板上印刷）
- [ ] 用户手上的屏：**触摸芯片型号**（排线上那颗黑色小 IC 的丝印，如 `CST816S`）
- [ ] 那块的"CAM扩展板"上 **X25（20-pin FPC）** 的走线去向：接 F411 排针，还是引到底部 2×8 排针
      （用万用表通断档量最快）
