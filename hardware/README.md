# 硬件参考资料

> 这里放的是**两个项目用到的硬件原始资料**：芯片/模块的原理图、规格手册、接线说明。
>
> **不是代码**，但排查硬件问题时非常有用 —— 比如"这个引脚到底能不能复用""模块的 VIH 是多少"，
> 都得回来查这里。

---

## 目录结构

```
hardware/
├── core-board/              项目1 的主控板资料
│   └── FryPi_Core.pdf       FryPi STM32F411RET6 核心板原理图
│
├── rs485-module/            项目1 的 RS485 收发模块
│   ├── 野火小智TTL转RS485模块原理图.pdf
│   ├── 野火小智TTL转RS485模块尺寸.pdf
│   ├── 野火小智TTL转RS485模块规格手册_20240805.pdf
│   ├── TTL转RS485模块与STM32开发板引脚连接说明.xlsx
│   └── readme.txt
│
└── ov-watch-screen/         项目1 曾考虑过的屏幕（**已确认不兼容，留档**）
    ├── OVWatch_Back.pdf     屏幕背面引脚定义
    ├── OVWatch_Case.zip     3D 打印外壳模型
    ├── OVWatch_Firmware.zip 出厂固件
    └── fw/Firmware/         解包后的固件（.hex / .bin）
```

---

## 一、core-board —— 核心板原理图

**`FryPi_Core.pdf`**（652 KB）

项目1 的主控板 `FryPi-STM32F411RET6` 的完整原理图。

**什么时候要查它**：

| 疑问 | 查哪里 |
|---|---|
| 哪些引脚被板载外设占了？ | 搜 `W25Q128`（PB12~PB15 被 Flash 占用）、`CH340`（USART1 被占）|
| PA13/PA14 能不能当普通 IO？ | 看着 SWD 部分 —— **不能**，要保留调试口 |
| 板子上有没有 5V 输出脚？ | 看电源部分 |
| LED / 按键接在哪个脚？ | 搜 `LED` / `KEY` |

**踩过的坑**：`docs/01-hardware.md` §4.1 记录了一处**原理图与代码不一致**的地方 ——
舵机在图上标的是 PA0，但实际代码用的是 PB0。**以代码为准**，这里是留档。

---

## 二、rs485-module —— RS485 收发模块

项目1 的核心外设。`firmware-stm32` 和 `firmware-esp32` 都靠它通信。

| 文件 | 内容 | 什么时候用 |
|---|---|---|
| **原理图.pdf** | 内部电路（SP3485/MAX485 + 保护） | 查 DE/RE 脚怎么接的、有没有上下拉 |
| **规格手册.pdf** | 电气参数 | 查**最高波特率**、共模电压范围、节点数上限 |
| **尺寸.pdf** | 机械尺寸 | 画外壳/装壳时用 |
| **引脚连接说明.xlsx** | 和 STM32 开发板的对应接法 | **第一次接线时看这个** |
| readme.txt | 模块说明 | — |

**关键参数**（从规格手册查到，写在 `docs/01-hardware.md` 里）：
- 支持 **最高 10 Mbps**（本项目用 115200，远低于上限）
- **A/B 两线半双工**，需要 DE 脚控制方向
- 总线两端需要 **120 Ω 终端电阻**（本项目实测：加了反而没帮助，故障不在电气层）

**踩过的坑**：485 首试成功率只有 58%，一度怀疑是模块/接线问题，
最后定位到**主站事务间隔不足**（从机 TC 标志置位前无法重新武装接收）。
过程见 [`../docs/07-interview-notes.md`](../docs/07-interview-notes.md)。

---

## 三、ov-watch-screen —— 【已弃用】的屏幕

**这一整个目录都是留档，不要用它做新设计。**

这原本是打算给项目1 用的 SPI 彩屏，但核对后发现：

| 问题 | 结论 |
|---|---|
| **与控制板 18-pin 座不兼容** | 引脚定义完全不同，硬插会短路 |
| 需要自己飞线 18 根 | 工作量太大，且不如直接买 ILI9341 模块 |

**最终方案**：项目1 用的是 **2.4" ILI9341 SPI 彩屏**（320×240，LovyanGFX 驱动 + 自建 965 字中文字库）。

**留档的价值**：
- `OVWatch_Back.pdf` 是屏幕背面的引脚定义 —— 以后如果要用这块屏，不用再拆一次
- `OVWatch_Firmware.zip` / `fw/` 是出厂固件 —— 刷坏了可以恢复

**完整核对记录**：见 [`../docs/05-ovwatch-screen-check.md`](../docs/05-ovwatch-screen-check.md)

---

## 四、项目2 的硬件资料在哪

**项目2（声纹门禁）用到的模块资料不在这个目录**，因为都是现成的通用模块：

| 器件 | 资料怎么获取 |
|---|---|
| **STM32F103RET6** | 用 `Keil.STM32F1xx_DFP` 器件包（含官方数据手册链接）|
| **MAX9814 麦克风模块** | 常见模块，关键参数已写进 [`../voice-lock/README.md`](../voice-lock/README.md)：GAIN 接 GND = 60 dB，VIH 等 |
| **OLED SSD1306** | 驱动命令已实现在 `voice-lock/firmware-stm32-ret6/Core/Src/oled.c` |
| **WS2813E RGB 灯** | 曾评估过，模块资料已随弃用一起删除（见 `voice-lock/README.md` §八.6）|

**为什么没有原理图**：这些都是**成品模块**（自带 PCB 和丝印），
不像项目1 的核心板那样需要看板级原理图。引脚对应关系都写进了各自的 README。

---

## 五、整理说明

这份目录原来是两个地方：根目录的 `_hw_ref/` 和 `hardware/`，
而且 `hardware/Core.pdf` 和 `_hw_ref/FryPi_Core.pdf` **是同一个文件**（SHA256 相同，652 KB 重复）。

已合并为一份：

| 原位置 | 现位置 |
|---|---|
| `hardware/Core.pdf` | `hardware/core-board/FryPi_Core.pdf` |
| `_hw_ref/FryPi_Core.pdf` | **删除**（重复）|
| `_hw_ref/rs485_module/*` | `hardware/rs485-module/*` |
| `_hw_ref/OVWatch_*` + `_hw_ref/fw/` | `hardware/ov-watch-screen/*` |

**省了 652 KB，也更清楚了。**
