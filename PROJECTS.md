# 项目分类说明

> 本仓库包含**两个独立的嵌入式项目**。看这份文档能快速分清哪个是哪个、各自在哪、怎么用。

---

## 一、项目一览

| # | 项目 | 代码位置 | 主控 | 用途 |
|---|---|---|---|---|
| **1** | **双 MCU 工业网关**<br>（RS485 振动监测 + 端侧 AI）| `firmware-stm32/`<br>`firmware-esp32/` | STM32F411RET6<br>ESP32-S3 | **简历项目一** |
| **2** | **端云协同声纹门禁**<br>（离线语音触发 + 云端声纹验证）| `firmware-stm32-ret6/` | STM32F103RET6<br>（规划加 ESP32-S3）| **简历项目二** |
| — | **毕业论文原始代码**<br>（教育部门复检提交用）| `archive/thesis-original-C8T6/` | STM32F103C8T6 | **复检归档，勿动** |

---

## 二、目录结构

```
MyProject/
├── firmware-stm32/              ← 【项目1】Modbus 从机固件（F411RET6）
├── firmware-esp32/              ← 【项目1】Modbus 网关固件（ESP32-S3）
├── firmware-stm32-ret6/         ← 【项目2】声纹门禁固件（F103RET6）★升级版
│
├── archive/                     ← 【归档】论文原始代码，复检用
│   ├── thesis-original-C8T6/
│   │   └── MyBiye-Final1/       ← 与作者提交的原始包**逐字节一致**
│   └── _source-zip/
│       ├── MyBiye-Final1.zip    ← 原始包
│       └── 校验信息-SHA256.txt   ← SHA256 + 说明
│
├── docs/                        ← 设计文档、协议、面试材料
├── tools/                       ← 调试/验证脚本、代码生成工具
├── hardware/                    ← 硬件资料（原理图等）
├── model_out/                   ← TFLite 模型产物
├── logs/                        ← 联调日志（gitignore）
│
└── README.md / plan.md          ← 项目总览与计划
```

---

## 三、项目 1：双 MCU 工业网关

**简历标题**：双 MCU 工业网关 | RS485 振动监测与端侧 AI 异常识别平台

| 项 | 内容 |
|---|---|
| **架构** | STM32F411 采集节点 + ESP32-S3 边缘网关 |
| **总线** | RS485 两线半双工，**115200**，手写 Modbus RTU 主机/从机协议栈 |
| **采样** | 50Hz 六轴加速度（MPU6050），双缓冲维护 600 点窗口 |
| **推理** | TFLite Micro 端侧推理，**< 400 µs** |
| **界面** | 2.4" ILI9341 SPI 彩屏 320×240，LVGL + 自建 965 字中文字库 |
| **验收** | **4 小时长跑：47510 次事务首试成功率 100%**，超时/重试/CRC 错全为 0 |
| **文档** | [`firmware-esp32/README.md`](firmware-esp32/README.md) §9 排错速查<br>[`docs/06-decisions-and-results.md`](docs/06-decisions-and-results.md) 工程决策<br>[`docs/07-interview-notes.md`](docs/07-interview-notes.md) 面试问答 |

**核心亮点**：总线首试成功率从 **58% 调到 100%**（自研双串口增量对账脚本定位根因）。
详见 `firmware-esp32/README.md` §9.2.1。

---

## 四、项目 2：端云协同声纹门禁（升级版）

**简历标题**：端云协同声纹门禁系统 | 离线语音触发与云端声纹验证

| 项 | 内容 |
|---|---|
| **主控** | **STM32F103RET6**（512KB Flash / 64KB RAM，LQFP64）|
| **原版** | STM32F103C8T6（64KB / 20KB），见 `archive/` |
| **采集** | MAX9814 麦克风（AGC）→ PA1 / ADC1_IN1 |
| **✅ 已升级** | **采样率 250Hz → 8kHz**（TIM2 触发 ADC + DMA，见下）|
| **执行** | 舵机（PB0 / TIM3_CH3，50Hz PWM）+ SSD1306 OLED（I2C1）|
| **规划** | 加 ESP32-S3 做边缘网关：收音频 → MFCC → TFLite 推理 → 回传判定 |

### ⚠️ 为什么要升级采样率（这是本项目的核心）

原版是"单次转换 + `HAL_Delay(2)`"，等效采样率只有 **250 Hz**：

| | 数值 |
|---|---|
| 采样率 | 250 Hz |
| 奈奎斯特极限 | **125 Hz** |
| 人说话基频 | **85 ~ 255 Hz** |

⇒ **采到的数据里连人声基频都没有**，只剩"音量包络"。
**这才是原论文要写"云端声纹仅作接口验证"的根本原因** —— 数据本身不支持声纹识别。

升级后 **8 kHz**（32 倍），能覆盖完整语音频段，才谈得上做 MFCC / 声纹特征。

### 升级版的改动清单（相对论文原版）

| 类别 | 文件 | 说明 |
|---|---|---|
| **新增** | `Core/Src/audio_capture.c`<br>`Core/Inc/audio_capture.h` | 8kHz 采集模块（TIM2_CH2 触发 ADC1 + DMA1 循环）|
| **改写** | `Core/Src/main.c` | 加 `audio_init()`、**主循环命令分发**（原来 `while(1){}` 是空的）、音频自检 `t` |
| **器件移植** | `stm32f103xe.h` + `startup_stm32f103xe.s` | C8T6(xB) → RET6(xE) |
| **CubeMX 重生成** | `adc.c` `i2c.c` `tim.c` `usart.c`<br>`stm32f1xx_hal_msp.c` `MyBiye.ioc` | 换芯片型号的必然结果 |

**未被改动的原有代码**（论文主体逻辑）：
`MeasureBaseline` / `WaitForVoiceTrigger` / `CaptureVoiceWindow` /
`ExtractVoiceFeature` / `ESP_CloudVerifyOnce` / `Servo_SetAngle` /
`Door_Open` / `Door_Close` / `OLED_UpdateStatus` / `oled.c`

### 开发笔记（踩过的坑，值得记）

| 坑 | 说明 |
|---|---|
| **F1 的 ADC 外部触发源没有 `T2_TRGO`** | 那是 F4 的选项。F1 只有 `T1_CC1/2/3`、`T2_CC2/CC3`、`T3_CC1/T3_TRGO`、`T4_CC4` 等。用 `arm-none-eabi-gcc -fsyntax-only` 才发现 |
| **F1 的 `ADC_InitTypeDef` 没有 `ExternalTrigConvEdge`** | 上升沿使能编码在 `ExternalTrigConv` 的宏值里 |
| **不能用 TIM3 做 8kHz 触发** | TIM3 正在驱动舵机（ARR=19999 → 50Hz）。定时器的 ARR 只有一套值，改成 999 舵机会失控 → 改用**空闲的 TIM2** |
| **`slow_u16` 式的"越界返回 0"陷阱** | （项目1 的教训）越界返回默认值时，调用方必须区分"0 是非法还是真值" |

---

## 五、归档区（`archive/`）

**用途**：教育部门论文复检 / 学位材料提交。

| 项 | 值 |
|---|---|
| 原始包 | `_source-zip/MyBiye-Final1.zip`（9.46 MB）|
| **SHA256** | `5B3FCF1655B659B166D29A5791300B1B87C8795EE2A5B79B2C3946FE0E7B1065` |
| 解压工程 | `thesis-original-C8T6/MyBiye-Final1/`（196 个文件）|
| 目标芯片 | STM32F103C8T6（LQFP48，64KB/20KB）|
| 工程名 | `MyBiye` |

> ⚠️ **本目录内容从未被修改**，与提交的原始包逐字节一致。
> 所有升级改动都在 `firmware-stm32-ret6/` 里，两者**完全独立、互不影响**。
>
> **复检提交时用 `archive/` 里的版本**，不要用 `firmware-stm32-ret6/`（那是毕设之后的迭代）。

---

## 六、固件编译入口

| 项目 | 打开这个文件 |
|---|---|
| 项目1 · F411 从机 | `firmware-stm32/cubemx/Projects/`（Makefile，`mingw32-make -j4`）|
| 项目1 · ESP32 网关 | `firmware-esp32/`（PlatformIO，`pio run -t upload`）|
| **项目2 · 声纹门禁** | **`MDK-ARM/MyBiye.uvprojx`**（Keil µVision5，F7 编译）|

**烧写工具**：DAP-Link（SWD）

| 项目2 接线 | H1 排针 |
|---|---|
| SWDIO | 7 脚 |
| SWCLK | 9 脚 |
| GND | 4/6/8/10… |
| 3.3V（可选）| 1 或 2 脚 |

---

## 七、串口命令（项目2）

USART1（PA9/PA10），**115200**：

```
a = ESP8266 AT        r = ESP8266 复位     o = 开门
c = 关门              w = 连 WiFi          s = 连服务器
v = 语音验证          t = 音频采集自检 ★   h = 帮助
```

`t` 会连续测 5 轮，输出 `avg / min / max / p2p / dma_pos`：

| 观察 | 正常 |
|---|---|
| `dma_pos` | **一直在变**（说明定时器+ADC+DMA 都在跑）|
| `avg` | 接近 **2048**（MAX9814 @3.3V 偏置 = 1.65V）|
| 拍手/说话时 `p2p` | **明显变大** |
