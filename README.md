# 嵌入式项目集 · 双 MCU 架构实践

> **程锦泉** · 电子信息工程（2026 届）· 求职方向：嵌入式软件工程师（MCU / RTOS / 工业通信）
>
> 本仓库包含**两个完整的双 MCU 项目**，都已完成实物验证并留有调试记录与实测数据。

---

## 项目一览

| # | 项目 | 架构 | 核心亮点 | 实测结果 |
|---|---|---|---|---|
| **1** | [**边缘智能 Modbus 网关**](#项目一边缘智能-modbus-网关) | STM32F411 ⇄ **RS485** ⇄ ESP32-S3 | 手写 Modbus RTU **主/从双端协议栈**、TFLite Micro 端侧推理、LVGL HMI | **4 小时长跑 47510 次事务，首试成功率 100%**，零超时/零重试/零 CRC 错 |
| **2** | [**双 MCU 声纹识别门禁**](voice-lock/README.md) | STM32F103RET6 ⇄ **UART** ⇄ ESP32-S3 | **MFCC 全链路**（自写 FFT/Mel/DCT）、σ 自动标定、全流程离线 | **本人 0.59~0.73 / 他人 0.49~0.52**，区分度 0.2，端到端 **1.3 秒** |

**两个项目的共同点**：
- 都是**双 MCU 分工**（一个负责实时采集，一个负责算力密集的处理）
- 都**自己写了通信协议**（不是调库）
- 都有**完整的踩坑记录**（这是最有价值的部分）
- 都**实测验证过**，不是"设计文档"

---

## 快速导航

| 想了解 | 去哪里 |
|---|---|
| **项目2（声纹门禁）** | [**`voice-lock/README.md`**](voice-lock/README.md) ← 主文档 |
| 项目2 的**算法原理** | [`voice-lock/docs/algorithm.md`](voice-lock/docs/algorithm.md)（含 5 轮迭代血泪史）|
| 项目2 的**通信协议** | [`voice-lock/docs/protocol.md`](voice-lock/docs/protocol.md) |
| 项目2 的**演示视频脚本** | [`voice-lock/docs/demo-video-script.md`](voice-lock/docs/demo-video-script.md) |
| **演示视频与截图** | [`media/README.md`](media/README.md) |
| **项目1（Modbus 网关）** | 下方 [项目一](#项目一边缘智能-modbus-网关) |
| 项目1 的**面试问答卡片** | [`docs/07-interview-notes.md`](docs/07-interview-notes.md) |
| 项目1 的**工程决策与结果** | [`docs/06-decisions-and-results.md`](docs/06-decisions-and-results.md) |
| **大白话技术基础** | [`docs/08-basics-in-plain-language.md`](docs/08-basics-in-plain-language.md) |

---

## 目录结构

```
MyProject/
│
├─ README.md                       ← 本文件（两个项目总览）
├─ PROJECTS.md                     项目分类说明（更详细）
│
├─ media/                          ★ 演示素材：视频 + 截图
│   ├─ README.md                   素材索引与录制说明
│   ├─ videos/                     3 支演示视频（10.5 MB，已压缩）
│   └─ frames/                     5 张关键截图
│
├─ voice-lock/                     ★★ 项目2：双 MCU 声纹识别门禁
│   ├─ README.md                   主文档（架构/接线/编译/命令/踩坑/实测）
│   ├─ docs/
│   │   ├─ algorithm.md            MFCC + 声纹比对算法详解（含 5 轮迭代）
│   │   ├─ protocol.md             二进制帧协议规范
│   │   └─ demo-video-script.md    演示视频分镜 + 面试 Q&A
│   ├─ firmware-stm32-ret6/        采集节点（Keil MDK）
│   │   ├─ Core/Src/
│   │   │   ├─ main.c              ★ 主逻辑 + 门禁流程 + 诊断命令
│   │   │   ├─ audio_capture.c     ★ 8 kHz 采集 + DMA + VAD
│   │   │   ├─ uart_proto.c        ★ 二进制协议 + CRC16
│   │   │   └─ oled.c              ★ SSD1306 驱动 + I2C 错误追踪
│   │   └─ MDK-ARM/MyBiye.uvprojx
│   └─ firmware-esp32-voicelock/   声纹引擎（PlatformIO）
│       ├─ include/{mfcc,speaker,uart_link}.h
│       └─ src/{mfcc,speaker,uart_link}.cpp
│
├─ docs/                           项目1 文档（01 硬件 / 02 协议 / 03 上手 …）
├─ firmware-stm32/                 项目1 下位机（STM32F411 + FreeRTOS）
│   ├─ App/                        手写代码（平台无关，可在 PC 上单测）
│   │   ├─ modbus_crc.[ch]         CRC16 + 测试向量
│   │   ├─ modbus_slave.[ch]       手写 Modbus RTU 从机协议栈
│   │   └─ regbank.[ch]            寄存器表 + 六轴窗口双缓冲
│   ├─ test/test_modbus.c          89 项 PC 端单元测试
│   └─ cubemx/                     CubeMX 工程（GCC Makefile）
├─ firmware-esp32/                 项目1 上位机（主站 / LVGL / 推理 / MQTT）
├─ model_out/                      TFLite 模型产物
├─ tools/                          PC 侧脚本（协议对拍 / 压测 / 采样 / 训练 / 长跑）
├─ hardware/                     硬件参考资料（原理图/模块手册，见其 README）
└─ archive/                        论文原始代码归档（C8T6 版，留档对照）
```

---

# 项目一：边缘智能 Modbus 网关

> STM32F411（采集 + 手写 Modbus RTU 从机） ⇄ RS485 ⇄ ESP32-S3（LVGL HMI + 端侧推理 + MQTT）

## 这是什么

双机边缘智能网关：
- **下位机** `STM32F411RET6` + FreeRTOS：传感器采集、**手写 Modbus RTU 从机协议栈**、本地自治与故障降级
- **上位机** `ESP32-S3`：**手写 Modbus 主站**轮询调度、LVGL 触摸 HMI、1D-CNN 端侧推理（TFLite Micro）、WiFi/MQTT 上云

## 核心成果

| 指标 | 结果 |
|---|---|
| **总线首试成功率** | **58% → 100%** |
| **长跑验证** | 4 小时，**47510 次事务** |
| 超时 / 重试 / CRC 错 | **全部 0** |
| 内存泄漏 | heap 净变化 **0** |
| **端侧推理耗时** | **< 400 µs** |
| Flash 占用 | 32.9 KB / 512 KB |

> **最有价值的一段排查**：485 总线出现"首试必失败、重试必成功"。
> 按电气方向排查（降波特率、加 120 Ω 终端电阻、放宽超时）**全部被反证**后，
> 自己写的**双串口增量对账脚本**——同时监听主站发出和从机收到的帧，逐帧比对序号，
> 测得**主站发 176 次而从机只收到 100 次请求**，据此把问题锁定在**请求方向**。
> 最终根因：从机在 `TC` 标志置位前无法重新武装接收，主站事务间隔不足导致整帧被静默丢弃。
>
> 完整过程见 [`docs/07-interview-notes.md`](docs/07-interview-notes.md)。

## 文档索引

| 文档 | 内容 |
|---|---|
| [`plan.md`](plan.md) | **主计划**：架构、硬件现状、场景选型、时间表、风险 |
| [`docs/07-interview-notes.md`](docs/07-interview-notes.md) | ★★ **面试问答卡片**：自我介绍、快速数字表、**最难问题（58%→100%）的完整标准答案**、追问防御清单 |
| [`docs/08-basics-in-plain-language.md`](docs/08-basics-in-plain-language.md) | ★ **大白话基础**：引脚/外设/寄存器、什么是"复用"、I2C 与上拉电阻、为什么降速反而更可靠、DMA |
| [`docs/06-decisions-and-results.md`](docs/06-decisions-and-results.md) | ★ **工程决策与关键结果**（答辩/简历素材）：架构、范围取舍、6 个技术问题与解法、量化指标、已知限制 |
| [`docs/04-plan-15days.md`](docs/04-plan-15days.md) | 15 天执行计划：范围分级、逐日日程、模型路线把关 |
| [`docs/03-bringup.md`](docs/03-bringup.md) | 分阶段上手指南 + 每阶段验收标准 + 踩坑清单 |
| [`docs/01-hardware.md`](docs/01-hardware.md) | 引脚分配表、接线、电平与供电红线 |
| [`docs/02-modbus-protocol.md`](docs/02-modbus-protocol.md) | 寄存器映射、CRC 测试向量、实战帧示例、一致性机制、从机状态机 |
| [`firmware-esp32/README.md`](firmware-esp32/README.md) | ESP32 侧固件说明 + **排错速查**（485 从 58%→100%、SPI 屏 5 个坑、中文字库）|
| [`firmware-stm32/CUBEMX-SETUP.md`](firmware-stm32/CUBEMX-SETUP.md) | CubeMX 逐项配置清单（HSE=25MHz、ADC/DMA、USART2+IDLE、DE 时序）|

## 现在就跑得起来的东西

### 1. PC 端单元测试（不需要任何硬件）

```powershell
powershell -ExecutionPolicy Bypass -NoProfile -File firmware-stm32\test\build_and_run.ps1
```
编译 `App/` 下的手写协议栈并跑 **89 项**单元测试：CRC 向量（含 `docs/02` §6 全部实战帧回归）、
异常码、静默丢弃、广播写、FC10 原子写入、六轴窗口双缓冲一致性、通信看门狗。
**当前状态：89 PASS / 0 FAIL。**

> ⚠️ Windows 默认禁止运行 `.ps1`，所以必须带 `-ExecutionPolicy Bypass`。
> 另外本仓库的 `.ps1` **一律只用 ASCII 字符**——PowerShell 5.1 会把无 BOM 的 UTF-8
> 中文脚本按 GBK 解析，中文字符串会变乱码并直接报语法错误。

### 2. 编译固件（已实测通过）

```powershell
cd firmware-stm32\cubemx\Projects
make -j8
```
产物在 `cubemx\Projects\build\`：`ModbusGatewayF411.elf / .hex / .bin / .map`。

### 3. 已踩过并已绕开的 CubeMX 坑

见 [`firmware-stm32/CUBEMX-SETUP.md`](firmware-stm32/CUBEMX-SETUP.md) §7。
**重新 GENERATE CODE 之后，务必跑一次 `firmware-stm32\apply_app_patch.ps1`**（幂等）。

## 关键约束（别踩）

- Modbus 必须用 **USART2 (PA2/PA3)**，USART1 被板载 CH340 占用
- **PB12~PB15 被板载 W25Q128 占用**；套件里的 W25Q64 模块不用接
- **PA13/PA14 保留 SWD**；CubeMX 里选 Serial Wire，别选 JTAG（会吃掉 PA15/PB3/PB4）
- **485 默认 115200**：9600 下窗口读取要 1.37s、占总线约 83%，与 500ms 慢车道冲突
- **C 区（0x0020~0x002F）只允许主站写，从机只读**：否则主站"读-改-写"会与从机互相覆盖
- **六轴特征布局必须是 `per-feature`**：写成 `per-axis` 是转置关系，模型照跑不报错、但准确率掉到随机
- MQ-2/MQ-135 的 AO、HC-SR04 的 ECHO **必须分压**后才能进 3.3V 引脚
- 舵机/电机/风扇用**外部 5V 供电并共地**，否则随机复位
- 485 总线访问在 S3 侧**必须串行化**（单一 master task）

---

# 归档：论文原始代码

[`archive/thesis-original-C8T6/`](archive/thesis-original-C8T6/) 保存了毕业设计**升级前**的
STM32F103C8T6 版本源码，用于和 `voice-lock/` 的升级版对照。

| 对比项 | 原毕设（C8T6）| 升级版（`voice-lock/`）|
|---|---|---|
| 架构 | 单 MCU + ESP8266 + Python 服务器 | **双 MCU（F103 + ESP32-S3）** |
| 采样 | ADC 软件轮询 | **TIM2 硬件触发 + DMA，8 kHz** |
| 特征 | 13 维时域统计量 | **84 维 MFCC 超矢量** |
| 判定 | 云端 Python / 讯飞 WebAPI | **端侧实现，全离线** |
| 网络 | 必需 | **不需要** |

**原始压缩包**：[`archive/_source-zip/`](archive/_source-zip/)（含 SHA256 校验信息）

---

# 关于本仓库

**作者**：程锦泉 · 电子信息工程（2026 届）
**方向**：嵌入式软件工程师（MCU / RTOS / 工业通信 / 端侧算法）

**说明**：
- 两个项目均为**个人独立完成**（项目2 基于本人的毕业设计升级）
- 所有实测数据都来自**真实硬件运行**，未做美化
- 已知局限（如声纹在 8 kHz 下的区分度上限）在各项目文档里**明确写出**，没有隐藏

**联系方式**：见简历（本仓库不公开私人联系方式，避免被爬虫抓取）。
如需就技术细节交流，欢迎在 Issues 里留言。
