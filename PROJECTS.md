# 项目分类说明

> 本仓库包含**两个独立的嵌入式项目** + 一份论文原始代码归档。
> 看这份文档能快速分清哪个是哪个、各自在哪、怎么用。

---

## 一、项目一览

| # | 项目 | 代码位置 | 主控 | 状态 |
|---|---|---|---|---|
| **1** | **双 MCU 工业网关**<br>（RS485 振动监测 + 端侧 AI）| [`firmware-stm32/`](firmware-stm32/)<br>[`firmware-esp32/`](firmware-esp32/) | STM32F411RET6<br>ESP32-S3 | ✅ **已完成，长跑验收通过** |
| **2** | **双 MCU 声纹识别门禁**<br>（端侧全离线声纹验证）| [`voice-lock/`](voice-lock/) | STM32F103RET6<br>ESP32-S3 | ✅ **已完成，实测通过** |
| — | **毕业论文原始代码**<br>（教育部门复检提交用）| [`archive/thesis-original-C8T6/`](archive/thesis-original-C8T6/) | STM32F103C8T6 | 📦 **归档，勿动** |

---

## 二、目录结构（当前实际结构）

```
MyProject/
│
├── README.md                    ← 两个项目的总览（先看这个）
├── PROJECTS.md                  ← 本文件：项目分类说明
│
├── media/                       ★ 演示素材
│   ├── README.md                素材索引
│   ├── videos/                  3 支演示视频（10.5 MB）
│   └── frames/                  5 张关键截图
│
├── voice-lock/                  ★★ 【项目2】双 MCU 声纹识别门禁
│   ├── README.md                主文档
│   ├── docs/
│   │   ├── algorithm.md         MFCC + 比对算法（含 5 轮迭代）
│   │   ├── protocol.md          二进制帧协议
│   │   └── demo-video-script.md 演示视频脚本 + 面试 Q&A
│   ├── firmware-stm32-ret6/     采集节点（Keil MDK）
│   └── firmware-esp32-voicelock/ 声纹引擎（PlatformIO）
│
├── firmware-stm32/              【项目1】Modbus 从机固件（F411RET6）
├── firmware-esp32/              【项目1】Modbus 网关固件（ESP32-S3）
├── docs/                        【项目1】设计文档、协议、面试材料
├── model_out/                   【项目1】TFLite 模型产物
├── hardware/                    ★ 硬件参考资料（两个项目共用）
│   ├── README.md                每个文件是什么
│   ├── core-board/              项目1 核心板原理图
│   ├── rs485-module/            RS485 模块资料
│   └── ov-watch-screen/         【已弃用】的屏幕资料（留档）
│
├── tools/                       调试/验证脚本、代码生成工具
├── archive/                     【归档】论文原始代码，复检用
│   ├── thesis-original-C8T6/
│   │   └── MyBiye-Final1/       与提交的原始包**逐字节一致**
│   └── _source-zip/
│       ├── MyBiye-Final1.zip    原始包
│       └── 校验信息-SHA256.txt  SHA256 + 说明
│
├── plan.md                      【项目1】主计划
└── logs/                        联调日志（.gitignore，不入库）
```

---

## 三、项目 1：双 MCU 工业网关

**简历标题**：双 MCU 工业网关 | RS485 振动监测与端侧 AI 异常识别平台

| 项 | 内容 |
|---|---|
| **架构** | STM32F411 采集节点 + ESP32-S3 边缘网关 |
| **总线** | RS485 两线半双工，**115200**，**手写 Modbus RTU 主机/从机双端协议栈** |
| **采样** | 50 Hz 六轴加速度（MPU6050），双缓冲维护 600 点窗口 |
| **推理** | TFLite Micro 端侧推理，**< 400 µs** |
| **界面** | 2.4" ILI9341 SPI 彩屏 320×240，LVGL + **自建 965 字中文字库** |
| **验收** | **4 小时长跑：47510 次事务首试成功率 100%**，超时/重试/CRC 错全为 0 |
| **代码量** | Flash 32.9 KB / 512 KB，RAM 33.9 KB / 128 KB |
| **单测** | PC 端 **89 项单元测试全通过**（不需要硬件）|

**核心亮点**：总线首试成功率从 **58% 调到 100%**（自研双串口增量对账脚本定位根因）。

**文档入口**：
- [`docs/07-interview-notes.md`](docs/07-interview-notes.md) ★★ 面试问答卡片（含 58%→100% 的完整答案）
- [`docs/06-decisions-and-results.md`](docs/06-decisions-and-results.md) 工程决策与结果
- [`docs/08-basics-in-plain-language.md`](docs/08-basics-in-plain-language.md) 大白话技术基础
- [`firmware-esp32/README.md`](firmware-esp32/README.md) ESP32 侧固件说明 + 排错速查
- [`docs/02-modbus-protocol.md`](docs/02-modbus-protocol.md) 寄存器映射与协议细节

---

## 四、项目 2：双 MCU 声纹识别门禁（毕业设计升级版）

**简历标题**：双 MCU 声纹识别门禁系统 | 端侧全离线声纹验证

| 项 | 内容 |
|---|---|
| **架构** | STM32F103RET6 采集节点 + ESP32-S3 声纹引擎 |
| **主控** | **STM32F103RET6**（512 KB Flash / 64 KB RAM，LQFP64）|
| **原版** | STM32F103C8T6（64 KB / 20 KB），见 [`archive/`](archive/) |
| **采集** | MAX9814 麦克风 → PA1 / ADC1_IN1，**TIM2 硬件触发 + DMA，严格 8 kHz**（实测 7996–8000 Hz）|
| **触发** | 帧能量 VAD + **最小值统计法估噪声底**（取最近 10 帧最小值）|
| **传输** | 自研二进制帧协议（帧头 + 序号 + 长度 + CRC16），1 秒音频分 32 块上传 |
| **特征** | **自研 84 维 MFCC 超矢量**（14 静态 + 14 Δ + 14 ΔΔ，各取均值/标准差）|
| **比对** | 全局 z-score + 加权欧氏距离 + 高斯核，**σ 自动标定** |
| **指示** | SSD1306 OLED（I2C1，地址 0x78）：`*** PASS ***` / `--- FAIL ---` |
| **验收** | **本人 0.59–0.73 / 他人 0.49–0.52**，区分度 **0.2**，端到端 **1.3 秒** |
| **网络** | **完全不需要** |

### ⚠️ 为什么原版采样率不够（这是升级的核心动机）

原版是"单次转换 + `HAL_Delay(2)`"，等效采样率只有 **250 Hz**：

| | 数值 |
|---|---|
| 采样率 | 250 Hz |
| 奈奎斯特极限 | **125 Hz** |
| 人说话基频 | **85 ~ 255 Hz** |

⇒ **采到的数据里连人声基频都没有**，只剩"音量包络"。
**这才是原论文要写"云端声纹仅作接口验证"的根本原因** —— 数据本身不支持声纹识别。

升级后 **8 kHz**（32 倍），能覆盖完整语音频段（0~4 kHz），才谈得上做 MFCC / 声纹特征。

### 和原论文的对比

| 维度 | 原论文（C8T6）| 升级版（`voice-lock/`）|
|---|---|---|
| 架构 | 单 MCU + ESP8266 + Python 服务器 | **双 MCU（F103 + ESP32-S3）** |
| 采样 | ADC 软件轮询 @250 Hz | **TIM2 硬件触发 + DMA @8 kHz** |
| 特征 | 13 维时域统计量 | **84 维 MFCC 超矢量** |
| 算法 | 调讯飞 WebAPI | **完全自研**（自写 FFT / Mel / DCT / 比对）|
| 判定 | 云端 Python | **端侧高斯核 + σ 自动标定** |
| 网络 | 必需 | **不需要** |
| 延迟 | 网络往返 | **1.3 秒** |

**诚实说明**：升级版是**独立的端侧方案**，不复用原论文的服务器代码。
论文里的"端云协同 + 讯飞 API" 仍可作为**扩展路径**（本地粗筛 + 云端精确的两级验证），
但主线已经是 `voice-lock/` 这套离线实现。

### 文档入口
- [`voice-lock/README.md`](voice-lock/README.md) ★ 主文档（架构 / 接线 / 编译 / 命令 / 8 个踩坑 / 实测）
- [`voice-lock/docs/algorithm.md`](voice-lock/docs/algorithm.md) ★ MFCC 推导 + 算法迭代 5 版血泪史
- [`voice-lock/docs/protocol.md`](voice-lock/docs/protocol.md) 帧格式 / CRC16 / 三级容错
- [`voice-lock/docs/demo-video-script.md`](voice-lock/docs/demo-video-script.md) 演示脚本 + 面试追问 Q&A

### 升级过程中踩的坑（摘要）

| 坑 | 根因 |
|---|---|
| **F1 的 ADC 外部触发源没有 `T2_TRGO`** | 那是 F4 的选项。F1 只有 `T1_CC1/2/3`、`T2_CC2/CC3`、`T3_CC1/T3_TRGO`、`T4_CC4` |
| **F1 的 `ADC_InitTypeDef` 没有 `ExternalTrigConvEdge`** | 上升沿使能编码在 `ExternalTrigConv` 的宏值里 |
| **VAD 用单点阈值永远触发不了** | 8 kHz 下波形每 8~24 个样本就穿过基线，必须用**帧 RMS** |
| **算法相似度出现负数** | 逐样本 z-score 把说话人个性信息减掉了（v2 版本的失败）|
| **σ 硬编码成 84，自己只有 0.40** | 实测 d² ≈ 350，差一个数量级 → 改成自动标定 |
| **OLED 突然黑屏但 `err=0`** | 电压跌落让 SSD1306 掉显示，数据链路正常 → 加显示看门狗 |
| **敲大写命令没反应** | `switch(c)` 的 case 标签是小写，没做大小写归一化 |

**完整版**：见 [`voice-lock/README.md`](voice-lock/README.md) §八（8 个坑，每个都有根因分析）

---

## 五、归档：论文原始代码

**位置**：[`archive/thesis-original-C8T6/`](archive/thesis-original-C8T6/)

| 项 | 内容 |
|---|---|
| **代码** | `MyBiye-Final1/`，与提交给学校的原始包**逐字节一致** |
| **原始包** | [`archive/_source-zip/MyBiye-Final1.zip`](archive/_source-zip/) |
| **校验** | `校验信息-SHA256.txt`（SHA256：`5B3FCF16...0E7B1065`）|
| **用途** | **教育部门复检**，以及和 `voice-lock/` 升级版做对照 |

**⚠️ 不要修改这个目录** —— 它是"提交时是什么样，现在就是什么样"的证据。

---

## 六、两个项目的共同点

面试时可以这样概括自己的技术主线：

| 共同点 | 说明 |
|---|---|
| **都是双 MCU 分工** | 一个负责实时采集（不占 CPU），一个负责算力密集处理 |
| **都自己写了通信协议** | 不是调库：Modbus RTU 主/从双端 + 自定义二进制帧 |
| **都有完整的踩坑记录** | 而且记录的是**根因**，不只是现象 |
| **都经过实测验证** | 项目1 靠 4 小时长跑日志，项目2 靠实物演示视频 + 相似度数据 |
| **都做了工程取舍** | 项目1 的 485 波特率/终端电阻，项目2 的舵机→OLED（接插件可靠性）|

**技术栈重合度**：STM32 + FreeRTOS/裸机 + UART/I2C/SPI/ADC/DMA + ESP32-S3 + C
**技术栈差异**：项目1 偏工业通信 + 端侧 AI；项目2 偏音频信号处理 + 算法工程

---

## 七、怎么快速验证

### 项目 1（不需要硬件）

```powershell
powershell -ExecutionPolicy Bypass -NoProfile -File firmware-stm32\test\build_and_run.ps1
```
→ 应看到 **89 PASS / 0 FAIL**

### 项目 2（需要硬件）

```
1. 接线：F103 PB10 → ESP32 IO7，PB11 ← IO6，GND 共地
2. F103 烧写 voice-lock/firmware-stm32-ret6（Keil + AC6 编译器）
3. ESP32 烧写 voice-lock/firmware-esp32-voicelock（pio run -t upload）
4. ESP32 串口敲 S 确认模板数 ≥ 3
5. ESP32 敲 V，F103 敲 U 并说一句话 → OLED 显示 *** PASS ***
```

详细步骤：[`voice-lock/README.md`](voice-lock/README.md) §七
