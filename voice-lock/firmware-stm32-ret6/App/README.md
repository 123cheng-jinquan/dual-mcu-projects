# App/ —— 手写应用代码（项目二，STM32F103RET6 采集端）

> **`Core/` 里是 CubeMX 自动生成的**（时钟、外设初始化、`main.c`）。
> **这里放全部手写代码。** 两者必须分开，否则 CubeMX 重新生成会冲掉业务代码。

---

## 目录结构

| 目录 | 职责 | 文件 |
|---|---|---|
| **`audio/`** | 8 kHz 采样（TIM2 触发 ADC + DMA）、帧能量 VAD 语音触发 | `audio_capture.c` / `.h` |
| **`protocol/`** | 二进制帧协议：收发状态机 + CRC16 | `uart_proto.c` / `.h` |
| **`ui/`** | SSD1306 OLED 驱动（含 I2C 错误追踪 + 显示看门狗）| `oled.c` / `.h` |

---

## ⚠️ 为什么 `main.c` 还在 `Core/Src/`

它包含 **35 个 `USER CODE BEGIN/END` 标记**，说明它是 **CubeMX 生成并托管**的文件：

```
Core/Src/main.c  ← CubeMX 生成（含 SystemClock_Config / MX_*_Init 调用）
```

**如果把它移出 `Core/Src/`**，CubeMX 下次"GENERATE CODE"会在 `Core/Src/` 里
**重新生成一个新的 `main.c`** → 同时存在两份 → 重复定义。

**正确的做法（还没做）**是把它瘦身成"薄壳"：
`main.c` 只保留 CubeMX 的初始化，业务逻辑挪到 `App/app/app_main.c`，
在 `USER CODE BEGIN 2` 里调一次 `app_init()`。这也是**项目一**的做法：

```
项目一：Core/Src/main.c（CubeMX 薄壳）→ 调 App/app/app_main.c（业务）
项目二：Core/Src/main.c（80KB，业务也在这里）
```

**项目二为什么没改**：`main.c` 有 80 KB，拆它要动 5 个任务/流程函数和
一堆共享状态，改了只能靠 Keil 重新编译验证 —— 而死线只剩几天。
**先标清楚边界，留作下一步。**

---

## 分层设计

```
Core/Src/main.c        CubeMX 初始化 + 门禁流程 + 命令分发
        ↓ 调用
App/audio/       audio_capture  8kHz 采集 + VAD
App/protocol/    uart_proto      帧封装/解析 + CRC16
App/ui/          oled            SSD1306 驱动
        ↓ 调用
Drivers/         ST HAL + CMSIS（CubeMX 管理）
```

**依赖单向向下**：`audio/`、`protocol/`、`ui/` 三个模块**互不依赖**，
也不依赖 `main.c` 里任何东西 —— 每个都能单独编译检查。

---

## 各模块的关键设计

### `audio/audio_capture.c`

| 设计点 | 为什么 |
|---|---|
| **TIM2 比较输出触发 ADC** | 题目要求"严格 8 kHz"。软件轮询做不到稳定采样率；硬件触发由定时器保证，实测 7996–8000 Hz |
| **DMA 循环搬运 16384 点** | 采样过程**不占 CPU**，主循环可以正常跑串口和 OLED |
| **帧能量 VAD（10ms 一帧）** | 早期用"单样本超阈值"触发，8kHz 下波形每 8~24 个点就穿过基线 → **永远触发不了** |
| **噪声底用最小值统计法** | 取最近 10 帧的最小值。如果用平均值，正在说话时噪声底会被抬高 → 越说越难触发 |

### `protocol/uart_proto.c`

帧格式：`SYNC(0xAA) | CMD | SEQ | LEN(2,LE) | PAYLOAD | CRC16`

| 设计点 | 为什么 |
|---|---|
| **CRC 不覆盖 SYNC** | 和 Modbus 从机地址不参与 CRC 同理：SYNC 只用来找帧头，滑动搜索时不必重算 |
| **长度合法性检查（>1024 丢弃）** | 音频数据里出现 `0xAA` 的概率约 1/256，每 8KB 平均 31 次假帧头。只靠 CRC 会被假的 LEN 卡住几秒 |
| **逐字节喂状态机** | 串口中断一次只来 1 字节。在中断里攒帧会很慢；状态机几微秒返回，主循环再查"有没有完整帧" |

### `ui/oled.c`

| 设计点 | 为什么 |
|---|---|
| **I2C 错误计数** | 原版 `OLED_Init()` 完全丢掉 `HAL_I2C_Master_Transmit` 的返回值 → 通信失败也打印 "OLED ok" → 屏幕黑但毫无线索 |
| **总线恢复（9 个时钟脉冲）** | STM32F1 的 I2C 有著名缺陷：从机被干扰后外设会卡在 BUSY，之后所有传输都失败且不报错 |
| **显示看门狗（每 2 秒重发开显示）** | 曾出现"屏幕黑但 `err=0`" —— 数据一直成功发出去，是 SSD1306 自己掉了显示（电压跌落）。周期性重发命令即可自愈 |

---

## 构建（Keil MDK）

```
1. 打开 MDK-ARM/MyBiye.uvprojx
2. Options for Target → Target → ARM Compiler 选 "compiler version 6"
3. Rebuild
4. DAP-Link 烧写（Flash 算法选 High-density 512k！）
```

**⚠️ 两个坑**：

| 坑 | 说明 |
|---|---|
| MDK-Lite 32 KB 上限 | 本项目约 44 KB，**必须用社区版或更高** |
| 器件包 | 需要 `Keil.STM32F1xx_DFP`（2.3.0 即可）|

**本目录加进 Keil 后**，`MyBiye.uvprojx` 里的 include 路径需要包含这三个子目录：

```
../Core/Inc;../App/audio;../App/protocol;../App/ui;...
```

（已配好。如果 Keil 报 `cannot open source input file "audio_capture.h"`，
就是这个路径丢了。）
