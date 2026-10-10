# firmware-esp32 · Modbus 边缘智能网关（上位机）

ESP32-S3 侧固件：**Modbus RTU 主站 + LVGL 触摸 HMI + TFLite Micro 端侧推理 + WiFi/MQTT**。
下位机是 STM32F411（手写 Modbus RTU 从机，地址 `0x01`），两者通过 RS485 两线半双工相连。

- 协议、寄存器映射、帧示例、数据一致性机制 → [`docs/02-modbus-protocol.md`](../docs/02-modbus-protocol.md)
- 主机侧调度与重试 → 同上 §9；任务划分与内存预算 → [`plan.md`](../plan.md) §7.2
- 分阶段上手指南与验收标准 → [`docs/03-bringup.md`](../docs/03-bringup.md)
- 15 天排期与硬件选型（屏幕怎么买）→ [`docs/04-plan-15days.md`](../docs/04-plan-15days.md)
- PC 侧脚本清单与用法 → [`tools/README.md`](../tools/README.md)

> **当前状态：已在真实 ESP32-S3 上编译、烧写、验证通过。**
> 函数签名、任务划分、队列、超时/重试、数据一致性、注释（为什么这样做）都是完整的；
> 需要真实硬件或真实模型才能定的部分用 `TODO:` 标出（见文末清单）。
> - ✅ 编译：`pio run -e esp32-s3-devkitc-1`（RAM 23.8% / Flash 14.7%）
> - ✅ 烧写：COM6（CH343，`1A86:55D3`），115200 8N1 可监视日志
> - ✅ **485 总线：115200，首试成功率 100%**（`TX == RX`，超时 0 / 重试 0 / CRC错 0）
>   —— 从 58% 一路调到 100% 的完整过程见 **§9.2.1**（含一个"降速有效但根因不是位速率"的教训）
> - ✅ 本地 OLED（路径 C）：SSD1306 @0x3C / SDA=GPIO8 / SCL=GPIO9，500ms 刷一次
> - ✅ **LVGL SPI 屏（路径 B）：已点亮并调通**（2026-10）
>   - 硬件：2.4" SPI 模块，**ILI9341**，**物理 320×240**，白底黑字
>   - 驱动：**LovyanGFX**（不是 TFT_eSPI —— 原因见 §9.3）
>   - 界面：数据 / 波形 / 诊断 三页，`gui_tick()` 在 core1 刷屏
>   - **最终配置三件套（缺一必错，见 §9.3）**：
>     `panel/memory = 320×240`（gui.cpp）、`TFT_ROTATION = 4`（gui.cpp）、
>     `LVGL_HOR_RES/VER_RES = 320/240`（config.h）
>   - 开机自检日志会打 `尺寸一致性：… ✓ 一致`，**必须看到 ✓ 才是对的**

---

## 1. 目录结构与文件职责

```
firmware-esp32/
├─ platformio.ini          构建配置：板型/PSRAM、串口监视器、7 个依赖库（含取舍注释）
├─ README.md               本文件
├─ include/
│  └─ config.h             ✱唯一参数表✱：引脚/周期/超时/优先级/栈/缓冲/寄存器地址/位定义
└─ src/
   ├─ main.cpp             setup()/loop() + 任务与队列的 ASCII 数据流图
   ├─ modbus_master.h/.cpp ★核心★ 唯一总线持有者：请求队列 + FC03/04/06/10 + 重试 + 统计
   │                         + 慢车道 500ms + 快车道 2s（含窗口序号一致性校验）
   ├─ app_tasks.h/.cpp     队列、窗口三缓冲池、seqlock 快照、看门狗、写车道任务
   ├─ gui.h/.cpp           LVGL：数据页 / 波形页 / 诊断页 + 控件创建 + 更新入口 + 命令按钮
   ├─ oled.h/.cpp          ★路径 C★ SSD1306 128×64 I2C 本地显示：分类/振动 std/485 在线
   │                         （手写驱动，不依赖 LVGL，自己一条任务，只读共享数据）
   ├─ infer.h/.cpp         TFLite Micro：arena(PSRAM) + 标准化→int8量化→推理→反量化
   ├─ net.h/.cpp           WiFi + MQTT：telemetry 上行、cmd 下行转 g_cmd_queue
   └─ model_data.h         （**尚不存在**）由 tools/train_mlp.py 生成，接口契约见 infer.h
```

### 任务与核绑定（plan.md §7.2）

| 任务 | 核 | 优先级 | 栈(B) | 周期 | 职责 |
|---|---|---|---|---|---|
| `mb_bus` | 0 | 6 | 4096 | 事件 | **唯一总线持有者**：消费 `g_mb_req_queue`，串行收发 + 5s 心跳日志 |
| `mb_slow` | 0 | 5 | 3072 | 500ms | 读 `0x0000~0x001B`（28 个）→ 快照 + `g_ui_queue` |
| `mb_fast` | 0 | 5 | 4096 | 2s | 窗口头 → 5 块 → 序号校验 → `g_acq_queue`/`g_ui_queue`；再读 E 区 36 特征 |
| `cmd` | 0 | 5 | 3072 | 事件 | 消费 `g_cmd_queue` → FC06/FC10 |
| `infer` | 0 | 3 | 12288 | 事件 | 消费 `g_acq_queue` → TFLite Micro → `g_ui_queue` |
| `gui` | 1 | 4 | 8192 | 10ms | `lv_timer_handler()` + 排空 `g_ui_queue` |
| `net` | 0 | 3 | 8192 | 20ms | WiFi/MQTT、2s 上报、下行转 `g_cmd_queue` |
| `oled` | 0 | 2 | 4096 | 500ms | 刷 SSD1306（只读快照/推理结果/统计，**不碰总线**） |

三条队列：`g_acq_queue`（窗口/特征→推理）、`g_ui_queue`（状态→GUI）、`g_cmd_queue`（GUI/MQTT→写）。

### ★硬约束★
**只有 `task_modbus_bus` 允许操作 RS485 串口与 DE 引脚。** 其它任务一律通过
`mb_read_holding() / mb_write_single() / mb_write_multiple()`（内部走请求队列）或往
`g_cmd_queue` 投命令。GUI 任务**绝不**调用同步 API（会阻塞刷屏）。详见 `modbus_master.h` 顶部。

---

## 2. 编译与烧写

> 本仓库没有预装工具链，下面这些命令需要你在**已装好 PlatformIO 的机器**上执行。

```bash
cd firmware-esp32

# 编译（真机模式，USE_SIM_SLAVE=0）
pio run

# 烧写 + 串口监视器（115200）
pio run -t upload
pio device monitor            # 或者 pio run -t upload -t monitor
```

### 2.1 PSRAM 必须先配对（最常见的第一个坑）

`platformio.ini` 里：

```ini
board_build.arduino.memory_type = qio_opi     ; N16R8：16MB QIO Flash + 8MB OPI PSRAM
```

| 你的模组 | memory_type | 说明 |
|---|---|---|
| **N16R8**（本项目目标） | `qio_opi` | 16MB QIO Flash + 8MB Octal PSRAM |
| N16R8V / 部分自绘板 | `opi_opi` | Flash 也是 OPI |
| N8R2 / N8R8 | `qio_qspi` | 8MB Flash + Quad PSRAM |
| 无 PSRAM | — | **跑不动本项目**：LVGL 缓冲 + 48KB arena + 窗口池 + WiFi 内部 RAM 会爆 |

配错的症状：串口打印 `PSRAM ID read error`、`heap_caps_malloc(MALLOC_CAP_SPIRAM)` 返回 NULL、
或者干脆 boot loop。上电第一件事看串口有没有 `PSRAM: 有`。

### 2.2 串口监视器

`monitor_speed = 115200`，启用了 `esp32_exception_decoder`（把 `Guru Meditation Error` 的地址
翻译成函数名+行号，排 HardFault 必备）和 `time` 时间戳。**`monitor_rts = 0 / monitor_dtr = 0`**：
关掉自动握手，避免"一开监视器板子就复位"。

---

## 3. 先用 `USE_SIM_SLAVE` 调通主站，再上真 F411

`docs/03` 阶段 4 的第一句话就是"先脱机验证，再联调"。上位机这边对应的做法：

```
PC (tools/modbus_slave_sim.py, pymodbus 从机)  ⇄  USB 转 RS485  ⇄  ESP32-S3
```

```bash
# 1) 编一个"仿真从机"固件：离线判定放宽到 5 次、每帧打印原始十六进制
pio run -e esp32-s3-devkitc-1-sim -t upload
```

对应的地址：`cmd_queue` 的写命令、慢/快车道的时间参数都不变，只是：

| 项 | 真机 | `USE_SIM_SLAVE=1` |
|---|---|---|
| 串口打印 | 只打错误 | **每一帧 TX/RX 十六进制**，可直接和 `docs/02 §6` 的示例逐字节对拍 |
| 离线判定 | 连续 3 次超时 | 连续 5 次超时（PC 上 pymodbus 的响应抖动比真机大） |
| 波特率 | `config.h` 的 `MODBUS_BAUD`（默认 115200） | **必须和模拟器的 `--baud` 一致**。⚠ `tools/modbus_slave_sim.py` 的 `--baud` **默认是 9600** |

```bash
# PC 侧（tools/README.md 的推荐用法）
python tools/modbus_slave_sim.py --port COM7 --baud 115200 --addr 1 --rate 2 --sample-rate 50 --verbose
#   ↑ 若不加 --baud 115200 就是 9600，请相应把 config.h 的 MODBUS_BAUD 改成 9600
```

### 主站联调顺序建议

1. **自检先过**：上电看串口有没有
   `Modbus 自检通过（CRC 向量 + §6 示例帧逐字节一致）`。
   这行由 `mb_selftest()` 打出，它把 `docs/02 §3.1` 的 10 组 CRC 向量、
   `§6 ①` 的慢车道帧、`§6 ②` 的 5 个分块帧、`§6 ④⑤⑥` 的写帧全部构造出来逐字节比对。
   见到 ERROR 就先修协议层，别往下走。
2. **慢车道**：模拟器里 `0x0000~0x001B` 随时间变化 → 日志能看到读成功、TX/RX 计数在涨。
3. **快车道**：模拟器按 `--rate 2` 每 2s 更新窗口，`0x0100` 会置成
   `0x8000 | 100`（bit15 = 有效窗口就绪，低 8 位 = 有效样本数，与我们的检查一致）。
   故意在读到一半时改 `0x0101`，验证 `序号失配重读 N 次` 会增长、且不会拼出错窗口。
4. **写车道**：诊断页按钮（或 MQTT `cmd/`）→ 看模拟器的写入日志（`--verbose`）。
5. 全绿之后把 `USE_SIM_SLAVE` 换回 0、接真 F411。

> `tools/` 已经在仓库里了（`tools/README.md` 是它的用法总览）。
> 与上位机直接相关的两个脚本：
> - [`tools/modbus_slave_sim.py`](../tools/modbus_slave_sim.py) —— PC 当从机，让主站逻辑脱机跑通
> - [`tools/train_mlp.py`](../tools/train_mlp.py) —— 训练 + int8 量化 + 导出部署物（见 §5）

---

## 4. 引脚与接线（RS485 侧）

全部在 `include/config.h` §1 里改，代码里没有裸引脚号。

| 信号 | 默认 GPIO | 说明 |
|---|---|---|
| `RS485_TX_PIN` | 17 | → 485 模块 `DI` |
| `RS485_RX_PIN` | 18 | ← 485 模块 `RO` |
| `RS485_DE_PIN` | 16 | → 485 模块 `DE`（与 `~RE` 短接）。**必须避开 GPIO33~37（N16R8 的 PSRAM 占用）** |
| A / B / GND | — | A→A、B→B、**共地**；双绞；两端 120Ω（短线可只在主机端） |

**如果套件里的 485 模块是"自动收发"型（没有 DE 脚）**：
把 `RS485_USE_DE_PIN` 改成 `0`（`RS485_DE_PIN` 就不再被使用）。
不改的后果是固件去 bit-bang 一个空脚，表现为"能发不能收"。

### 4.1 本地 OLED（路径 C，SSD1306 128×64 I2C）

| 信号 | GPIO | 说明 |
|---|---|---|
| `OLED_PIN_SDA` | 8 | → 模块 `SDA`（很多模块丝印是 `SDA`/`SCL`，个别是 `D0`/`D1`，别接反） |
| `OLED_PIN_SCL` | 9 | → 模块 `SCL` |
| `OLED_I2C_ADDR` | — | `0x3C`（模块背面 `SA0` 悬空/接地都是 0x3C；拉高才是 0x3D） |
| VCC / GND | — | **3V3**（SSD1306 模块板载稳压，接 5V 一般也能亮，但本项目统一 3V3）/ 共地 |

- 只用 2 根线、1 个固定协议，与 LVGL 屏（路径 A/B）**互不冲突，可以同时接**：
  `Wire` 只被 `task_oled` 使用，I2C 初始化在 `setup()` 里单线程完成。
- **没插屏也不影响任何功能**：`oled_init()` 返回 false → `task_oled` 每 10s 重试一次
  （所以支持"跑着的时候再插上"），期间日志每 60s 提示一次接线检查；总线/推理/网络照常。
- 屏上显示（8 行 × 21 列 ASCII，行 0 是 2 倍字号标题）：

  ```
  GW-485
  CLS:normal   87%      ← 分类结果（推理；未部署模型时显示 no model）
  VIB std   42 pp  180  ← 振动强度：三轴加速度 std 最大值 / 峰峰值最大值
  SEQ win 1234          ← 窗口序号 0x0101
  485:ONLINE    to=0    ← 485 在线状态（连续 3 次超时→OFFLINE）/ 连续超时计数
  TX  231 RX    0
  ENV  25.3C  60%       ← 温度/湿度（证明慢车道数据在更新）
  ```

  `~` 出现在行尾 = 该行数据超过 `OLED_STALE_MS`(15s) 没更新（别把旧值当实时值）。
- 中文字库不在这块屏的预算里（Flash 只留了 ASCII 5×7 点阵），所以标签用英文缩写。

---

## 5. 模型部署（TFLite Micro 的输入从哪来）

`src/infer.h` 顶部的接口契约是**照着 [`tools/train_mlp.py`](../tools/train_mlp.py)
的真实产出写的**（不是猜的）：

```bash
#                   ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 这个参数不能省！
python tools/train_mlp.py --data data --split data/split.json \
       --out model_out --feat-layout per-feature --epochs 200
```

> ### ⚠⚠ 最容易踩的坑：`--feat-layout` 必须显式指定为 `per-feature`
>
> - `docs/02 §5-E` 规定 E 区是「**每个特征** 6 轴连续」：
>   `0x0400~0x0405` 6 轴均值 → `0x0406~0x040B` 6 轴标准差 → … → `0x0423`
>   （`ax_mean, ay_mean, az_mean, gx_mean, gy_mean, gz_mean, ax_std, …`）
> - `tools/train_mlp.py` 的**默认** `--feat-layout` 是 **`per-axis`**（该文件第 73 行），
>   即「每**轴** 6 个特征连续」（`ax_mean, ax_std, ax_p2p, …, ay_mean, …`）——
>   和 E 区顺序是**转置**关系
> - 两者都是 36 维：模型能加载、能推理、不报任何错，**只是准确率掉到随机水平**
>
> 所以：**训练时必须加 `--feat-layout per-feature`**。
> `infer_init()` 会用 `norm_params.h` 里的 `FEATURE_LAYOUT` 宏做**硬校验**，
> 对不上直接拒绝启动推理并打 ERROR，不会让你带着错位模型往下跑。

`model_out/` 里会得到：

| 文件 | 去哪 | 内容 |
|---|---|---|
| `model_data.h` / `model_data.cc` | **拷进 `firmware-esp32/src/`** | `extern "C"`：`model_data[]`（`alignas(16)`）+ `model_data_len` |
| `norm_params.h` / `norm_params.cc` | **拷进 `firmware-esp32/src/`** | `#define FEATURE_DIM 36`、`FEATURE_LAYOUT "per-feature"`、`MODEL_INPUT_SCALE`/`ZERO_POINT`、`MODEL_OUTPUT_SCALE`/`ZERO_POINT`、`feature_mean[36]`、`feature_std[36]` |
| `model.tflite` / `norm_params.py` | 留在 PC | 给 `verify_tflite.py` 用 |
| `model_info.json` | 留在 PC | `class_names`（**字母序**）—— 类别名的唯一出处 |
| `confusion_matrix.txt` | 留在 PC | PC 端精度留档 |

三条容易踩的点（都已在 `infer.h` 里写明）：

1. 这两个头文件是 `extern "C"` 的，**必须 `#include`**，不能自己在固件里写
   `extern const unsigned char model_data[];`（那会变成 C++ 链接 → undefined reference）。
2. `train_mlp.py` **不把类别名导出到 C 头**。所以：
   - 类别数从**输出张量的维度**读（运行时权威值）；
   - 类别名在 `infer.cpp` 的 `kClassLabels[]`，必须按 `model_info.json` 的
     `class_names`（`sorted(set(labels))`，字母序）逐项对齐，条数 = `INFER_MAX_CLASSES`。
3. 没有这两个头文件时固件**照样能编、能烧、能跑**（`__has_include` 兜底，
   推理恒返回 false，GUI 显示"模型未部署"）—— `plan.md §8 A` 表就是"先交双机联调，
   模型晚两周"。**不要**先建空壳头文件占位，那会把兜底逻辑变成假模型。

启动时会做四项自检（都是真实存在的坑）：
- `static_assert(FEATURE_DIM == INFER_FEATURE_DIM)`：训练用的维度 ≠ E 区长度 → 直接编译报错
- **`FEATURE_LAYOUT` 必须是 `"per-feature"`** → 否则推理硬失败（见上面的坑）
- 输入张量维度必须是 36；输出类别数必须 ≤ `INFER_MAX_CLASSES`
- **量化参数交叉校验**：张量里的 `scale/zero_point` 与 `norm_params.h` 的宏对不上 →
  报 ERROR，八成是把不同训练的 `model_data.h` 和 `norm_params.h` 混着拷了

### 5.1 ⚠ 还缺一个接口：`verify_tflite.py` 的串口对拍协议

[`tools/verify_tflite.py`](../tools/verify_tflite.py) 要求 MCU 侧实现一条串口协议
（该文件第 17~34 行定死，**目前固件里还没有实现**，协议原文抄在 `src/infer.h` 末尾）：

```
PC → MCU :  0xA5 0x01 <count> <36 个 int8> <checksum>
MCU → PC :  0x5A 0x01 0x24 <36 个 int8> <checksum>          (int8 模式，默认)
            0x5A 0x02 0x03 <class_id> <conf_lo> <conf_hi> <cksum>   (class 模式)
```

要点：走**调试串口**（不是 485 总线），收到的 36 个 int8 是**已经量化好的**，
必须直接灌进输入张量 —— 固件侧对应 `infer_run_quantized()`（已实现），
**不能**再调 `infer_run()` 做一次 `(x-mean)/std`（那就必然对不上）。
没有这条协议时 `verify_tflite.py --pc-output-json/--print-input` 仍可离线对拍，不是阻塞项。

---

## 6. 内存预算（为什么这么分）

| 用途 | 位置 | 大小 | 理由 |
|---|---|---|---|
| 窗口三缓冲池 | **PSRAM** | 3 × ~1.2KB | 大块、写一次读两次，内部 RAM 让给 WiFi |
| TFLite tensor arena | **PSRAM** | 48KB（`TFLM_ARENA_SIZE`） | 推理期间集中访问，平时闲置；内部 RAM 挤不下 |
| 标准化/量化中间缓冲 | **内部 RAM（栈）** | 144B | PSRAM 延迟高，先在内部算完再一次性拷进张量 |
| LVGL 绘制缓冲 | **内部 DMA RAM** | 320×40×2 = 25.6KB | PSRAM 缓冲做 SPI DMA 更慢且和 WiFi 抢 cache |
| WiFi / LWIP | 内部 | ~60KB | 无法挪走 |
| 7 个任务栈 | 内部 | ~40KB | 合计，见上表 |
| 模型 | Flash | <100KB | `xxd -i` 成 C 数组随固件走 |

想收紧 arena：跑起来后看串口打印的 `arena 用量 N 字节`（来自 `arena_used_bytes()`），
按实际用量的 1.2~1.5 倍设 `TFLM_ARENA_SIZE`。
想收紧各任务栈：用 `uxTaskGetStackHighWaterMark()` 量余量，留 30% 以上。

---

## 7. 待确认清单（会直接改变代码取值）

来源：`plan.md` §11 + `docs/03` 阶段 0 + `docs/04-plan-15days.md`。

| 编号 | 待确认项 | 影响 | 怎么定 |
|---|---|---|---|
| **TBD-1** | **屏幕驱动 IC / 分辨率 / 颜色格式 / 是否电容触摸** | `config.h` 的 `LVGL_HOR_RES`/`LVGL_VER_RES`/`LVGL_COLOR_DEPTH`/`GUI_USE_ESP_LVGL_PORT`；`gui.cpp` 的显示移植路径 | 看屏背面丝印（ILI9341 / ST7789 / ST7796 / GC9A01…）+ 触摸芯片（XPT2046 / FT6236 / GT911）。**这是当前最大的一个未知，界面部分就等它。** 选型约束见 `docs/04` §屏幕：SPI 四线 / 3.3V / 带驱动 IC，不要 8080 或 RGB 并口、不要 MIPI、不要裸屏 |
| **TBD-2** | 485 模块有没有 DE/RE 脚 | `RS485_USE_DE_PIN` | 模块上有 `DE`/`RE`/`DIR` 丝印就是手写时序型 → 置 1；否则自动收发 → 置 0 |
| **TBD-3** | ESP32-S3 模组型号（N16R8 / N8R2 / 无 PSRAM） | `platformio.ini` 的 `memory_type` | 看模组丝印；无 PSRAM 跑不动本项目 |
| **TBD-4** | 485 的 TX/RX 最终落哪两个 GPIO | `RS485_TX_PIN`/`RS485_RX_PIN` | 避开 GPIO33~37（PSRAM）、GPIO19/20（原生 USB）、屏幕/触摸占用脚 |
| **TBD-5** | 是否仍按 15 天排期（`docs/04`）跑 | 决定要不要现在就塞 MQTT/模型 | `docs/04` 的数据集+训练路线**不依赖 S3**；模型可以晚于双机联调 |
| **TBD-6** | 主模型任务是否确认为「电机振动异常分类」，类别名是什么 | `infer.cpp` 的 `kClassLabels[]`（必须对齐 `model_info.json` 的 `class_names`）、GUI 波形页语义 | `plan.md §4-B` 是推荐主线（正常/不平衡/松动），手势作副模式 |
| **TBD-9** | E 区 0x0400~0x0423 的**实际写入顺序**（F411 侧） | 训练时必须用 `--feat-layout per-feature` 才与 `docs/02 §5-E` 一致；固件已做硬校验 | 以 `docs/02 §5-E` 为准（每特征 6 轴连续）。**若 F411 侧实现成别致顺序，必须同步改 docs/02 与 `--feat-layout`**，不要在固件里偷偷转置 |
| **TBD-7** | 与 485 联调时的波特率（9600 还是 115200） | `MODBUS_BAUD` 与模拟器/从机的 `--baud`、`0x002D` | 先在 9600 调通再升 115200（`docs/02 §1`）；注意 `modbus_slave_sim.py` 默认就是 9600 |
| **TBD-8** | 是否要实现 `verify_tflite.py` 的串口对拍协议 | 需要一个调试串口上的收帧状态机（协议见 §5.1） | 部署模型之后做对拍时再补；不实现也能用离线对拍 |

---

## 8. 本骨架里所有 `TODO` / 待补项

按文件列出，方便逐条清账。

### `platformio.ini`
- [ ] 按实际模组改 `board_build.arduino.memory_type`（TBD-3）
- [ ] TFT_eSPI / LovyanGFX **二选一**，把另一个注释掉（TBD-1）
- [ ] 若用 S3 原生 USB 看日志，取消 `-DARDUINO_USB_CDC_ON_BOOT=1` 的注释

### `include/config.h`
- [ ] `WIFI_SSID` / `WIFI_PASS` / `MQTT_BROKER` 填真实值
- [ ] `LVGL_HOR_RES` / `LVGL_VER_RES` / `GUI_USE_ESP_LVGL_PORT` 按屏定（TBD-1）
- [ ] `RS485_USE_DE_PIN` / `RS485_DE_PIN` 按 485 模块定（TBD-2）
- [ ] `RS485_TX_PIN` / `RS485_RX_PIN` 按实际接线定（TBD-4）
- [ ] `MODBUS_BAUD` 与从机/模拟器对齐（TBD-7）

### `src/gui.cpp`
- [ ] **`gui_port_init()`：显示 + 触摸移植（两条互斥路径，全是 TODO）** ← 界面能不能亮就看这一步
      - 路径 A `esp_lvgl_port`（一体板）：`esp_lvgl_port_init` + `lvgl_port_add_disp/touch`，
        并把 `gui_tick()` 改成 `lvgl_port_lock/unlock` 包住 LVGL 调用
      - 路径 B 自绘（TFT_eSPI/LovyanGFX）：画缓冲（内部 DMA 优先）→ `lv_disp_drv_register`
        → `lv_indev_drv_register`；`flush_cb` 里必须调 `lv_disp_flush_ready()`
- [ ] 中文字库（默认字体只有 ASCII，中文会显示成方框）
- [ ] 字号（需在 `lv_conf.h` 打开 `LV_FONT_MONTSERRAT_12/16/20`）
- [ ] 数据页：按 `0x001A` 报警位 / `0x0027~0x0029` 阈值给卡片染色
- [ ] 波形页：加陀螺仪曲线、暂停/继续按钮、序号失配窗口标黄
- [ ] 推理结果：多类别概率并排显示（对比实验要同时看三类）
- [ ] 诊断页：离线时灰显控件 + 红字（`docs/03` 阶段 4 验收项）
- [ ] 舵机/RGB 的"使能 + 参数"两条命令不是原子的（见对应注释），必要时改一条 FC10

### `src/infer.cpp` / `src/infer.h`
- [ ] **`kClassLabels[]` 改成 `model_info.json` 的 `class_names`（字母序）**（TBD-6）
- [ ] **训练时必须加 `--feat-layout per-feature`**（TBD-9）—— 固件已硬校验，漏了会拒绝启动推理
- [ ] **`verify_tflite.py` 的串口对拍协议**（协议原文见 `src/infer.h` 末尾 / README §5.1）
      入口函数 `infer_run_quantized()` 已就绪，缺的是收帧状态机
- [ ] **1D-CNN 路径**（原始 100×6 窗口）—— 第二个模型 + 输入张量填充；
      需要把 `infer_run()` 拆成 `fill_input_*()` + `invoke_common()`
- [ ] 报警联动：置信度超阈值 → 蜂鸣器 + `alarm/` 主题（阈值待定）
- [ ] `infer_deinit()` 的干净释放（v0.4 换模型/OTA 时才需要）
- [ ] 用实测 `arena_used_bytes()` 收紧 `TFLM_ARENA_SIZE`

### `src/net.cpp` / 命令链路
- [ ] `cmd_msg_t` 的执行结果回传 GUI（按钮回弹 + 失败提示）
- [ ] MQTT 下行的鉴权/白名单（现在任何能发 `cmd/` 的客户端都能控执行器）
- [ ] 报警主题 `alarm/{id}` 的触发规则（v0.4）
- [ ] 下行 JSON 的键位约定（`reg`/`value`/`values`/`fan`/`ctrl_set`/`ctrl_clr`/
      `safe_shutdown`/`force_window`）需要写进 `tools/` 的文档并加测试

### `src/app_tasks.cpp`
- [ ] 控制字 RMW 与 F411 本地自治的竞争（见 `ctrl1_rmw_write()` 注释；
      建议从机提供一个"置位/清位"命令寄存器，或改用 FC10 连写完整控制字）

### 外部依赖（在 `firmware-esp32/` 之外，本次未改动）
- [x] `tools/modbus_slave_sim.py`（已存在）
- [x] `tools/train_mlp.py`（已存在）→ 产出要**手工拷进 `src/`**
- [ ] 把模型部署物拷贝这一步自动化（脚本或 `extra_scripts`），避免手抄出错

---

## 9. 排错速查

| 现象 | 先看这里 |
|---|---|
| **上电后除启动横幅外串口一行都没有** | 先看有没有 `总线心跳`（每 5s 一行）。**没有心跳** = 持有者任务被卡住；**有心跳但 `TX` 不涨** = 生产端没进来；**`TX` 在涨** = 总线好着呢，是"没有从机应答"而已（`超时` 会跟着涨）。见下面 §9.1 |
| 上电就 boot loop / `PSRAM ID read error` | `platformio.ini` 的 `memory_type`（TBD-3） |
| 自检打 ERROR | `mb_selftest()` 的日志会指出是 CRC 向量还是哪个分块帧不一致 |
| 全是超时，一帧都收不到 | 485 的 A/B 有没有接反；DE 引脚配错（自动收发模块却置了 `RS485_USE_DE_PIN=1`）；波特率与从机不一致 |
| **偶尔通、大部分超时（约 1/4 成功率）** | 先做**字节级对账**（两侧同时看）：`[诊断] 字节累计：已发/已收` 对比 F411 的 `[mb] rx=…(…B) tx=…(…B)`。**发送 ≠ 对方接收 → 物理层丢字节**（120Ω 终端/A-B 双绞/共地/线长，见 §9.2 表）；**对得上但 tx=0 → 协议层**（地址/CRC 静默丢弃） |
| 偶尔 CRC 错 | 帧间静默不够（`MODBUS_T35_US`）；电机/舵机与 485 线平行走线；未共地 |
| 慢车道读到全 0 | 通信没通 vs 传感器没接是两回事：先看诊断页 `RX` 计数有没有涨 |
| 窗口读回来是"缝合怪" | 看 `序号失配重读 N 次`（诊断页 / MQTT `mb_seqmiss`）—— 这是 `docs/02 §7` 机制在起作用，不是 bug |
| 界面刷不出来 | `gui_port_init()` 的 TODO 还没补（TBD-1）；或者 `flush_cb` 里漏了 `lv_disp_flush_ready()` |
| OLED 一直不亮 | 日志里有没有 `OLED 就绪`。没有就查：VCC=3V3 / GND / SDA=GPIO8 / SCL=GPIO9 / 地址 0x3C；`task_oled` 每 10s 会重试并在日志里提示 |
| WiFi 连不上但 CPU 很忙 | 我们的任务优先级抢过了 WiFi（`TASK_PRIO_*` 必须低于 WiFi 的 18~23） |
| 任务创建失败 | 内部 RAM 不够：把 LVGL 缓冲 / tensor arena 挪到 PSRAM，或收紧任务栈 |
| 随机复位 | 看门狗在复位你：某个任务登记后没在 5s 内 `app_wdt_feed()` |
| 编译报 `FEATURE_DIM ... 不一致` | `norm_params.h` 的 `FEATURE_DIM` ≠ `INFER_FEATURE_DIM`：训练输入维度和 E 区 0x0400~0x0423 对不上 |
| 启动报"特征布局不匹配" | 训练时漏了 `--feat-layout per-feature`（默认是 `per-axis`，与 `docs/02 §5-E` 转置）→ 重新训练导出，**不要**注释掉这段检查 |
| 启动报"量化参数不匹配" | 把**不同训练**的 `model_data.h` 和 `norm_params.h` 混着拷了 |
| 推理结果总是同一个类 | 输入没填进去（检查 `INFER_FEATURE_DIM`）或标准化参数用错；先用 `verify_tflite.py --print-input` 拿一组已知输入离线比 |
| GUI 上类别名显示成 `classN` | `infer.cpp` 的 `kClassLabels[]` 还没按 `model_info.json` 的 `class_names`（字母序）改 |
| 模型没部署时界面不亮推理区 | 这是**预期行为**（`__has_include` 兜底），拷入 `model_data.h` + `norm_params.h` 后重启即可 |

### 9.1 为什么"总线上什么日志都没有"是个陷阱（已修，留档）

**症状**：烧完固件上电，串口只有启动横幅，之后 20 秒以上一行都没有、不重启、不崩溃；
看起来像"Modbus 任务根本没在轮询、`g_mb_req_queue` 一直是空的"。

**真相**：轮询一直在跑。当时**成功路径一行日志都不打**，而没有从机应答时：
- `mb_execute()` 走 silent 分支（`stats_on_fail()` 只加计数不打日志）；
- 在线状态翻转 → `ui_post()` → 但 `gui_tick()` 开头就是
  `if (!s_ui_ready) continue;`（LVGL 屏还没落地 → `s_ui_ready=false`），
  消息被静默丢弃，连 `gui_set_online()` 的 `APP_LOGI` 都到不了；
- 慢车道读失败只 `continue`，一个字的日志都没有；
- 快车道的「采样序号未变化」告警在没读成功时永远不会触发。

于是**整个系统在串口上表现为"死了"，实际 CPU 跑得好好的**。

**顺带查出的真 bug（`mb_call()` 的等待预算算错）**：请求要在队列里排队，
但预算只算了"它自己"那一份：
`budget = timeout_ms × (retries+1) + 30ms` → 慢车道只有 180ms。
实测单个请求（3 次尝试全超时）要 **153~183ms**，再排到快车道那 7 个请求后面
（头 + 5 块 + 序号校验）就必然超预算 → 慢车道/快车道**大量请求被误判成"等不到完成通知"**，
上层看到的就是"队列里的请求像没人处理"。修法见 `config.h` 的
`MODBUS_MAX_SINGLE_REQ_MS` 与 `mb_call()`：预算按
`(入队时排在前面的请求数 + 1) × MODBUS_MAX_SINGLE_REQ_MS + SLACK` 算，并封顶。

**现在的观测口径**（照这三行就能把故障域分开）：
1. `总线心跳：TX … RX … 超时 … 等待处理 …` —— 每 5s 一行，**证明总线活着**；
2. `485 节点离线：连续 N 次超时` —— 状态翻转时会打（不再只走 UI 队列）；
3. `慢车道读失败（第 N 次）：TIMEOUT`（每 10 次节流一次）+ `等待总线完成通知超时`
   —— 前者=从机没应答，后者=持有者任务真被卡住（两者的处置完全不同）。
4. `[诊断] 收帧失败定性：静默(线上没字节) N 次 / 有字节但不成帧 M 次`
   —— **静默**为主：从机没上电 / 地址不对 / A-B 没接 / 它把我们静默丢弃了；
   **垃圾字节**为主：波特率与从机不一致 / 干扰 / 半双工时序。两者修法相反。

### 9.2 实机联调实测：115200 vs 9600（已定性为**位速率电气裕量**问题）

2026-10 在某台 ESP32-S3 + 一块真 F411 上实测（线长 15cm、已共地、**无终端电阻**）。

**115200 下的现象**：主站成功率只有 ~26%，失败**全部是静默**（线上一个字节都没有），
`CRC错 0 / 异常 0`（收到的帧全对）。两侧字节级对账（当时 ESP32→F411 丢 32%、F411→ESP32 丢 59%）
指向物理层，而 15cm 短线排除了反射、已共地排除了共模问题 —— 只剩"位速率裕量"。

**降到 9600 后的结果**（两端同改，见下）：

| 指标 | 115200 | 9600（配好周期后） |
|---|---|---|
| 节点在线状态 | 反复 上线/离线 闪 | **稳定在线** |
| 慢车道成功率 | ~26% | 首 15 次 **100%**，长跑 ~65% |
| 从机侧 `drop=0 long=0 0len=0` | — | **是**（没有队列溢出、没有超长帧） |
| 从机收到的帧类型 | 只有短帧 | **已包含窗口分块请求**（`01 03 01 81 …`），快车道开始工作 |
| 看门狗复位 / 队列满 | 无 | **0 / 0** |

**结论：这是位速率电气裕量问题**（驱动器/负载/相位），**不是**软件、不是线长、不是共地。
15cm 短线 + 无终端电阻的组合在 115200 下裕量不足，9600 就够。
→ 想回到 115200 需要 **120Ω 终端电阻**（两端各一个）+ 更好的收发器模块；
在那之前，**9600 是可长期使用的可用配置**。

> ⚠️⚠️ **上面这段结论在 2026-10 被推翻了，保留原文作为"错误推理"的反面教材。**
> 真实根因是**主站事务间隔不够**（与波特率、终端电阻、信号质量**全都无关**），
> 已修复，并已成功切回 **115200**（首试成功率 **100%**）。
> 详见下面的 **§9.2.1**。教训：**"降速有效"不等于"根因是位速率"** ——
> 降速同时也改变了时间尺度，掩盖了时序问题。

#### 9600 下必须同步改的 5 处（漏一处就会误判成"降速更糟"）

| 位置 | 值 | 为什么必须改 |
|---|---|---|
| `MODBUS_BAUD` | 9600 | 主机波特率 |
| `cubemx/Src/usart.c` 的 `huart2.Init.BaudRate` | 9600 | 从机波特率（`0x002D` 只是记录值，改它没用） |
| `MODBUS_INTER_FRAME_GUARD_US` | `MODBUS_T35_US(MODBUS_BAUD)` | 原来固定 200µs，9600 下只有 0.2 个字符时间，从机（IDLE 判帧尾）会把两帧粘起来 |
| `MODBUS_TIMEOUT_MS_SLOW/_FAST/_WRITE` | 400/700/400 | 9600 下一帧 260 字节光"发完"就 298ms |
| `MODBUS_MAX_SINGLE_REQ_MS` / `MODBUS_NOTIFY_BUDGET_MAX_MS` | 1000 / 1200 | 排队最坏耗时跟着线上时间涨 |
| `MODBUS_SLOW_PERIOD_MS` / `MODBUS_FAST_PERIOD_MS` | 1000 / 15000 | **不改这两个会被自己的队列打满**：一次"全超时"请求 = 超时×3 = 1.2s > 500ms 周期；快车道 7 个请求最坏 14.7s > 10s 周期 |

#### ⚠ 两个降速时踩到的真坑（都已修，留档）

1. **长周期任务不喂狗 → 看门狗复位**：任务一旦 `app_wdt_register()` 登记，
   **两次喂狗间隔超 5s 就 panic**。`mb_fast` 原本一口气 `vTaskDelayUntil(10000ms)`
   睡完，直接触发 `task_wdt: - mb_fast (CPU 0)` + `rst:0xc`，而且它自己的第一行
   日志都来不及打，看着像"任务卡死"。
   → 修法：按 `BUS_WDT_FEED_PERIOD_MS`(500ms) **切片睡眠**，每片喂狗；
   窗口读的每一步之间也补喂狗（`window_read_once` 里）。
   **115200 时代周期 2s < 5s，所以这个坑一直藏着。**
2. **周期必须大于"最坏情况下一次完整操作的时间"**：否则请求入队比出队快，
   队列必满。判据就是串口里的 `总线请求队列满` + 心跳里的 `等待处理` 一直 >0。

#### 仍未定论的一点（诚实记录）

**现象**：9600 配好之后，长跑稳定在
```
总线心跳：TX 56  RX 35  超时 21  重试 21  异常 0  CRC错 0  在线  等待处理 0
```
即 **第一次尝试成功率 ~63%，而"重试次数恰好等于超时次数" ⇒ 每次重试都成功**。
失败**全部是静默**（`有字节但不成帧 0`），从机侧 `CRC错/parse0` 都不涨
（`parse0` 冻在 844 不动 —— 那是**启动瞬态**，与链路质量无关）。

**已经排除的解释**（都做过实验）：
| 假设 | 实验 | 结果 |
|---|---|---|
| 从机应答太慢，超时给短了 | 超时 400ms → 700ms | 重试数没变 ✗ |
| 半双工撞车（下一帧顶到从机还没释放的驱动器上） | 加 100ms 事务后保护间隔（`MODBUS_TXN_GUARD_US`） | 重试数没变 ✗ |
| A/B 相位边缘 | 对调 A/B | 直接彻底离线（`RX 0`）→ 原接法是对的 ✗ |
| 线接触不良 | 重新插紧 | `CRC错` 70 → 0、`parse0` 停止增长 ✓（这一条是真因，已修） |
| 空闲差分死区导致幽灵帧 | 由 `parse0` 是**一次性跳变**而非持续增长否定 | ✗ |

**剩下最可能的解释**：从机的响应**偶尔根本发不出来**（不是发得慢），或者主站
第一个请求**偶尔根本没被从机收到且没留下任何痕迹**。要定这个性，下一步应该在
**从机侧**加一个"每 5 秒的请求到达时间戳直方图"：看那 37% 的失败是集中在某个
固定时刻（比如采样任务/printf 锁竞争的窗口），还是均匀分布。
判据：
- **集中** → 从机侧调度竞争（`printf` 互斥锁、50Hz 采样任务），把 Modbus 任务
  提到更高优先级、或在关键路径上禁掉日志即可。
- **均匀** → 位级裕量仍然不够，需要 120Ω 终端电阻或换模块。

**✅ 反过来说，已经确定的事**：9600 下从机**对自己发出的响应从不丢**
（`CRC错 0`、`异常 0`、`有字节但不成帧 0`），节点稳定在线、看门狗 0 次复位、
队列不堆积。**"位速率裕量"这个结论仍然成立**（115200 首试 25% → 9600 首试 63%），
只是它**不是全部原因**。

---

### 9.2.1 ★ 已解决：首试成功率 58% → **100%**（2026-10）★

**结论：问题出在"请求方向"，不在"响应方向"；根因是主站事务间隔不够。**
修法只有一行（`config.h`）：

```c
#define MODBUS_TXN_GUARD_US   (125 * MODBUS_T35_US(MODBUS_BAUD))   /* 原 25 */
```
9600 下从 ≈100ms 提到 **≈500ms**。实测 **0 超时 / 0 重试 / 100%**。

#### 定位方法（这次能成功的关键）

新增工具 **`tools/measure_bus_pair.ps1`**：**同时**打开主站(COM6)和从站(COM8)，
各取两次读数，用**增量**对比。

> ⚠ 为什么必须用增量：两侧计数器是各自累计的，运行时长不同
> （实测从站跑过 8.6 小时、主站可能刚重启）。直接比绝对值会得出荒谬结论 ——
> 我一开始就被"从站收到 29245 个请求、主站只发了 3008 个"误导过。

对账结果（100 秒窗口，同一时间段）：

| 指标 | 值 |
|---|---|
| 主站发送尝试 | 176 |
| **从站收到请求** | **100** ← 少了 76 次 |
| 从站静默丢弃 `parse0` | **0**（收到的全应答了） |
| 从站发出响应 | 9278 字节 |
| 主站收到响应 | 102 |

**"从站只收到 100 个请求、却对应上主站收到的 102 个响应"** ——
这两个数吻合得太好，说明**丢的 76 次是请求压根没到从站**。

#### 根因

从机发送响应时会把接收器关掉（`DIR` 拉高 → 模块内部 `~RE` 同时拉高，
见野火小智模块手册「DIR = 输出输入方向使能端」）。它要等 **TC 标志**
（最后一字节移出移位寄存器）才放开总线、重新武装 UART 接收 ——
这段窗口在 9600 下 ≈ **66ms（响应）+ 处理时间**。

主站只等 100ms 就发下一个请求，撞上这个窗口 → **整帧被静默丢弃**；
等 400ms 超时后重试时从机早已就绪 → **必然成功**。

这完美解释了那个"100% 复现"的现象：
```
重试数 == 超时数        ← 每次首试都失败、重试都成功
失败全记为"静默"        ← 帧根本没进从机的接收器
CRC 错 ≈ 0             ← 信号质量本身很好（所以不是电气问题）
```

#### 调参实测（每档 200 秒、约 100 次事务）

| `MODBUS_TXN_GUARD_US` | 9600 下间隔 | 首试成功率 |
|---|---|---|
| 25 倍 | ≈100ms | 58% ← 原值 |
| 60 倍 | ≈240ms | 73% |
| 75 倍 | ≈300ms | 73% |
| **125 倍** | **≈500ms** | **100%** ✓ |
| 150 倍 | ≈600ms | 100%（等价，取更小者） |

#### ⚠️ 二次修正：这个间隔**不能随波特率缩放**（又踩了一次）

定稿 9600 之后切 115200，我把它写成了**与波特率成比例**：

```c
#define MODBUS_TXN_GUARD_US  (125 * MODBUS_T35_US(MODBUS_BAUD))   /* 错 */
```
9600 下它是 500ms（好用），**115200 下变成 42ms → 立刻退回 50% 失败**。

**原因**：这个间隔要覆盖的是从机的**固定开销**，它不随波特率缩小 ——

| 组成部分 | 与波特率的关系 | 9600 | 115200 |
|---|---|---|---|
| 响应传输时间 | **成正比** | 66ms | 6ms |
| RTOS 调度延迟 + 响应构建 + 重新武装接收 | **固定** | ← 大头 → | ← 大头 → |

⇒ 间隔必须按**绝对时间**给。最终定为固定值：

```c
#define MODBUS_TXN_GUARD_US  400000UL    /* 400ms，与波特率无关 */
```

**实测标定（两端同一份配置）**：

| 波特率 | 间隔 | 首试成功率 |
|---|---|---|
| 9600 | 500ms | **100%**（870 次事务，超时 0 / 重试 0 / CRC错 0）|
| 115200 | 42ms（125×T35）| **50%** ✗ ← 这个坑 |
| **115200** | **400ms** | **100%**（477 次事务，超时 0 / 重试 0 / CRC错 0）✓ |

#### 切回 115200 的同步改动清单（**漏一处就一帧都收不到**）

| 位置 | 值 |
|---|---|
| `firmware-esp32/include/config.h` `MODBUS_BAUD` | 115200 |
| `firmware-esp32/include/config.h` `MODBUS_TXN_GUARD_US` | **400000UL**（固定，不乘 T35）|
| `firmware-esp32/include/config.h` `MODBUS_TIMEOUT_MS_SLOW/_FAST/_WRITE` | 150 / 200 / 150 |
| `firmware-esp32/include/config.h` `MODBUS_MAX_SINGLE_REQ_MS` / `_NOTIFY_BUDGET_MAX_MS` | 200 / 600 |
| `firmware-esp32/include/config.h` `MODBUS_SLOW_PERIOD_MS` / `_FAST_PERIOD_MS` | **1000** / 2000 |
| `firmware-stm32/cubemx/Src/usart.c` `huart2.Init.BaudRate` | 115200 |
| `firmware-stm32/App/regbank.c` `DEF_BAUD_CODE` | **4**（=115200，见 `docs/02` 编码表 0=9600…4=115200）|
| `firmware-stm32/App/app_main.c` `MB_TX_TIMEOUT_MS` | 100（260 字节 @115200 ≈ 25ms 的 4 倍余量）|

> ⚠️ 慢车道周期不能设 500ms：**400ms 间隔 + 事务本身 > 500ms**，队列会持续积压。
> 设 1000ms 后队列深度恒定在 2（快车道的分块请求在排队），**不增长即无害**，
> `队列满` 告警 0 次。

#### 走过的弯路（都是被"单侧日志"误导的）

| 曾经的假设 | 反证 |
|---|---|
| 位速率电气裕量不足 | 9600 已经比 115200 好，但仍有 42% 失败 |
| 需要 120Ω 终端电阻 | **加上后 58.2% → 52.6%，没有改善**；CRC 错仅 2/3020（0.07%） |
| 从机应答慢 | 超时 400→700ms，重试数不变 |
| 半双工撞车（事务后间隔） | 加 100ms **有改善但不够**（这就是线索，只是没加大到足够） |
| 主站 DE 切回时机 / RX 残留 | 加 `rx_drain()` 后 59.6%，几乎无变化 |
| A/B 相位反 | 对调后彻底离线 → 原接法正确 |

**两条最大的教训**：
1. **只盯一侧日志永远查不出"对端有没有收到"** ⇒ 必须**双口同时抓 + 用增量对比**
2. **"降速有效"不等于"根因是位速率"** —— 降速同时改变了时间尺度，
   会把**时序类**问题一起掩盖。这次就是被它骗了很久（120Ω 电阻白买了）

#### 遗留与后续

- 115200 下这个宏只有 `125 × 334µs ≈ 42ms`，**代价几乎消失** ——
  所以"装好 120Ω 后切回 115200"比看起来更值（README §9.2 有 6 处耦合改法）
- 模块自带 **120Ω 拨码开关**（野火小智模块手册写明"近距离可关闭"）。
  当前拨码**关着**，外接电阻已可移除 —— 因为实测证明丢帧与终端匹配无关
- 如果要**根治**而不是用间隔规避：应让从机在发送完成后**更快重新武装接收**
  （比如把 `task_modbus` 提到更高优先级，排除 50Hz 采样任务的调度延迟）。
  当前用间隔规避已经够用，暂不做


---

### 9.3 SPI 屏点亮全过程（2026-10，踩了 5 个坑，全部留档）

屏：2.4" SPI 模块 / **ILI9341** / 卖家页面与实物丝印一致 / **物理 320×240**。
代码：`gui.cpp` 路径 B（LovyanGFX）。**这一节是排错索引，遇到屏的问题先看这里。**

#### 坑 1：TFT_eSPI 在本工程的核心版本下必崩 ✗ → 换 LovyanGFX

```
Guru Meditation Error: Core 1 panic'ed (StoreProhibited)
EXCVADDR: 0x00000010
Backtrace: writecommand → begin_tft_write → TFT_eSPI::init
```
崩点是 `SET_BUS_WRITE_MODE`（`*_spi_user = SPI_USR_MOSI`）—— 该库为速度
**直接操作 SPI 寄存器**，与 Arduino-ESP32 **2.0.17** 不兼容。
已排除：`TFT_MISO` 未定义、`SPI.begin()` 未调用、`SPI_PORT` 宏未解析、
**以及"屏没接"（接上照崩）**。
**⇒ 换 LovyanGFX**：自带 ESP32-S3 驱动、走 esp-idf 的 spi_master，
不依赖 Arduino `SPIClass`、不碰寄存器宏，一次通过。

#### 坑 2：`LV_COLOR_16_SWAP` + 缓冲类型 → 规则条纹（**看起来极像镜像**）

```c
s_lcd->pushImage(..., (lgfx::rgb565_t *)color_p);   // ✗ 错
s_lcd->pushImage(..., (const uint16_t *)color_p);   // ✓ 对
```
LVGL 在 `LV_COLOR_16_SWAP=1` 时**已经交换过字节序**，再转成 `rgb565_t`
结构体会让 LovyanGFX **再解释一遍** → 转换两遍 → **横向条纹 + 发白**。
而条纹**看起来非常像镜像**，导致长时间误判成"旋转不对"。
- 规律：**SPI 屏 + LVGL v8 + ESP32 ⇒ `LV_COLOR_16_SWAP` 基本都要置 1**（`lv_conf.h`）
- 判据：内容是**规则条纹/发白乱块**（认不出字）→ 字节序问题；
  内容**能认出来但方向不对** → 才是旋转问题

#### 坑 3：白底黑字 → 其实是**我自己多设了深色**

`lv_conf.h` 的 `LV_THEME_DEFAULT_DARK` **本来就是 0（浅色）**，
LVGL 默认主题就是白底黑字。是我在 `gui_build_ui()` 里写了
`lv_obj_set_style_bg_color(scr, 0x101820)` 把主题盖掉了。
**⇒ 删掉深色覆盖即可**，不要"手动设成白色"。

#### 坑 4：朝向 —— `setRotation` 的 `_internal_rotation` **bit2 被我忽略**

读 `LovyanGFX/.../Panel_LCD.inl` 源码：
```cpp
_internal_rotation = ((r + offset_rotation) & 3) | ((r & 4) ^ (offset_rotation & 4));
                                            // ↑ bit2 = **翻转标志**，不参与 &3
_width = (_internal_rotation & 1) ? 交换 : 不交换;   // ← 画布尺寸也由它决定！
_colstart = (_internal_rotation & 2) ? mw-(pw+ox) : ox;
```
两个要点：
1. **不能"调完 setRotation 再自己覆写 MADCTL"**：`setRotation` 会同时算
   `_colstart/_rowstart/_width/_height`，单独覆盖寄存器会让两者不一致
   → 画面**整体转 90° + 镜像**。
2. `_internal_rotation & 1` **同时决定画布是否交换** —— 忽略它就会让
   "朝向对了但画布尺寸和 LVGL 冲突"。

**定位方法（一次定死，别学我试一整天）**：
直接枚举 **MADCTL 寄存器的 8 种有效组合**（0x00/0x20/0x40/0x60/0x80/0xA0/0xC0/0xE0），
每种在屏上画**几何标记**（左上角白方块 + 上边绿线 + 左边蓝线 + 大字编号），
让用户回答"哪一张方框在左上角"。
- **不要问"文字是不是反的"** —— 字节序错、镜像、旋转三种问题下
  文字看起来都"像反的"，人眼分不开（这是我绕最久的根本原因）
- **几何位置是客观的**，不受颜色/字节序干扰

本屏最终值：**`_internal_rotation = 4`**（MADCTL `MV|MY|ML|BGR` = 0xA8）
⇒ 取 **`TFT_ROTATION = 4`、`offset_rotation = 0`**。

#### 坑 5：右侧 80px 黑边 —— **面板物理尺寸配错**

我按"ILI9341 芯片原生 240×320"配 `panel_width/height`，
但这块**模块的玻璃是 320×240**，于是内容只画到 240px 宽，
右边空出 **320−240 = 80px**。
**⇒ `panel_width/height` 和 `memory_width/height` 都必须是 320×240。**

#### ★ 最终配置三件套（缺一必错）★

| # | 参数 | 位置 | 值 |
|---|---|---|---|
| ① | `panel_width/height` + `memory_width/height` | `gui.cpp` 的 `class Lcd` | **320 / 240** |
| ② | `TFT_ROTATION` | `gui.cpp` 顶部 | **4**（→ MADCTL 0xA8） |
| ③ | `LVGL_HOR_RES` / `LVGL_VER_RES` | `config.h` | **320 / 240** |

**调试口诀**（照做就不会再绕）：
1. 烧进去，看日志 `硬件参数：width=W height=H` 和 `尺寸一致性：… ✓/✗`
2. 把 ③ 改成 **W 和 H**（一个字都不要差）
3. 方向不对就只改 ②，**再回到第 1 步**

**其他已确认的参数**（这套组合在实机上跑通）：
- `TFT_SPI_WRITE_HZ = 10000000`（10MHz；20MHz 以上在 10cm 杜邦线上会出雪花）
- `cfg.dma_channel = 0`（**关掉 DMA**；开 DMA 时雪花位置会移动，是传输抢跑的特征）
- 画缓冲 = **整屏**（320×240×2 = 153600 字节，内部 DMA RAM）
- `cfg.rgb_order = false`、`cfg.invert = false`

---

### 9.4 中文字库（LVGL 默认字体只有 ASCII，中文全是方框）

**症状**：所有中文渲染成小方框（□）。LVGL 自带的 Montserrat 只有 ASCII。

**做法**：用 `lv_font_conv` 从系统黑体生成**稀疏字库**（只打进用到的字，
而不是整段 CJK 区 2 万多字 —— 那样要几 MB）。

```powershell
cd D:\MyProject
npm install lv_font_conv                    # 只需一次
node firmware-esp32\tools\gen_cn_font.js 14 2   # 字号 14、bpp 2
node firmware-esp32\tools\gen_cn_font.js 16 2
node firmware-esp32\tools\gen_cn_font.js 20 2
```

产物落在 `src/lv_font_cn_{14,16,20}.c`（**PlatformIO 会自动编译 `src/` 下的 .c**）。
在 `lv_conf.h` 里声明并设为默认字体：

```c
#define LV_FONT_CN_14 1
#define LV_FONT_CN_16 1
#define LV_FONT_CN_20 1
#define LV_FONT_CUSTOM_DECLARE  LV_FONT_DECLARE(lv_font_cn_14) \
                                LV_FONT_DECLARE(lv_font_cn_16) \
                                LV_FONT_DECLARE(lv_font_cn_20)
#define LV_FONT_DEFAULT &lv_font_cn_14
```

#### 坑 1：**必须显式带 ASCII 区**，否则数字也变方框

```js
'--range', '0x20-0x7F',      // ← 少了这行，字库里只有汉字
'--symbols', <汉字>,
```
漏了它 → 生成的字体 cmap 里只有 `range_start=19968`（汉字）→
一旦设成 `LV_FONT_DEFAULT`，`25.3`、`%RH` 这些**数字和字母也会变成方框**
（从"中文方框"变成"数字方框"，更难查）。
加上之后 cmap 是**两段**：ASCII（32~127）+ 汉字稀疏表。

#### 坑 2：**14px 的汉字会被笔画糊成"小方块"**

用户实机反馈：标签页（14px）读得清，但**卡片标题看着是方框**。
原因是 14px 下汉字笔画密度太高、糊在一起。**汉字最小可用字号约 16px**。
⇒ 分档使用：**14 默认 / 16 标题 / 20 数值**。

#### 坑 3：**数字标签不要用 Montserrat**

原来数值用 `lv_font_montserrat_20`（好看），但它是纯英文字体 ——
以后数值里**万一带单位汉字**（`25.3℃`）就会变方框。
⇒ 数值也换成 `lv_font_cn_20`（自带 ASCII，数字照常显示）。

#### ⚠ 加中文之后必须重跑字库

**这是唯一的陷阱，而且不会报错**：源码里加了新汉字、但没重跑
`gen_cn_font.js` → 那个字显示成方框。字库是"按需打进去"的。

| 字库 | 大小(2bpp) | 用途 |
|---|---|---|
| `lv_font_cn_14` | 333 KB | 默认字体（标签页、说明文字） |
| `lv_font_cn_16` | 395 KB | 卡片标题 |
| `lv_font_cn_20` | 550 KB | 卡片数值 |
| **合计** | **1.24 MB** | Flash 14.0%，**RAM 零增量**（LVGL 从 Flash 读字形） |

**最终效果（实机确认）**：标签页 `数据 / 波形 / 诊断`、卡片标题
`温度 / 湿度 / 气压 / 光照 / 烟雾 / 空气质量 / 距离 / 人体感应 / 心率 / 血氧`
全部正常显示，无方框。



---

### 9.5 页面操作：**没有触摸屏也没有按键怎么切页？**

**背景**：触摸那 5 根线当初特意没接（省引脚、免校准），实体按键也还没焊。
结果三页界面（数据/波形/诊断）**切不了**，只能一直盯着数据页。

**解决办法（三种，前两种零成本已实现）**：

#### ① 自动轮播（默认开，10 秒一页）

```c
// config.h
#define GUI_PAGE_CYCLE_MS  10000   /* 轮播周期 */
#define GUI_PAGE_COUNT     3
```
周期怎么选：波形页在 9600 总线下 **15 秒**才更新一轮窗口数据（见 §9.2），
周期比 15s 短的话切到波形页可能还是同一帧。**等加上 120Ω 终端电阻、
快车道回到 2 秒后，可以调小到 5 秒。**

#### ② 串口命令行（`console.cpp`，core0 最低优先级）

串口工具（**115200 8N1，COM6**）里直接敲：

| 命令 | 作用 |
|---|---|
| `page=0\|1\|2` | 切到指定页（0=数据 1=波形 2=诊断） |
| `next` / `n` | 下一页（循环） |
| `prev` / `p` | 上一页（循环） |
| `auto=0\|1` | 关/开自动轮播（也认 `on`/`off`） |
| `status` | 当前页 / 轮播 / 总线统计 / 队列 / 内存 |
| `help` | 命令帮助 |

- 大小写不敏感，`=` 和空格都行（`PAGE = 2`、`page 2`、`page=2` 等价）
- 手动切页会**重置轮播计时**，不会"刚切完又被轮播切走"
- ⚠ `pio device monitor` 默认会**复位设备**，用 `--no-reset` 或换串口助手

#### ③ MQTT 远程（`net.cpp`）

```json
{"page":1}          // 切到第 1 页
{"page":"next"}     // 下一页
{"page":"prev"}     // 上一页
```
⚠ 这是**本地 UI 操作，不进 `g_cmd_queue`**（那个队列是往 485 写寄存器的）。
两者目标完全不同，混在一起会让人以为 `page` 也下发给了 STM32。

#### ★ 线程安全：为什么三种入口都不直接碰 LVGL ★

LVGL **不是线程安全的**，而这三个入口分别来自 core0 的
`task_console` 和 `task_net`。所以：

```
console / net  ──►  gui_request_page()  ──►  只写【原子变量】s_page_want
                                                  │
task_gui (core1)  ──►  gui_tick()  ──►  读 s_page_want，调 lv_tabview_set_act()
```

真正调用 LVGL 的**只有 core1 的 `task_gui`** —— 和 LVGL 的其它调用在同一条
任务上，天然串行。**绝对不要在 core0 的任务里直接调 LVGL API。**

#### ⚠ 踩到的坑：自动轮播"永远停在数据页"

第一版代码里，轮播把 `s_page_cur` 改成 1 之后**没有同步 `s_page_want`**，
于是下一拍 `want(0) != cur(1)` 又成立 → 被误判成"手动请求" → 立刻切回 0。
**表现：日志每 10 秒都打"切到页面 0"，永远回数据页。**

⇒ 修复：手动和轮播两条分支都**同时更新 `want` 和 `cur`**。
判据：日志应该是 `轮播到页面 1 → 2 → 0` 依次循环。

#### 将来要加实体按键

代码位置已经留好，接线很简单（**按键一端接 GPIO，另一端接 GND，用内部上拉**）：

| 按键 | 建议 GPIO | 动作 |
|---|---|---|
| 上一页 | GPIO4 | `gui_request_page_step(-1)` |
| 下一页 | GPIO5 | `gui_request_page_step(+1)` |
| 功能键 | GPIO6 | 自定义（比如切轮播开关） |

⚠ 避开已占用的脚：OLED 8/9、485 的 16/17/18、USB 19/20、
模组 Flash 26~32、Octal PSRAM 33~37、strapping 0/3/45/46、板载 RGB 48。
（当前 TFT 用的是 10/11/12/13/14/15/21。）
---

### 9.6 波形页"曲线是直线"——两层原因（2026-10）

**症状**：波形页能显示，推理也正常，但三条加速度曲线**都是平直线**，晃动板子看不出变化。

#### 第一层：Y 轴固定 ±20000，而波动只有 ±200

从机实测的 IMU 原始值：

```
ax ≈ -10700    ay ≈ -10500    az ≈ +6350      ← 重力占大头
相邻样本变化 ≈ ±200                            ← 振动引起的波动
```

固定 Y 轴 ±20000 时，波动只占屏高的 **0.5%** ⇒ 看起来就是直线。
⇒ **改成每次按当前窗口的 min/max 自适应**（上下各留 10% 余量，最小跨度锁 100 counts）。

#### 第二层（关键）：三个轴的**均值差太远**，共用一根 Y 轴必然压扁

修完第一层后曲线**仍然是直线**。原因是：

```
ax ≈ -10700   ay ≈ -10500   az ≈ +6350
```

三条线的均值相差 **17160**，**各占 Y 轴一端、永不交叠**。
自适应后范围必须同时容下三个均值 ⇒ 跨度仍 ≈17000 ⇒ 波动依旧只占 1%。

**"共用一根 Y 轴"和"显示波动"在数学上是互斥的**，只能二选一。
⇒ 显示 **`v − 本轴均值`**（振动监测的标准做法，只看波动量）。

#### 最终方案：阈值自动切换去均值

```c
#define GUI_WAVE_DEMEAN_SPAN  2000   /* ≈0.12g */
```

| 情况 | 显示 | 理由 |
|---|---|---|
| 峰峰值 ≤ 2000（静止/微振） | **去均值**（`v - mean[axis]`） | 否则三条直线，什么都看不出 |
| 峰峰值 > 2000（剧烈振动） | **原始值** | 保留"哪个方向受力"的方向信息；去均值会把波形压成对称的，丢失这个信息 |

实现位置：`gui.cpp` 的 `gui_update_wave()`，**两遍扫描**（先求均值与范围、再写点）。

#### 顺带确认的事

- **三条 series 一直都在**（红=ax 绿=ay 蓝=az），之前"只看到红绿"是
  **三条线全被压成直线并重叠**造成的视觉错觉，不是 LVGL 的问题
- `seq / rate=50.0Hz / ts / valid` 那一行信息正常，说明窗口头解析正确

#### 排查这类"曲线不对"的通用顺序

1. **先看有没有数据**：`seq` 在涨吗？`valid` 吗？→ 否则查窗口读（§9.2.1）
2. **再看范围**：把均值/峰峰值从从机串口打出来（`[imu]` 那行），
   和图表 Y 轴范围比一下 —— **波动占屏高 < 5% 就必然看着像直线**
3. **最后看语义**：多轴的**均值差异**是否远大于波动？是则必须去均值
---

### 9.7 写方向验证 + 一个"读错基准值"的真 bug（2026-10）

背景：本项目此前**只验证过读**（采集 → 显示）。写方向（控制下发）从未实测
—— 因为诊断页的按钮需要触摸屏，而触摸特意没接（省引脚 + 免校准）。

#### 9.7.1 怎么在不接硬件的前提下验证写

给 `console.cpp` 加了 5 条下行命令（`beep` / `rgb` / `fan` / `w` / `cset`·`cclr`）。
**关键点：它们走的是和 MQTT 下行完全相同的路径**
（`cmd_post` → `g_cmd_queue` → `task_cmd` → Modbus FC06/FC10），
所以验证它们等于验证了整条下行链路，而且**不需要 broker、不需要额外硬件**。

| 命令 | 作用 | 用到的 Modbus 功能码 |
|---|---|---|
| `beep [0\|1]` | 蜂鸣器（无参数 = 响 1 秒） | FC06 写控制字 |
| `rgb R G B` / `rgb off` | 设三色灯 | **FC10 连写 3 个**（R/G/B 地址连续） |
| `fan 0~1000` | 风扇 PWM | **FC10 连写 2 个**（控制字+PWM，原子） |
| `w 寄存器 值` | 通用写 | FC06 |
| `cset/cclr 掩码` | 控制字置位/清位 | FC06（读-改-写） |

#### 9.7.2 ★ 发现的 bug：RMW 拿"慢车道快照"当基准值，导致覆盖式写入 ★

**现象**：执行 `beep` 之后，从机控制字从 `0x0020` 变成 `0x0001` ——
**bit5（允许本地自治）被无声清掉了**。再执行 `rgb` 变成 `0x0008`，bit5 依然是 0。

**根因链**（三段，每段都独立成立）：

```c
// 原实现：
const uint16_t cur = slow_u16(&snap, REG_CTRL1);
const uint16_t nv  = (cur | set_mask) & ~clr_mask;
```

1. **越界返回 0**：`slow_regs_t.regs[]` 只覆盖
   `MODBUS_SLOW_START(0x0000) ~ +MODBUS_SLOW_REG_COUNT(28)` = **0x0000~0x001B**；
   而 `REG_CTRL1 = 0x0020` **不在范围内** → `slow_u16()` 的越界检查直接 `return 0`
2. **于是 nv 只剩 mask**：`(0 | set) & ~clr` = 只有 set 里的位
3. **FC06 是写整字**：把整个控制字覆盖掉 → 其余使能位全丢

**还有第二层错误（更本质）**：**即使范围覆盖到了，用快照做 RMW 也是错的** ——
快照是慢车道每 1s 采一次的缓存，可能已过期近 1 秒；
用它算新值会把这 1 秒内发生的改动**无声地覆盖掉**。

**修法**：RMW 之前**实时读一次从机的真值**（不查缓存）：

```c
uint16_t cur = 0;
if (mb_read_holding(REG_CTRL1, 1, &cur, ...) != MB_OK) { ... }
const uint16_t nv = (cur | set_mask) & ~clr_mask;
```

`fan_set()` 同理：一次 `FC03` 读 `0x0020~0x0021` 拿基准，
再一次 `FC10` 原子写回（读写各一次，中间窗口最小）。

**修复后实测日志**：
```
控制字 RMW：读回 0x0008，置 0x0020 清 0x0000 → 写 0x0028   ← 正确保留 bit3
控制字 RMW：读回 0x0028，置 0x0008 清 0x0000 → 写 0x0028   ← 正确保留 bit5（幂等）
```
从机侧确认：`[hb] ctrl=0x0028` ✅（bit5 本地自治 + bit3 RGB 使能都在）

#### 9.7.3 这个 bug 的通用教训

| 教训 | 说明 |
|---|---|
| **"越界返回默认值"是危险的设计** | `slow_u16()` 越界返回 0 是合理的（防御性），但**调用方必须知道 0 是"非法"还是"真值"**。这里 0 被当成了"当前控制字"，于是写入变成了覆盖 |
| **缓存值不能用于 RMW** | 读-改-写的"读"必须是**当前真值**。用缓存 = 用历史值算现在的值 = 必然覆盖中间改动 |
| **"读"的 bug 会以"写"的形式表现** | 现象是"写坏了别的位"，根因却在读路径。**排查时要往上游多走一步** |
| **写方向一定要单独验证** | 这个 bug 在纯读场景下**永远不会暴露**。项目跑了 4 小时长跑（只读）都没发现它 |

#### 9.7.4 验证结果汇总

| 项 | 结果 |
|---|---|
| FC06 写单寄存器（`w 0x0027 450`） | ✅ 成功 |
| FC10 连写 3 个（`rgb 255 0 0`） | ✅ 成功 |
| FC10 连写 2 个原子（`fan`） | ✅ 代码就绪（未接风扇硬件） |
| 控制字 RMW 保位 | ✅ **修复后** `0x0008` → `0x0028`，bit5 保留 |
| 总线统计 | ✅ `TX 333 RX 333  超时 0 重试 0 异常 0 CRC错 0` |
| 从机侧确认 | ✅ `[hb] ctrl=0x0028` |