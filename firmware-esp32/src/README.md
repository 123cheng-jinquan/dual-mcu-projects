# src/ —— ESP32-S3 网关端源码（项目一）

> 原来 21 个文件全平铺在 `src/` 下，还混着 **1.6 MB 自动生成的字库**（`lv_font_cn_*.c`），
> 以及**不该手改的生成文件**（`model_data.cc` / `norm_params.cc`）。
> 现在按职责分开，并且用目录把"手写"和"生成"隔开。

---

## 目录结构

| 目录 | 职责 | 文件 |
|---|---|---|
| **`app/`** | 启动流程、FreeRTOS 任务划分、串口命令台 | `main.cpp`<br>`app_tasks.cpp` / `.h`<br>`console.cpp` |
| **`protocol/`** | Modbus RTU **主机**（主动轮询采集端）| `modbus_master.cpp` / `.h` |
| **`dsp/`** | 特征归一化 + TFLite 推理 | `infer.cpp` / `.h` |
| **`ui/`** | 彩屏界面（LVGL）+ OLED 备用显示通道 | `gui.cpp` / `.h`<br>`oled.cpp` / `.h` |
| **`net/`** | WiFi + MQTT | `net.cpp` / `.h` |
| **`model/`** | ★ **自动生成，不要手改** | `model_data.cc` / `.h`<br>`norm_params.cc` / `.h` |
| **`assets/`** | ★ **自动生成的字库**（1.6 MB）| `lv_font_cn_14.c` / `16` / `20` |

`include/` 里保留两个**全局配置**：`config.h`（所有引脚/时序/阈值）和 `lv_conf.h`（LVGL 配置）。

---

## 为什么这样分

### 1. 把"自动生成"隔出来

| 文件 | 谁生成 | 手改会怎样 |
|---|---|---|
| `model/model_data.cc` | `tools/train_mlp.py` | 下次训练覆盖，改动丢失 |
| `model/norm_params.cc` | 同上 | **模型和归一化参数不匹配 → 推理结果全错且不报错** |
| `assets/lv_font_cn_*.c` | `lv_font_conv`（npm）| 下次生成字库覆盖 |

放在 `model/` 和 `assets/` 里，命名上就是"别动这里"的信号。

### 2. 靠 `-I` 让跨模块 include 保持原样

PlatformIO **只自动加 `-Isrc`，不会递归子目录**。所以 `platformio.ini` 里显式加了 6 个：

```ini
-Isrc/app
-Isrc/protocol
-Isrc/dsp
-Isrc/model
-Isrc/ui
-Isrc/net
```

这样各文件里的 `#include "gui.h"`、`#include "config.h"` **一行都不用改** ——
纯搬家，不动代码。

### 3. 依赖方向

```
app/  ──┬──> protocol/   （轮询采集端）
        ├──> dsp/        （推理）
        ├──> ui/         （显示）
        └──> net/        （上报）

dsp/   ────> model/      （模型 + 归一化参数，只读）
ui/    ────> assets/     （字库，只读）
```

`protocol/`、`dsp/`、`ui/`、`net/` 之间**基本互不依赖**，都由 `app/` 串起来。

---

## 验证过没问题

重组后重新编译通过，**没有任何 `#include` 需要改**（因为 include 路径补全了）。
产物与重组前一致：

```
RAM:   20.8%
Flash: 4.7%
```

---

## 各模块的关键设计（面试会被问）

### `app/app_tasks.cpp` —— 任务划分

按实时性分成两条"车道"：

| 车道 | 任务 | 优先级 | 干什么 |
|---|---|---|---|
| **快车道** | `task_modbus_bus` | 最高 | 独占 485 总线，做单帧读 |
| **慢车道** | `task_modbus_slow` | 中 | 分 5 帧拼一个完整窗口 |
| 推理 | `task_infer` | 中 | 特征 → 推理 |
| 输出 | `task_output` | 中 | 刷屏 |
| 健康 | `task_health` | 最高 | 喂看门狗 |

**关键约束：485 总线访问必须串行化** —— 只有一个任务能碰总线，
否则两个任务同时发帧会互相踩（这是 docs/06 §技术问题里记过的一条）。

### `dsp/infer.cpp` —— 模型部署

- `model_data[]` 直接编进固件（5.5 KB），**不存在"烧模型文件"这一步**
- **48 KB tensor arena 放 PSRAM**（`heap_caps_malloc(..., MALLOC_CAP_SPIRAM)`）——
  内部 RAM 塞不下，硬塞会导致 WiFi/LVGL 任务创建失败
- **只注册 8 个算子**（不用 `AllOpsResolver`，那个占几百 KB Flash）
- 启动时做**两道自检**：张量维度、量化参数（`scale`/`zero_point` 必须和
  `norm_params.h` 里的宏一致）—— 这两处出错**程序不会崩、只会结果错**

完整部署流程见 [`../../docs/model-deployment-on-esp32.md`](../../docs/model-deployment-on-esp32.md)。

### `ui/oled.cpp` —— 为什么有两个屏

彩屏走 LVGL，**OLED 是一条不依赖 LVGL 的独立通道**：
LVGL 崩了或彩屏没接，OLED 上仍然能看到总线状态和推理结果。

> 关键状态不应该依赖 GUI 框架 —— 这是一个可讲的设计取舍。

---

## 构建

```bash
cd firmware-esp32
pio run                  # 编译
pio run -t upload        # 烧写（先改 platformio.ini 的 upload_port）
pio device monitor -b 115200
```

**⚠️ 改 `platformio.ini` 时注意**：这个文件**必须保存为 UTF-8 无 BOM**。
用 PowerShell 5.1 的 `Set-Content` 改它，会按 GBK 写 → PlatformIO 报
`UnicodeDecodeError: invalid continuation byte`（这个坑真踩过）。
