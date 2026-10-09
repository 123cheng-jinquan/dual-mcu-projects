# PC 侧工具集（`tools/`）

> 这些脚本是 `docs/02-modbus-protocol.md`（协议规格）与 `docs/03-bringup.md`（分阶段上手）
> 的**可执行验收工具**：凡是能拆到 PC 上验证的，就不留到双机联调时才发现
> （`docs/03` 开篇原则）。
>
> 安装依赖：`pip install -r tools/requirements.txt`（只写文件、不自动装；见文末平台差异说明）

---

## 1. 脚本一览

| 脚本 | 用途 | 典型命令 | 依赖 | 对应 `docs/03` 阶段 |
|---|---|---|---|---|
| `crc16_test.py` | Modbus CRC16 测试向量对拍（`docs/02` §3.1 全量 10 条）+ 整帧 `0x0000` 自检 + 位运算/查表交叉验证 | `python crc16_test.py`<br>`pytest crc16_test.py -v` | 无（纯标准库） | **阶段 3** 一开始（手写从机第一步） |
| `modbus_master_test.py` | PC 当 Modbus **主机**压测真实 F411 从机，覆盖 `docs/02` §10 全部用例（异常码/静默/广播/压力） | `python modbus_master_test.py --port COM5 --baud 115200 --addr 1 --count 1000` | `pymodbus>=3.7,<3.10`、`pyserial` | **阶段 3**（脱机验证从机协议栈） |
| `modbus_slave_sim.py` | PC 当 Modbus **从机**，实现 A/B/C/D/E 五区寄存器，让 S3 主站逻辑脱离硬件先跑通 | `python modbus_slave_sim.py --port COM7 --baud 115200 --addr 1 --rate 2` | `pymodbus>=3.7,<3.10`、`pyserial` | **阶段 4** 第 2 步（换真 F411 之前） |
| `capture_window.py` | 从 USART1(CH340) 直接收 F411 打点的六轴窗口 → 每窗口一个 CSV（不依赖 S3） | `python capture_window.py --port COM4 --baud 115200 --label normal --flush-every 10` | `pyserial` | **阶段 6** 第 1~2 步（数据集采集） |
| `label_tool.py` | 数据集体检（坏样本检测/隔离）+ 键盘打标 + 分层划分写 `split.json` | `python label_tool.py --data data --quarantine` | 无（纯标准库） | **阶段 6** 第 2~3 步 |
| `train_mlp.py` | 36 维特征（`0x0400~0x0423`）训练 MLP → int8 全整数量化 `.tflite` → C 数组 + 标准化参数 | `python train_mlp.py --data data --split data/split.json --out model_out --epochs 200 --test-ratio 0.2` | `tensorflow`（回退 `scikit-learn`）、`numpy` | **阶段 6** 第 4~5 步（模型与部署物） |
| `verify_tflite.py` | PC（tflite-runtime）与 MCU（TFLite Micro）同输入推理对拍，逐类打最大绝对偏差 | `python verify_tflite.py --model model_out/model.tflite --port COM5 --baud 115200 --tol 0.05` | TFLite 运行时（见平台差异）、`pyserial` | **阶段 6** 第 6 步（部署后回归） |
| `_console.py` | 内部共享模块：UTF-8 控制台 + 等宽表格（Windows GBK 控制台不再因 `µ`/中文崩掉）。**不单独运行** | — | 无 | — |

---

## 2. 三条推荐流水线

### 2.1 阶段 3：把手写从机调通（不碰 S3）

```powershell
python crc16_test.py                                   # 先证明 CRC 实现是对的
python modbus_slave_sim.py --port COM7 --selftest      # 顺带验证模拟器寄存器逻辑
python modbus_master_test.py --port COM5 --cases basic,boundary,write,value,consistency
python modbus_master_test.py --port COM5 --raw-tests --raw-port COM5 --cases error
python modbus_master_test.py --port COM5 --count 10000 --progress
```

验收锚点：`01 03 00 4B 02 00 34 BC` → 异常 `03`；CRC 错帧静默；10000 帧 `err_crc == 0`。

### 2.2 阶段 4：先把 S3 主站逻辑调通（不接 F411）

```powershell
# 终端 A：PC 当从机（默认 115200，与 firmware-esp32 的 MODBUS_BAUD 一致）
python modbus_slave_sim.py --port COM7 --baud 115200 --addr 1 --rate 2 --verbose
# 终端 B：PC 当主机自测这条链路（可选）
python modbus_master_test.py --port COM7 --baud 115200 --cases basic,consistency
```

S3 侧按 `plan.md` §5.4 的调度表打即可：慢车道 500ms 读 `0x0000~0x001B`，
快车道 2s 读窗口头 + 5 块 + `0x0101` 校验。

### 2.3 阶段 6：采数据 → 打标 → 训练 → 部署对拍

```powershell
python capture_window.py --port COM4 --baud 115200 --label unlabeled --target 80
python label_tool.py --data data --quarantine
python label_tool.py --data data --label-keys 0=normal,1=imbalance,2=loosen
python label_tool.py --data data --split-only --test-ratio 0.2
python train_mlp.py --data data --split data/split.json --out model_out --epochs 200
python verify_tflite.py --model model_out/model.tflite --port COM5 --baud 115200 --tol 0.05 --samples-count 10
```

---

## 3. 为什么默认波特率是 115200

**因为 9600 跑不动这个项目的数据量。** 算一笔账（8N1，1 字节 = 10 bit）：

| 项目 | 数值 |
|---|---|
| 一个六轴窗口 | 600 个寄存器 = 1200 字节数据 + 5 帧协议开销（5×(1+1+1+2+2) ≈ 35 字节）≈ **1315 字节** |
| @115200 | 1315 × 10 / 115200 ≈ **114 ms** → 占 2s 快车道周期的 **约 7%** |
| @9600 | 1315 × 10 / 9600 ≈ **1.37 s** → 占 2s 周期的 **约 83%** |

@9600 时，快车道一帧窗口就吃掉 1.37s，再叠加 500ms 一次的慢车道（28 个寄存器 ≈ 64ms，
几乎每 500ms 都要插队），总线会长期饱和、超时和重试接连发生，**系统根本跑不起来**。
所以 9600 **只适合"链路到底通不通"的单点自检**（`docs/03` 阶段 3 的调试期），
确认能通之后必须立刻升回 115200。

因此本工具集所有脚本的 `--baud` 默认值都是 **115200**，与
`firmware-esp32/include/config.h` 的 `MODBUS_BAUD` 保持一致 —— 否则联调第一步就是满屏超时。
（F411 侧波特率可用 `0x002D` 运行时切换，见 `docs/02` §5 C 区。）

---

## 4. 36 维特征的两种布局：别把矩阵转置了 ⚠

**这是"模型能训、能跑、不报错，但准确率掉到随机水平"的头号原因。**

`docs/02` §5 E 区是**权威定义**，它按**特征**分段：

| 地址段 | 内容 |
|---|---|
| `0x0400~0x0405` | 6 个轴的**均值** |
| `0x0406~0x040B` | 6 个轴的**标准差** |
| `0x040C~0x0411` | 6 个轴的峰峰值 |
| `0x0412~0x0417` | 6 个轴的过零率 |
| `0x0418~0x041D` | 6 个轴的主频 |
| `0x041E~0x0423` | 6 个轴的主频带能量占比 |

即 **"每个特征 6 个轴连续" = `per-feature`（默认值）**：

```
下标:  0    1    2    3    4    5  |  6    7    8    9   10   11 | ...
含义: ax均 ay均 az均 gx均 gy均 gz均 | ax标 ay标 az标 gx标 gy标 gz标 | ...
       └──── 均值 6 轴 ────┘         └──── 标准差 6 轴 ────┘
```

另一种常见写法是 **`per-axis`（每轴 6 个特征连续）**，它与上面的矩阵**互为转置**：

```
下标:  0    1    2    3    4    5  |  6    7    8    9   10   11 | ...
含义: ax均 ax标 ax峰 ax过 ax频 ax能 | ay均 ay标 ay峰 ay过 ay频 ay能 | ...
```

**两者的维数都是 36、训练都不会报错**，但每一维的含义完全不同 → 模型学到的全是噪声。

规则很简单：

* **默认用 `per-feature`**（与 `docs/02` §5 E 区、本项目固件一致），不要改；
* 只有当你**同步修改了 F411 写入 E 区的顺序**（改成按轴分段）时，才用 `--feat-layout per-axis`；
* `train_mlp.py` 在 `per-axis` 时会显著警告，并把 `feature_layout` 写进 `model_info.json`；
  训练时还会用 1-NN 可分性自检，**布局用错时直接提示并给出转置布局的对照分数**；
* ESP32 侧 `infer_init()` 会用 `FEATURE_LAYOUT` 宏对 `model_info.json` 里的
  `feature_layout` 做**硬校验**，不一致直接拒绝启动推理 —— 这是最后一道保险。

---

## 5. `train_mlp.py` 产出物 → ESP32 工程摆放位置

`train_mlp.py` 跑完会在 `--out`（默认 `model_out/`）下产出下面这些文件。
**ESP32 侧已经在等这几样东西**，按下表拷贝：

| 产出文件 | 拷贝到 `firmware-esp32/` 的位置 | 用途 / 注意事项 |
|---|---|---|
| `model_data.cc` | `src/model_data.cc` | `alignas(16) const unsigned char model_data[]` + `model_data_len`；TFLite Micro 直接引用。**arena 建议 `heap_caps_malloc(..., MALLOC_CAP_SPIRAM)` 放 PSRAM**（`plan.md` §7.2） |
| `model_data.h` | `include/model_data.h`（或 `src/` 同级） | 与 `.cc` 配套的声明；`model_data.cc` 里 `#include "model_data.h"` |
| `norm_params.h` | `include/norm_params.h` | 标准化参数（`feature_mean[]`/`feature_std[]`）+ 量化参数（`MODEL_INPUT_SCALE`/`MODEL_INPUT_ZERO_POINT`/`MODEL_OUTPUT_SCALE`/`MODEL_OUTPUT_ZERO_POINT`）+ **`FEATURE_LAYOUT` 宏**。**必须与训练时同一套**：PC 侧 `norm_params.py` 是同一份数据的 Python 版 |
| `norm_params.cc` | `src/norm_params.cc` | `norm_params.h` 里那两个 `float` 数组的定义 |
| `model_info.json` | `src/model_info.json`（或仅存档，供 CI/工具读取） | 含 `class_names`（**按字母序**，与固件 `kClassLabels[]` 一一对应）、`num_classes`、`feature_dim`、`feature_layout`、量化参数、测试集准确率、混淆矩阵。`infer_init()` 用它做硬校验 |
| `model.tflite` | 不直接进固件 | 中间产物：`.cc` 由它生成；`verify_tflite.py` 用它做 PC 侧对拍 |
| `confusion_matrix.txt` | 存档（写进 README / 报告） | 留档用，证明测试集准确率（`docs/03` 验收线 ≥90%） |

**一致性检查清单（换模型后逐条过）**：

1. `FEATURE_LAYOUT`（固件宏）== `model_info.json:feature_layout` == `norm_params.py:FEATURE_LAYOUT`
   —— 都应为 `"per-feature"`；
2. `kClassLabels[]` 顺序 == `model_info.json:class_names`（字母序）；
3. 固件用 `norm_params.h` 的 mean/std 做 `(x-mean)/std`，再按 `MODEL_INPUT_SCALE` /
   `MODEL_INPUT_ZERO_POINT` 量化成 int8 —— **不要做第二次缩放**；
4. 输出按 `MODEL_OUTPUT_SCALE` / `MODEL_OUTPUT_ZERO_POINT` 反量化；
5. 最后跑 `verify_tflite.py` 对拍，确认逐类偏差在容差内。

> 注意：`0x0400` 特征通道是**只读**的（`docs/02` §5 E 区）。
> `train_mlp.py --feature-source auto` 既能读 `capture_window.py` 采的 100×6 窗口 CSV
> （现场算 36 维特征），也能读现成的 36 列特征 CSV。

---

## 6. 每个脚本都能 `--selftest`（无硬件自检）

没有板子/串口时，先验证逻辑与格式，避免"插上板子才发现解析写错"：

| 命令 | 验证内容 |
|---|---|
| `python crc16_test.py` | 44 条用例（含 `docs/02` §3.1 全量向量）全 PASS |
| `python modbus_slave_sim.py --selftest` | A/B/C/D/E 区生成、读/写请求校验（异常 02/03）、E 区与 D 区自洽 |
| `python capture_window.py --selftest --label demo --out data` | 两种输入格式解析、CSV 命名/落盘、异常行拒绝 |
| `python label_tool.py --selftest --data data` | 坏样本检测与隔离、打标移动、分层划分与 `split.json` 可复现 |
| `python train_mlp.py --selftest --data data` | 窗口→36 维特征、两种数据源、布局转置检查、标准化、混淆矩阵渲染 |
| `python verify_tflite.py --selftest` | MCU 串口协议组帧/解析、softmax、容差判定 |

`--selftest` 全部只碰临时目录（`data/_selftest*`、`data/../_selftest_tmp`），跑完自动清理。

---

## 7. 寄存器地址口径（**统一取自 `docs/02` §5，不要自己发明**）

| 区块 | 地址 | 访问 | 本工具集中的出现位置 |
|---|---|---|---|
| A 环境量 | `0x0000~0x000F` | R | `--cases basic` 读 A+B（一次 28 个） |
| B 状态与统计 | `0x0010~0x001F` | R | 帧计数 `0x0013~0x0015` 用于压测核对 |
| C 控制与配置 | `0x0020~0x002F` | R/W | 只读区拒绝/非法值用例、模拟器写入校验 |
| D 六轴窗口 | `0x0100~0x035B` | R | `capture_window.py` CSV 的 600 个值；5 块分读 |
| E 特征向量 | `0x0400~0x0423` | R | `train_mlp.py` 的 36 维输入（布局见 §4） |

**允许的连续读区间**（模拟器按此返回异常 02）：A 区、B 区、A+B 整体
（`0x0000~0x001F`）、C 区、D 区整窗（`0x0100~0x035B`）、E 区。其它跨区读一律异常 02。

---

## 8. 依赖与平台差异（`requirements.txt` 里也有注释）

| 依赖 | 说明 |
|---|---|
| `pymodbus>=3.7,<3.10` | **区间是实测的**：3.10 起 pymodbus 删除了模拟器用的 `ModbusSlaveContext`/`ModbusSequentialDataBlock`（改用 `SimData`/`SimDevice`），模拟器会起不来；3.6.x 的服务端不接受 `custom_pdu`，异常码覆盖会弱化。`modbus_master_test.py` 本身在 3.6~3.15 上都能跑 |
| `pyserial` | 串口读写；`capture_window.py` / 原始帧用例 / 对拍都依赖它 |
| `numpy` | 推理张量构造与量化校准 |
| `tensorflow` | 训练 + `.tflite` 转换。**Windows 上用 `tensorflow.lite.Interpreter` 跑推理**，因为 `tflite-runtime` 在 Windows 没有官方 wheel |
| `tflite-runtime` **或** `ai-edge-litert` | 轻量推理运行时。**Linux/macOS/树莓派**推荐 `tflite-runtime`；Google 已迁移到 `ai-edge-litert`，两者 API 同名（`Interpreter`），`verify_tflite.py` 会依次尝试 |
| `scikit-learn` | TensorFlow 不可用时的回退训练路径（**不产出 tflite**，只验证特征可分性与准确率） |
| `pytest` | `crc16_test.py` 除直接运行外也兼容 pytest |
| `pandas` | 可选：用 notebook 做数据集统计分析与可视化 |

> 只装核心两个也能跑通 `docs/03` 阶段 3/4：
> `pip install "pymodbus>=3.7,<3.10" pyserial`。

---

## 9. 常见问题（脚本里的中文提示已覆盖，这里给出源头）

| 现象 | 排查顺序 |
|---|---|
| `串口打不开` | ① CH340/CP210x 驱动 ② 端口号（`--list-ports` 或设备管理器）③ **被 Modbus Poll / 串口助手占用**（同口只能一个进程）④ 误选了板载 USB 调试口而不是 485 口 |
| 联调一开始就满屏超时 | ① 波特率两端不一致（都应为 **115200**，见 §3）② 从机地址不一致 ③ A/B 未共地或接反 |
| `CRC 错注入` 用例失败（收到了响应） | 从机 CRC 校验分支没走"静默丢弃"；检查是否在 CRC 失败时回了异常帧（`docs/02` §8 明确禁止） |
| 压测 `err_crc` 增长 | 波特率/DE 切换时序；`docs/03` 阶段 3 的坑清单第 1、2、5 条 |
| 窗口 5 块读完 `0x0101` 变了 | 从机没实现 `docs/02` §7 的"读期间锁缓冲"（500ms 兜底锁） |
| 模型测试集准确率只有随机水平 | **99% 是特征布局转置**：见 §4，确认 `--feat-layout` 与固件 E 区一致（默认 `per-feature`） |
| `verify_tflite.py` 偏差巨大 | ① 标准化参数不是同一套 ② 已量化输入被 MCU 二次缩放 ③ `FEATURE_LAYOUT` 与固件不一致 ④ 输出反量化参数用错 |
| 模拟器寄存器整体偏 1 位 | pymodbus `ModbusSlaveContext` 的地址偏移差异；`modbus_slave_sim.py` 启动时会打印**地址偏移探测结果**，以此为准 |

---

## 10. 已知偏差与需要注意的假设

1. **`docs/02` §6 示例②第 1 行的 CRC 笔误已修正**（本次复核发现）：原文写作 `01 03 01 04 00 7D 84 17`，
   正解为 `01 03 01 04 00 7D C5 D6`——`84 17` 其实是块地址 `0x0100` 那一帧的 CRC，属抄错一行；同表其余 4 行本来正确。
   **该笔误造成过真实故障**：ESP32 侧把文档帧抄进 `mb_selftest()` 常量后，开机自检失败、网关无法启动。
   现已修文档、修固件常量，并在 `firmware-stm32/test/test_modbus.c::test_doc_frames()` 与 ESP32 的
   `mb_selftest()` 两处加了复算，防止再次漂移。
   本工具集**全程不硬编码 CRC**（`crc16_test.py` 现算现比，`-v` 可打印全部实战帧的现场计算值），因此天然不受此类笔误影响。
2. **相位/能量类特征的算法差异**：PC 侧主频与能量占比用朴素 DFT（100 点 @50Hz → 0.5Hz 一格）；
   F411 侧按 `plan.md` §5.3 用 CMSIS-DSP 128 点实数 FFT。两者**频点分辨率不同**，
   特征值本就不会逐位相等——对拍时看的是**分类输出**是否一致，不要拿特征值逐位比。
3. **E 区特征按"窗口内 100 个样本"计算**（`train_mlp.py` 与 `modbus_slave_sim.py` 都这么算）。
   若 F411 固件改用其它统计口径（例如对多窗口做平均），特征分布会变，
   需要用真实数据重新训练——但**布局（§4）不变**。
4. **模拟器对广播/静默的实现方式**：广播靠服务端 `broadcast_enable=True`，
   地址过滤靠"服务端 context 只注册本机地址"（`NoSuchSlaveException` → 静默）。
   这两条在 pymodbus 内部完成，真 F411 必须自己实现（`docs/02` §2/§8）。
5. **`--include-risky` 会打断总线**：写 `0x002C`（从机地址）/`0x002D`（波特率）立即生效，
   脚本会尝试改回并重连，但请自己心里有数；默认跳过这些用例。
6. **`capture_window.py` 的 `W,` 行格式**：约定 `W,<seq>,<600 个整数>`（也接受无序号、
   `W:` / `[W]` 变体）。F411 侧打点实现请与此对齐，否则先加 `--verbose` 看原始行。
