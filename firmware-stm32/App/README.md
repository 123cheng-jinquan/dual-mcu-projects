# App/ —— 手写应用代码（项目一，STM32F411 采集端）

> 这里放**全部手写代码**。`../cubemx/` 里是 CubeMX 自动生成的 HAL 初始化和 FreeRTOS 配置，
> **不要手动改那些文件**（重新生成会被覆盖）。两者分开，是为了让 CubeMX 重生成不会冲掉业务代码。

---

## 为什么这样分层

原来这 4 个文件全平铺在 `App/` 下，其中 `app_main.c` 一个文件 **1547 行 / 72 KB**，
把传感器驱动、Modbus 收发、FreeRTOS 任务、36 维特征算法全塞在一起 ——
想单独看"特征是怎么算的"，得先翻 900 行无关代码。

按**依赖方向**拆开之后：

```
app/        任务编排 + 初始化      ← 最上层，认识所有人
  ↓ 调用
features/   特征算法              ← 纯函数，只吃窗口数组
regmap/     寄存器表 + 双缓冲      ← 数据模型
protocol/   Modbus 协议栈          ← 通信
  ↓ 调用
drivers/    外设驱动              ← 最底层（目前仍在 app_main.c 里，见下）
```

**关键约束：依赖只能向下，不能反向。**
`features/` 不知道 FreeRTOS 的存在，`protocol/` 不知道有传感器 ——
这样任何一层都能单独在 PC 上编译测试。

---

## 目录说明

| 目录 | 职责 | 文件 |
|---|---|---|
| **`app/`** | 任务创建、初始化流程、任务主循环 | `app_main.c` / `.h` |
| **`features/`** | 信号处理算法 | `vibration_feat.c` / `.h` |
| **`regmap/`** | 寄存器表定义、六轴窗口双缓冲与锁定 | `regbank.c` / `.h` |
| **`protocol/`** | Modbus RTU 从机 + CRC16 | `modbus_crc.c` / `.h`<br>`modbus_slave.c` / `.h` |

---

## 已经抽出来的：`features/vibration_feat.c`

**这是整个项目唯一的算法核心**，价值最高，所以第一个抽。

- 输入：`100 样本 × 6 轴` 的采样窗口
- 输出：`36 维特征`（均值 / 标准差 / 峰峰值 / 过零率 / 主频 / 频带占比）
- 依赖：只有 `<math.h>` 和 `regbank.h` 里的两个常量 —— **零句柄依赖**

抽出来之后可以**在 PC 上直接编译单测**（不需要 HAL、不需要 FreeRTOS、不需要板子）。

实现上用的是 **Goertzel** 而不是整段 FFT：只关心 3~25Hz，按 0.5Hz 步进扫 45 个频点，
比做 128 点 FFT 便宜。实测 45 × 100 × 6 ≈ 2.7 万次浮点运算，F411 有硬件 FPU，
耗时 1~2ms，占 2 秒窗口的千分之一。

---

## 还没拆的：`drivers/`（诚实说明）

`app_main.c` 里目前还留着这些驱动代码：

| 函数 | 归属 |
|---|---|
| `i2c_write_reg` / `i2c_read_regs` / `i2c_bus_unstick` | `drivers/i2c_bus.c` |
| `mpu6050_init` / `mpu6050_read` | `drivers/mpu6050.c` |
| `adc_filtered` / `adc_dma_manual_init` / `adc_config_ranks_manual` | `drivers/adc_scan.c` |
| `dht11_read` / `hcsr04_read_mm` / `bmp280_read` / `max30102_read` | `drivers/sensors_todo.c`（目前全是 TODO 桩）|

**为什么没拆**：这些函数共享一组静态句柄（`s_hi2c` / `s_hadc` / `s_huart_mb` 等），
拆分要么把这些句柄改成参数传递（调用点很多），要么引入一个共享 context 结构体。
两种改法都要动到 5 个任务函数，**回归风险明显高于收益** ——
而 `make` 只能验证"能编过"，验证不了"跑起来还对"。

**所以保留现状，但把边界标清楚了。** 如果以后要拆，按上表逐个搬即可，
每个目标文件里只依赖 `s_hi2c` / `s_hadc` 这两个句柄，搬的时候一并传进去。

---

## 构建

### 编译（GCC Makefile）

```powershell
cd cubemx/Projects
mingw32-make -j8
```

产物：`cubemx/Projects/build/ModbusGatewayF411.elf/.hex/.bin`

**⚠️ CubeMX 重新生成代码后**，`Makefile` 里的 5 个源文件和 4 个 `-I` 会被冲掉，
必须补回：

```powershell
cd firmware-stm32
pwsh -File apply_app_patch.ps1
```

这个脚本是**幂等**的（已经在就跳过）。

### 单元测试（PC 端，不需要硬件）

```powershell
cd firmware-stm32
pwsh -ExecutionPolicy Bypass -File test\build_and_run.ps1
```

89 项：CRC 向量（含全部实战帧回归）、异常码、静默丢弃、广播写、
FC10 原子写入、六轴窗口双缓冲一致性、通信看门狗。

> ⚠️ 如果 `gcc` 报"空文件都退出 1"这种怪现象，是 MSYS2 的 gcc 在这个环境里不可用，
> 不是代码问题。可以用 `-Compiler <完整路径>` 指定另一个宿主编译器。

---

## 分层是怎么验证没改坏的

拆分 `vibration_feat` 之后重新编译，**二进制字节数与拆分前完全一致**：

```
拆分前： text 46184  data 116  bss 35932
拆分后： text 46184  data 116  bss 35932   ← 一模一样
```

说明只是**代码搬家**，编译器的优化结果没变，行为没有任何差异。
