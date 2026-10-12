# Modbus Edge Gateway · 边缘智能 Modbus 网关

> STM32F411（采集 + 手写 Modbus RTU 从机） ⇄ RS485 ⇄ ESP32-S3（LVGL HMI + 端侧推理 + MQTT）
> 状态：**v0.2 规划完成，待开工**（详见 [`plan.md`](plan.md)）

## 这是什么

双机边缘智能网关：
- **下位机** `FryPi-STM32F411RET6` + FreeRTOS：12 路传感器采集、手写 Modbus RTU 从机协议栈、本地自治与故障降级
- **上位机** `ESP32-S3`：Modbus 主站轮询调度、LVGL 触摸 HMI、1D-CNN 端侧推理（TFLite Micro）、WiFi/MQTT 上云
- **可选** CAN 总线 → Modbus↔CAN↔MQTT 协议转换（v1.1）

## 文档索引

| 文档 | 内容 |
|---|---|
| [`plan.md`](plan.md) | **主计划**：架构、硬件现状、场景选型、时间表、风险 |
| [`docs/07-interview-notes.md`](docs/07-interview-notes.md) | ★★ **面试问答卡片**（本项目专用）：30 秒/3 分钟自我介绍、快速数字表、按主题的 Q&A、**最难问题（485 58%→100%）的完整标准答案**、追问防御清单、三个"千万别答错" |
| [`docs/08-basics-in-plain-language.md`](docs/08-basics-in-plain-language.md) | ★ **大白话基础**：引脚/外设/寄存器、**什么是"复用"**、**I2C 与上拉电阻**、**为什么降速反而更可靠**、DMA —— 纯生活比喻、零术语 |
| [`docs/06-decisions-and-results.md`](docs/06-decisions-and-results.md) | ★ **工程决策与关键结果**（答辩/简历素材）：架构、范围取舍、6 个技术问题与解法、量化指标、工具清单、已知限制、3 分钟演示脚本 |
| [`docs/04-plan-15days.md`](docs/04-plan-15days.md) | ★ **15 天执行计划**：范围分级（必做/加分/不做）、逐日日程、模型路线把关、屏幕选购、简历与面试题 |
| [`docs/03-bringup.md`](docs/03-bringup.md) | 分阶段上手指南 + 每阶段验收标准 + 踩坑清单 |
| [`docs/01-hardware.md`](docs/01-hardware.md) | 引脚分配表（基于 `Core.pdf` 原理图）、接线、电平与供电红线、**§4.1 已知限制（舵机/PA0）** |
| [`docs/02-modbus-protocol.md`](docs/02-modbus-protocol.md) | 寄存器映射 v0.2、CRC 测试向量、实战帧示例、一致性机制、从机状态机 |
| [`firmware-esp32/README.md`](firmware-esp32/README.md) | ESP32 侧固件说明 + **§9 排错速查**（485 从 58%→100%、SPI 屏 5 个坑、中文字库、页面操作、波形页） |
| [`firmware-stm32/CUBEMX-SETUP.md`](firmware-stm32/CUBEMX-SETUP.md) | CubeMX 逐项配置清单（HSE=25MHz、ADC/DMA、USART2+IDLE、DE 时序骨架） |
| [`docs/05-ovwatch-screen-check.md`](docs/05-ovwatch-screen-check.md) | OV-Watch 屏可用性核对（结论：与 Core 板 18-pin 座不兼容，留档） |
| [`docs/plan-v0.1.md`](docs/plan-v0.1.md) | 原始草案（留档） |

## 目录结构

```
MyProject/
├─ plan.md                       主计划
├─ README.md
├─ docs/                         设计文档（01 硬件 / 02 协议 / 03 上手 / 04 十五天计划）
├─ firmware-stm32/
│  ├─ CUBEMX-SETUP.md            CubeMX 配置清单
│  ├─ App/                       手写代码（平台无关，可在 PC 上单测）
│  │  ├─ modbus_crc.[ch]         CRC16 + 测试向量
│  │  ├─ modbus_slave.[ch]       手写 Modbus RTU 从机协议栈
│  │  └─ regbank.[ch]            寄存器表 + 六轴窗口双缓冲/锁定
│  ├─ test/test_modbus.c         78 项 PC 端单元测试
│  └─ cubemx/                    CubeMX 工程（待你生成 .ioc）
├─ firmware-esp32/               PlatformIO 工程（主站 / LVGL / 推理 / MQTT）
├─ tools/                        PC 侧脚本（CRC 对拍 / 主机压测 / 从机模拟 / 采样 / 训练）
└─ hardware/Core.pdf             核心板原理图
```

## 现在就跑得起来的东西

### 1. PC 端单元测试（不需要任何硬件）

```powershell
powershell -ExecutionPolicy Bypass -NoProfile -File firmware-stm32\test\build_and_run.ps1
```
编译 `App/` 下的手写协议栈并跑 **89 项**单元测试：CRC 向量（含 `docs/02` §6 全部实战帧回归）、
异常码、静默丢弃、广播写、FC10 原子写入、六轴窗口双缓冲一致性、通信看门狗。
**当前状态：89 PASS / 0 FAIL。** 另有 `-SyntaxOnly` 模式用 HAL 桩检查 `app_main.c`。

> ⚠️ Windows 默认禁止运行 `.ps1`，所以必须带 `-ExecutionPolicy Bypass`。
> 另外本仓库的 `.ps1` **一律只用 ASCII 字符**——PowerShell 5.1 会把无 BOM 的 UTF-8
> 中文脚本按 GBK 解析，中文字符串会变乱码并直接报语法错误。

### 2. 编译固件（已实测通过）

```powershell
cd firmware-stm32\cubemx\Projects
make -j8
```
产物在 `cubemx\Projects\build\`：`ModbusGatewayF411.elf / .hex / .bin / .map`。
**当前体积：Flash 32.9KB / 512KB，RAM 33.9KB / 128KB。**

VSCode 里更省事：用 VSCode 打开 `firmware-stm32\cubemx` 目录，`Ctrl+Shift+B` 编译，
`F5` 用 DAP-Link 调试（配置在 `.vscode\` 里，已按实际目录写好）。

### 3. 已踩过并已绕开的 CubeMX 坑

见 [`firmware-stm32/CUBEMX-SETUP.md`](firmware-stm32/CUBEMX-SETUP.md) §7。
**重新 GENERATE CODE 之后，务必跑一次 `firmware-stm32\apply_app_patch.ps1`**（幂等）。

## 快速上手（从零到双机联调）

1. **阶段 0** 打通工具链：F411 点灯 + printf；S3 点亮屏 + LVGL demo
2. **阶段 1** 传感器逐个跑通（ADC / I2C / 单总线 / 脉冲 / GPIO / PWM 六组）
3. **阶段 2** 寄存器表 + FreeRTOS 任务化 + 双缓冲窗口
4. **阶段 3** 手写 Modbus RTU 从机 → **用 PC 的 USB-485 当主机先验证**
5. **阶段 4** S3 主站轮询 + LVGL 实时显示 + 控制下发
6. **阶段 5** 本地自治与故障降级（拔掉 485 仍能报警）
7. **阶段 6** 数据采集 → 训练 → TFLite Micro 部署
8. **阶段 7** MQTT 上云 + 文档 + 演示视频

详见 [`docs/03-bringup.md`](docs/03-bringup.md)。

## 关键约束（别踩）

- Modbus 必须用 **USART2 (PA2/PA3)**，USART1 被板载 CH340 占用
- **PB12~PB15 被板载 W25Q128 占用**；套件里的 W25Q64 模块不用接
- **PA13/PA14 保留 SWD**；CubeMX 里选 Serial Wire，别选 JTAG（会吃掉 PA15/PB3/PB4）
- **485 默认 115200**：9600 下窗口读取要 1.37s、占总线约 83%，与 500ms 慢车道冲突，系统跑不起来（带宽预算见 `docs/02` §1.1）
- **C 区（0x0020~0x002F）只允许主站写，从机只读**：从机的报警/自治行为不得回写控制字，否则主站"读-改-写"会与从机互相覆盖（RMW 竞争）
- **六轴特征布局必须是 `per-feature`**（权威定义见 `docs/02` §5-E）：写成 `per-axis` 是转置关系，模型照跑不报错、但准确率掉到随机——本项目最隐蔽的坑
- MQ-2/MQ-135 的 AO、HC-SR04 的 ECHO **必须分压**后才能进 3.3V 引脚
- 舵机/电机/风扇用**外部 5V 供电并共地**，否则随机复位
- 485 总线访问在 S3 侧**必须串行化**（单一 master task）

## 演示（待补）

- [ ] 实时环境监测仪表盘
- [ ] 振动异常分类（正常 / 不平衡 / 松动）
- [ ] 手势控制执行器
- [ ] 断网本地自治 + 恢复自动重连
- [ ] MQTT 上云与远程控制
