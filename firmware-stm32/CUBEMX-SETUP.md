# CubeMX 配置清单（FryPi-STM32F411RET6）· 照抄即可

> 目标：按这份清单在 STM32CubeMX 里点一遍，就能得到一个"只剩业务代码要填"的工程。
> 所有数值都来自板子原理图（`hardware/core-board/FryPi_Core.pdf`）与 `docs/01-hardware.md` 的引脚分配。

---

## 0. 建工程：两条路径（先试 A）

### 0.A 快速路径：加载现成的 `cubemx/ModbusGatewayF411.ioc`

那份 `.ioc` 已经替你填好：**33 个引脚的分配与 GPIO 标签、上电初始电平（含 `RS485_DE` 默认接收态）、HSE 25MHz + PLL(25/200/2) → SYSCLK 100MHz、USART1/USART2 = 115200 8N1、调试口 Serial Wire**。

它**故意没写**（参数多、写错代价高，所以留给 GUI）：ADC 的扫描/排秩/DMA、TIM 的 PWM 模式与预分频、I2C1 速率、IWDG、FreeRTOS、NVIC 优先级。

> ⚠️ 该 `.ioc` 是**手工编写、未实测加载过**的。如果 CubeMX 报错打不开，直接走 §0.B 手点（约 15 分钟），别在它上面耗时间。

**步骤 1 · 加载**
`File` → `Load Project` → 选 `firmware-stm32/cubemx/ModbusGatewayF411.ioc`。
若提示 migrate / 更新固件包，选确认（让它用你本机已装的 F4 包）。

**步骤 2 · 加载后必须核对这 6 项（别跳过）**

| 核对项 | 期望值 | 在哪看 |
|---|---|---|
| HSE 频率 | **25 MHz**（不是 8MHz！填错所有串口波特率都会整体错位） | Clock Configuration 页 |
| 时钟树 | SYSCLK **100 MHz**，HCLK 100 / APB1 50 / APB2 100 | Clock Configuration 页 |
| 调试口 | PA13/PA14 显示 SWDIO/SWCLK（**不能是 JTAG**，否则吃掉 PA15/PB3/PB4） | 引脚图 |
| 引脚 | 与 `docs/01-hardware.md` 的 H2/H3 表逐个对一遍 | 引脚图（绿色标签） |
| `RS485_DE` | PA1，Output，**初始电平 Low** | 右键 PA1 → GPIO 设置 |
| 报错 | 右下角无红色报错 | 状态栏 |

**步骤 3 · 再手点这 8 项（顺序即操作顺序）**

1. **SYS** → Timebase Source：`SysTick` → **`TIM4`**（FreeRTOS 下必须，否则 `HAL_Delay`/I2C 超时会出玄学问题）
2. **USART1 / USART2** → 确认 115200、8N1、无硬件流控（已填好，扫一眼）
3. **I2C1** → Parameter Settings → I2C Speed Mode = **Fast Mode**（400kHz）
4. **TIM 通道模式**：TIM1→Channel1、TIM2→Channel1、TIM3→Channel1/2/3/4，全部选 **`PWM Generation CHx`**
   ⚠️ 光在引脚图上分配 `TIMx_CHy` **不会**自动开 PWM 模式，必须来这一页选
5. **TIM 时基**：TIM1/TIM3 → PSC=**99**、ARR=**999**（1kHz）；TIM2 → PSC=**99**、ARR=**19999**（50Hz 舵机）
6. **ADC1** → Scan Conversion Mode = **Enabled**、Continuous Conversion Mode = **Enabled**、Number of Conversions = **8**、Sampling Time = **84 Cycles**；
   **Rank 顺序必须是 `PB0, PB1, PC0, PC1, PC2, PC3, PC4, PC5`**（对应 `app_main.c` 的 `ADC_IDX_*` 宏，顺序错了会出现"吹口气烟雾值和光照值一起变"）；
   DMA Settings → Add → `ADC1` → **DMA2 Stream 0**、Mode = **Circular**、Data Width = Half Word
7. **NVIC** → 使能 `USART2 global interrupt`，抢占优先级 = **5**（其它要调 `FromISR` API 的中断也必须 ≥5，见 §3 的 FreeRTOS 优先级说明）
8. **IWDG** 勾上：Prescaler = **64**、Reload = **1000**（约 2s）；
   **Middleware → FREERTOS**：Interface = **CMSIS_V1**、`TOTAL_HEAP_SIZE` = **24576**，**不要在这里建任务**（任务由 `app_start_tasks()` 用代码创建）

**步骤 4 · 生成代码**
Project Manager → Toolchain 选你的 IDE（文件里默认写的是 Makefile）→ 勾选 *Generate peripheral initialization as a pair of .c/.h files* → `GENERATE CODE`。生成目录就是 `firmware-stm32/cubemx/`。

**步骤 5 · 把 `App/` 挂进生成的工程（VSCode + Makefile）**

在 `firmware-stm32/cubemx/Makefile` 的 `C_INCLUDES` / `C_DEFS` 定义之后追加——
```make
C_SOURCES  += ../App/modbus_crc.c ../App/modbus_slave.c ../App/regbank.c ../App/app_main.c
C_INCLUDES += -I../App
C_DEFS     += -DREGBANK_USE_FREERTOS
```
> ⚠️ **`-DREGBANK_USE_FREERTOS` 不能漏**。`regbank.h` 里靠这个宏决定临界区用不用
> `taskENTER_CRITICAL()`；漏掉它会**静默退化成"空临界区"**——采集任务和 Modbus 任务
> 就会无保护地同时改寄存器表，出现偶发数据撕裂（表现为"温度偶尔跳一个离谱值"），
> 而且编译完全正常、没有任何警告。
>
> ⚠️ CubeMX **每次重新生成都会覆盖 Makefile**，重生成后这三行要再贴一次。

**步骤 6 · VSCode 工程（配置已给你写好了）**

用 VSCode **打开 `firmware-stm32/cubemx` 这个目录**（不是整个 MyProject——Makefile 的相对路径以它为根）。
`.vscode/` 下已经放好：`tasks.json`（build/flash/size）、`launch.json`（DAP-Link + OpenOCD 调试）、
`settings.json`、`c_cpp_properties.json`（IntelliSense，含 `REGBANK_USE_FREERTOS`）。

**环境准备（你机器实测的缺口）**

| 需要什么 | 现状 | 怎么办 |
|---|---|---|
| `arm-none-eabi-gcc` | ✅ 已有：`D:\STM32\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin` | 无需处理 |
| `cmake` / `ninja` / `git` | ✅ 已有（CubeCLT / D:\Git） | 无需处理 |
| `make` | ❌ PATH 里没有（只有 `mingw32-make`） | MSYS2 里执行 `pacman -S --noconfirm make` |
| `openocd`（**DAP-Link 烧写必需**） | ❌ 没有。CubeCLT 只带 ST-LINK 的 gdbserver，**不支持 CMSIS-DAP** | MSYS2 里执行 `pacman -S --noconfirm mingw-w64-ucrt-x86_64-openocd` |
| VSCode 扩展 | ❌ 只有中文语言包 | `code --install-extension ms-vscode.cpptools marus25.cortex-debug` |

> 装完 `openocd` 后若 `tasks.json` 里调用不到，把 `settings.json` 的 `cortex-debug.openocdPath`
> 指向 `D:/MSYS2/ucrt64/bin/openocd.exe`（已填好），或把 `D:\MSYS2\ucrt64\bin` 加进 PATH。
> 不想装 `make` 的话，把 `.vscode/tasks.json` 里的 `"command": "make"` 改成 `"command": "mingw32-make"`
> 也能编过，只是 `make clean` 删不掉文件（Makefile 里 `RM = rm -f` 在 cmd.exe 下不存在）。

之后按 §4 在 `Core/Src/main.c` 里接入 `app_init()` / `app_start_tasks()`。

### 0.B 从零手动建工程（`.ioc` 加载不成功时走这条）

## 0. 工程组织（**先做这一步，否则后面手写代码会被生成器覆盖**）

```
firmware-stm32/
├─ cubemx/                ← CubeMX 工程：只有 .ioc + 生成的 Core/ 与 Drivers/（可被重新生成）
├─ App/                   ← 手写代码（协议栈/寄存器表/业务），CubeMX 永远不会碰
└─ test/                  ← PC 端单元测试
```
- MCU 选型：`STM32F411RETx`（LQFP64）
- Toolchain：**Makefile**（配 VSCode + Cortex-Debug / CLion）或 **MDK-ARM V5**（Keil），任选
- 在 CubeMX 的 `Project Manager → Code Generator` 里勾选 **"Generate peripheral initialization as a pair of .c/.h files per peripheral"**（把初始化和业务分开，好维护）
- 把 `App/` 加进 include path；在工程 defines 里加 **`REGBANK_USE_FREERTOS`**（让寄存器表自动使用 `taskENTER_CRITICAL`）

---

## 1. RCC / 时钟树（**HSE 必须是 25MHz**）

| 项 | 值 | 说明 |
|---|---|---|
| High Speed Clock (HSE) | **Crystal/Ceramic Resonator** | 原理图已确认板上晶振 = **25MHz**，填错会导致所有串口波特率一致错位 |
| Low Speed Clock (LSE) | Crystal/Ceramic Resonator | 板上 32.768kHz |
| PLL Source | HSE | |
| PLL: M / N / P / Q | **25 / 200 / 2 / 4** | SYSCLK = 100MHz；APB1 = 50MHz，APB2 = 100MHz |
| Voltage Regulator | Scale 1 | Flash latency 由 CubeMX 自动算（100MHz → 3WS） |

> ⚠️ **100MHz 与 USB 不能同时要**：25MHz HSE 下 VCO=200MHz，Q=4 只能得到 50MHz ≠ USB 要求的 48MHz。
> 若确实要用 USB：改成 **M=25 / N=192 / P=2 / Q=4 → SYSCLK 96MHz + USB 48MHz**。
> 本项目不需要 USB（调试用 USART1），保持 100MHz。

---

## 2. SYS

| 项 | 值 | 说明 |
|---|---|---|
| Debug | **Serial Wire** | 选 JTAG 会吃掉 PA15/PB3/PB4 三个 GPIO |
| Timebase Source | TIM（不要用 SysTick） | 用了 FreeRTOS 后 SysTick 归它管，HAL 需要独立时基 |
| IWDG | 打开（在 Watchdog 里配） | 预分频 **64**、重装载 **1000** → 约 2s 超时 |

---

## 3. 外设配置

### USART1 —— 调试串口 + 数据集打点
- Mode: Asynchronous，115200 8N1，无硬件流控
- NVIC: 可不使能（只用阻塞发送 + `printf` 重定向）
- 用途：`printf` 日志；阶段 6 直接把六轴窗口打到 PC 采数据（不依赖 ESP32-S3）

### USART2 —— **Modbus RTU 从机**
- Mode: Asynchronous，**115200** 8N1（与 S3 的 `MODBUS_BAUD`、tools 脚本默认值一致）
  ⚠️ 不要长期用 9600 跑业务：带宽预算见 `docs/02` §1.1——9600 下窗口读取要 1.37s，占 2s 周期的 83%，与 500ms 慢车道冲突。9600 只用于"链路能否通"的单点自检
- 引脚：PA2 = USART2_TX、PA3 = USART2_RX（`docs/01` H3 第 8 行，物理相邻，接线方便）
- DMA: 添加 `USART2_RX` → **DMA2 Stream 5 / Channel 4**，Mode = **Circular**
- NVIC: 使能 `USART2 global interrupt`，抢占优先级 **5**
  ⚠️ FreeRTOS 里 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5`，任何要调用 `xQueueSendFromISR` 的中断，优先级数值必须 ≥ 5（数值越大优先级越低）

### RS485 方向控制
- PA1 配成 `GPIO_Output`，**Initial Output Level = Low**（上电默认接收态）
- 建议给它起 Label：`RS485_DE`

### ADC1 —— 8 路模拟量
| 通道 | 引脚 | 传感器 |
|---|---|---|
| IN8 | PB0 | MQ-135 空气（AO 经分压） |
| IN9 | PB1 | 火焰（AO） |
| IN10 | PC0 | 光敏（分压） |
| IN11 | PC1 | 热敏 NTC（分压） |
| IN12 | PC2 | 电位器 |
| IN13 | PC3 | 土壤湿度 |
| IN14 | PC4 | 雨量 |
| IN15 | PC5 | MQ-2 烟雾（AO 经分压） |

- 参数：Resolution = **12-bit**；**Scan Conversion Mode = Enabled**；**Continuous Conversion Mode = Enabled**；**DMA Continuous Requests = Enabled**；Number of Conversions = **8**；Sampling Time = **84 Cycles**（MQ 系列输出阻抗高，采样时间宁长勿短）；Data Alignment = Right
- DMA: `ADC1` → DMA2 Stream 0 / Channel 0，Circular，Half Word
- 代码里 `HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw, 8)`
- ⚠️ **数组下标 = 你在这里的 Rank 顺序，不是引脚号顺序**。按上表自上而下使能，`adc_raw[0]` 就是 PB0(MQ-135)、`adc_raw[7]` 才是 PC5(MQ-2)。
  `App/app_main.c` 里的 `ADC_IDX_*` 宏按这套顺序定义好了；如果你在 CubeMX 里调换顺序，**必须同步改那些宏**，否则会出现"吹口气烟雾值和光照值一起变"这种诡异现象。
- ⚠️ 不要再把 PA4/PA5 勾成 ADC（PA4 用作数字输入、PA5 用作舵机 PWM）

### I2C1 —— 传感器总线
- I2C，**400kHz**（Fast Mode），7 位地址
- 引脚：PB6 = I2C1_SCL、PB7 = I2C1_SDA
- 从设备：MPU6050 `0x68`、MAX30102 `0x57`、BMP280 `0x76/0x77`、SSD1306 `0x3C`、VL53L0X `0x29`
- ⚠️ 先写一个 I2C 扫描程序把 0x08~0x77 遍历一遍，确认地址全在线再写驱动

### 定时器（PWM）
| 定时器 | 通道/引脚 | 配置 | 用途 |
|---|---|---|---|
| TIM1 | CH1 = PA8 | PSC=99, ARR=999 → 1kHz | 三色灯 R |
| TIM2 | CH1 = PA5 | PSC=99, ARR=19999 → **50Hz** | SG90 舵机（CCR 500~2500 → 0.5~2.5ms） |
| TIM3 | CH1 = PA6 / CH2 = PA7 / CH3 = PC8 / CH4 = PC9 | PSC=99, ARR=999 → 1kHz | 风扇、电机 ENA、RGB 的 G/B |

### GPIO
| 引脚 | 方向 | Label | 说明 |
|---|---|---|---|
| PA1 | Output (Low) | RS485_DE | 收发方向 |
| PA4 | Input | PIR_IN | HC-SR501 输出 |
| PB2 | Input | RADAR_IN | 人体雷达（BOOT1，上电需为低 → 不要给它上拉） |
| PA0 | Input / EXTI | KEY1 | 板载按键（本地打标/模式切换） |
| PC10 / PC11 | Output | MOT_IN1 / MOT_IN2 | 电机驱动方向 |
| PC12 | Output | BUZZER | 有源蜂鸣器 |
| PD2 | Output | LED_STATUS | 状态灯 |
| PA15 | Output | SPARE_OUT | 备用（继电器等） |
| PC13 | Output | LED_T | 板载 LED1~4 |
| PB8 / PB9 | 暂不配 | — | 备用（CAN1_RX/TX，v1.1） |
| PB12~PB15 | 暂不配 | — | 板载 W25Q128 的 SPI2（v0.4 断网缓存再用） |
| PC6 / PC7 | 暂不配 | — | 备用 USART6（第二路 485 / 调试） |

### FreeRTOS
- Interface: **CMSIS_V1**；`USE_PREEMPTION = Enabled`
- `TOTAL_HEAP_SIZE`：建议 **24576~32768 字节**（F411 共 128KB RAM）
- `MAX_PRIORITIES` = 7；`MINIMAL_STACK_SIZE` = 128 words
- 任务（栈按"字"算，1 word = 4 字节）：

| 任务 | 优先级 | 栈(words) | 周期 | 职责 |
|---|---|---|---|---|
| `task_acq_fast` | 5 (osPriorityHigh) | 256 | 20ms | MPU6050 50Hz 采样 → `regbank_push_sample()` |
| `task_modbus` | 4 | 512 | 事件驱动 | 解析队列里的帧 → `mb_slave_process()` → DE 发送 |
| `task_acq_slow` | 3 | 384 | 500ms | ADC/AHT/I2C 读取 → 更新 A 区 → `REG_A_SEQ++` |
| `task_output` | 3 | 256 | 事件驱动 | 执行 C 区控制字 → PWM/GPIO |
| `task_health` | 2 | 384 | 200ms | `regbank_tick()`、`regbank_sync_stats()`、IWDG 喂狗、本地自治判断 |
| `task_dsp` | 2 | **768** | 2s | 窗口满后做 FFT + 统计特征 → 写 E 区（**FFT 很吃栈，栈别省**） |

---

## 4. 手写代码如何接入（关键三步）

在 `Core/Src/main.c` 里：
```c
/* USER CODE BEGIN Includes */
#include "app_main.h"
/* USER CODE END Includes */

/* USER CODE BEGIN 2 */
app_init(&huart1, &huart2, &hadc1, &hi2c1, &htim1, &htim2, &htim3);
/* USER CODE END 2 */

/* USER CODE BEGIN RTOS_THREADS */
app_start_tasks();          /* 在 App/app_main.c 里统一创建上面 6 个任务 */
/* USER CODE END RTOS_THREADS */
```
`App/app_main.c` 里做的事：
1. `regbank_init()`
2. 填 `mb_slave_t`：`slave_addr = regbank_get_slave_addr()`、`areas = regbank_areas(&n)`、
   `get = regbank_get`、`set = regbank_set`、`check = regbank_check`、`on_block_read = regbank_on_block_read`
3. 启动 USART2 的 IDLE+DMA 接收
4. 创建任务

---

## 5. USART2 收发骨架（**最容易踩坑的地方，按这个写**）

### 接收
```c
#define MODBUS_FRAME_MAX 256
static uint8_t rx_dma_buf[MODBUS_FRAME_MAX];

void modbus_start_rx(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma_buf, sizeof(rx_dma_buf));
    /* 关掉半传输中断，否则每半帧就触发一次回调 */
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

/* HAL 在"IDLE 空闲"或"缓冲区满"时回调，Size = 本次收到的字节数 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2) {
        BaseType_t woken = pdFALSE;
        frame_t f;
        if (Size <= MODBUS_FRAME_MAX) {
            f.len = (uint16_t)Size;
            memcpy(f.data, rx_dma_buf, Size);
            xQueueSendFromISR(g_modbus_rx_q, &f, &woken);
        }
        portYIELD_FROM_ISR(woken);
        modbus_start_rx();          /* 立刻重启接收 */
    }
}
```

### 发送（DE 时序）—— **已实现，见 `App/app_main.c` 的 `modbus_send()`**

采用"**任务内等待 TC 后切 DE**"的写法，好处是**不用改 `stm32f4xx_it.c`**：Modbus 任务本来就是总线的唯一持有者，等几十毫秒（9600 下 60 字节约 63ms）不影响其它任务。

```c
RS485_DE_ON();                                  /* PA1 = 1，切到发送 */
HAL_UART_Transmit_DMA(s_huart_mb, buf, len);

t0 = HAL_GetTick();
/* ① 等 DMA 把整帧搬进数据寄存器 */
while ((HAL_UART_GetState(s_huart_mb) != HAL_UART_STATE_READY) &&
       ((HAL_GetTick() - t0) < MB_TX_TIMEOUT_MS)) { taskYIELD(); }

/* ② 再等 TC —— 移位寄存器里的最后一字节移出去。
 *    只等 DMA 完成就切 DE 是经典 bug，会把最后一字节（常常是 CRC 高字节）削掉 */
while ((__HAL_UART_GET_FLAG(s_huart_mb, UART_FLAG_TC) == RESET) &&
       ((HAL_GetTick() - t0) < MB_TX_TIMEOUT_MS)) { taskYIELD(); }
__HAL_UART_CLEAR_FLAG(s_huart_mb, UART_FLAG_TC);

RS485_DE_OFF();                                 /* PA1 = 0，切回接收 */
modbus_start_rx();                              /* ③ 必须在 DE 拉低之后重启接收 */
```

> 想改成"纯中断切 DE"（CPU 占用更低）：在 `stm32f4xx_it.c` 的 `USART2_IRQHandler` 里、调用 `HAL_UART_IRQHandler` 之前调用 `app_uart2_irq_hook()`（`App/app_main.c` 已预留空实现），在那里处理 `UART_FLAG_TC` 并切 DE。
>
> ⚠️ **三个必踩的坑**：① DE 在 DMA 完成时就清 → 最后一个字节被截断；② 接收在 DE 还没拉低时就重启 → 收到自己发的数据；③ 忘了关 DMA 半传输中断 → 半帧被当成一帧。

### printf 重定向（USART1）
```c
int __io_putchar(int ch) { uint8_t c = (uint8_t)ch; HAL_UART_Transmit(&huart1, &c, 1, 10); return ch; }
/* main 里：setvbuf(stdout, NULL, _IONBF, 0); */
```

---

## 6. 顺手要做的验证（做完再往下走）

1. **时钟对不对**：`printf("%lu\r\n", HAL_RCC_GetHCLKFreq());` 应为 100000000
2. **DMA 采样对不对**：8 路 ADC 原始值随遮挡/吹气变化
3. **I2C 扫全**：把 0x08~0x77 打印出来，与 `docs/01-hardware.md` 的地址表对照
4. **DE 时序**：用逻辑分析仪抓 TX/DE 两路，确认 DE 在整帧发完后才落下
5. **IWDG 有效**：故意在某任务里 `while(1);`，板子应在 ~2s 后复位，且 `REG_B_RESET_CAUSE` 能看出 IWDG

---

## 7. 已知的 CubeMX 6.17 坑，与我们的应对（全部实测）

> 这一节是**用真机工具链把固件编出来**才发现的。重建工程（GENERATE CODE）之后如果又出问题，先来这里对照。

| # | 现象 | 根因 | 应对 |
|---|---|---|---|
| 1 | `make` 报 `No rule to make target 'build/sysmem.o'` | CubeMX 把 `../Src/sysmem.c`、`../Src/syscalls.c` 写进了 `C_SOURCES`，**却根本没生成这两个文件** | `apply_app_patch.ps1` 自动删掉"列了但不存在"的源文件行（只在文件确实缺失时才删） |
| 2 | `main.c` 报 `'RCC_PLLQ_DIV4' undeclared` | **F4 HAL 的 PLLQ 是数字(2~15)，没有 `RCC_PLLQ_DIVx` 宏**（只有 `RCC_PLLP_DIVx`）。坑源是 `.ioc` 里 `RCC.PLLQ` 被写成了宏名 | 已同时改掉 `.ioc`（`RCC.PLLQ=4`）与生成的 `main.c`（`PLLQ = 4`） |
| 3 | 链接报 `undefined reference to vTaskDelayUntil` | CubeMX 生成的 `FreeRTOSConfig.h` 里 **`INCLUDE_vTaskDelayUntil` 默认是 0**，该函数不参与编译 | `app_main.c` 里自实现等价的 `task_delay_until()`（只用 `xTaskGetTickCount` + `vTaskDelay`，恒定可用），不再依赖这个配置项 |
| 4 | ADC 的 8 个通道配不上：界面上勾了 8 个，`.ioc` 里只有 1 个 rank | CubeMX 的 ADC Mode 面板与内部模型不同步 | `app_main.c` 的 `adc_config_ranks_manual()` 开机时用 `HAL_ADC_Init` + `HAL_ADC_ConfigChannel` **显式配置 Rank 1..8**，并顺带把采样时间设成 84 cycles、ADC 时钟设成 PCLK2/4。开关是文件顶部的 `ADC_RANKS_CONFIG_IN_CODE` |
| 5 | 运行 `.ps1` 报 "not digitally signed" | Windows 默认执行策略 | 一律用 `powershell -ExecutionPolicy Bypass -NoProfile -File xxx.ps1` |
| 6 | `.ps1` 里的中文变乱码并报语法错误 | PowerShell 5.1 按 ANSI(GBK) 读取无 BOM 的 UTF-8 脚本 | 仓库里的 `.ps1` **一律只用 ASCII**（注释也写英文） |

### 7.1 重新 GENERATE CODE 之后的固定动作

```
1) CubeMX 里 GENERATE CODE
2) 在 firmware-stm32 目录下执行（或在 VSCode 里跑任务 "app patch (重新生成后执行)"）：
   powershell -ExecutionPolicy Bypass -NoProfile -File apply_app_patch.ps1
3) 检查 .ioc 里的 PLLQ 是否仍是 RCC.PLLQ=4（若被改回宏名就改回来）
4) 检查 FreeRTOS 的 TOTAL_HEAP_SIZE 仍是 24576、IWDG 仍是 64/1000
5) 检查 ADC1 的 Clock Prescaler（代码里已强制 PCLK2/4，GUI 里填不填都不影响运行）
```

### 7.2 硬件侧还建议改的三处（不影响编译，影响接传感器时的行为）

代码已按**当前 .ioc 的实际值**配好（见 `app_main.c` 顶部的引脚映射宏），改回规划值后只需改那三组宏：

| 信号 | 规划(`docs/01`) | 当前 | 为什么建议改回去 |
|---|---|---|---|
| `RS485_DE` | PA1 | **PA5** | 现在能用；PA1 空着更规整 |
| 舵机 PWM | PA5 | **PA0** | ⚠️ **PA0 是板载 KEY1 按键那一脚**，PWM 会与按键/RC 电路互相拖累 |
| 人体感应 | PA4 | **PB2** | ⚠️ **PB2 = BOOT1**，做输入要求上电为低；某些模块上电即输出高，有进不了应用的风险 |
| 人体雷达 | PB2 | **PB10** | 顺手挪回即可 |

### 7.3 工程实际布局（和 §0 里描述的不一样，以实测为准）

```
firmware-stm32/
├─ App/                      ← 手写代码（CubeMX 永不改动）
├─ cubemx/
│  ├─ ModbusGatewayF411.ioc
│  ├─ Inc/                   ← CubeMX 生成的公共头（main.h、FreeRTOSConfig.h…）
│  ├─ Src/                   ← CubeMX 生成的源码（main.c、adc.c、freertos.c…）
│  ├─ Drivers/、Middlewares/ ← HAL 与 FreeRTOS（随项目自带）
│  ├─ Projects/              ← Makefile + 启动文件 + 链接脚本 + build/
│  └─ .vscode/               ← VSCode 构建/调试配置（用 VSCode 打开 cubemx 目录）
└─ apply_app_patch.ps1       ← 重新生成后把 App/ 补回 Makefile
```
