#pragma once
/* =============================================================================
 *  config.h — 上位机（ESP32-S3）**全部可调参数**的唯一出处
 * =============================================================================
 *  规则：src/ 下的任何文件都不允许出现裸的魔法数字。
 *        引脚、周期、超时、优先级、栈、缓冲大小、寄存器地址、位定义 —— 全在这里。
 *        换屏幕 / 换 485 模块 / 换板子，只改本文件 + platformio.ini 的 PSRAM 型号。
 *
 *  ⚠⚠ 「待确认」清单（会直接改变本文件取值，来源 plan.md §11 / docs/03 阶段 0）
 *  ---------------------------------------------------------------------------
 *  [TBD-1] 屏幕驱动 IC / 分辨率 / 颜色格式 / 是否电容触摸 —— **完全未定**
 *          → 影响 LVGL_HOR_RES / LVGL_VER_RES / LVGL_COLOR_DEPTH / GUI_USE_ESP_LVGL_PORT
 *          → 当前值只是「2.4~2.8" SPI TFT + 触摸」这一类最常见组合的占位
 *          → docs/04-plan-15days.md 的选型结论：**SPI 四线 + 3.3V 逻辑 + 带驱动 IC**；
 *            不要 8080/RGB 并口（吃 16+ 引脚与带宽），不要 MIPI（S3 不支持），
 *            不要裸屏。省钱路线 = 2.4" SPI TFT + 电容触摸模块（自己配 TFT_eSPI）；
 *            省时路线 = ESP32-S3 + LCD 一体板（走 esp_lvgl_port）。
 *  [TBD-2] 套件里的 485 模块是「带 DE/RE 的手写时序型」还是「自动收发型」
 *          → 带 DE 脚：RS485_USE_DE_PIN = 1 且 RS485_DE_PIN 填真实 GPIO
 *          → 自动收发：RS485_USE_DE_PIN = 0（本文件已给出两种写法，见 §2）
 *  [TBD-3] ESP32-S3 模组型号（N16R8 / N8R2 / 无 PSRAM）
 *          → 影响 platformio.ini 的 board_build.arduino.memory_type
 *  [TBD-4] 485 的 TX/RX 最终落在哪两个 GPIO
 *          → ESP32-S3 的 GPIO 矩阵可以任意映射，但必须避开：
 *            · GPIO33~GPIO37 = N16R8 的 **Octal PSRAM / Flash** 占用，动不得
 *            · GPIO19/20 = 原生 USB（若用 USB-CDC 调试也动不得）
 *            · 屏幕 / 触摸占用的 SPI 与 I2C 脚
 *  [TBD-5] 触摸屏是否与 485 抢同一个 UART/SPI 中断 -> 目前不抢（485 用 UART1）
 * =============================================================================
 */

/* =============================================================================
 * §0  仿真从机开关（由 platformio.ini 的 build_flags 注入，默认 0）
 * -----------------------------------------------------------------------------
 *  USE_SIM_SLAVE = 1 时：
 *    · 允许连 PC 上的 tools/modbus_slave_sim.py（下位机还没写好的时候先调主站逻辑）
 *    · 离线判定放宽到 5 次连续超时（PC 上 pymodbus 的响应抖动比真机大）
 *    · 串口打印每一帧原始 TX/RX 十六进制，方便和 docs/02 §6 的表逐字节对拍
 *  为什么用宏而不是运行期变量：仿真模式下不需要的那部分代码直接被编译器删掉，
 *  真机固件里不会残留调试分支。
 * ============================================================================= */
#ifndef USE_SIM_SLAVE
#define USE_SIM_SLAVE 0
#endif

/* =============================================================================
 * §1  RS485 / Modbus RTU 主站
 * ============================================================================= */

/* UART 编号：用 UART1。
 * 为什么不是 UART0：UART0 是 Arduino 的 Serial（USB/CH340 调试口），
 * 若拿它接 485，日志和数据会互相踩；UART2 也可以，但 UART1 是约定俗成。 */
#define RS485_UART_NUM              1

/* TX = 主机 -> 485 收发器 DI；RX = 收发器 RO -> 主机。
 * ESP32-S3 引脚可任意映射（GPIO matrix），所以这里只是"选两个没人用的脚"。
 * 默认 17/18：ESP32-S3-DevKitC-1 的排针上这两脚空闲且不在 PSRAM/USB 区。
 * [TBD-4] 上板前请对着丝印复核，并把屏幕占用的脚让开。 */
#define RS485_TX_PIN                17
#define RS485_RX_PIN                18

/* DE/RE 方向控制。
 * 为什么要有这个开关：套件里的 485 模块如果自带"自动收发"（靠 TX 电平自己判方向），
 * 它就没有 DE 脚可接 —— 那就必须把 RS485_USE_DE_PIN 置 0，否则固件会去 bit-bang
 * 一个不存在的脚，表现为"能发不能收"或干脆收不到任何东西。
 * 怎么判断：模块上如果有 DE / RE / RSE / DIR 丝印就是手写时序型，置 1。 */
#define RS485_USE_DE_PIN            1
#define RS485_DE_PIN                16
/* DE 有效电平：MAX3485 / SP3485 的 DE 高有效、RE 低有效（通常 DE 与 ~RE 短接），
 * 所以"发送 = 高"。若模块用了反相驱动，把这里改成 0。 */
#define RS485_DE_ACTIVE_HIGH        1

/* 事务之间的保护间隔（**不是** 3.5T 帧间隔，是"等从机彻底静下来"的额外余量）。
 *
 * 为什么要单独有这个东西（2026-10 实测的现象逼出来的）：
 *   9600 下从机一个 61 字节响应在线上要 **70ms**。主机发完就切回接收、收到帧就
 *   立刻发下一个请求 —— 这时从机可能刚发完最后一字节、**transceiver 的驱动器
 *   还没完全释放**，主机这一帧会被顶在总线上弄坏 → 从机静默丢弃（不回异常）。
 *   表现就是：`TX 55 RX 35 超时 20 重试 20` —— **重试次数恰好等于超时次数**，
 *   即"第一次尝试失败、重试立刻成功"；而且从机侧 `CRC错/parse0` 都不涨、
 *   主机侧 `CRC错 0`，因为坏帧在从机那儿被静默丢掉了。
 *   加了这个间隔后第一次尝试的成功率应显著上升（重试数下降）。
 *
 * ★★ 2026-10 实测：100ms **不够**，最终定为 125 倍（9600 下 ≈500ms）★★
 *   定位用的是 **tools/measure_bus_pair.ps1 双口增量对账**（同时抓两块板、
 *   用**增量**对比，避开"两侧计数器时间窗口不同"这个坑）：
 *
 *       主站发送尝试 176 次 / 从站只收到 **100** 个请求 / parse0 = 0
 *       （从站对收到的 100 个**全部应答**了）/ 主站收到 102 个响应
 *   ⇒ **丢的是"请求方向"**：76 次请求从站压根没收到，不是响应回不来。
 *   （注意：只看单侧日志是**查不出**这一点的 —— 我一开始一直以为是响应丢。）
 *
 *   机理：从机发送响应时把接收器关掉（`DIR` 拉高 → 模块内部 ~RE 同时拉高，
 *   见野火小智模块手册的「DIR = 输出输入方向使能端」）。它要等 **TC 标志**
 *   （最后一字节移出移位寄存器）才放开总线并重新武装 UART 接收 ——
 *   这段窗口在 9600 下 ≈ 66ms(响应) + 处理时间。主站间隔不够就发下一个请求，
 *   撞上这个窗口 → **整帧被静默丢弃**；等 400ms 超时后重试时从机早已就绪
 *   → **必然成功**。这就是"首试失败率恒定、重试数恒等于超时数"的根源。
 *
 *   调参实测（每档 200 秒、约 100 次事务）：
 *     25  倍(≈100ms) → 58%   ← 原值，明显不够
 *     60  倍(≈240ms) → 73%
 *     75  倍(≈300ms) → 73%
 *     **125 倍(≈500ms) → 100%** ✓ 采用
 *     150 倍(≈600ms) → 100%（与 125 倍等价，取更小者）
 *
 *   代价：每次事务多几百 ms。慢车道 500ms / 快车道 2s 的周期放得下。
 *
 * ★★★ 2026-10 最终定值：**固定 400ms，不随波特率缩放** ★★★
 *
 * ⚠ 这里踩过一个坑（记录一下，别再犯）：
 *   一开始我把间隔写成 `125 * MODBUS_T35_US(MODBUS_BAUD)`，即**与波特率成比例**。
 *   9600 下它是 500ms（成功），切到 115200 后变成 **42ms** —— **立刻退回 50% 失败**。
 *   原因：这个间隔要覆盖的是**从机的固定开销**，它**不随波特率缩小**：
 *     · 响应传输时间 —— 与波特率成正比（9600: 66ms / 115200: 6ms）
 *     · **RTOS 调度延迟 + 响应构建 + 重新武装接收 —— 固定，与波特率无关** ← 大头
 *   ⇒ 间隔必须按**绝对时间**给，**不能乘 T35**。
 *
 * 实测标定（两端同一份配置）：
 *   · 9600   + 500ms(125×T35) → 100%（870 次事务，超时 0 / 重试 0 / CRC错 0）
 *   · 115200 +  42ms(125×T35) → **50%**（静默超时，重试恒成功）← 这个坑
 *   · **115200 + 400ms** → 见下方实测（目标 100%）
 * 取 **400ms**：对 9600 也够（原来 500ms 只是略宽），对 115200 也够。
 *
 * 代价：每次事务多 400ms。**但换来的是零超时零重试** ——
 *   比"靠重试补齐"其实更快：一次重试要白等一个 150ms 超时。 */
#define MODBUS_TXN_GUARD_US         150000UL
/* 波特率：docs/02 §1「调试期 9600 → 稳定后 115200」。
 * 为什么固件里默认直接写 115200：主机侧的帧间隔容错比从机好做得多，
 * 而 600 个寄存器的窗口在 9600 下要约 1.5s，会吃掉整个 2s 周期 —— 根本跑不动。
 * 所以：**先用 9600 调通链路，确认无误后回 115200**。
 * ⚠⚠ 两个必须注意的地方：
 *   1. tools/modbus_slave_sim.py 的 `--baud` **默认是 9600**（tools/README.md 的
 *      示例也全是 9600）。用 USE_SIM_SLAVE 调试时，要么把这里改成 9600，
 *      要么给模拟器加 `--baud 115200` —— **两边必须一致**，否则只会看到满屏超时。
 *   2. 从机侧可以用 C 区 0x002D 写波特率代码在线切换，但主机改这里之后要重启。
 * 建议：真机联调第一阶段就把这一行临时改成 9600（和从机 0x002D=0 一致），
 *      链路稳定后再改回 115200 并把从机的 0x002D 写成 4。
 *
 * ★ 2026-10 实机判别实验（当前值 = 9600）★
 *   现象：115200 下主站成功率只有 ~25%，两侧字节级对账显示**物理层丢字节**
 *         （ESP32→F411 丢 32%、F411→ESP32 丢 59%），而线只有 15cm、已共地、
 *         没有终端电阻可用。
 *   措施：两端一起降到 9600（唯一不用加电阻就能加大位裕量的手段）。
 *   结论怎么读：
 *     · 9600 下丢字节消失 → 是位级电气裕量问题（驱动器/负载/A-B 相位），
 *       9600 可长期使用；想回 115200 就需要 120Ω 终端电阻 + 更好的收发器模块。
 *     · 9600 下照样丢     → 与位速率无关，是离散时序/接线问题（DE 手动方向被占空、
 *       A/B 接反、模块本身），降速无用，直接去查那三样。
 *   ⚠ 降速后**必须同步放宽**下面这几处（否则会误判成"改成 9600 更糟"）：
 *     MODBUS_T35 相关、MODBUS_TIMEOUT_MS_*、MODBUS_MAX_SINGLE_REQ_MS、
 *     MODBUS_NOTIFY_BUDGET_MAX_MS、MODBUS_FAST_PERIOD_MS。 */

/* ★★★ 2026-10 切回 115200 ★★★
 * ---------------------------------------------------------------------------
 * 之前降速到 9600 的**唯一原因**是"115200 下首试成功率只有 ~26%"。
 * 但那个问题在 2026-10 已经**定到根因并修好**了（详见 README §9.2.1）：
 *
 *   真正的原因是**主站事务间隔不够** —— 从机发送响应时会关掉接收器
 *   （DIR 拉高 → 模块内部 ~RE 同时拉高），它要等 `TC` 标志（最后一字节移出
 *   移位寄存器）才放开总线并重新武装接收。"主站发太快 → 请求整帧被从机
 *   静默丢弃 → 400ms 后重试必然成功"就是这个现象。
 *   修法是 `MODBUS_TXN_GUARD_US` 从 25 倍加到 125 倍（**与波特率成比例**，
 *   见那里的注释）。**它和波特率无关** —— 所以这个根因在 9600 上暴露、
 *   在 115200 上同样存在（而且因为时间更短，反而更容易撞上）。
 *
 * 修复后的实测（9600）：**首试成功率 100%、超时 0、重试 0、CRC错 0**（870 次事务）。
 * ⇒ 降速的前提消失了，**切回 115200**，理由是它实在好太多：
 *     · `MODBUS_TXN_GUARD_US` 从 ≈500ms 降到 ≈42ms（间隔与波特率成反比）
 *     · 快车道窗口读（600 个寄存器）从 ≈7s 降到 ≈0.6s
 *     · 慢车道周期可以从 2000ms 回到 **500ms** → 界面刷新快 4 倍
 *     · 波形页从"15 秒才动一下"变成"2 秒一轮"
 *
 * ⚠ 切回是**两端一起改**（主机 ESP32 + 从机 F411 的 USART2），
 *   漏一处就会变成"一帧都收不到"。清单见 README §9.2 / §9.2.2。 */
#define MODBUS_BAUD                 115200

/* 从机地址。docs/02 §1：F411 = 从机地址 0x01。
 * 0x00 是广播（只对写有意义、从机不响应），主机读操作绝不能用 0x00。 */
#define MODBUS_SLAVE_ADDR           0x01

/* 3.5 字符的帧静默判尾时间（8N1，1 字符 = 11 bit）：
 *   3.5 * 11 / baud 秒 = 38.5 / baud 秒 = (38500000 / baud) 微秒
 * 115200 → 334us；9600 → 4010us（与 docs/02 §1 的表一致）。
 * 主机侧用它做"两次发送之间的静默"：不等够，从机会把上一帧的尾巴和这一帧的头粘着解析。 */
#define MODBUS_T35_US(baud)         (38500000UL / (baud))
/* 帧与帧之间额外多留一点余量（从机软件兜底定时器是 5ms，我们不和它抢边界）。
 * ★ 9600 下的关键修正（原来固定 200µs 是错的）★
 *   3.5T 是**按波特率缩放**的：115200 时 334µs，9600 时 4010µs。
 *   原来固定加 200µs，在 9600 下帧间的"静默"只有 200µs ≪ 3.5T，
 *   从机（靠 USART IDLE 判帧尾）会收到"上帧尾巴 + 这一帧头"粘在一起的两帧 ✗。
 *   在 115200 下 334+200=534µs 侥幸够用，降速后必须让余量也跟着 T35 走。 */
#define MODBUS_INTER_FRAME_GUARD_US MODBUS_T35_US(MODBUS_BAUD)

/* ---- 慢车道：docs/02 §6 ① / docs/02 §9 --------------------------------------
 * 500ms 一次读 0x0000~0x001B（A 区 16 + B 区 12 = 28 个寄存器，一帧读完）。
 * 为什么是 28 而不是分两次：0x0000~0x001B 连续，125 上限内，一帧最省总线时间；
 * 而且 A 区和 B 区（含帧计数、报警位）同帧返回，天然保证"数据与状态一致"。
 * 9600 下这一帧的线上时间只有 8+61=69 字节 ≈ 79ms，500ms 周期绰绰有余 ✓
 * ★ 但 9600 实测（2026-10）把这个结论推翻了：
 *   一次"全超时"的请求 = timeout × 3 = 400×3 = 1.2s，已经**超过 500ms 周期** ✗ →
 *   队列被填满（`等待处理 8` → `总线请求队列满`），慢车道自己也再排不进去。
 *   所以在 9600 下周期必须放宽到 ≥ 1 次/秒，等回到 115200（请求 ~180ms）再改回 500。
 * ★ 实测再收紧一档：必须 > "一次请求的最坏耗时" ★
 *   9600 下最坏 = 超时 400ms + 一次重试 63ms ≈ 470ms。取 1000ms 时队列仍在涨
 *   （心跳里 `等待处理` 稳定在 4），说明 1000 还是不够 —— 因为失败率高的时候
 *   一个周期内要花掉 400(超时)+63(重试)+400(再超时) ≈ 900ms，已经逼近 1000ms。
 *   取 2000ms，让"最坏一次请求"只占周期的 1/4，队列才有机会排空。 */
#define MODBUS_SLOW_PERIOD_MS       1000
#define MODBUS_SLOW_START           0x0000
#define MODBUS_SLOW_REG_COUNT       28          /* 0x0000~0x001B */

/* ---- 快车道：docs/02 §6 ② / §7 / §9 ----------------------------------------
 * 2s 一次：读窗口头 0x0100~0x0103 → 5 次分块读 600 个寄存器 → 再读 0x0101 校验序号。
 * 为什么 2s：F411 是 50Hz 采样、100 样本一窗 → 2.0s 出一个新窗口；主机按 2s 拉，
 * 恰好一个窗口一个周期，既不积压也没浪费带宽（600 寄存器 @115200 约 125ms，占 6%）。
 *
 * ★ 9600 判别实验期间：2s 必须放宽到 10s ★
 *   600 个寄存器在 9600 下光"数据字节"就要 600×2×11/9600 ≈ 1.38s（不含帧头与响应间隔），
 *   2s 周期装不下 7 帧往返 → 会被判成大面积超时，把"降速有没有用"这个实验本身搞脏。
 *   10s 周期下总线占用 ≈ 1.6s/10s = 16%，安全。
 *   ⚠ 回到 115200 时把这里改回 2000，否则波形页/特征更新会慢 5 倍。
 *   15s 而不是 10s 的理由：一次窗口读是 7 个**串行**请求，每个"全超时"要
 *   700ms×3 = 2.1s；7 个就是 14.7s。周期必须大于这个值，否则上一轮还没读完
 *   下一轮又来了 —— 这正是 9600 实测里队列被填满的直接原因。
 *
 * ★★ 2026-10 切回 115200 后的**实际定值：5000ms**（不是上面说的 2000）★★
 *   一次窗口读是 **7 次串行事务**（窗口头 + 5 分块 + 序号复校验），每次成功后
 *   要等 `MODBUS_TXN_GUARD_US` = **400ms** ⇒ **光间隔就 2.8 秒**；再加 6 帧
 *   260 字节的传输（115200 下 ≈23ms/帧）⇒ 一次完整窗口读 ≈ **3.1~3.3 秒**。
 *   周期 2000ms < 3.3s ⇒ **每轮都读不完就被下一次唤醒打断**，
 *   现象是界面永远停在"等待窗口..."、一条曲线都不画（**实测踩到**）。
 *   取值原则：**周期必须 > 一次完整窗口读的最坏耗时**。
 *   ⚠ 想回到 2 秒周期，必须先降低 `MODBUS_TXN_GUARD_US`
 *     （该值现在服务于"从机重新武装接收"，标定过程见那里的注释）。 */
#define MODBUS_FAST_PERIOD_MS       5000
/* ★ 长周期任务的"切片喂狗"节拍 ★
 * 踩过的坑（2026-10 实测，代价是连着重启十几次）：
 *   任务一旦用 app_wdt_register() 登记进看门狗，**两次喂狗间隔超过
 *   WDT_TIMEOUT_S(5s) 就会被 panic 复位** —— 所以"周期 > 5s 的任务"绝不能
 *   一口气 vTaskDelayUntil(period) 睡完，否则它连自己的第一行日志都来不及打，
 *   日志上表现为 `task_wdt: - mb_fast (CPU 0)` + `rst:0xc`，看着像任务卡死，
 *   其实是"睡太久没喂狗"。
 *   115200 时代快车道周期 2s（< 5s）碰巧不触发，一降速/一改周期就暴露。
 * 做法见 task_modbus_fast()：按本值切片睡眠，每片都 app_wdt_feed()，
 * 累加到 MODBUS_FAST_PERIOD_MS 才真正干一次活。
 * 取值 500ms：是 5s 超时的 1/10，留足余量；又足够粗，空转开销可忽略。 */
#define BUS_WDT_FEED_PERIOD_MS      500
#define MODBUS_WIN_HEAD_START       0x0100
#define MODBUS_WIN_HEAD_COUNT       4           /* 0x0100 头 / 0x0101 序号 / 0x0102 采样率 / 0x0103 时间戳 */
#define MODBUS_WIN_SEQ_ADDR         0x0101      /* 一致性校验用（docs/02 §7 第 5 条） */
#define MODBUS_WIN_DATA_START       0x0104      /* 100 样本 × 6 轴，行优先 */
#define MODBUS_WIN_DATA_REGS        600         /* 0x0104~0x035B */

/* 分块（docs/02 §6 表②，逐帧 CRC 已在该文档核算过）：
 *   125 + 125 + 125 + 125 + 100 = 600
 * 为什么是 125：FC03 单帧最多 125 个寄存器（PDU ≤ 253B，docs/02 §4）；
 * 为什么最后一块是 100：600 = 4×125 + 100，剩下多少读多少，凑不满就少读，
 * 不能补到 125（会越过 0x035B 读到未映射区 → 从机回异常 02）。 */
#define MODBUS_WIN_CHUNK_COUNT      5
#define MODBUS_WIN_CHUNK_MAX        125
#define MODBUS_WIN_CHUNK_TOTAL      600         /* 编译期自检用，必须等于 MODBUS_WIN_DATA_REGS */

/* ---- 特征通道（E 区）：docs/02 §5-E / §6 ③ / plan.md §5.3 第 4 条 -----------
 * 36 个寄存器 = 6 轴 × (均值/标准差/峰峰值/过零率/主频/主频带能量占比)。
 * 和原始窗口同周期（2s）拉，作为"带宽受限时的降级通道"，也是对比实验的素材。 */
#define MODBUS_FEAT_START           0x0400
#define MODBUS_FEAT_COUNT           36          /* 0x0400~0x0423 */
#define MODBUS_FEAT_PERIOD_MS       2000

/* ---- 超时与重试：docs/02 §9 的调度表 ---------------------------------------
 * 慢车道 50ms / 快车道 60ms / 写车道 50ms，读重试 2 次、写重试 1 次。
 * 为什么是 50~60ms 这个量级：115200 下一帧 28 寄存器的响应约 5.5ms、
 * 125 寄存器约 22ms，从机就算忙也该在 30ms 内回；给到 50~60ms 已留了 2 倍以上余量。
 * 再大就会拖慢整个调度表（快车道 6 帧 × 3 次重试 × 60ms = 1.08s，2s 周期就危险了）。
 * 为什么写只重试 1 次：写操作必须幂等——只能整块重发，绝不允许"半个 FC10"
 * （docs/02 §9 的原话），重试次数越少风险越小。
 *
 * ★ 9600 判别实验期间的修正（这几行是"降速后仍然满屏超时"的常见原因）★
 *   9600 下**光是把响应字节发完**就要很久，超时必须按"最长一帧的线上时间"给：
 *     28 寄存器响应 61 字节  → 63.5ms
 *     36 寄存器响应 77 字节  → 80.2ms
 *     125 寄存器响应 260 字节 → 271ms   ← 快车道分块读的大头
 *   所以慢车道给 200ms（=63.5ms 线上时间 + 从机处理余量），
 *   快车道给 500ms（=271ms + 余量）。留 2~3 倍余量是刻意的：
 *   从机那边 50Hz 采样任务的临界区、printf 互斥锁都会占时间。
 *   ⚠ 回 115200 时改回 50/60，否则超时形同虚设（失败要等半秒才发现）。 */
#define MODBUS_TIMEOUT_MS_SLOW      150
#define MODBUS_TIMEOUT_MS_FAST      200
#define MODBUS_TIMEOUT_MS_WRITE     150
#define MODBUS_RETRY_READ           2
#define MODBUS_RETRY_WRITE          1

/* 快车道"读完发现窗口序号变了 → 整窗丢弃重读"的最大次数（docs/02 §7 第 5 条）。
 * 为什么是 2：序号失配意味着 6 个帧（头 + 5 块）里恰好跨了窗口切换，
 * 概率本来就不高；连丢 3 次说明从机的窗口周期和主机不同步（比如 0x002E 被改过），
 * 这时候继续重读只是浪费总线，应该让诊断页把 seq_mismatch_count 显示出来给人看。 */
#define MODBUS_WIN_SEQ_RETRY        2

/* 连续多少次超时判定"节点离线"：docs/02 §9 规定 3 次。
 * 为什么不能在第一次超时就灰屏：485 上偶发一个 CRC 错（电机干扰）很常见，
 * 3 次连续超时（≈ 慢车道 1.5s）才既有反应速度又不会误报。
 * 仿真从机（PC 上的 pymodbus）响应抖动比真机大得多，放宽到 5 次，
 * 否则调试时会看到界面反复"离线→在线"闪。 */
#if USE_SIM_SLAVE
#define MODBUS_OFFLINE_TIMEOUTS     5
#else
#define MODBUS_OFFLINE_TIMEOUTS     3
#endif

/* 请求队列深度与投递等待：
 * 深度 8 的依据 = 最坏情况同时排队的请求数上界：
 * 慢车道 1 + 快车道每周期最多 7（头 + 5 块 + 序号校验）+ 写车道若干 + 余量。
 * 快车道那 7 个是串行同步投递的（投一个等一个），所以实际并发需求很小，
 * 8 已经非常宽裕；再大只是白占内部 RAM。 */
#define MODBUS_REQ_QUEUE_LEN        8
#define MODBUS_REQ_QUEUE_WAIT_MS    20
/* ---- 同步等待的预算（把"排队"这件事算进去）---------------------------------
 * 总线是串行的：请求要等前面所有请求做完才轮到自己，预算必须覆盖这个过程。
 *   budget = (入队时排在前面的请求数 + 1) × MODBUS_MAX_SINGLE_REQ_MS + SLACK
 * 为什么不是 (timeout_ms × 尝试次数)：那只算了"自己"，没算排队 ——
 *   实测（这个工程上踩过）：慢车道 50ms×3+30 = 180ms 的旧预算，
 *   而快车道一次要连发 7 个请求（头 + 5 块 + 序号校验），
 *   排在它们后面的慢车道请求**经常在预算内等不到完成通知**，
 *   被误判成超时；上层看到的现象就是"队列像没人处理"。
 *
 * MODBUS_MAX_SINGLE_REQ_MS = 一个请求占住总线的最坏时长（实测基准）。
 *
 * ★★★ 2026-10 修正：**必须把 MODBUS_TXN_GUARD_US 算进来** ★★★
 *   它现在是 **400ms 固定值**（见那里的实测标定），而它**属于每次尝试的固定开销**：
 *       单请求最坏耗时 ≈ (间隔 + 传输 + 驱动开销) × 尝试次数
 *                      ≈ (400 + 23 + 25) × 3 ≈ 1344ms
 *   我一开始只写了 200ms（只算了传输），结果 `budget` 被算成
 *   `(0+1)×200 + 150 = 350`，再被上限 600 封顶 ——
 *   **比一次请求的真实耗时还短** → `ulTaskNotifyTake()` 提前返回 →
 *   上层拿到假的 `MB_ERR_TIMEOUT`：**从机明明回了，主站却说超时**。
 *   现象（实测踩到）：快车道读窗口头**恒定失败（成功 0 / 失败 N）**、
 *   状态码 = 1（TIMEOUT），而慢车道读数 100% 成功 —— 因为慢车道读小数据、
 *   单请求耗时恰好没超预算。
 *   ⚠ 这类"假超时"极难查：总线侧一切正常、从机侧也一切正常。
 *
 *   取值：400(间隔) + 100(传输+开销) + 余量 ≈ **600ms**。
 *   ★ 9600 下要更大（间隔 400ms 不变，但传输 298ms）→ 800ms。 */
#if MODBUS_BAUD <= 19200
#define MODBUS_MAX_SINGLE_REQ_MS    800
#else
#define MODBUS_MAX_SINGLE_REQ_MS    600
#endif
#define MODBUS_NOTIFY_SLACK_MS      200
/* 预算上限：宁可偶尔误判一次超时（上层会重试/丢弃），也绝不能让调用方
 * 睡到看门狗（WDT_TIMEOUT_S = 5s）超时被复位 —— 那会把整块板子拖重启。
 * 队列深 8 → 最坏 (8+1)×600+200 = 5.6s 会被封到这里。
 * ⚠ 上限必须**明显小于看门狗**（WDT_TIMEOUT_S = 5s），而且理由很硬：
 *   调用方 `ulTaskNotifyTake()` 阻塞期间**没法喂狗**。上限开大时，
 *   一次窗口读（快车道连发 7 个请求）里只要有 2 个请求各等满上限，
 *   mb_fast 就超过 5s 没喂狗 → **看门狗复位**
 *   （2026-10 在 9600 下实测踩到：`task_wdt: - mb_fast (CPU 0)` + rst:0xc）。
 *
 * ★ 2026-10 从 600 提到 **1500** ★
 *   600 连"一个请求"都盖不住（单请求最坏 ≈1344ms），必须 ≥ 单请求耗时 × 1.5。
 *   1500ms 仍在看门狗 5s 的安全范围内（一次窗口读里最多 2 个请求等满，
 *   中间每次请求之间都会 `app_wdt_feed()`，见 window_read_once）。 */
#define MODBUS_NOTIFY_BUDGET_MAX_MS 1500
/* 寄存器地址与位定义（**逐条抄自 docs/02 §5，禁止自行发明**） */

/* A 区 环境量 0x0000~0x000F */
#define REG_TEMP                    0x0000      /* int16 ×10 ℃ */
#define REG_HUMI                    0x0001      /* uint16 ×10 %RH */
#define REG_PRESS                   0x0002      /* uint16 ×10 hPa */
#define REG_LIGHT                   0x0003      /* uint16 ADC 0~4095 */
#define REG_SMOKE                   0x0004      /* uint16 ADC 0~4095 */
#define REG_AIR                     0x0005      /* uint16 ADC 0~4095 */
#define REG_FLAME                   0x0006
#define REG_SOIL                    0x0007      /* 0~100 */
#define REG_RAIN                    0x0008
#define REG_DIST                    0x0009      /* mm */
#define REG_PIR                     0x000A      /* 0/1 */
#define REG_RADAR                   0x000B      /* 0/1 */
#define REG_HR                      0x000C      /* bpm, 0 = 无效 */
#define REG_SPO2                    0x000D      /* %, 0 = 无效 */
#define REG_POT                     0x000E
#define REG_SAMPLE_SEQ              0x000F      /* A 区每轮采集 +1，主机判数据新旧 */

/* B 区 状态与统计 0x0010~0x001F */
#define REG_STATUS1                 0x0010
#define REG_STATUS2                 0x0011
#define REG_FW_VER                  0x0012
#define REG_FRAME_OK                0x0013
#define REG_FRAME_CRCERR            0x0014
#define REG_FRAME_EXC               0x0015
#define REG_UPTIME_MIN              0x0016
#define REG_RESET_REASON            0x0017
#define REG_COMM_WDT_CNT            0x0018
#define REG_MODE                    0x0019      /* 0 = 本地自治，1 = 远程受控 */
#define REG_ALARM                   0x001A
#define REG_WIN_STATUS              0x001B      /* 同 0x0100 头 */

/* C 区 控制与配置 0x0020~0x002F（读写） */
#define REG_CTRL1                   0x0020
#define REG_FAN_PWM                 0x0021      /* 0~1000 */
#define REG_MOTOR_PWM               0x0022      /* int16 −1000~+1000 */
#define REG_SERVO_ANGLE             0x0023      /* 0~180，0xFFFF = 保持 */
#define REG_RGB_R                   0x0024      /* 0~255 */
#define REG_RGB_G                   0x0025
#define REG_RGB_B                   0x0026
#define REG_ALARM_TEMP_HI           0x0027      /* ×10℃ */
#define REG_ALARM_SMOKE_HI          0x0028
#define REG_ALARM_HR_LO             0x0029
#define REG_COMM_WDT_MS100          0x002A      /* ×100ms，0 = 禁用 */
#define REG_SAFE_BITMAP             0x002B
#define REG_SLAVE_ADDR              0x002C
#define REG_BAUD_CODE               0x002D      /* 0=9600 1=19200 2=38400 3=57600 4=115200 */
#define REG_SAMPLE_PERIOD           0x002E      /* ms */
#define REG_COMMAND                 0x002F      /* 0xA5A5 恢复默认 / 0x5A5A 软重启 */

/* 控制字 1（0x0020）位定义 —— docs/02 §5 C 区 */
#define CTRL1_BEEP                  0x0001      /* bit0 蜂鸣器 */
#define CTRL1_FAN_EN                0x0002      /* bit1 风扇使能 */
#define CTRL1_MOTOR_EN              0x0004      /* bit2 电机使能 */
#define CTRL1_RGB_EN                0x0008      /* bit3 RGB 使能 */
#define CTRL1_SERVO_EN              0x0010      /* bit4 舵机使能 */
#define CTRL1_ALLOW_LOCAL           0x0020      /* bit5 允许本地自治 */
#define CTRL1_CLEAR_STATS           0x0040      /* bit6 清统计（自清） */
#define CTRL1_SAVE_FLASH            0x0080      /* bit7 保存配置到 Flash */
#define CTRL1_BEEP_LONG             0x0100      /* bit8 蜂鸣器长鸣 */
#define CTRL1_SAFE_SHUTDOWN         0x0200      /* bit9 全部输出安全关断 */
#define CTRL1_FORCE_WIN             0x0400      /* bit10 强制窗口重采 */

/* 状态字 2（0x0011）位定义（只列用到的） */
#define STATUS2_WIN_OVERRUN         0x0004      /* bit2 窗口缓冲溢出 */
#define STATUS2_LOCAL_MODE          0x0008      /* bit3 处于本地自治 */
#define STATUS2_COMM_WDT            0x0010      /* bit4 通信看门狗超时 */

/* D 区 六轴采样窗口 */
#define REG_WIN_HEAD                0x0100      /* bit15=1 有效窗口就绪；低 8 位 = 有效样本数 */
#define REG_WIN_SEQ                 0x0101
#define REG_WIN_RATE_X10            0x0102      /* ×10 Hz */
#define REG_WIN_TS_10MS             0x0103
#define WINDOW_SAMPLES              100         /* 每窗 100 样本 */
#define WINDOW_AXES                 6           /* ax ay az gx gy gz */
#define WINDOW_HEAD_VALID_BIT       0x8000

/* E 区 特征向量 */
#define REG_FEAT_BASE               0x0400

/* =============================================================================
 * §2  任务划分 / 优先级 / 栈 / 核绑定（plan.md §7.2）
 * -----------------------------------------------------------------------------
 *  ESP32-S3 双核：core 0 跑"实时 + 通信"，core 1 只跑 GUI。
 *  为什么 GUI 独占 core 1：LVGL 刷屏 + SPI DMA 对时序敏感，和 WiFi 的射频任务
 *  抢 core 0 会看到明显的刷新抖动（docs/03 阶段 4 的坑列表里写得很清楚）。
 *
 *  ⚠ 栈大小单位是**字节**（ESP-IDF 的 xTaskCreatePinnedToCore 就是字节，
 *    不是像 vanilla FreeRTOS 那样按 word 算）——写错一位就会栈溢出。
 *  上板后一律用 uxTaskGetStackHighWaterMark() 量余量，留够 30% 以上。
 * ============================================================================= */

#define CORE_BUS                    0           /* 总线 + 采集 + 命令 + 网络 */
#define CORE_INFER                  0
#define CORE_NET                    0
#define CORE_GUI                    1           /* GUI 独占 */

/* 优先级数值越大越高。基准：Arduino 的 loopTask = 1，
 * WiFi/LWIP 内部任务约 18~23 —— 我们的任务必须**低于** WiFi，
 * 否则会把射频任务饿死，表现为"WiFi 连不上但 CPU 很忙"。 */
#define TASK_PRIO_BUS               6           /* 唯一总线持有者，最高 */
#define TASK_PRIO_SLOW              5           /* 慢车道（只是生产者，阻塞等总线） */
#define TASK_PRIO_FAST              5           /* 快车道（同上） */
#define TASK_PRIO_CMD               5           /* 写车道：消费 cmd_queue */
#define TASK_PRIO_GUI               4           /* core1 上最高，没人跟它抢 */
#define TASK_PRIO_INFER             3           /* 推理可以被总线抢占（有 2s 的余量） */
#define TASK_PRIO_NET               3
/* 串口命令行：**最低优先级**。它对实时性没有任何要求（人敲命令的节奏），
 * 而且只在有输入时才干活，平时就是 50ms 一次的 delay。见 console.cpp。 */
#define TASK_PRIO_CONSOLE           1

#define TASK_STACK_BUS              4096
#define TASK_STACK_SLOW             3072
#define TASK_STACK_FAST             4096        /* 里面有 600×int16 的解包循环，多给一点 */
#define TASK_STACK_CMD              3072
#define TASK_STACK_GUI              8192        /* LVGL 控件树 + 图表重绘，最容易栈溢出 */
#define TASK_STACK_INFER            12288       /* TFLite Micro 的调用栈比想象中深 */
#define TASK_STACK_NET              8192        /* WiFi + MQTT + JSON 解析 */
/* 命令行：一行缓冲 64 字节 + printf 栈，3KB 足够 */
#define TASK_STACK_CONSOLE          3072

/* 队列深度 */
#define QLEN_ACQ                    4           /* 窗口/特征 → 推理 */
#define QLEN_UI                     8           /* 状态 → GUI */
#define QLEN_CMD                    8           /* GUI/MQTT → 总线写请求 */

/* 命令的保鲜期：超过这个时间的命令直接丢掉。
 * 为什么需要：GUI 按钮/MQTT 下行的命令在 cmd_queue 里排队时，如果总线正在做
 * 一次 6 帧的窗口读（最坏 1s+），等轮到这条命令时人早就松手了。
 * 3s 是"还能接受的操作延迟"上限，超过就该让用户重按而不是执行一条幽灵命令。 */
#define CMD_MAX_AGE_MS              3000

/* =============================================================================
 * §3  看门狗
 * -----------------------------------------------------------------------------
 *  超时 5s：慢车道 500ms、快车道 2s，5s 足够容下快车道一次"3 次重试全超时"
 *  （6 帧 × 3 × 60ms ≈ 1.08s）而不误复位。IWDG 式的硬复位是"活得久"的底线
 *  （plan.md §3.1 从机侧也强制要求 IWDG，主机侧同理）。
 * ============================================================================= */
#define WDT_TIMEOUT_S               5

/* =============================================================================
 * §4  LVGL
 * ============================================================================= */

/* [TBD-1] 屏幕已选定（2026-10）：**2.4" IPS 240×320，ST7789（或 ILI9341），8 针 SPI 模块**
 *   为什么是它：¥28.8 的模块（中景园/黄保凯），4-SPI 接口 + 2.54mm 单排排针，
 *   能直接插杜邦线，不需要转接板。（另一款 ¥34.5 的带 GT30L32S4W 硬件中文字库，
 *   也可以；两者 TFT_eSPI 都原生支持。）
 *   ⚠ 这**不是**裸屏：裸屏只有一根 FPC 排线、没有排针，买错就点不亮。
 *   换成 480×320 ST7796 / 240×240 GC9A01 只改下面这两行 + User_Setup.h。 */
/* ★ 屏幕方向：**240×320 竖屏**（用户要求）★
 * ---------------------------------------------------------------------------
 * 2026-10 最终的**三处配对关系**（今天绕最久的地方，必须一起理解）：
 *   ① **面板物理/显存几何**（gui.cpp 的 class Lcd）：
 *        panel_width/height = **320×240**、memory 同 —— **硬件真实像素数**。
 *   ② **MADCTL**（由 gui.cpp 的 TFT_ROTATION 决定）：把 320×240 的硬件
 *        旋转成用户想要的方向。
 *   ③ **LVGL canvas**（下面这两个宏）：**必须等于库报告的可寻址尺寸**，
 *        否则 LVGL 按 A 尺寸排版、库按 B 尺寸推进去 → 内容只占一部分/错位。
 *        ⚠ "库报告的尺寸"以串口日志 `硬件参数：width=… height=…` 为准，
 *          不要靠推算！今天多次栽在这里。
 *
 * 调试口诀（照着做就不会再绕）：
 *   1. 烧进去，看日志的 `硬件参数：width=W height=H`
 *   2. 把下面两个宏改成 **W 和 H**（一个字都不要差）
 *   3. 如果方向不对，只改 gui.cpp 的 TFT_ROTATION，**再回到第 1 步**
 * =========================================================================== */
#define LVGL_HOR_RES                320
#define LVGL_VER_RES                240
#define LVGL_COLOR_DEPTH            16          /* RGB565。SPI 屏几乎都是 16 位，省一半带宽 */

/* 绘制缓冲高度（行）。320×40×2B = 25.6KB。
 * 为什么用"部分缓冲 + 40 行"而不是全屏双缓冲：全屏双缓冲 320×240×2×2 = 300KB，
 * 只能放 PSRAM；而 PSRAM 走 cache，SPI DMA 从 PSRAM 搬数据比从内部 RAM 慢且
 * 会和 WiFi 抢 cache（docs/03 阶段 4 的坑）。40 行 = 6 次 flush 刷完一屏，
 * 肉眼完全看不出分块，却把缓冲塞进内部 DMA 内存。
 * 只有在内部 RAM 实在不够时才把下面的 LVGL_DRAW_BUF_IN_PSRAM 打开。 */
#define LVGL_DRAW_BUF_LINES         40
#define LVGL_DRAW_BUF_PIXELS        (LVGL_HOR_RES * LVGL_DRAW_BUF_LINES)
#define LVGL_DRAW_BUF_IN_PSRAM      0

/* 驱动路径二选一（见 gui.cpp 的 TODO）：
 *   1 = 一体板（板载屏 + 官方 BSP）：走 esp_lvgl_port，由 BSP 提供 esp_lcd 驱动
 *   0 = 分体（自购 SPI 屏）：TFT_eSPI / LovyanGFX 自己写 flush_cb + 触摸 read_cb
 * [TBD-1] 取决于最终拿到的是哪种板子。 */
#define GUI_USE_ESP_LVGL_PORT       0

/* GUI 任务节拍：LVGL 要求 lv_timer_handler 被 5~30ms 周期调用。
 * 10ms 是手感与 CPU 占用的平衡点（100Hz 内部定时器精度足够，CPU 占比 <5%）。 */
#define GUI_TICK_PERIOD_MS          10
#define GUI_TAB_BAR_H               40

/* 波形页：显示多少个点。
 * 窗口有 100 个样本，但屏幕宽度只有 320px 还要放 3 条轴 —— 显示 50 点
 * （即每 2 个样本取 1 个，50Hz → 25Hz 显示）刚好，既不糊也不浪费像素。 */
#define GUI_WAVE_POINTS             50
/* 曲线 Y 轴范围：**只在"不去均值"模式下用作初值** ——
 * 实际范围由 gui.cpp 每次自适应计算（见 §波形页 的去均值说明）。
 * MPU6050 ±2g 量程时 1g ≈ 16384 LSB，所以 ±20000 能容下静止+晃动。 */
#define GUI_WAVE_Y_MIN              (-20000)
#define GUI_WAVE_Y_MAX              (20000)

/* ★ 是否"去均值显示"的阈值（counts）★
 *   加速度原始值里**重力占大头**，且三个轴的重力分量方向不同
 *   （实测 ax≈-10700 / ay≈-10500 / az≈+6350）。如果直接画原始值，
 *   三条线各占 Y 轴一端、永不交叠，而波动只有 ±200 —— 看起来就是三条直线。
 *   ⇒ 当"窗口内峰峰值 ≤ 本阈值"时，显示 `v - 本轴均值`（只看波动）；
 *     超过阈值（剧烈振动/明显晃动）则显示原始值（保留方向信息）。
 *   2000 ≈ 0.12g，是个经验分界：小于它基本是噪声+微振。 */
#define GUI_WAVE_DEMEAN_SPAN        2000
/* 波形刷新节流：2s 一个新窗口，GUI 每 10ms 醒一次没必要每帧都重画曲线 */
#define GUI_WAVE_MIN_INTERVAL_MS    200

/* 诊断页刷新周期：它是"人盯着看的慢变化数据"，1s 一次足够，
 * 刷太快反而看不清（而且 mb_get_stats 要进临界区，别让 GUI 高频打扰 core0）。 */
#define GUI_DIAG_REFRESH_MS         1000

/* =============================================================================
 * §3.x  页面控制（本项目**没有触摸屏、没有按键**）
 * -----------------------------------------------------------------------------
 *  三个界面（数据 / 波形 / 诊断）靠两种方式切换：
 *    ① **自动轮播**：每 GUI_PAGE_CYCLE_MS 切下一页
 *    ② **手动**：串口命令（console.cpp）/ MQTT 下行（net.cpp）
 *  手动切换会**重置轮播计时**，不会出现"刚切完又被轮播切走"。
 *
 *  轮播周期怎么选：
 *    · 波形页在 9600 总线下 **15 秒**才更新一轮窗口数据（见 §9.2），
 *      周期比 15s 短的话，切到波形页可能看到的是同一帧 → 失去意义。
 *    · 10 秒是个折中：数据页/诊断页信息量够看，波形页也能偶尔撞到新数据。
 *    · 等 485 加上 120Ω 终端电阻、快车道回到 2 秒后，可以调小到 5 秒。
 * ============================================================================= */
#define GUI_PAGE_CYCLE_MS           10000

/* 页面总数（数据/波形/诊断）。改界面时这里要跟着改。 */
#define GUI_PAGE_COUNT              3

/* 调试串口命令行（console.cpp）：
 *   1 = 起一个低优先级任务解析 `page=1` / `next` / `help` 之类的命令。
 *   命令行走 **USB 调试口（115200，COM6）**，和 485 总线无关、互不影响。
 *   ⚠ 这是"没有按键也没有触摸"时唯一的直接操作手段，默认开着。 */
#define CONSOLE_ENABLE              1

/* =============================================================================
 * §4.1  本地 OLED（SSD1306 0.96" 128×64，I2C）—— gui.cpp 的"路径 C"
 * -----------------------------------------------------------------------------
 *  为什么要有它：LVGL 那条路（路径 A/B，见 gui.cpp）卡在"屏幕型号未定"上，
 *  一条数据都看不见；而 OLED 只用 2 根线 + 一个固定协议，10 分钟就能点着。
 *  它是**不依赖 LVGL 的显示通道**：OLED 挂了也不影响总线/上位机。
 *
 *  ⚠ 与 LVGL 屏的取舍：
 *    OLED 只显示字符（无图形/触摸），分辨率 128×64 → 8 行 × 21 列 6×8 像素字。
 *    它的定位是"没有 TFT 时的本地观察窗 / TFT 到手后的独立告警窗"。
 *
 *  ⚠ 与总线的解耦（这是刻意的设计，不是巧合）：
 *    · OLED 由**自己的一条任务**驱动（task_oled，core0 + 低优先级），
 *      不在 GUI 任务里做 I2C —— 一帧 1024 字节 @400kHz ≈ 25ms，放在 GUI 任务里
 *      会把 core1 的 LVGL 刷新拖出 25ms 的抖动；
 *    · task_oled 只读"最新值"（慢车道快照 / 推理结果 / 总线统计），
 *      从不向总线发请求 → OLED 死掉、没插、I2C 拉死，总线照常轮询。
 *
 *  [TBD-6] 接线（用户已给出，与 docs/01 的 F411 侧 OLED 是同一颗屏）：
 *    VCC=3V3  GND=GND  SDA=GPIO8  SCL=GPIO9  地址 0x3C
 * ============================================================================= */
#ifndef OLED_ENABLE
#define OLED_ENABLE                 1
#endif

#define OLED_I2C_ADDR               0x3C        /* SSD1306 常见地址（0x3D 是另一种 SA0 接法） */
#define OLED_I2C_CLOCK_HZ           400000      /* 400kHz 快速模式：25ms/帧 */
#define OLED_PIN_SDA                8           /* ESP32-S3-DevKitC-1 上空闲脚，避开 19/20 USB 与 33~37 PSRAM */
#define OLED_PIN_SCL                9
#define OLED_HOR_RES                128
#define OLED_VER_RES                64
#define OLED_PAGES                  (OLED_VER_RES / 8)      /* 8 页，每页 8 像素高 */
#define OLED_BUF_SIZE               (OLED_HOR_RES * OLED_PAGES)
#define OLED_TEXT_COLS              (OLED_HOR_RES / 6)      /* 6×8 字体：21 列 */
#define OLED_TEXT_ROWS              (OLED_VER_RES / 8)      /* 8 行 */
#define OLED_I2C_TIMEOUT_MS         50          /* 一次 I2C 传输的上限（未接屏时靠它不卡死） */
#define OLED_REFRESH_PERIOD_MS      500         /* 刷屏节拍（人眼够用；2Hz 远低于 I2C 带宽上限） */
#define OLED_RETRY_PERIOD_MS        10000       /* 初始化失败（没插屏）后的重试间隔，支持热插 */
#define OLED_WRITE_CHUNK            32          /* 每次 Wire 传输的字节数（避免撑爆 I2C 缓冲） */

/* 数据"保鲜期"：超过这么久没有新推理结果/新窗口，就在屏上标明 stale，
 * 避免把一帧几分钟前的旧数据当成实时值看。 */
#define OLED_STALE_MS               15000

/* OLED 任务（core0 + 最低优先级，见上面"与总线的解耦"） */
#define TASK_PRIO_OLED              2
#define TASK_STACK_OLED             4096

/* =============================================================================
 * §4.2  SPI TFT（2.4" IPS 240×320）—— gui.cpp 的"路径 B"
 * -----------------------------------------------------------------------------
 *  ⚠⚠ 当前默认 **TFT_ENABLE = 0（关）**，原因见下面"已知问题"。屏到货后再开。
 * -----------------------------------------------------------------------------
 *  这是**主界面**：LVGL 在这里画数据页/波形页/诊断页。OLED（§4.1）是副屏，
 *  两者并存、互不依赖（OLED 挂了 LVGL 照跑，反之亦然）。
 *
 *  ┌─ 历史：TFT_eSPI 在这个核心版本上必崩（已解决，留档）──────────────────────┐
 *  │  现象：置 1 后开机立刻 panic 复位循环，串口只到 `[tft] step1/step1b`：    │
 *  │    Guru Meditation Error: Core 1 panic'ed (StoreProhibited)             │
 *  │    EXCVADDR: 0x00000010                                                 │
 *  │    Backtrace: writecommand → begin_tft_write → TFT_eSPI::init           │
 *  │  已排查掉的：                                                            │
 *  │    · TFT_MISO 未定义（-1 被当引脚算 GPIO 偏移）→ 补成 GPIO21 后仍崩 ✗    │
 *  │    · SPI.begin() 没被调用 → 已在 gui_port_init() 里显式先调一次，仍崩 ✗  │
 *  │    · SPI_PORT 宏没解析 → 已确认 CONFIG_IDF_TARGET_ESP32S3 在第 9 行定义、 │
 *  │      qio_opi/include 在编译单元的 -I 里，SPI_PORT = FSPI ✓               │
 *  │    · **屏没接**（把真屏接上照崩）✗                                       │
 *  │  → 结论：TFT_eSPI 2.5.43 为速度**直接操作 SPI 寄存器**                │
 *  │    （`SET_BUS_WRITE_MODE` = `*_spi_user = SPI_USR_MOSI`），与           │
 *  │    Arduino-ESP32 **2.0.17** 不兼容。                                    │
 *  │  → 解决：**换 LovyanGFX**（自带 ESP32-S3 的 SPI 驱动、走 esp-idf 的     │
 *  │    spi_master，不依赖 Arduino SPIClass、不碰寄存器宏）。换完一次通过 ✓  │
 *  └─────────────────────────────────────────────────────────────────────────┘
 *
 *  本屏排针（**以实物丝印为准**）：左侧单排 14 针，从上到下——
 *    T_IRQ  T_DO  T_DIN  T_CS  T_CLK      ← 触摸 5 根（本项目先不接）
 *    SDO(MISO)  LED  SCK  SDI(MOSI)  DC  RESET  CS  GND  VCC
 *  ⚠ 顺序和常见模块不同（触摸在前、显示在后），**别照着记忆接**。
 *
 *  接线表（屏 → ESP32-S3-DevKitC-1）：
 *    VCC   → 3V3        （不要接 5V：虽然多数模块能扛，但统一 3V3 最稳）
 *    GND   → GND
 *    CS    → GPIO10
 *    RESET → GPIO14
 *    DC    → GPIO13      （数据/命令选择，四线 SPI 必需）
 *    SDI   → GPIO11      （= MOSI，写数据）
 *    SCK   → GPIO12      （= 时钟）
 *    LED   → GPIO15      （背光；**高电平点亮**）
 *    SDO   → GPIO21      （= MISO，只写屏可不接，但代码里必须给合法脚）
 * ============================================================================= */

/* ★ 主开关：1 = 启用 LVGL SPI 屏（路径 B）；0 = 不碰 TFT，只用 OLED。
 * 2026-10：屏已到货（2.4" ILI9341 SPI，红色 PCB，背面丝印
 *   `2.4" TFT SPI 240X320 V1.3`，带 XPT2046 触摸 + microSD 卡槽）→ 置 1。
 * 为什么要留这个开关：一旦 TFT 初始化出问题，GUI 任务会在 init() 里 panic，
 * 表现为**整块板子复位循环**，连 485/推理/OLED 都跑不了。
 * 留个总开关就能一键回退到"只用 OLED"的可用状态。 */
#define TFT_ENABLE                  1

/* 驱动 IC 二选一：ST7789 或 ILI9341。
 * 本屏背面丝印与卖家详情页都写 **ILI9341** → 置 1。
 * ⚠ 这两个 IC 在 2.4"/240×320 上是"同板不同芯"，初始化序列不同，
 *   **选错的表现是白屏或花屏**，不会烧坏，改回来即可。 */
#define TFT_USE_ILI9341             1

#define TFT_PIN_SCK                 12
#define TFT_PIN_MOSI                11
/* MISO：**必须给一个合法引脚号**（本工程只写屏不读屏，接不接都行）。
 * ⚠ 不能省、也不能给 -1：TFT_eSPI 的 init() 会走
 *   `spi.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, -1)`，把 -1 当引脚去算 GPIO
 *   寄存器偏移 → 非法写内存 → StoreProhibited → 复位循环。 */
#define TFT_PIN_MISO                21
#define TFT_PIN_CS                  10
#define TFT_PIN_DC                  13
#define TFT_PIN_RST                 14
#define TFT_PIN_BL                  15

/* SPI 写屏时钟 —— **这是"背光亮但屏黑/花屏/条纹"时第一个要调的值**。
 * 40MHz：内部 RAM + 短排线时的常用值
 * 27MHz：杜邦线较长/接触一般/面包板 的稳妥值（**出问题先降到这个**）
 * 20MHz：还不稳就再降（肉眼几乎看不出刷新变慢）
 * 现象对照：
 *   全黑（背光亮）    → 先查 SDI/SCK/CS/DC 线序，再降频
 *   花屏/条纹/随机亮块 → 基本都是时钟太快，降频即可
 * ★ 2026-10 实测结论（这块 2.4" ILI9341 + 10cm 杜邦线）★
 *   40MHz → 条纹；27MHz → 仍有条纹；20MHz → 基本干净但**左侧仍有一条雪花带**；
 *   **10MHz → 完全干净**。杜邦线 + 面包板的连线质量就这样，别为难它。
 *   10MHz 刷全屏约 8fps（320×240×2B / 10Mbit ≈ 123ms），
 *   局部刷新（LVGL 只重画脏区）时完全够用，肉眼感觉不到卡。
 * ⚠ 雪花/条纹的**另一个更常见成因是 LV_COLOR_16_SWAP 没开**（见 lv_conf.h）——
 *   两个坑长得一样，先确认那个宏是 1，再来降这里。
 * ⚠ 这个值同时被 gui.cpp 的 LovyanGFX `cfg.freq_write` 使用。 */
#define TFT_SPI_WRITE_HZ            10000000

/* ★ MADCTL 寄存器值：**直接控制朝向/镜像/BGR 的权威参数** ★
 * ============================================================================
 * 最终值 **0x60**（2026-10 实机 8 组穷举测试定出来的，用户选定"竖屏、编号 5"）
 *
 * 为什么必须绕过库的 setRotation：
 *   LovyanGFX 的 `setRotation + offset_rotation` 是"**两张表叠加**"，而它内置的
 *   4 个 rotation 只覆盖 MX/MY 的**对称组合**。这块 ZJY 模块需要的组合是
 *   **MY=1, MX=0, MV=1（= 0x60）**，这个组合**根本不在库的表里** ——
 *   所以无论怎么调 setRotation/offset_rotation 都到不了，我卡在这上面一整天。
 *   ⇒ 结论：**朝向问题绕不过寄存器层，直接写 0x36 命令才是正解。**
 *
 * ILI9341 数据手册 §8.2.29 位定义：
 *   bit7 MY  行方向(1=从下往上)     bit6 MX  列方向(1=从右往左)
 *   bit5 MV  行列交换(1=横屏)       bit3 BGR (1=BGR，多数模块需要 1)
 *
 * 实机 8 组穷举结果（用户目视确认）：
 *   0x00 / 0x20 / 0x80 / 0xA0 / 0xC0 / 0xE0 → 不对
 *   **0x40 → 正确，但呈横屏**（用户"2 号，横过来看正常"）
 *   **0x60 → 正确，且呈竖屏**（用户"5 号，竖过来看正常"）★ 采用
 *
 * 常用参考值：
 *   0x60 = 竖屏 + 不镜像（本工程）
 *   0x40 = 横屏 + 不镜像（同上，但 MV=0）
 *   0x48 = 0x40 + BGR（若发现红蓝互换就换这个）
 *   0x68 = 0x60 + BGR（竖屏 + BGR，红蓝互换时优先试它）
 * ⚠ 改这里之后**不用改别处**：gui.cpp 的 lcd_apply_madctl() 会写下去，
 *   并覆写库自己算的 MADCTL。 */
#define TFT_MADCTL                  0x60

/* RGB565 常用色（以前这几个宏是 TFT_eSPI 的字体头提供的；
 * 换成 LovyanGFX 后它不再自带，所以在自己这里定义一份，避免依赖库的内部头）。 */
#define TFT_BLACK                   0x0000
#define TFT_WHITE                   0xFFFF
#define TFT_RED                     0xF800
#define TFT_GREEN                   0x07E0
#define TFT_BLUE                    0x001F
#define TFT_YELLOW                  0xFFE0

/* 背光是否由 GPIO 控制：
 *   1 = 接 TFT_PIN_BL，初始化时拉高（本文件的默认）
 *   0 = 模块的 LED 脚直接接 3V3（常亮），程序里不碰
 * 有些模块的 LED 脚是"背光升压使能"，必须给高电平才亮 —— 所以默认给 1 更保险。 */
#define TFT_BL_USE_GPIO             1

/* 是否把 LVGL 的绘制缓冲放到 PSRAM（0 = 优先内部 DMA 内存）。
 * 240×40×2B = 19.2KB，内部 RAM 放得下，且**SPI DMA 从内部 RAM 搬更快**；
 * 只有在内部 RAM 紧张（WiFi + LVGL + 模型都上了）时才改成 1。
 * 详见 LVGL_DRAW_BUF_* 上面的说明。 */
#define TFT_DRAW_BUF_IN_PSRAM       0

/* =============================================================================
 * §5  TFLite Micro / 推理
 * ============================================================================= */

/* 特征维度：E 区 36 个寄存器 = 6 轴 × 6 类统计量（docs/02 §5-E）。
 * 必须和 tools/train_mlp.py 的输出维度、以及 0x0400 区的长度三者一致
 * （infer.cpp 里有 static_assert 兜底）。
 *
 * ⚠ 顺序也必须一致，而且这一点**极易踩坑**：
 *   docs/02 §5-E 的 E 区是「**每个特征** 6 轴连续」：
 *     0x0400~0x0405 = 6 轴均值，0x0406~0x040B = 6 轴标准差，… ，0x0423
 *   而 tools/train_mlp.py 的**默认** --feat-layout 是 per-axis
 *     （「每**轴** 6 个特征连续」），与上面是转置关系！
 *   → 训练时必须加 `--feat-layout per-feature`，
 *     否则 36 维输入被打乱、模型照跑、准确率掉到随机水平且不报错。
 *   infer.cpp 的 infer_init() 用 norm_params.h 的 FEATURE_LAYOUT 宏做了硬校验。 */
#define INFER_FEATURE_DIM           36
/* 类别数上界：主任务（振动：正常/不平衡/松动）+ 副任务（手势 4 类）→ 取 8 个位置 */
#define INFER_MAX_CLASSES           8
/* 类别名的最大长度（GUI 标签用） */
/* 类别名字符串的长度上限（16 = "unbalance" + '\0' 还绰绰有余）。
 * 当前 tools/train_mlp.py **不把类别名导出到 C 头**（只在 model_info.json 里），
 * 所以这个值现在只是给 GUI/日志留的排版上界；将来若把类别名也生成成 C 数组，
 * 数组的第二维就用它。 */
#define INFER_LABEL_LEN             16

/* Tensor arena 大小。
 * 为什么 48KB：plan.md §6.2 规定模型 int8 约 15~40KB；arena 要装下
 * 输入/输出张量 + 中间激活 + 算子 scratch，一般取模型的 1.2~1.5 倍。
 * 先用 48KB 起步，用 MicroInterpreter::arena_used_bytes() 打印实际用量再收紧。
 * 为什么一定放 PSRAM：内部 RAM 要留给 WiFi/LWIP + LVGL 缓冲 + 各任务栈，
 * arena 是"大而访问不算频繁"的块，正是 PSRAM 的适用场景；
 * 但**输入张量的填充缓冲和量化中间值仍然建议走内部 RAM**（PSRAM 读写延迟高），
 * 详见 infer.cpp 的注释。 */
#define TFLM_ARENA_SIZE             (48 * 1024)

/* =============================================================================
 * §6  WiFi / MQTT（v0.4 才真正用起来，这里先占位）
 * ============================================================================= */

#define WIFI_SSID                   "YOUR_WIFI_SSID"      /* TODO: 填真实 SSID */
#define WIFI_PASS                   "YOUR_WIFI_PASS"      /* TODO: 填真实密码 */
#define WIFI_CONNECT_TIMEOUT_MS     15000
#define WIFI_RETRY_BACKOFF_MS       2000                  /* 连不上时的重试间隔，避免打死射频 */

#define MQTT_BROKER                 "192.168.1.100"       /* TODO: PC 上跑的 mosquitto */
#define MQTT_PORT                   1883
#define MQTT_CLIENT_ID              "esp32s3-gw-01"
#define MQTT_USER                   ""                    /* 公共 broker 留空 */
#define MQTT_PASS                   ""
#define MQTT_KEEPALIVE_S            30                    /* PubSubClient 默认 15s，30s 更省流量 */
#define MQTT_SOCKET_TIMEOUT_S       5

/* 设备 ID：主题前缀 telemetry/{device_id}、cmd/{device_id}（docs/03 阶段 7） */
#define MQTT_DEVICE_ID              "gw01"
#define MQTT_TOPIC_TELEMETRY        "telemetry/" MQTT_DEVICE_ID
#define MQTT_TOPIC_CMD              "cmd/" MQTT_DEVICE_ID
#define MQTT_TOPIC_ALARM            "alarm/" MQTT_DEVICE_ID

/* 上报周期：为什么 2s —— 和快车道同拍，一次把环境量 + 状态字 + 推理结果打包发走，
 * 少一次 JSON 组包、少一次 TCP 写。500ms 上报对 MQTT 太重（一天 17 万条）。 */
#define NET_TELEMETRY_PERIOD_MS     2000
#define NET_LOOP_PERIOD_MS          20                    /* client.loop() 的调用节拍 */
#define NET_RECONNECT_BACKOFF_MS    2000
#define NET_JSON_BUF_SIZE           768                   /* 一条 telemetry 的 JSON 上界 */

/* =============================================================================
 * §7  其它
 * ============================================================================= */
#define APP_LOG_TAG                 "gw"                  /* log_x 的统一 TAG */

/* 日志宏：直接走 Serial（UART0/CH340 调试口），不依赖 ESP-IDF 的 TAG 约定
 * —— arduino-esp32 各内核版本对 log_x 的 TAG 处理不一致（有的要求用户自己
 *   定义 static const char *TAG，有的有默认值），用 Serial.printf 永远编得过。
 * 需要更细的日志时再换 ESP_LOGI 并显式定义 TAG。 */
#define APP_LOGI(fmt, ...)  Serial.printf("[I][" APP_LOG_TAG "] " fmt "\n", ##__VA_ARGS__)
#define APP_LOGW(fmt, ...)  Serial.printf("[W][" APP_LOG_TAG "] " fmt "\n", ##__VA_ARGS__)
#define APP_LOGE(fmt, ...)  Serial.printf("[E][" APP_LOG_TAG "] " fmt "\n", ##__VA_ARGS__)

/* 宏值转字符串（打印引脚号用）。两级展开是必须的，否则打出来是 "RS485_DE_PIN" */
#define STR_INNER(x)                #x
#define STR(x)                      STR_INNER(x)
