/**
 * @file    app_main.c
 * @brief   应用层胶水：CubeMX 句柄 → 6 个 FreeRTOS 任务 + 485 收发 + 传感器采集
 *
 * 本文件负责"把协议栈和数据接起来"，三件最难的事都在这里做完：
 *   ① USART2 的 IDLE + DMA 收帧（不用自己数 3.5T 定时器，交给硬件）
 *   ② RS485 的 DE/RE 时序（**必须等 TC 而不是等 DMA 完成**，否则最后一字节被截断）
 *   ③ 50Hz 快采样任务与慢采样任务的分工（对应 regbank 的双缓冲窗口）
 *
 * 传感器驱动部分是 TODO 桩：它们和硬件强相关，请在阶段 1（docs/03-bringup.md）
 * 逐个把它们填实——注意 ADC/I2C 的读写骨架已经写好了，照抄即可。
 */
#include "app_main.h"
#include "regbank.h"
#include "modbus_slave.h"
#include "modbus_crc.h"

#include <stdio.h>
#include <string.h>
#include <math.h>       /* sqrtf/cosf：特征提取用（F411 有 FPU，开销可忽略） */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"     /* printf 串行化用的互斥锁 */

/* ===========================================================================
 * 调试开关 —— ⚠️ **所有配置宏必须放文件顶部，不能放在用到它的 #if 后面** ✗→✓
 * ---------------------------------------------------------------------------
 * 踩过的坑：`DBG_WINDOW_DUMP` 原来定义在文件中部（打点模式那一节），
 *   而 [hb]/[imu]/[dsp] 的静音判断写在它**前面** ✗ →
 *   预处理是**按顺序**展开的，前面的 `#if !DBG_WINDOW_DUMP` 里该宏尚未定义（=0）✗
 *   → `!0` 为真 → 静音**完全失效** → 日志与打点数据混在同一个串口流里 ✗
 *   → 采集脚本一行都解析不出来（实测卡 300 秒、0 个窗口 ✗）。
 *
 *   1 = 打点模式：每 2 秒把 100×6 窗口以 "W,<seq>,600 个 int16" 打出，
 *       同时**静音** [hb]/[imu]/[dsp]，保证串口流只有数据行 ✓
 *   0 = 正常模式：看日志（调 Modbus / 485 时用）✓
 *
 * ⚠⚠ 2026-10 实测：**打点模式开着的时候，Modbus 从机会大面积不应答** ✗
 *   因果链（ESP32 主站侧抓到"静默超时"，查到这里）：
 *     打点一行 4.2KB，dbg_transmit() 里是 HAL_UART_Transmit(..., 2000ms) 阻塞写，
 *     并且**持着 s_printf_mutex ~370ms**（4.2KB @115200）。
 *     task_modbus 处理完请求要打日志 → 抢这把锁 → 被拖住最多 370ms ✗
 *     而主站慢车道超时只有 50ms → 从机的响应**晚到主站已经放弃** ✗
 *   实测数据：主站 TX 232 / RX 60（≈26% 成功率），失败**全部是静默**
 *   （线上一个字节都没有），而且 E 区特征(0x0400, 36 个寄存器)那组帧
 *   几乎每次都成功 —— 因为那组最紧凑，容易挤进未被堵住的空档 ✓ 现象完全对得上。
 *   → 结论：**调 485 / 跑双机联调时必须置 0**；采数据集时再置回 1 重烧。 */
#define DBG_WINDOW_DUMP   0

/* ---- 常量（与 docs/02-modbus-protocol.md 对齐） ------------------------ */
#define MODBUS_FRAME_MAX      256u
#define MODBUS_RX_QUEUE_LEN   4u
/* 等待发送完成的兜底时间。**必须按"最长一帧"算**，否则会被截帧：
 *   窗口分块响应 = 地址+功能+字节数+250 数据+CRC = 260 字节
 *   9600   → 260 × 11 / 9600   ≈ 298ms
 *   115200 → 260 × 11 / 115200 ≈  25ms
 * 所以改波特率时必须同步改这个值，它是"能跑通"的前提之一。
 *
 * 2026-10 切回 115200 后取 **100ms**：约为 25ms 的 4 倍余量。
 * 不用 400ms 是因为它是**阻塞等待的上界**，太大只会在异常时拖长任务占用；
 * 也不用 50ms，因为裕量太薄（这里还包含 taskYIELD 的调度延迟）。 */
#define MB_TX_TIMEOUT_MS      100u

#define TASK_ACQ_FAST_STACK   256u
#define TASK_MODBUS_STACK     512u
#define TASK_ACQ_SLOW_STACK   384u
#define TASK_OUTPUT_STACK     256u
#define TASK_HEALTH_STACK     384u
#define TASK_DSP_STACK        768u     /* FFT 很吃栈，别省 */

#define PRIO_ACQ_FAST         5u
#define PRIO_MODBUS           4u
#define PRIO_ACQ_SLOW         3u
#define PRIO_OUTPUT           3u
#define PRIO_HEALTH           6u     /* 看门狗喂狗任务必须是最高优先级：任何任务卡住都不能饿死它 */
#define PRIO_DSP              2u

#define ADC_CH_COUNT          8u
#define MPU6050_ADDR          0x68u
#define MPU6050_REG_PWR_MGMT1 0x6Bu
#define MPU6050_REG_SMPLRT_DIV 0x19u
#define MPU6050_REG_CONFIG    0x1Au
#define MPU6050_REG_ACCEL_XOUT 0x3Bu

/* ADC 扫描数组下标 = CubeMX 里的 Rank 顺序（见 CUBEMX-SETUP.md §3），不是引脚号顺序！
 * 若你在 CubeMX 里调整了通道使能顺序，这里必须同步改，否则"吹一口气烟雾值变了、光照也在变"。 */
#define ADC_IDX_AIR           0u   /* Rank1  PB0  MQ-135 空气 */
#define ADC_IDX_FLAME         1u   /* Rank2  PB1  火焰 */
#define ADC_IDX_LIGHT         2u   /* Rank3  PC0  光敏 */
#define ADC_IDX_NTC           3u   /* Rank4  PC1  热敏（A 区无寄存器，暂不外露） */
#define ADC_IDX_POT           4u   /* Rank5  PC2  电位器 */
#define ADC_IDX_SOIL          5u   /* Rank6  PC3  土壤 */
#define ADC_IDX_RAIN          6u   /* Rank7  PC4  雨量 */
#define ADC_IDX_SMOKE         7u   /* Rank8  PC5  MQ-2 烟雾 */

/* PWM 量程换算：TIM1/TIM3 均为 PSC=99、ARR=999 → CCR 取值 0~999 对应占空比 0~100% */
#define PWM_CCR_FROM_PERMIL(x) ((uint32_t)(((uint32_t)(x) * 999u) / 1000u))  /* 0~1000 → 0~999 */
#define PWM_CCR_FROM_U8(x)     ((uint32_t)(((uint32_t)(x) * 999u) / 255u))   /* 0~255  → 0~999 */

/* B 区状态字 1 中由"慢车道采集任务"负责的位（其余位由别的任务维护，写回时要保留） */
#define ST1_OWN_MASK_SLOW     (0x03FBu)   /* bit0 DHT11,bit1 BMP280,bit3 MAX30102,bit4~bit9 其余 */

/* ---- 句柄与状态 -------------------------------------------------------- */
static UART_HandleTypeDef *s_huart_dbg;
static UART_HandleTypeDef *s_huart_mb;
static ADC_HandleTypeDef  *s_hadc;
static I2C_HandleTypeDef  *s_hi2c;
static TIM_HandleTypeDef  *s_htim_rgb_r;
static TIM_HandleTypeDef  *s_htim_servo;
static TIM_HandleTypeDef  *s_htim_misc;

extern IWDG_HandleTypeDef hiwdg;           /* 看门狗（CubeMX 生成） */

static mb_slave_t   s_slave;
static QueueHandle_t s_rx_q;

/* ---- 临时诊断：USART2 上到底收到了什么（调 485 用，见 task_modbus 的 [mb] 行）----
 * 为什么需要：主站报"静默超时"时，从机侧必须能回答两个问题：
 *   ① 帧到了没有？（g_mb_rx_events 涨 = 到了）
 *   ② 到了之后为什么没回？（队列丢 / 解析判坏 / 发了但总线被占）
 * 这三个计数就能把责任分到"物理层 / 队列 / 协议层"。 */
volatile uint32_t g_mb_rx_events   = 0u;
volatile uint32_t g_mb_rx_size0    = 0u;
volatile uint32_t g_mb_rx_q_drop   = 0u;
volatile uint32_t g_mb_rx_overlong = 0u;
volatile uint32_t g_mb_tx_frames   = 0u;
volatile uint32_t g_mb_parse_zero  = 0u;
volatile uint32_t g_mb_rx_bytes    = 0u;   /* USART2 收到的总字节（与主站 tx 对账） */
volatile uint32_t g_mb_tx_bytes    = 0u;   /* USART2 发出的总字节（与主站 rx 对账） */
volatile uint16_t g_mb_rx_last_len = 0u;
volatile uint32_t g_mb_rx_last_ms  = 0u;
volatile uint16_t g_mb_rx_last_n   = 0u;
volatile uint8_t  g_mb_rx_last[MODBUS_FRAME_MAX];

/* ---- 直方图实验（2026-10）：失败是"集中"还是"均匀"？------------------------
 * 背景：9600 下主站第一次尝试成功率只有 ~61%，但**每次重试都成功**，
 *       且失败全是静默、从机侧 parse0 不涨（说明那帧从机连"收到判错"都没有）。
 * 判据：
 *   · 失败在时间上**集中**（某几个桶明显偏高）→ 从机侧调度竞争
 *     （printf 互斥锁 / 50Hz 采样任务抢占）→ 提 Modbus 任务优先级即可修，不花钱。
 *   · 失败在时间上**均匀**（每个桶都差不多）→ 位级裕量不足
 *     → 需要 120Ω 终端电阻或换收发器模块。
 * 做法：按 (运行秒数 % 桶数) 分桶，每个桶同时记"到达次数"和"收到但没应答次数"，
 *       这样既能看周期性（比如每秒一次的采样/打印节拍），也能看各时段的成功率。 */
#define MB_HIST_BUCKETS   64u
volatile uint32_t g_hist_rx[MB_HIST_BUCKETS];   /* 该秒内到达的帧数 */
volatile uint32_t g_hist_nr[MB_HIST_BUCKETS];   /* 该秒内"到达但没应答"的帧数 */

static uint8_t  s_rx_dma[MODBUS_FRAME_MAX];
static uint8_t  s_resp[MODBUS_FRAME_MAX];
static uint16_t s_adc_raw[ADC_CH_COUNT];
static DMA_HandleTypeDef s_hdma_adc1;   /* ADC1 的 DMA 句柄（CubeMX 没配时由代码兜底建立） */

typedef struct {
    uint16_t len;
    uint8_t  data[MODBUS_FRAME_MAX];
} mb_frame_t;

/* ---- 引脚映射（**以当前 CubeMX 工程里的实际分配为准**） --------------------
 * ⚠️ docs/01-hardware.md 里的"规划值"与当前 .ioc 的"实际值"不一致，代码按实际值配：
 *
 *      信号         规划    当前     原因
 *      RS485_DE     PA1     PA5      PA0 被 TIM2_CH1(舵机) 占了，PA1 当时没配上
 *      人体感应      PA4     PB2      PB2 = BOOT1，做输入要求上电时为低电平
 *      人体雷达      PB2     PB10
 *
 *   在 CubeMX 里把这三处改回规划值并重新生成后，只要改下面这三组宏即可。
 *   （改动建议：RS485_DE 一定要挪走 PA5/PA0 这两个引脚之外的位置，
 *     舵机 PWM 也不要留在 PA0——PA0 是板载按键那一脚。）
 */
#define PIN_RS485_DE_PORT   GPIOA
#define PIN_RS485_DE_PIN    GPIO_PIN_5      /* 规划 PA1 */
#define PIN_PIR_PORT        GPIOB
#define PIN_PIR_PIN         GPIO_PIN_2      /* 规划 PA4 */
#define PIN_RADAR_PORT      GPIOB
#define PIN_RADAR_PIN       GPIO_PIN_10     /* 规划 PB2 */

#define RS485_DE_ON()   HAL_GPIO_WritePin(PIN_RS485_DE_PORT, PIN_RS485_DE_PIN, GPIO_PIN_SET)
#define RS485_DE_OFF()  HAL_GPIO_WritePin(PIN_RS485_DE_PORT, PIN_RS485_DE_PIN, GPIO_PIN_RESET)

/* ======================================================================= */
/*  传感器驱动（阶段 1 逐个填实）                                            */
/* ======================================================================= */

/* ---- I2C 小工具 -------------------------------------------------------- */
static bool i2c_write_reg(uint8_t dev, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(s_hi2c, (uint16_t)(dev << 1), reg, I2C_MEMADD_SIZE_8BIT,
                             &val, 1u, 50u) == HAL_OK;
}

static bool i2c_read_regs(uint8_t dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(s_hi2c, (uint16_t)(dev << 1), reg, I2C_MEMADD_SIZE_8BIT,
                            buf, len, 50u) == HAL_OK;
}

/**
 * @brief I2C 总线解锁（工业标准做法）★ 解决"冷启动好、一按复位就认不到"✗→✓
 *
 * 根因：**按复位（NRST）不会给从机断电** ✗。若 MCU 复位那一刻传感器正在发数据，
 *   它会一直把 SDA 拉低不放 ✗ → 总线锁死 → 复位后 MCU 怎么重试都读不到 ✗✗
 *   （只有断电重上电才恢复 ✓ —— 这正是"来回折腾"的原因 ✓）
 *
 * 解法：在**初始化 I2C 外设之前**，把 SCL 当普通 GPIO 开漏输出敲 9 个脉冲 ✓，
 *   把从机移位寄存器里卡住的位全部移出 ✓；再补一个 STOP ✓；
 *   最后把 PB6/PB7 交还给 I2C1（AF4 + 开漏）✓。
 *   这样**复位之后也能自动恢复** ✓✓
 */
static void i2c_bus_unstick(void)
{
    GPIO_InitTypeDef g = {0};
    uint8_t i;

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* SCL(PB6)/SDA(PB7) 先都设为开漏输出，写 1 = 释放（靠外部上拉拉高） */
    g.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &g);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);           /* 释放 SDA */
    for (i = 0u; i < 9u; i++) {                                   /* 9 个时钟脉冲 */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        { volatile uint32_t d = 300u; while (d--) { } }
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        { volatile uint32_t d = 300u; while (d--) { } }
    }
    /* 补一个 STOP：SCL 保持高时，SDA 由低变高 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    { volatile uint32_t d = 300u; while (d--) { } }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    { volatile uint32_t d = 300u; while (d--) { } }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    { volatile uint32_t d = 300u; while (d--) { } }

    /* 引脚交还给 I2C1：F4 上 I2C1 的复用功能是 AF4，开漏 + 上拉 */
    g.Mode      = GPIO_MODE_AF_OD;
    g.Pull      = GPIO_PULLUP;
    g.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &g);
}

/**
 * @brief 开机 I2C 总线自检 —— **无论传感器是否在线都要跑** ✓
 *
 * 为什么必须加：传感器一直离线时，原来只有"窗口填 0"这一个现象 ✗，
 * 分不清是"模块没接好"还是"总线被拉死" ✗✗。本函数把三种情况分开：
 *   ① 打印总线上所有应答的地址（0x68 = MPU6050 ✓）
 *   ② 总线通但无人应答（NACK/HAL_ERROR）→ 传感器没接好 / 模块坏 / 地址不对 ✗
 *   ③ 返回 TIMEOUT 或 BUSY → **总线被拉死**：SDA/SCL 接反、短路、或没上拉 ✗✗
 *
 * 这三种的排查方向完全不同，所以先把它测出来再动手 ✓。
 */
void i2c_bus_selftest(void)
{
    uint8_t addr;
    uint8_t found = 0u;

    printf("[i2c] 总线自检：扫描 0x08~0x77 ...\r\n");
    for (addr = 0x08u; addr < 0x78u; addr++) {
        if (HAL_I2C_IsDeviceReady(s_hi2c, (uint16_t)(addr << 1), 1u, 10u) == HAL_OK) {
            printf("[i2c]   应答地址: 0x%02X\r\n", addr);
            found++;
        }
        HAL_IWDG_Refresh(&hiwdg);          /* 扫描期间喂狗，别被 IWDG 打断 */
    }
    printf("[i2c] 扫描结束：共 %u 个设备\r\n", (unsigned)found);

    if (found == 0u) {
        HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(s_hi2c, (uint16_t)(0x68u << 1), 1u, 20u);
        printf("[i2c] 单独测 MPU6050(0x68) 返回码 = %d  (0=OK 1=ERROR/NACK 2=BUSY 3=TIMEOUT)\r\n",
               (int)st);
        if ((st == HAL_TIMEOUT) || (st == HAL_BUSY)) {
            printf("[i2c] ==> 总线被拉死：多半 SDA/SCL 接反、短路、或上拉缺失 ✗✗\r\n");
            printf("[i2c]     先断电，用万用表/肉眼确认 PB6=SCL PB7=SDA 没接反、没短到一起\r\n");
        } else {
            printf("[i2c] ==> 总线是通的，但没有任何设备应答 ✗\r\n");
            printf("[i2c]     检查：① 模块电源灯亮不亮 ② VCC/GND 有没有接反（接反可能已损坏）\r\n");
            printf("[i2c]           ③ SDA/SCL 有没有接反 ④ 换一组杜邦线（断芯极常见）\r\n");
        }
    }
}

/** I2C 扫描：阶段 1 第一件事，把在线地址全打印出来 */
void sensors_i2c_scan(void)
{
    uint8_t addr;
    printf("[i2c] scan:");
    for (addr = 0x08u; addr < 0x78u; addr++) {
        /* 探测参数刻意收紧（1 次 / 5ms）：112 个地址 × 最坏 5ms ≈ 0.56s。
         * 若按 2 次 / 20ms 最坏要 4.5s —— 那会直接超过 IWDG 的 2s，把人复位。 */
        if (HAL_I2C_IsDeviceReady(s_hi2c, (uint16_t)(addr << 1), 1u, 5u) == HAL_OK) {
            printf(" 0x%02X", addr);
        }
        HAL_IWDG_Refresh(&hiwdg);      /* 扫描期间也喂狗 */
    }
    printf("\r\n");
}

/* ---- MPU6050（核心数据源） -------------------------------------------- */
/* MPU6050 是否在线。
 * ⚠️ **没接传感器时，绝不能每 20ms 去戳 I2C**：对不存在的从机，每次事务都要等满超时，
 *    50Hz 调用会把 CPU 全部吃光（远超 100%），结果是低优先级任务（Modbus、慢采集、
 *    输出）全被饿死 —— 我们踩过这个坑。这个标志由 task_health 里带重试的探测置位，
 *    并且连续读失败会自动清掉它。 */
static volatile bool s_mpu_online = false;

/* 开机静止标定出的陀螺仪零偏（LSB），采样时减掉。标定见 task_health。 */
static int16_t s_gyro_bias[3] = { 0, 0, 0 };

/* printf 串行化用的互斥锁。
 * ⚠️ printf 不是线程安全的：多个任务同时打印时，后到的 HAL_UART_Transmit 会看到
 *    UART 正忙而直接返回 HAL_BUSY，**那一整行日志被静默丢掉**。
 *    我们踩过这个坑（[hb] 心跳整片消失），所以必须串行化。 */
static SemaphoreHandle_t s_printf_mutex = NULL;

static bool mpu6050_init(void)
{
    printf("[mpu] 1/3 wake\r\n");
    if (!i2c_write_reg(MPU6050_ADDR, MPU6050_REG_PWR_MGMT1, 0x00u)) return false; /* 唤醒 */
    printf("[mpu] 2/3 dlpf\r\n");
    if (!i2c_write_reg(MPU6050_ADDR, MPU6050_REG_CONFIG, 0x04u))    return false; /* DLPF 20Hz */
    /* 采样率 = 1kHz / (1 + 9) = 100Hz，再在任务里按 50Hz 取用；留余量避免混叠 */
    printf("[mpu] 3/3 rate\r\n");
    if (!i2c_write_reg(MPU6050_ADDR, MPU6050_REG_SMPLRT_DIV, 0x09u)) return false;
    return true;
}

/**
 * @brief 读六轴原始值
 * @param six 输出 [ax, ay, az, gx, gy, gz]
 * @note  原始值即可，量程换算放到推理侧统一做，便于对比实验
 */
static bool mpu6050_read(int16_t six[6])
{
    uint8_t raw[14];
    uint8_t i;

    if (!i2c_read_regs(MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT, raw, sizeof(raw))) {
        return false;
    }
    /* 大端：高字节在前 */
    for (i = 0u; i < 6u; i++) {
        six[i] = (int16_t)(((uint16_t)raw[i * 2u] << 8) | raw[i * 2u + 1u]);
    }
    /* raw[6..7] 是温度，这里不用 */
    return true;
}

/* ---- 以下传感器请在阶段 1 按 docs/03-bringup.md 逐个实现 ---------------- */
static bool dht11_read(uint16_t *temp_x10, uint16_t *humi_x10)
{
    /* TODO(阶段1): 单总线时序，微秒级延时 + 关中断；DATA 需 4.7k~10k 上拉
     *   起始信号：拉低 >18ms → 释放 → 读 40 bit（湿度8+湿度8+温度8+温度8+校验8） */
    (void)temp_x10; (void)humi_x10;
    return false;
}

static bool hcsr04_read_mm(uint16_t *mm)
{
    /* TODO(阶段1): TRIG 拉高 10µs → 测 ECHO 高电平宽度 t(µs)，距离 = t * 0.343 / 2
     *   ⚠️ ECHO 是 5V 输出，必须分压后再进 GPIO */
    (void)mm;
    return false;
}

static bool bmp280_read(uint16_t *press_x10)
{
    /* TODO(阶段1): 读 0x88~0xA1 校准系数 + 0xF7~0xF9 原始值，按数据手册算补偿气压 */
    (void)press_x10;
    return false;
}

static bool max30102_read(uint16_t *hr_bpm, uint16_t *spo2)
{
    /* TODO(加分项): 需要 FIFO + 算法（峰值检测 / R 比值），先把原始红光红外值读出来看波形 */
    (void)hr_bpm; (void)spo2;
    return false;
}

/* ---- 模拟量与滤波 ------------------------------------------------------ */
/** 中值 + 均值滤波：ADC 读数跳动 ±50 是常态，不滤会在界面上乱跳 */
static uint16_t adc_filtered(uint8_t ch)
{
    static uint16_t hist[ADC_CH_COUNT][5];
    static uint8_t  idx[ADC_CH_COUNT];
    uint16_t tmp[5];
    uint32_t sum = 0u;
    uint8_t  i, j;

    if (ch >= ADC_CH_COUNT) return 0u;

    hist[ch][idx[ch]] = s_adc_raw[ch];
    idx[ch] = (uint8_t)((idx[ch] + 1u) % 5u);

    for (i = 0u; i < 5u; i++) tmp[i] = hist[ch][i];
    for (i = 0u; i < 4u; i++) {                 /* 冒泡排序取中值 */
        for (j = 0u; j < 4u - i; j++) {
            if (tmp[j] > tmp[j + 1u]) { uint16_t t = tmp[j]; tmp[j] = tmp[j + 1u]; tmp[j + 1u] = t; }
        }
    }
    for (i = 1u; i < 4u; i++) sum += tmp[i];    /* 去掉最低最高再平均 */
    return (uint16_t)(sum / 3u);
}

/* ======================================================================= */
/*  485 收发                                                              */
/* ======================================================================= */

static void modbus_start_rx(void)
{
    /* 用「中断 + IDLE」收帧，**不用 DMA**：
     *   - 本项目一帧最长 256 字节、每秒最多几帧。115200 下一帧 250 字节约 12k 次中断/秒，
     *     折算不到 0.4% CPU，完全可接受；
     *   - 好处是 CubeMX 里**不用再给 USART2 配 RX DMA**，少一处容易配错的地方。
     * HAL 在检测到 IDLE（总线空闲 = 帧尾）或缓冲区满时回调 HAL_UARTEx_RxEventCallback()。 */
    HAL_UARTEx_ReceiveToIdle_IT(s_huart_mb, s_rx_dma, sizeof(s_rx_dma));
}

/** HAL 在"总线空闲"或"缓冲区满"时回调，Size = 本次收到的字节数 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if ((s_huart_mb != NULL) && (huart->Instance == s_huart_mb->Instance)) {
        BaseType_t woken = pdFALSE;

        /* 临时诊断计数（**不在 ISR 里 printf**：那会阻塞几百微秒，
         * 把"主站超时"变成"被日志拖出来的超时"，掩盖真问题）。
         * 真正的打印在 task_modbus 里做，见那里的 [mb] 行。 */
        if (Size > 0u) {
            g_mb_rx_events++;
            g_mb_rx_bytes += (uint32_t)Size;
            g_mb_rx_last_len = Size;
            /* 直方图：按"到达的那一秒"分桶（见上方 g_hist_rx 的注释） */
            g_hist_rx[(HAL_GetTick() / 1000u) % MB_HIST_BUCKETS]++;
            g_mb_rx_last_ms  = HAL_GetTick();
            if (Size <= MODBUS_FRAME_MAX) {
                uint16_t k;
                for (k = 0u; k < Size; k++) {
                    g_mb_rx_last[k] = s_rx_dma[k];
                }
                g_mb_rx_last_n = Size;
            }
        } else {
            g_mb_rx_size0++;
        }

        if ((Size > 0u) && (Size <= MODBUS_FRAME_MAX) && (s_rx_q != NULL)) {
            mb_frame_t f;
            f.len = Size;
            memcpy(f.data, s_rx_dma, Size);
            if (xQueueSendFromISR(s_rx_q, &f, &woken) != pdTRUE) {
                g_mb_rx_q_drop++;
            }
        } else if (Size > MODBUS_FRAME_MAX) {
            g_mb_rx_overlong++;
        }
        modbus_start_rx();
        portYIELD_FROM_ISR(woken);
    }
}

/** 发送一帧，并在"最后一字节真正移出移位寄存器"之后才切回接收 */
static void modbus_send(const uint8_t *buf, uint16_t len)
{
    uint32_t t0;

    RS485_DE_ON();
    if (HAL_UART_Transmit_IT(s_huart_mb, (uint8_t *)buf, len) != HAL_OK) {
        RS485_DE_OFF();
        modbus_start_rx();
        return;
    }

    t0 = HAL_GetTick();
    /* ① 等中断发送把整帧送出（HAL 的状态会回到 READY） */
    while ((HAL_UART_GetState(s_huart_mb) != HAL_UART_STATE_READY) &&
           ((HAL_GetTick() - t0) < MB_TX_TIMEOUT_MS)) {
        taskYIELD();
    }
    /* ② 再等 TC：移位寄存器里的最后一字节移出去。
     *    只等 DMA 完成就切 DE 是**经典 bug**，会让最后一字节（常常是 CRC 高字节）被削掉。 */
    while ((__HAL_UART_GET_FLAG(s_huart_mb, UART_FLAG_TC) == RESET) &&
           ((HAL_GetTick() - t0) < MB_TX_TIMEOUT_MS)) {
        taskYIELD();
    }
    __HAL_UART_CLEAR_FLAG(s_huart_mb, UART_FLAG_TC);

    RS485_DE_OFF();
    /* ③ 必须在 DE 拉低之后才重启接收，否则会收到自己刚发出去的数据 */
    modbus_start_rx();
}

void app_uart2_irq_hook(void)
{
    /* 预留：若改成"纯中断切 DE"，在这里处理 UART_FLAG_TC 并调用 RS485_DE_OFF()。
     * 当前实现不需要它（见 modbus_send）。 */
}

/* ======================================================================= */
/*  精确周期延时（自带实现，不依赖 FreeRTOS 的 vTaskDelayUntil）            */
/* ======================================================================= */

/**
 * @brief 以固定周期为基准的延时至（语义与 vTaskDelayUntil 相同）
 *
 * 为什么不用 vTaskDelayUntil：CubeMX 生成的 FreeRTOSConfig.h 里
 * `INCLUDE_vTaskDelayUntil` 默认为 0，即该函数根本不参与编译，链接会报
 * `undefined reference to vTaskDelayUntil`。而它是一个随 CubeMX 重新生成
 * 而变化的配置项，不值得为它反复改 CubeMX 设置。
 *
 * 本实现只用到 xTaskGetTickCount() + vTaskDelay()（两者恒定可用），
 * 并且利用 TickType_t 的无符号回绕特性，跨 tick 计数器溢出同样安全。
 * 与 vTaskDelay 的区别：这个不会因为任务本身执行时间而累积漂移，
 * 所以 50Hz 采样任务的采样率是稳的。
 */
static void task_delay_until(TickType_t *prev_wake, TickType_t period)
{
    TickType_t now    = xTaskGetTickCount();
    TickType_t target = *prev_wake + period;

    if ((int32_t)(target - now) > 0) {
        vTaskDelay(target - now);
    }
    *prev_wake = target;
}

/* ======================================================================= */
/*  任务                                                                  */
/* ======================================================================= */

/** 5 号任务：50Hz 采六轴，喂给寄存器表的双缓冲窗口 */
static void task_acq_fast(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    int16_t six[6];
    uint8_t fail = 0u;

    (void)arg;
    for (;;) {
        if (s_mpu_online) {
            if (mpu6050_read(six)) {
                /* 减掉开机标定的陀螺仪零偏（加速度不处理） */
                six[3] = (int16_t)(six[3] - s_gyro_bias[0]);
                six[4] = (int16_t)(six[4] - s_gyro_bias[1]);
                six[5] = (int16_t)(six[5] - s_gyro_bias[2]);
                regbank_push_sample(six);
                fail = 0u;

                /* ---- 调试打印：每秒一行原始六轴 ----
                 * 这是验证"量程/单位/字节序/零偏"是否正确的唯一可靠手段：
                 *   水平静止放置： az ≈ +16384（±2g 量程下 1g，具体正负看芯片贴装方向）
                 *                  ax ≈ 0、ay ≈ 0
                 *                  gx/gy/gz ≈ 0（几百 LSB 的零偏是正常的，属静态误差）
                 * 翻开板子/晃动时对应轴应明显变化 → 说明轴映射也对。 */
                {
                    static uint32_t dbg_ms = 0u;
                    if ((uint32_t)(HAL_GetTick() - dbg_ms) >= 1000u) {
                        dbg_ms = HAL_GetTick();
#if !DBG_WINDOW_DUMP
                        printf("[imu] ax=%6d ay=%6d az=%6d | gx=%6d gy=%6d gz=%6d\r\n",
                               (int)six[0], (int)six[1], (int)six[2],
                               (int)six[3], (int)six[4], (int)six[5]);
#endif
                    }
                }
            } else if (++fail >= 10u) {
                /* 连续失败 → 判定掉线：**立即停止 50Hz 访问 I2C**（否则会饿死其他任务），
                 * 交回 task_health 按 5 秒节奏重新探测。 */
                s_mpu_online = false;
                fail = 0u;
                regbank_write_internal(REG_B_STATUS1,
                    (uint16_t)(regbank_get(REG_B_STATUS1) & ~(uint16_t)(1u << 2)));
                printf("[sensor] mpu6050 连续读失败 -> 标记掉线，停止高频采样\r\n");
            }
        } else {
            /* 离线：窗口仍按 50Hz 推进（时间轴不能停），但填 0，且**不做任何 I2C 事务** */
            int16_t zero[6] = { 0, 0, 0, 0, 0, 0 };
            regbank_push_sample(zero);
        }
        task_delay_until(&last, pdMS_TO_TICKS(20));   /* 50Hz */
    }
}

/** 4 号任务：唯一的 Modbus 处理者（从机） */
static void task_modbus(void *arg)
{
    mb_frame_t f;
    uint32_t   last_report = 0u;

    (void)arg;
    for (;;) {
        if (xQueueReceive(s_rx_q, &f, pdMS_TO_TICKS(1000)) == pdPASS) {
            uint16_t n = mb_slave_process(&s_slave, f.data, f.len, s_resp, sizeof(s_resp));
            if (n > 0u) {
                /* 收到合法主站请求 → 喂通信看门狗（广播帧不应答，也不喂） */
                regbank_notify_master_request(HAL_GetTick());
                modbus_send(s_resp, n);
                g_mb_tx_frames++;
                g_mb_tx_bytes += (uint32_t)n;
            } else {
                g_mb_parse_zero++;      /* 收到了但被协议层判为"不该回"（地址/CRC/异常静默） */
                g_hist_nr[(HAL_GetTick() / 1000u) % MB_HIST_BUCKETS]++;
            }
        }

        /* ---- 每 5 秒报一次 USART2 侧的收发统计（**常驻**，调 485 时最有用的一行）----
         * 判读：
         *   rx=0            → 主站的帧**根本没到** F411（接线/DE/收发器方向）
         *   rx>0, q_drop>0  → 到了但队列溢出（task_modbus 没被及时调度）
         *   rx>0, tx=0      → 到了但不回（地址不匹配 / CRC 错 → 协议层静默丢弃）
         *   rx>0, tx>0      → 从机回帧了；丢帧在物理层或主站侧
         * 2026-10 实测（ESP32 主站 25% 成功率那次）：rx=58 tx=58 parse0=0，
         * 即 **从机对收到的帧 100% 应答**，问题不在协议层 —— 就是靠这一行定的性。 */
        if ((HAL_GetTick() - last_report) >= 5000u) {
            last_report = HAL_GetTick();
            uint16_t k;
            printf("[mb] rx=%u(0len=%u drop=%u long=%u %uB) parse0=%u tx=%u(%uB) q=%u last=",
                   (unsigned)g_mb_rx_events, (unsigned)g_mb_rx_size0,
                   (unsigned)g_mb_rx_q_drop, (unsigned)g_mb_rx_overlong,
                   (unsigned)g_mb_rx_bytes,
                   (unsigned)g_mb_parse_zero, (unsigned)g_mb_tx_frames,
                   (unsigned)g_mb_tx_bytes,
                   (unsigned)uxQueueMessagesWaiting(s_rx_q));
            for (k = 0u; (k < g_mb_rx_last_n) && (k < 12u); k++) {
                printf("%02X ", (unsigned)g_mb_rx_last[k]);
            }
            printf("\r\n");
        }

        /* ---- 直方图汇总：每 10 秒打一次（见 g_hist_rx 的注释）----
         * 判读：
         *   无效帧(第3段)在时间上**集中** → 调度竞争；**均匀** → 位级裕量。
         * 数字含义：rx=该秒到达的帧数，nr=该秒"到达但没应答"的帧数。 */
        {
            static uint32_t last_hist_ms = 0u;
            if ((HAL_GetTick() - last_hist_ms) >= 10000u) {
                last_hist_ms = HAL_GetTick();
                uint32_t tot_rx = 0u, tot_nr = 0u, k;
                uint32_t worst_b = 0u, worst_n = 0u;
                for (k = 0u; k < MB_HIST_BUCKETS; k++) {
                    tot_rx += g_hist_rx[k];
                    tot_nr += g_hist_nr[k];
                    if (g_hist_nr[k] > worst_n) {
                        worst_n = g_hist_nr[k];
                        worst_b = k;
                    }
                }
                printf("[hist] 按秒累计: 到达=%u 无效=%u (%.0f%%)  最差桶=%u(无效%u)\r\n",
                       (unsigned)tot_rx, (unsigned)tot_nr,
                       (tot_rx > 0u) ? (100.0 * (double)tot_nr / (double)tot_rx) : 0.0,
                       (unsigned)worst_b, (unsigned)worst_n);
                /* 每 8 个桶一行，便于肉眼看"是否成片聚集" */
                printf("[hist] nr:");
                for (k = 0u; k < MB_HIST_BUCKETS; k += 8u) {
                    uint32_t s = 0u, j;
                    for (j = 0u; (j < 8u) && ((k + j) < MB_HIST_BUCKETS); j++) {
                        s += g_hist_nr[k + j];
                    }
                    printf(" %u", (unsigned)s);
                }
                printf("\r\n");
            }
        }
    }
}

/** 3 号任务：500ms 慢车道，采环境量并更新 A 区 */
static void task_acq_slow(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    uint16_t v;
    uint16_t humi;
    uint16_t status1;

    (void)arg;
    for (;;) {
        /* 传感器探测已经移到 task_health（优先级最高，不会被 50Hz 采样饿死）。
         * 这里只负责慢车道环境量采集。 */
        status1 = 0u;

        /* ① 模拟量 8 路（ADC1 扫描 + DMA 已在后台循环搬运 s_adc_raw）
         *    ADC 通道本身不会"掉线"，所以这几路直接标在线 */
        regbank_write_internal(REG_A_LIGHT, adc_filtered(ADC_IDX_LIGHT));
        regbank_write_internal(REG_A_SMOKE, adc_filtered(ADC_IDX_SMOKE));
        regbank_write_internal(REG_A_AIR,   adc_filtered(ADC_IDX_AIR));
        regbank_write_internal(REG_A_FLAME, adc_filtered(ADC_IDX_FLAME));
        regbank_write_internal(REG_A_SOIL,  adc_filtered(ADC_IDX_SOIL));
        regbank_write_internal(REG_A_RAIN,  adc_filtered(ADC_IDX_RAIN));
        regbank_write_internal(REG_A_POT,   adc_filtered(ADC_IDX_POT));
        /* 热敏 NTC 原始值：A 区 16 个寄存器已排满，暂不外露；
         * 如需上抛，可用 B 区保留位 0x001C~0x001F 扩展（见 docs/02 §5） */
        (void)adc_filtered(ADC_IDX_NTC);
        status1 |= (uint16_t)((1u << 4) | (1u << 5) | (1u << 6));

        /* ② 数字量 */
        if (HAL_GPIO_ReadPin(PIN_PIR_PORT, PIN_PIR_PIN) == GPIO_PIN_SET) {
            regbank_write_internal(REG_A_PIR, 1u);
            status1 |= (uint16_t)(1u << 8);
        } else {
            regbank_write_internal(REG_A_PIR, 0u);
            status1 |= (uint16_t)(1u << 8);   /* 传感器在线但当前无触发 */
        }
        if (HAL_GPIO_ReadPin(PIN_RADAR_PORT, PIN_RADAR_PIN) == GPIO_PIN_SET) {
            regbank_write_internal(REG_A_RADAR, 1u);
        } else {
            regbank_write_internal(REG_A_RADAR, 0u);
        }
        status1 |= (uint16_t)(1u << 9);

        /* ③ DHT11 / 气压 / 超声（失败则保留上次值，并把对应在线位置 0） */
        if (dht11_read(&v, &humi)) {
            regbank_write_i16(REG_A_TEMP, (int16_t)v);
            regbank_write_internal(REG_A_HUMI, humi);
            status1 |= (uint16_t)(1u << 0);
        }
        if (bmp280_read(&v)) {
            regbank_write_internal(REG_A_PRESS, v);
            status1 |= (uint16_t)(1u << 1);
        }
        if (hcsr04_read_mm(&v)) {
            regbank_write_internal(REG_A_DIST, v);
            status1 |= (uint16_t)(1u << 7);
        }

        /* ④ 心率血氧（间断测，采不到就写 0） */
        {
            uint16_t hr = 0u, spo2 = 0u;
            if (max30102_read(&hr, &spo2)) {
                regbank_write_internal(REG_A_HR, hr);
                regbank_write_internal(REG_A_SPO2, spo2);
                status1 |= (uint16_t)(1u << 3);
            }
        }

        /* ⑤ 只覆盖本任务负责的位，保留 task_acq_fast 维护的 MPU6050 在线位（bit2） */
        {
            uint16_t s1 = regbank_get(REG_B_STATUS1);
            s1 = (uint16_t)(s1 & (uint16_t)(~ST1_OWN_MASK_SLOW));
            regbank_write_internal(REG_B_STATUS1, (uint16_t)(s1 | status1));
        }

        /* ⑥ 最后才更新采样序号：主站看到序号变化时，本轮数据一定已经全部写好 */
        v = regbank_get(REG_A_SEQ);
        regbank_write_internal(REG_A_SEQ, (uint16_t)(v + 1u));

        task_delay_until(&last, pdMS_TO_TICKS(500));
    }
}

/** 3 号任务：把 C 区控制字落到硬件上 */
static void task_output(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    uint16_t ctrl;
    uint16_t safe;
    uint16_t alarms;
    uint16_t rgb_r;

    (void)arg;
    for (;;) {
        ctrl   = regbank_get(REG_C_CTRL);        /* 主站写入的控制字（本任务只读，绝不回写） */
        safe   = regbank_get(REG_C_SAFE_STATE);
        alarms = regbank_get(REG_B_ALARM_BITS);
        rgb_r  = regbank_get(REG_C_RGB_R);

        /* 本地自治时的输出仲裁。
         * ⚠️ 下位机**绝不写 C 区**（C 区是主站的地盘），本地行为只在这里"叠加"到输出上。
         *    否则主站的读-改-写（0x0020）会与下位机的写入互相覆盖 —— 典型的 RMW 竞争。 */
        if (regbank_get(REG_B_MODE) == 0u) {
            bool local_allowed = ((ctrl & CTRL_BIT_LOCAL_AUTO) != 0u);   /* 此时 ctrl 仍是主站原值 */

            if (safe != 0u) {
                ctrl = safe;                     /* 失效安全：安全状态位图覆盖远程控制 */
            }
            if ((alarms != 0u) && local_allowed) {
                ctrl |= (uint16_t)(CTRL_BIT_BUZZER | CTRL_BIT_RGB_EN);   /* 本地报警：蜂鸣 + 红灯 */
                rgb_r = 255u;
            }
        }

        /* 蜂鸣器（PC12） */
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12,
            ((ctrl & CTRL_BIT_BUZZER) != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        /* 风扇（TIM3_CH1 / PA6） */
        if ((ctrl & CTRL_BIT_FAN_EN) != 0u) {
            uint16_t duty = regbank_get(REG_C_FAN_PWM);
            if (duty > 1000u) duty = 1000u;
            __HAL_TIM_SET_COMPARE(s_htim_misc, TIM_CHANNEL_1, PWM_CCR_FROM_PERMIL(duty));
        } else {
            __HAL_TIM_SET_COMPARE(s_htim_misc, TIM_CHANNEL_1, 0u);
        }

        /* 电机（TIM3_CH2 / PA7 + 方向 PC10/PC11），PWM 带符号表示正反转 */
        if ((ctrl & CTRL_BIT_MOTOR_EN) != 0u) {
            int16_t pwm = (int16_t)regbank_get(REG_C_MOTOR_PWM);
            uint16_t duty = (uint16_t)((pwm < 0) ? -pwm : pwm);
            if (duty > 1000u) duty = 1000u;
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, (pwm >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, (pwm >= 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(s_htim_misc, TIM_CHANNEL_2, PWM_CCR_FROM_PERMIL(duty));
        } else {
            __HAL_TIM_SET_COMPARE(s_htim_misc, TIM_CHANNEL_2, 0u);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
        }

        /* 三色灯：R 在 TIM1_CH1，G/B 在 TIM3_CH3/CH4（0~255 → 0~999 必须换算，否则最亮只有 25%） */
        if ((ctrl & CTRL_BIT_RGB_EN) != 0u) {
            __HAL_TIM_SET_COMPARE(s_htim_rgb_r, TIM_CHANNEL_1, PWM_CCR_FROM_U8(rgb_r));
            __HAL_TIM_SET_COMPARE(s_htim_misc, TIM_CHANNEL_3, PWM_CCR_FROM_U8(regbank_get(REG_C_RGB_G)));
            __HAL_TIM_SET_COMPARE(s_htim_misc, TIM_CHANNEL_4, PWM_CCR_FROM_U8(regbank_get(REG_C_RGB_B)));
        } else {
            __HAL_TIM_SET_COMPARE(s_htim_rgb_r, TIM_CHANNEL_1, 0u);
            __HAL_TIM_SET_COMPARE(s_htim_misc, TIM_CHANNEL_3, 0u);
            __HAL_TIM_SET_COMPARE(s_htim_misc, TIM_CHANNEL_4, 0u);
        }

        /* 舵机（TIM2_CH1 / PA5，50Hz）：CCR = 500 + angle * 2000 / 180 */
        if ((ctrl & CTRL_BIT_SERVO_EN) != 0u) {
            uint16_t ang = regbank_get(REG_C_SERVO_ANGLE);
            if (ang <= 180u) {
                __HAL_TIM_SET_COMPARE(s_htim_servo, TIM_CHANNEL_1,
                                      (uint32_t)(500u + (uint32_t)ang * 2000u / 180u));
            }
        }

        task_delay_until(&last, pdMS_TO_TICKS(20));   /* 50Hz 刷新输出即可 */
    }
}

/** 2 号任务：健康检查 —— 看门狗、统计同步、本地自治降级 */
static void task_health(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    uint16_t alarms = 0u;

    (void)arg;
    for (;;) {
        /* ---- 心跳（调试期打开）：证明调度器真的跑起来了 ----
         * 如果串口上**完全没有** [hb] 行 → 卡在调度器启动之前，不是任务里的问题。 */
        {
            static uint32_t hb_last = 0xFFFFFFFFu;
            uint32_t sec = HAL_GetTick() / 1000u;
            if (sec != hb_last) {
                hb_last = sec;
#if !DBG_WINDOW_DUMP
                printf("[hb] t=%lus cmd=0x%04X ctrl=0x%04X alarm=0x%04X mpu=%u\r\n",
                       (unsigned long)sec,
                       (unsigned)regbank_get(REG_C_COMMAND),
                       (unsigned)regbank_get(REG_C_CTRL),
                       (unsigned)regbank_get(REG_B_ALARM_BITS),
                       (unsigned)(s_mpu_online ? 1u : 0u));
#endif
            }
        }

        /* ---- MPU6050 探测（带重试）----
         * 放在优先级最高的本任务里，就不会被 50Hz 采样任务饿死；
         * 探测前先喂狗。在线期间**完全不探测**，所以正常情况下这里是零开销。 */
        {
            static uint32_t next_probe_ms = 0u;
            if (!s_mpu_online && ((int32_t)(HAL_GetTick() - next_probe_ms) >= 0)) {
                next_probe_ms = HAL_GetTick() + 5000u;     /* 5 秒重试一次 */
                HAL_IWDG_Refresh(&hiwdg);

                /* ★ 每次重试前先做一次"总线自愈"：
                 *   解锁 9 时钟 + 重新初始化 I2C 外设。
                 *   为什么放在这里：实测"按复位键后总线被拉死 ✗，拔插 USB 就好 ✓"，
                 *   说明从机会卡在坏状态 ✗。有了这段，固件能自己把总线救回来 ✓，
                 *   不必再依赖断电重上电 ✓✓ */
                i2c_bus_unstick();
                s_hi2c->Init.ClockSpeed = 100000u;
                (void)HAL_I2C_Init(s_hi2c);
                if (mpu6050_init()) {
                    s_mpu_online = true;
                    printf("[sensor] mpu6050 ok -> 开始 50Hz 采样\r\n");
                    sensors_i2c_scan();                     /* 顺便把在线地址报出来 */

                    /* ---- 开机陀螺仪零偏标定 ----
                     * MPU6050 零偏规格可达 ±20°/s，不减掉会给特征引入固定偏移。
                     * ⚠️ 必须**在静止时**标定：若标定时板子被拿着/晃动，算出来的是"运动平均值"，
                     *    标了反而更糟（实测踩过：标定值 gy=-247，标完残余仍有 -800 LSB ✗）。
                     *    所以这里同时统计每轴的峰峰值做**静止性检查**，不合格就丢弃重来。 */
                    {
                        int32_t sum[3]  = { 0, 0, 0 };
                        int16_t gmin[3] = { 32767, 32767, 32767 };
                        int16_t gmax[3] = { -32768, -32768, -32768 };
                        int16_t s6[6];
                        uint32_t ok = 0u, k;
                        uint16_t a;
                        int16_t  span = 0;

                        for (k = 0u; k < 50u; k++) {
                            if (mpu6050_read(s6)) {
                                for (a = 0u; a < 3u; a++) {
                                    int16_t g = s6[3u + a];
                                    sum[a] += g;
                                    if (g < gmin[a]) { gmin[a] = g; }
                                    if (g > gmax[a]) { gmax[a] = g; }
                                }
                                ok++;
                            }
                            HAL_IWDG_Refresh(&hiwdg);
                            vTaskDelay(pdMS_TO_TICKS(2));
                        }
                        for (a = 0u; a < 3u; a++) {
                            int16_t d = (int16_t)(gmax[a] - gmin[a]);
                            if (d > span) { span = d; }
                        }

                        if (ok < 40u) {
                            /* ★ 标定失败**绝不能让传感器掉线** ✗→✓
                             * 教训：原来这里写了 s_mpu_online = false，
                             * 结果"零偏不准"这个**小问题**被升级成"整机离线"这个**灾难** ✗✗
                             * —— 窗口全填 0，看起来像传感器坏了/掉了 ✓（排查了很久 ✗）。
                             * 正确做法：标定不成就不减偏置（按 0 算），**继续采数据** ✓。 */
                            printf("[sensor] 零偏标定样本不足(%u/50) -> 本轮不减偏置，继续采样\r\n",
                                   (unsigned)ok);
                        } else if (span > 300) {
                            /* 抖动大 → 本次平均值不可用。同样**不置离线** ✓ */
                            printf("[sensor] 标定时抖动偏大(%d LSB) -> 本轮不减偏置，继续采样\r\n",
                                   (int)span);
                        } else {
                            s_gyro_bias[0] = (int16_t)(sum[0] / (int32_t)ok);
                            s_gyro_bias[1] = (int16_t)(sum[1] / (int32_t)ok);
                            s_gyro_bias[2] = (int16_t)(sum[2] / (int32_t)ok);
                            printf("[sensor] 陀螺仪零偏标定: gx=%d gy=%d gz=%d LSB (抖动 %d)\r\n",
                                   (int)s_gyro_bias[0], (int)s_gyro_bias[1], (int)s_gyro_bias[2], (int)span);
                        }
                    }
                } else {
                    printf("[sensor] mpu6050 未就绪（未接线？），5 秒后重试\r\n");
                }
            }
        }

        regbank_tick(HAL_GetTick());
        regbank_sync_stats();

        /* ---- 本地阈值报警（网关离线时这就是"自治"的核心） ---- */
        if ((int16_t)regbank_get(REG_A_TEMP) > (int16_t)regbank_get(REG_C_TH_TEMP_MAX)) {
            alarms |= (1u << 0);
        }
        if (regbank_get(REG_A_SMOKE) > regbank_get(REG_C_TH_SMOKE_MAX)) {
            alarms |= (1u << 1);
        }
        {
            uint16_t hr = regbank_get(REG_A_HR);
            uint16_t hr_min = regbank_get(REG_C_TH_HR_MIN);
            if ((hr != 0u) && (hr < hr_min)) {
                alarms |= (1u << 3);
            }
        }
        regbank_write_internal(REG_B_ALARM_BITS, alarms);

        /* ⚠️ 这里**故意不写 C 区**：C 区是主站的写入区，从机只读。
         *    "本地报警时蜂鸣器要响"这类行为统一在 task_output 里仲裁（见那里的注释），
         *    否则主站对 0x0020 的读-改-写会与本任务的写入互相覆盖（典型 RMW 竞争）。 */

        /* 软重启命令（0x5A5A）。
         * ⚠️ 清标志必须用 regbank_write_internal()，**不能用 regbank_set()**：
         *    regbank_set() 是"主站写入路径"，要先过 C 区所有权与取值范围校验；
         *    万一被拒绝，复位后 RAM 里仍是 0x5A5A → 无限复位。
         *    同时打印一行，保证"谁触发的复位"永远有据可查。 */
        if (regbank_get(REG_C_COMMAND) == 0x5A5Au) {
            printf("[health] 收到软重启命令 0x5A5A -> NVIC_SystemReset()\r\n");
            regbank_write_internal(REG_C_COMMAND, 0u);
            HAL_IWDG_Refresh(&hiwdg);
            NVIC_SystemReset();
        }

        HAL_IWDG_Refresh(&hiwdg);   /* 喂狗必须放在所有任务都健康的假设下；单独建"看门狗任务"更严谨 */

        task_delay_until(&last, pdMS_TO_TICKS(200));
    }
}

/* ======================================================================= */
/*  特征提取：100×6 窗口 → 36 维特征（docs/02 §5 E 区）                     */
/* ======================================================================= */

#define FEAT_FS_HZ      50.0f    /* 采样率：与 task_acq_fast 的 20ms 周期一致 */
#define FEAT_FMIN_HZ     3.0f    /* 主频搜索下限（更低的多半是姿态漂移，不是振动） */
#define FEAT_FMAX_HZ    25.0f    /* 上限 = Nyquist（fs/2） */
#define FEAT_DF_HZ       0.5f    /* 频率分辨率 = fs/N = 50/100 */

/** D 区是"**每样本 6 轴**"的行优先布局：取第 k 个样本的第 axis 个轴 */
static int16_t win_at(const int16_t *w, uint16_t k, uint16_t axis)
{
    return w[(uint16_t)(k * REG_WIN_AXES + axis)];
}

/**
 * @brief Goertzel：单个频点上的能量
 *
 * 比整段 FFT 简单得多，而且只算我们关心的频段。40 个频点 × 100 点 × 6 轴
 * ≈ 2.4 万次运算，对带硬件 FPU 的 F411 只要 1~2ms —— 占 2 秒窗口的千分之一。
 * 用浮点换来的好处：同一份代码在 PC 上能直接跑单测。
 */
static float goertzel_pow(const int16_t *w, uint16_t axis, float freq)
{
    const float coeff = 2.0f * cosf(2.0f * 3.14159265f * freq / FEAT_FS_HZ);
    float s1 = 0.0f, s2 = 0.0f;
    uint16_t k;

    for (k = 0u; k < REG_WIN_SAMPLES; k++) {
        float s0 = (float)win_at(w, k, axis) + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    return (s1 * s1) + (s2 * s2) - coeff * s1 * s2;
}

/**
 * @brief 36 维特征，**严格按 per-feature 布局**写入 out[]
 *
 * ⚠️ 必须"**按特征分段**"写，不能"按轴循环"写：两者互为转置，维度都是 36，
 *    程序不报错、模型照训照跑，但**准确率会掉到随机水平**且极难排查。
 *    这是本项目最隐蔽的坑（docs/02 §5-E 有完整说明）。
 */
static void compute_features(const int16_t *w, int16_t out[36])
{
    uint16_t axis, k;

    /* ---- 第一段：均值 / 标准差 / 峰峰值 / 过零率 ---- */
    for (axis = 0u; axis < REG_WIN_AXES; axis++) {
        int32_t  sum = 0;
        int16_t  vmin = 32767, vmax = -32768;
        float    mean, acc = 0.0f;
        uint16_t zc = 0u;
        int      prev_sign = 0;

        for (k = 0u; k < REG_WIN_SAMPLES; k++) {
            int16_t v = win_at(w, k, axis);
            sum += v;
            if (v < vmin) { vmin = v; }
            if (v > vmax) { vmax = v; }
        }
        mean = (float)sum / (float)REG_WIN_SAMPLES;

        /* 过零率按"相对均值的符号变化"统计：直接数过 0 会被直流偏置骗到 */
        for (k = 0u; k < REG_WIN_SAMPLES; k++) {
            float d = (float)win_at(w, k, axis) - mean;
            int   s;
            acc += d * d;
            s = (d > 0.0f) ? 1 : ((d < 0.0f) ? -1 : 0);
            if ((s != 0) && (prev_sign != 0) && (s != prev_sign)) { zc++; }
            if (s != 0) { prev_sign = s; }
        }

        out[0u * REG_WIN_AXES + axis] = (int16_t)mean;                                    /* 均值 */
        out[1u * REG_WIN_AXES + axis] = (int16_t)sqrtf(acc / (float)REG_WIN_SAMPLES);      /* 标准差 */
        out[2u * REG_WIN_AXES + axis] = (int16_t)(vmax - vmin);                           /* 峰峰值 */
        out[3u * REG_WIN_AXES + axis] = (int16_t)((uint32_t)zc * 100u / REG_WIN_SAMPLES);  /* 过零率 ×100 */
    }

    /* ---- 第二段：主频 / 主频带能量占比（同样 per-feature）---- */
    for (axis = 0u; axis < REG_WIN_AXES; axis++) {
        float f, best_f = 0.0f, best_p = 0.0f, total = 0.0f;

        for (f = FEAT_FMIN_HZ; f <= (FEAT_FMAX_HZ + 0.01f); f += FEAT_DF_HZ) {
            float p = goertzel_pow(w, axis, f);
            total += p;
            if (p > best_p) { best_p = p; best_f = f; }
        }
        out[4u * REG_WIN_AXES + axis] = (int16_t)(best_f * 10.0f);                      /* 主频 ×10 Hz */
        out[5u * REG_WIN_AXES + axis] = (int16_t)(1000.0f * best_p / (total + 1e-3f));  /* 能量占比 ×1000 */
    }
}

/* ======================================================================= */
/*  打点模式：把窗口按 "W,<seq>,<600 个 int16>" 打到调试串口                  */
/* ======================================================================= */
/* 1 = 打开（配合 PC 端 tools/capture_window.py 采数据集）；0 = 关闭。
 *
 * 格式必须与 capture_window.py 的解析器严格一致：
 *   首字段 'W'、第二字段窗口序号、随后 600 个 int16，
 *   **行优先 = 样本 0 的 6 轴在前**（与 docs/02 §5 D 区一致）。
 * 打开时会把 [imu]/[dsp]/[hb] 等日志全部静音，保证串口流干净、不干扰采集。
 *
 * 一行约 4.2KB；115200 下约 0.37s，而窗口 2s 一个 —— 余量 5 倍以上。
 *
 * ⚠️ 打开它会把 [hb]/[imu]/[dsp] 全部静音（保持串口流干净），
 *    因此**调 Modbus / 485 时必须关掉**（否则 F411 侧日志全无，排查变成瞎猜）。
 *    采数据集时再打开。 */
/* ⚠️ 已移到文件顶部（所有 #if 之前）——见上方"调试开关"一节 ✗→✓ */

static void dbg_transmit(const char *ptr, uint16_t len);   /* 见文件末尾 _write 处 */

#if DBG_WINDOW_DUMP
/** int16 转十进制追加到 buf（自己实现：600 次 printf 既慢又受可变参数限制） */
static uint16_t append_i16(char *buf, uint16_t pos, int16_t v)
{
    char     tmp[8];
    uint16_t n = 0u;
    uint32_t u;

    if (v < 0) {
        buf[pos++] = '-';
        u = (uint32_t)(-(int32_t)v);
    } else {
        u = (uint32_t)v;
    }
    do {
        tmp[n++] = (char)('0' + (char)(u % 10u));
        u /= 10u;
    } while (u != 0u);
    while (n > 0u) {
        buf[pos++] = tmp[--n];
    }
    return pos;
}

/** 一个窗口打一行 */
static void dump_window(const int16_t *w, uint16_t seq)
{
    static char buf[16u + (REG_WIN_SAMPLES * REG_WIN_AXES * 7u)];   /* int16 最长 6 字符 + 逗号 */
    uint16_t pos = 0u;
    uint16_t i;

    buf[pos++] = 'W';
    buf[pos++] = ',';
    pos = append_i16(buf, pos, (int16_t)seq);
    for (i = 0u; i < (uint16_t)(REG_WIN_SAMPLES * REG_WIN_AXES); i++) {
        buf[pos++] = ',';
        pos = append_i16(buf, pos, w[i]);
    }
    buf[pos++] = '\r';
    buf[pos++] = '\n';
    dbg_transmit(buf, pos);
}
#endif

/** 6 号任务：窗口就绪就做特征提取，写 E 区 */
static void task_dsp(void *arg)
{
    /* 1200 字节的窗口缓冲放静态，避免吃掉任务栈（printf 本身也要几百字节） */
    static int16_t s_win[REG_WIN_SAMPLES * REG_WIN_AXES];
    TickType_t last = xTaskGetTickCount();
    uint16_t last_seq = 0u;
    uint16_t i;

    (void)arg;
    for (;;) {
        uint16_t seq = regbank_window_seq();

        if (regbank_window_ready() && (seq != last_seq) && !regbank_window_locked()) {
            int16_t feat[36];
            bool    consistent = true;

            /* 取窗口：读前读后校验序号 —— 期间若有新窗口写入，则本帧作废重来。
             * 这是"防跨窗口撕裂"的关键（docs/02 §7），配合 regbank 的读锁一起用。 */
            for (i = 0u; i < (uint16_t)(REG_WIN_SAMPLES * REG_WIN_AXES); i++) {
                s_win[i] = (int16_t)regbank_get((uint16_t)(REG_D_DATA + i));
            }
            if (regbank_window_seq() != seq) {
                consistent = false;
            }

            if (consistent) {
                compute_features(s_win, feat);
                for (i = 0u; i < 36u; i++) {
                    regbank_write_i16((uint16_t)(REG_E_FEATURES + i), feat[i]);
                }
                last_seq = seq;

#if DBG_WINDOW_DUMP
                dump_window(s_win, seq);          /* 打点模式：只吐数据 */
#else
                /* az 是最能反映振动/晃动的一路，打出来便于肉眼验收。
                 * 用整数打印频率（%d.%dHz）是刻意的：newlib-nano 默认不支持 %f。 */
                printf("[dsp] win#%u az: mean=%d std=%d pp=%d zcr=%d f=%d.%dHz e=%d/1000\r\n",
                       (unsigned)seq,
                       (int)feat[0u * REG_WIN_AXES + 2u],
                       (int)feat[1u * REG_WIN_AXES + 2u],
                       (int)feat[2u * REG_WIN_AXES + 2u],
                       (int)feat[3u * REG_WIN_AXES + 2u],
                       (int)(feat[4u * REG_WIN_AXES + 2u] / 10),
                       (int)(feat[4u * REG_WIN_AXES + 2u] % 10),
                       (int)feat[5u * REG_WIN_AXES + 2u]);
#endif
            }
        }
        task_delay_until(&last, pdMS_TO_TICKS(200));
    }
}

/* ======================================================================= */
/*  ADC 的 DMA 兜底初始化                                                  */
/* ======================================================================= */

/**
 * @brief 由代码建立 ADC1 → DMA2 Stream0 的通路
 *
 * 为什么需要它：CubeMX 里 ADC1 的 DMA 请求是"外设级"配置，如果 .ioc 里没有
 * DMA 模型（Mcu.IP 缺少 DMA），GUI 的 Add/Select 里就可能选不到 ADC1。
 * 本函数等价于 CubeMX 生成的 HAL_ADC_MspInit() 里那一段，效果完全相同。
 *
 * ⚠️ 它只在 `hadc->DMA_Handle == NULL` 时被调用（见 app_init），
 *    所以将来你在 CubeMX 里配好 DMA 请求后，这段会**自动让位**，不会重复初始化。
 */
static void adc_dma_manual_init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();

    s_hdma_adc1.Instance                 = DMA2_Stream0;          /* F411: ADC1 固定挂在 DMA2 Stream0 / Channel0 */
    s_hdma_adc1.Init.Channel             = DMA_CHANNEL_0;
    s_hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;  /* 外设 → 内存 */
    s_hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;      /* ADC 数据寄存器地址固定 */
    s_hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;       /* 内存地址递增，依次填 s_adc_raw[0..7] */
    s_hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    s_hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    s_hdma_adc1.Init.Mode                = DMA_CIRCULAR;          /* 循环：永远在后台搬运，不用中断 */
    s_hdma_adc1.Init.Priority            = DMA_PRIORITY_LOW;
    s_hdma_adc1.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(&s_hdma_adc1) != HAL_OK) {
        printf("[adc] 手动 DMA 初始化失败\r\n");
        return;
    }
    __HAL_LINKDMA(s_hadc, DMA_Handle, s_hdma_adc1);   /* 挂到 ADC 上，HAL_ADC_Start_DMA 才认 */
    printf("[adc] DMA 由代码手动建立（CubeMX 未配置 DMA 请求）\r\n");
}

/* ======================================================================= */
/*  ADC 规则通道序列（Rank）兜底配置                                        */
/* ======================================================================= */

/** 1 = 由代码强制配置 8 个 ADC rank；确认 CubeMX 的 Rank 表无误后可改成 0 */
#define ADC_RANKS_CONFIG_IN_CODE   1

/**
 * @brief 由代码显式配置 8 个 ADC 规则通道（Rank 1..8 = IN8..IN15 / PB0,PB1,PC0..PC5）
 *
 * 为什么要这么做：CubeMX 的 ADC "Mode" 面板与它写进 .ioc 的模型出现过不同步
 * （界面上 8 个通道都勾了，文件里却只有 1 个 rank），而 Rank 表改起来又很容易错。
 * 在这里显式配置，好处有三个：
 *   1) 通道顺序与 `ADC_IDX_*` 宏由代码保证一致，不依赖 GUI；
 *   2) 采样时间统一给足（84 cycles）—— MQ 系列与光敏/NTC 的输出阻抗较高，
 *      CubeMX 默认的 3 cycles 会明显欠采；
 *   3) 重新生成 CubeMX 代码不会覆盖它（本文件从不被 CubeMX 改动）。
 */
static void adc_config_ranks_manual(void)
{
    static const uint32_t kCh[ADC_CH_COUNT] = {
        ADC_CHANNEL_8,  ADC_CHANNEL_9,  ADC_CHANNEL_10, ADC_CHANNEL_11,
        ADC_CHANNEL_12, ADC_CHANNEL_13, ADC_CHANNEL_14, ADC_CHANNEL_15
    };
    ADC_ChannelConfTypeDef cfg;
    uint32_t i;

    printf("[init] adc ranks: CubeMX=%u -> code=%u\r\n",
           (unsigned)s_hadc->Init.NbrOfConversion, (unsigned)ADC_CH_COUNT);

    /* ⚠️ 这里**不能**用 HAL_ADC_Init() 重新初始化来改"序列长度"：
     *   HAL_ADC_Init() 结尾会把 ADC 使能（ADON=1），而 RM0090 规定
     *   ADC_SQRx / ADC_SMPRx 只能在 ADON=0 时修改 —— 使能状态下写是**无效**的。
     *   而 CubeMX 生成的 adc.c 里 ScanConvMode / ContinuousConvMode /
     *   DMAContinuousRequests / ClockPrescaler(=PCLK2/4) 已经全部正确，
     *   唯一缺的就是"序列长度 = 8 + 8 个 rank"（它的 Rank 表里只有 1 个通道）。
     *   所以正确顺序是：停 ADC → 写序列长度与 8 个 rank → 重新使能。 */
    __HAL_ADC_DISABLE(s_hadc);

    s_hadc->Init.NbrOfConversion = ADC_CH_COUNT;
    ADC1->SQR1 = (ADC1->SQR1 & ~(0xFu << 20)) | ((uint32_t)(ADC_CH_COUNT - 1u) << 20);

    cfg.Offset       = 0u;
    cfg.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    for (i = 0u; i < ADC_CH_COUNT; i++) {
        cfg.Channel = kCh[i];
        cfg.Rank    = i + 1u;
        if (HAL_ADC_ConfigChannel(s_hadc, &cfg) != HAL_OK) {
            printf("[init] adc rank %u FAIL\r\n", (unsigned)(i + 1u));
        }
    }

    __HAL_ADC_ENABLE(s_hadc);
    {   /* 等 ADC 稳定（等价于 HAL 内部的 ADC_STAB_DELAY_US） */
        volatile uint32_t d = 20000u;
        while (d-- != 0u) { }
    }
    printf("[init] adc ranks done\r\n");
}

/* ======================================================================= */
/*  复位原因（硬件寄存器说了算，不靠猜）                                     */
/* ======================================================================= */

/**
 * @brief 读 RCC->CSR 的复位标志，判断"上一次为什么重启"
 *
 * 这一条是排查"板子反复重启"的第一手证据：
 *   IWDG   → 有人卡死/没喂狗（软件问题）
 *   SFT    → 代码里调了 NVIC_SystemReset()（软件主动）
 *   POR    → 正常上电
 *   BOR    → **欠压复位 = 电源问题**（USB 供电不足、接触不良、线太细）
 *   PIN    → 复位按键被按 / 复位脚被拉低
 *
 * 结果同时写进 REG_B_RESET_CAUSE（0x0017），主站也能读到。
 * 读完必须清标志，否则下一次分不清"这一次"的原因。
 */
static void reset_cause_report(void)
{
    uint16_t cause = 0u;
    const char *why = "无标志(?)";

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET) {
        cause |= 0x0001u; why = "IWDG 看门狗复位（有任务卡死或没喂狗）";
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET) {
        cause |= 0x0002u; why = "软件复位 NVIC_SystemReset()";
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET) {
        cause |= 0x0004u; why = "上电复位 POR（正常）";
#ifdef RCC_FLAG_BORRST
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != RESET) {
        cause |= 0x0008u; why = "欠压复位 BOR —— 这是电源问题！";
#endif
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET) {
        cause |= 0x0010u; why = "复位引脚/按键";
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) != RESET) {
        cause |= 0x0020u; why = "低功耗复位";
    }

    printf("[boot] reset cause = 0x%04X : %s\r\n", (unsigned)cause, why);

    regbank_write_internal(REG_B_RESET_CAUSE, cause);
    __HAL_RCC_CLEAR_RESET_FLAGS();
}

/* ======================================================================= */
/*  初始化                                                                */
/* ======================================================================= */

void app_init(UART_HandleTypeDef *huart_dbg,
              UART_HandleTypeDef *huart_mb,
              ADC_HandleTypeDef  *hadc,
              I2C_HandleTypeDef  *hi2c,
              TIM_HandleTypeDef  *htim1,
              TIM_HandleTypeDef  *htim2,
              TIM_HandleTypeDef  *htim3)
{
    uint8_t n = 0u;

    s_huart_dbg  = huart_dbg;
    s_huart_mb   = huart_mb;
    s_hadc       = hadc;
    s_hi2c       = hi2c;

    /* ---- I2C 降到 100kHz（标准模式）✗→✓ ----
     * CubeMX 默认把 I2C1 配成 400kHz Fast mode ✗。用杜邦线接传感器时，
     * 400kHz 对线长 / 接触电阻 / 分布电容非常敏感 ✗ → 会**偶发 NACK** →
     * 固件判定"传感器掉线" → 窗口被填 0 ✗✗。
     *
     * 实测现象（tools/precheck_quiet.py）：az std 交替出现 0 和 3000+ ✓，
     * 就是"掉线-恢复"在反复抖 ✓ —— 很容易被误判成"传感器在剧烈振动" ✗。
     *
     * 降到 100kHz 后时序裕量大得多；我们只需 50Hz 采样，100kHz 完全够用 ✓。 */
    /* ---- ① 先解锁 I2C 总线（敲 9 个时钟脉冲）----
     * 必须在 I2C 外设接管引脚之前做，否则复位后总线可能仍是锁死状态 ✗ */
    i2c_bus_unstick();

    /* ---- ② 降到 100kHz（标准模式）✓ ----
     * CubeMX 默认 400kHz Fast mode ✗，用杜邦线时对线长/接触/分布电容很敏感 ✗
     * → 偶发 NACK → 固件误判"传感器掉线" → 窗口填 0 ✗✗ */
    s_hi2c->Init.ClockSpeed = 100000u;
    (void)HAL_I2C_Init(s_hi2c);

    /* 开机就做一次 I2C 总线自检：把"传感器离线"的三种原因分清楚 ✓ */
    i2c_bus_selftest();
    s_htim_rgb_r = htim1;
    s_htim_servo = htim2;
    s_htim_misc  = htim3;

    setvbuf(stdout, NULL, _IONBF, 0);

    regbank_init();

    /* ---- 第一件事：搞清楚"上一次为什么重启"（硬件寄存器，不靠猜）---- */
    reset_cause_report();

    s_slave.slave_addr    = (uint8_t)(regbank_get_slave_addr() & 0xFFu);
    s_slave.areas         = regbank_areas(&n);
    s_slave.area_count    = n;
    s_slave.get           = regbank_get;
    s_slave.set           = regbank_set;
    s_slave.check         = regbank_check;
    s_slave.on_block_read = regbank_on_block_read;

    /* printf 互斥锁：必须在任何任务开始打印之前建好（app_init 早于 MX_FREERTOS_Init） */
    s_printf_mutex = xSemaphoreCreateMutex();
    if (s_printf_mutex == NULL) {
        printf("[init] 警告：printf 互斥锁创建失败，多任务日志可能丢行\r\n");
    }

    s_rx_q = xQueueCreate(MODBUS_RX_QUEUE_LEN, sizeof(mb_frame_t));

    /* PWM 启动 */
    HAL_TIM_PWM_Start(s_htim_rgb_r, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(s_htim_servo, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(s_htim_misc, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(s_htim_misc, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(s_htim_misc, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(s_htim_misc, TIM_CHANNEL_4);

    /* ⚠️ IWDG 在 MX_IWDG_Init() 里就已经启动（2s），而初始化期间还没有任务在喂狗，
     *    所以每一步都要喂一次；否则初始化稍慢就会被看门狗打断成"复位循环"。 */
    HAL_IWDG_Refresh(&hiwdg);

    printf("[init] 1/6 adc dma handle\r\n");
    if (s_hadc->DMA_Handle == NULL) {
        adc_dma_manual_init();
    }
    HAL_IWDG_Refresh(&hiwdg);

    printf("[init] 2/6 adc ranks\r\n");
#if ADC_RANKS_CONFIG_IN_CODE
    adc_config_ranks_manual();   /* 显式配置 8 个 rank（不依赖 CubeMX 的 Rank 表） */
#endif
    HAL_IWDG_Refresh(&hiwdg);

    printf("[init] 3/6 adc start dma\r\n");
    /* 循环 DMA 不需要"完成中断"：若 CubeMX 勾了 DMA2 Stream0 的 NVIC 中断就摘掉，
     * 否则 8 通道扫描每约 5µs 一轮，HT/TC 触发约 40 万次/秒，白烧 ~25% CPU。 */
    HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);

    if (HAL_ADC_Start_DMA(s_hadc, (uint32_t *)s_adc_raw, ADC_CH_COUNT) != HAL_OK) {
        printf("[adc] start dma failed\r\n");
    }
    HAL_IWDG_Refresh(&hiwdg);

    /* ⚠️ 传感器探测（MPU6050 + I2C 扫描）**故意不放在启动初始化里**：
     *    它们是"慢且可选"的操作 —— I2C 上还没接从机、也没有上拉电阻时，驱动会一直等到超时。
     *    放在调度器启动之前会拖住整个启动，甚至把 IWDG 喂狗的机会都掐掉（实测就卡在这里）。
     *    改由 task_acq_slow 首次运行时探测一次（见那里的 [sensor] probe ... 日志），
     *    这样"外设没接"最坏只是那个任务慢一点，系统照样能起来、能通信。 */
    printf("[init] 4/5 sensors -> deferred to task_acq_slow\r\n");
    HAL_IWDG_Refresh(&hiwdg);

    printf("[init] 5/5 485 rx\r\n");
    /* 485 先置接收态，再启动接收 */
    RS485_DE_OFF();
    modbus_start_rx();

    printf("[boot] regbank ready, slave addr = %u\r\n", (unsigned)s_slave.slave_addr);
    /* 堆余量：调度器启动时还要再建 idle + timer 两个任务（约 1.5KB），
     * 若这里 free 已经很小，就是"堆耗尽 → configASSERT → 静默死循环"。 */
    printf("[boot] heap: total=%u, free=%u bytes\r\n",
           (unsigned)configTOTAL_HEAP_SIZE, (unsigned)xPortGetFreeHeapSize());
}

void app_start_tasks(void)
{
    (void)xTaskCreate(task_acq_fast, "acq_fast", TASK_ACQ_FAST_STACK, NULL, PRIO_ACQ_FAST, NULL);
    (void)xTaskCreate(task_modbus,   "modbus",   TASK_MODBUS_STACK,   NULL, PRIO_MODBUS,   NULL);
    (void)xTaskCreate(task_acq_slow, "acq_slow", TASK_ACQ_SLOW_STACK, NULL, PRIO_ACQ_SLOW, NULL);
    (void)xTaskCreate(task_output,   "output",   TASK_OUTPUT_STACK,   NULL, PRIO_OUTPUT,   NULL);
    (void)xTaskCreate(task_health,   "health",   TASK_HEALTH_STACK,   NULL, PRIO_HEALTH,   NULL);
    (void)xTaskCreate(task_dsp,      "dsp",      TASK_DSP_STACK,      NULL, PRIO_DSP,      NULL);
}

/* ======================================================================= */
/*  printf 重定向到 USART1（CH340）                                        */
/* ======================================================================= */
/* ======================================================================= */
/*  故障现场上报                                                          */
/* ======================================================================= */
/* CubeMX 不生成 HardFault 等处理函数，默认落到启动文件里的 Default_Handler
 * —— 那是个死循环，什么也不说，表现就是"板子反复重启、串口只重复第一行"。
 * 下面给出强定义覆盖它（启动文件里是 weak 符号），把原因和故障类型打出来。 */
static void fault_report(const char *name)
{
    printf("\r\n[FAULT] %s   CFSR=0x%08lX  HFSR=0x%08lX\r\n",
           name, (unsigned long)SCB->CFSR, (unsigned long)SCB->HFSR);
    printf("[FAULT] 已停机（2 秒后 IWDG 会复位，所以你会看到这条消息反复出现）\r\n");
    for (;;) { }
}

void HardFault_Handler(void)  { fault_report("HardFault"); }
void MemManage_Handler(void)  { fault_report("MemManage"); }
void BusFault_Handler(void)   { fault_report("BusFault"); }
void UsageFault_Handler(void) { fault_report("UsageFault"); }

/**
 * @brief FreeRTOS 断言失败（configASSERT）的落脚点
 *
 * 由 FreeRTOSConfig.h 里的宏调用并传入 __FILE__/__LINE__，
 * 所以能直接指出是**哪一个文件哪一行**的断言挂了。
 * （CubeMX 的默认实现是关中断 + 死循环，什么都不说。）
 */
void app_assert_failed(const char *file, int line)
{
    printf("\r\n[ASSERT] FreeRTOS 断言失败: %s:%d\r\n", file, (int)line);
    printf("[ASSERT] CFSR=0x%08lX HFSR=0x%08lX  heap_free=%u / total=%u\r\n",
           (unsigned long)SCB->CFSR, (unsigned long)SCB->HFSR,
           (unsigned)xPortGetFreeHeapSize(), (unsigned)configTOTAL_HEAP_SIZE);
    printf("[ASSERT] 已停机；2 秒后 IWDG 复位，所以这条消息会重复出现\r\n");
    __disable_irq();
    for (;;) { }
}

/* 所有调试输出的唯一出口：先拿互斥锁再发，避免多任务并发时 HAL_UART_Transmit
 * 看到 UART 忙而返回 HAL_BUSY、**整行日志被静默丢弃**。
 * 调度器还没启动时（app_init 阶段）直接发即可，那时只有一个执行流。 */
static void dbg_transmit(const char *ptr, uint16_t len)
{
    bool locked = false;

    if ((s_printf_mutex != NULL) && (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)) {
        if (xSemaphoreTake(s_printf_mutex, pdMS_TO_TICKS(200)) == pdPASS) {
            locked = true;
        }
    }
    if (s_huart_dbg != NULL) {
        /* ⚠️ 超时给 2000ms 而不是 100ms：
         *   普通日志行只有几十字节，100ms 绰绰有余；
         *   但**打点模式一行约 3200 字节**，115200 下要 ~278ms ✗，
         *   给 100ms 会导致 HAL_UART_Transmit **发到一半就中止** ✗
         *   → 每行只剩 ~1150 字节（约 262 个字段），采集脚本全部解析失败 ✗。
         *   这是实测踩到的坑：179 行解析失败、0 个窗口入库。 */
        HAL_UART_Transmit(s_huart_dbg, (uint8_t *)ptr, len, 2000u);
    }
    if (locked) {
        (void)xSemaphoreGive(s_printf_mutex);
    }
}

#if defined(__GNUC__)
int _write(int file, char *ptr, int len)
{
    (void)file;
    dbg_transmit(ptr, (uint16_t)len);
    return len;
}
#else
int fputc(int ch, FILE *f)
{
    char c = (char)ch;
    (void)f;
    dbg_transmit(&c, 1u);
    return ch;
}
#endif
