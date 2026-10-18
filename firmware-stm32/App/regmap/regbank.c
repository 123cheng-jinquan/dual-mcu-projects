/**
 * @file    regbank.c
 * @brief   寄存器表 + 六轴采样窗口双缓冲实现
 *
 * 为什么这么设计（面试可讲）：
 *   1. 主站要一次拿到 600 个寄存器，但 Modbus 单帧最多读 125 个 → 必须分 5 帧读。
 *      如果这 5 帧之间采集任务切换了缓冲，主站拼出来的窗口就是"半个旧窗口 + 半个新窗口"，
 *      训练数据被污染且极难排查。→ 用"双缓冲 + 读取期间锁定 + 窗口序号校验"解决。
 *   2. 锁定必须有兜底：主站读到一半崩了，窗口不能永远不更新 → 500ms 超时强制解锁。
 *   3. 锁定期间采集继续写"另一个"缓冲；写缓冲满了就丢样本并置溢出位——
 *      宁可丢样本，也绝不写正在被读的那块内存。
 */
#include "regbank.h"
#include <string.h>

/* ---- 出厂默认值 -------------------------------------------------------- */
#define DEF_SLAVE_ADDR      1u
#define DEF_BAUD_CODE       4u      /* 4 = 115200，见 docs/02 §寄存器表 0x002D 的编码：
                                     *     0=9600 1=19200 2=38400 3=57600 **4=115200**
                                     * ⚠️ 这个寄存器只是**记录值**，写它不会改 UART：
                                     *    真正的波特率在 cubemx/Src/usart.c 的 MX_USART2_UART_Init()，
                                     *    必须与 ESP32 的 MODBUS_BAUD 保持一致（否则一帧都收不到）。
                                     * 2026-10：曾为排查物理层丢字节临时降到 9600(0)，
                                     *    后定位到真正原因是**主站事务间隔不够**
                                     *    （见 firmware-esp32/README.md §9.2.1），与波特率无关
                                     *    → 已切回 115200，并同步改 usart.c 的 huart2。 */
#define DEF_ACQ_PERIOD_MS   500u
#define DEF_TH_TEMP_MAX     400u    /* 40.0 ℃ */
#define DEF_TH_SMOKE_MAX    2500u
#define DEF_TH_HR_MIN       50u
#define DEF_WD_TIMEOUT      30u     /* ×100ms = 3s */
#define DEF_SAFE_STATE      0u
#define DEF_SAMPLE_RATE_X10 500u    /* 50.0 Hz */

/* B 区状态字 2 的位定义 */
#define ST2_I2C_FAULT       (1u << 0)
#define ST2_SPI_FAULT       (1u << 1)
#define ST2_WIN_OVERRUN     (1u << 2)
#define ST2_LOCAL_AUTO      (1u << 3)
#define ST2_COMM_WD         (1u << 4)

/* ---- 存储 -------------------------------------------------------------- */
static uint16_t g_regs[REGBANK_SIZE];                                  /* A/B/C/E 区影子寄存器 */
static int16_t  g_win[2][REG_WIN_SAMPLES * REG_WIN_AXES];              /* D 区双缓冲（各 1200 字节） */

static volatile uint8_t  g_widx;      /* 写缓冲下标 */
static volatile uint8_t  g_ridx;      /* 读（有效）缓冲下标 */
static volatile uint16_t g_cnt;       /* 写缓冲已填样本数 */
static volatile uint16_t g_seq;       /* 窗口序号 */
static volatile bool     g_ready;     /* 是否已有完整窗口 */
static volatile bool     g_lock;      /* 窗口锁定（禁止切换缓冲） */
static volatile bool     g_overrun;   /* 锁定期间丢过样本 */
static bool              g_lock_served_data; /* 锁定期内是否已服务过 D 区数据读 */

static uint32_t g_now_ms;
static uint32_t g_lock_tick;
static uint32_t g_last_master_tick;
static bool     g_seen_master;
static bool     g_wd_prev;
static uint16_t g_slave_addr = DEF_SLAVE_ADDR;

static const mb_area_t k_areas[] = {
    { 0x0000u, 0x0010u, false },   /* A 环境量      0x0000~0x000F */
    { 0x0010u, 0x0010u, false },   /* B 状态统计    0x0010~0x001F */
    { 0x0020u, 0x0010u, true  },   /* C 控制配置    0x0020~0x002F（唯一可写区） */
    { 0x0100u, 0x025Cu, false },   /* D 采样窗口    0x0100~0x035B（604） */
    { 0x0400u, 0x0024u, false },   /* E 特征向量    0x0400~0x0423（36） */
};

/* ---- 内部工具 ---------------------------------------------------------- */
static void load_defaults(void)
{
    g_regs[REG_B_VERSION]      = 0x0100u;   /* v1.0 */
    g_regs[REG_C_FAN_PWM]      = 0u;
    g_regs[REG_C_MOTOR_PWM]    = 0u;
    g_regs[REG_C_SERVO_ANGLE]  = 0xFFFFu;   /* 保持 */
    g_regs[REG_C_RGB_R]        = 0u;
    g_regs[REG_C_RGB_G]        = 0u;
    g_regs[REG_C_RGB_B]        = 0u;
    g_regs[REG_C_TH_TEMP_MAX]  = DEF_TH_TEMP_MAX;
    g_regs[REG_C_TH_SMOKE_MAX] = DEF_TH_SMOKE_MAX;
    g_regs[REG_C_TH_HR_MIN]    = DEF_TH_HR_MIN;
    g_regs[REG_C_WD_TIMEOUT]   = DEF_WD_TIMEOUT;
    g_regs[REG_C_SAFE_STATE]   = DEF_SAFE_STATE;
    g_regs[REG_C_SLAVE_ADDR]   = DEF_SLAVE_ADDR;
    g_regs[REG_C_BAUD_CODE]    = DEF_BAUD_CODE;
    g_regs[REG_C_ACQ_PERIOD]   = DEF_ACQ_PERIOD_MS;
    g_regs[REG_C_CTRL]         = CTRL_BIT_LOCAL_AUTO;   /* 默认允许本地自治 */
    g_regs[REG_D_RATE]         = DEF_SAMPLE_RATE_X10;
    g_slave_addr               = DEF_SLAVE_ADDR;
}

static void win_unlock(void)
{
    g_lock = false;
    g_lock_served_data = false;
}

static bool addr_is_c(uint16_t addr)
{
    return (addr >= REG_C_CTRL) && (addr <= REG_C_COMMAND);
}

/** C 区各寄存器的取值范围校验 */
static mb_exc_t validate_c(uint16_t addr, uint16_t v)
{
    switch (addr) {
    case REG_C_FAN_PWM:     return (v <= 1000u) ? MB_EXC_NONE : MB_EXC_VALUE;
    case REG_C_SERVO_ANGLE: return ((v <= 180u) || (v == 0xFFFFu)) ? MB_EXC_NONE : MB_EXC_VALUE;
    case REG_C_RGB_R:
    case REG_C_RGB_G:
    case REG_C_RGB_B:       return (v <= 255u) ? MB_EXC_NONE : MB_EXC_VALUE;
    case REG_C_SLAVE_ADDR:  return ((v >= MB_ADDR_MIN) && (v <= MB_ADDR_MAX)) ? MB_EXC_NONE : MB_EXC_VALUE;
    case REG_C_BAUD_CODE:   return (v <= 4u) ? MB_EXC_NONE : MB_EXC_VALUE;
    case REG_C_ACQ_PERIOD:  return ((v >= 10u) && (v <= 60000u)) ? MB_EXC_NONE : MB_EXC_VALUE;
    case REG_C_WD_TIMEOUT:  return (v <= 60000u) ? MB_EXC_NONE : MB_EXC_VALUE;
    case REG_C_COMMAND:     return ((v == 0xA5A5u) || (v == 0x5A5Au) || (v == 0u)) ? MB_EXC_NONE : MB_EXC_VALUE;
    default:                return MB_EXC_NONE;
    }
}

/* ---- 生命周期 ---------------------------------------------------------- */
void regbank_init(void)
{
    memset(g_regs, 0, sizeof(g_regs));
    memset(g_win, 0, sizeof(g_win));

    g_widx = 0u;
    g_ridx = 1u;          /* 写缓冲 0，读缓冲 1（初始无有效数据） */
    g_cnt = 0u;
    g_seq = 0u;
    g_ready = false;
    g_lock = false;
    g_overrun = false;
    g_lock_served_data = false;
    g_now_ms = 0u;
    g_lock_tick = 0u;
    g_last_master_tick = 0u;
    g_seen_master = false;
    g_wd_prev = false;

    load_defaults();
}

/* ---- 协议栈钩子 -------------------------------------------------------- */
uint16_t regbank_get(uint16_t addr)
{
    /* D 区头部三个动态寄存器 */
    if ((addr >= REG_D_HEADER) && (addr <= REG_D_TIMESTAMP)) {
        switch (addr) {
        case REG_D_HEADER:
            return (uint16_t)((g_ready ? 0x8000u : 0u) | (g_ready ? REG_WIN_SAMPLES : 0u));
        case REG_D_SEQ: {
            uint16_t seq = g_seq;
            /* 只有在"锁定期内已经服务过 D 区数据读"之后读序号，才认为是主站的收尾校验 */
            if (g_lock && g_lock_served_data) {
                win_unlock();
            }
            return seq;
        }
        case REG_D_RATE:
            return g_regs[REG_D_RATE];
        case REG_D_TIMESTAMP:
        default:
            return (uint16_t)((g_now_ms / 10u) & 0xFFFFu);
        }
    }

    /* D 区数据：从"读缓冲"取值 */
    if ((addr >= REG_D_DATA) && (addr <= REG_D_DATA_END)) {
        uint16_t idx    = (uint16_t)(addr - REG_D_DATA);
        uint16_t sample = (uint16_t)(idx / REG_WIN_AXES);
        uint16_t axis   = (uint16_t)(idx % REG_WIN_AXES);
        if (sample >= REG_WIN_SAMPLES) {
            return 0u;
        }
        return (uint16_t)g_win[g_ridx][sample * REG_WIN_AXES + axis];
    }

    if (addr >= REGBANK_SIZE) {
        return 0u;
    }
    return g_regs[addr];
}

mb_exc_t regbank_check(uint16_t addr, uint16_t value)
{
    if (!addr_is_c(addr)) {
        return MB_EXC_ADDR;
    }
    return validate_c(addr, value);
}

mb_exc_t regbank_set(uint16_t addr, uint16_t value)
{
    mb_exc_t e = regbank_check(addr, value);

    if (e != MB_EXC_NONE) {
        return e;
    }

    REGBANK_ENTER_CRITICAL();
    g_regs[addr] = value;
    REGBANK_EXIT_CRITICAL();

    /* 自清位与命令寄存器的副作用 */
    if (addr == REG_C_CTRL) {
        if ((value & CTRL_BIT_CLEAR_STAT) != 0u) {
            mb_slave_clear_stats();
            g_regs[REG_C_CTRL] &= (uint16_t)~CTRL_BIT_CLEAR_STAT;
        }
        if ((value & CTRL_BIT_SAVE_CFG) != 0u) {
            /* TODO(应用层): 把 C 区配置写入板载 W25Q128，见 CUBEMX-SETUP.md */
            g_regs[REG_C_CTRL] &= (uint16_t)~CTRL_BIT_SAVE_CFG;
        }
        if ((value & CTRL_BIT_WIN_RECAP) != 0u) {
            REGBANK_ENTER_CRITICAL();
            g_cnt = 0u;              /* 丢弃当前写缓冲，重新采一窗 */
            REGBANK_EXIT_CRITICAL();
            g_regs[REG_C_CTRL] &= (uint16_t)~CTRL_BIT_WIN_RECAP;
        }
    } else if (addr == REG_C_SLAVE_ADDR) {
        g_slave_addr = value;
    } else if (addr == REG_C_COMMAND) {
        if (value == 0xA5A5u) {
            load_defaults();         /* 恢复出厂设置 */
        }
        /* 0x5A5A 软重启由应用层在 task_health 里检测并执行 NVIC_SystemReset() */
    }
    return MB_EXC_NONE;
}

void regbank_on_block_read(uint16_t start, uint16_t count)
{
    /* 只有真正触及 D 区**数据**才需要锁；只读头部不锁 */
    bool touches_data = (start <= REG_D_DATA_END) &&
                        (((uint32_t)start + (uint32_t)count) > (uint32_t)REG_D_DATA);

    if (!touches_data) {
        /* 只读 0x0100~0x0103 时不锁，但仍标记"头部已被读" */
        return;
    }

    REGBANK_ENTER_CRITICAL();
    if (!g_lock) {
        g_lock = true;
        g_lock_tick = g_now_ms;
        g_overrun = false;
    }
    g_lock_served_data = true;
    REGBANK_EXIT_CRITICAL();
}

const mb_area_t *regbank_areas(uint8_t *count_out)
{
    if (count_out != 0) {
        *count_out = (uint8_t)(sizeof(k_areas) / sizeof(k_areas[0]));
    }
    return k_areas;
}

uint16_t regbank_get_slave_addr(void)
{
    return g_slave_addr;
}

/* ---- 采集任务写入接口 -------------------------------------------------- */
void regbank_write_internal(uint16_t addr, uint16_t value)
{
    if (addr >= REGBANK_SIZE) {
        return;
    }
    REGBANK_ENTER_CRITICAL();
    g_regs[addr] = value;
    REGBANK_EXIT_CRITICAL();
}

void regbank_write_i16(uint16_t addr, int16_t value)
{
    regbank_write_internal(addr, (uint16_t)value);
}

void regbank_push_sample(const int16_t six[REG_WIN_AXES])
{
    uint8_t a;

    if (six == 0) {
        return;
    }

    REGBANK_ENTER_CRITICAL();

    /* 步骤 1：写缓冲已满 → 先提交（或锁定期间丢弃本样本） */
    if (g_cnt >= REG_WIN_SAMPLES) {
        if (!g_lock) {
            /* 提交：写缓冲变成读缓冲，另一块转为新的写缓冲（零拷贝） */
            g_ridx  = g_widx;
            g_widx  = (uint8_t)(g_widx ^ 1u);
            g_cnt   = 0u;
            g_seq++;
            g_ready = true;
            g_overrun = false;
        } else {
            /* 被主站锁定：丢弃本样本，绝不破坏正在被读的数据 */
            g_overrun = true;
            REGBANK_EXIT_CRITICAL();
            return;
        }
    }

    /* 步骤 2：写入本样本 */
    for (a = 0u; a < REG_WIN_AXES; a++) {
        g_win[g_widx][(uint16_t)g_cnt * REG_WIN_AXES + a] = six[a];
    }
    g_cnt++;

    /* 步骤 3：刚好写满 → 立即提交，使"窗口就绪"及时置位 */
    if ((g_cnt >= REG_WIN_SAMPLES) && (!g_lock)) {
        g_ridx  = g_widx;
        g_widx  = (uint8_t)(g_widx ^ 1u);
        g_cnt   = 0u;
        g_seq++;
        g_ready = true;
        g_overrun = false;
    }
    /* 若此时被锁定：本缓冲是写缓冲，保留不动，等解锁后的第一次 push 再提交 */

    REGBANK_EXIT_CRITICAL();
}

bool regbank_window_ready(void)  { return g_ready; }
uint16_t regbank_window_seq(void){ return g_seq; }
bool regbank_window_locked(void) { return g_lock; }
bool regbank_window_overrun(void){ return g_overrun; }

/* ---- 周期任务 ---------------------------------------------------------- */
void regbank_tick(uint32_t now_ms)
{
    bool expired;
    uint16_t st2;

    g_now_ms = now_ms;

    /* 1) 窗口锁超时兜底：主站读一半崩了不能永远锁着 */
    if (g_lock && ((uint32_t)(now_ms - g_lock_tick) > REG_WIN_LOCK_MS)) {
        win_unlock();
    }

    /* 2) 通信看门狗（"从没连上过主机"不报警，避免单板调试时乱叫） */
    expired = regbank_comm_watchdog_expired();
    if (expired && !g_wd_prev) {
        g_regs[REG_B_WD_TIMEOUTS]++;
    }
    g_wd_prev = expired;

    st2 = g_regs[REG_B_STATUS2];
    if (expired) {
        st2 |= (uint16_t)(ST2_COMM_WD | ST2_LOCAL_AUTO);
        g_regs[REG_B_MODE] = 0u;                 /* 本地自治 */
    } else {
        st2 &= (uint16_t)~(uint16_t)(ST2_COMM_WD | ST2_LOCAL_AUTO);
        g_regs[REG_B_MODE] = (g_seen_master ? 1u : 0u);
    }
    if (g_overrun) {
        st2 |= (uint16_t)ST2_WIN_OVERRUN;
    } else {
        st2 &= (uint16_t)~(uint16_t)ST2_WIN_OVERRUN;
    }
    g_regs[REG_B_STATUS2] = st2;

    /* 3) 运行时间（分钟） */
    g_regs[REG_B_UPTIME_MIN] = (uint16_t)((now_ms / 60000u) & 0xFFFFu);

    /* 4) 窗口状态镜像到 B 区，方便主站一帧拿到 */
    g_regs[REG_B_WIN_STATUS] = regbank_get(REG_D_HEADER);
}

void regbank_notify_master_request(uint32_t now_ms)
{
    g_last_master_tick = now_ms;
    g_seen_master = true;
    g_regs[REG_B_MODE] = 1u;
}

bool regbank_comm_watchdog_expired(void)
{
    uint32_t timeout;

    if (!g_seen_master) {
        return false;
    }
    timeout = (uint32_t)g_regs[REG_C_WD_TIMEOUT];
    if (timeout == 0u) {
        return false;                /* 0 = 禁用看门狗 */
    }
    return ((uint32_t)(g_now_ms - g_last_master_tick) > (timeout * 100u));
}

/* ---- 统计同步 ---------------------------------------------------------- */
void regbank_sync_stats(void)
{
    g_regs[REG_B_OK_CNT_LO] = (uint16_t)(g_mb_stats.ok_frames & 0xFFFFu);
    g_regs[REG_B_CRC_ERR]   = (uint16_t)(g_mb_stats.crc_errors & 0xFFFFu);
    g_regs[REG_B_EXC_CNT]   = (uint16_t)(g_mb_stats.exc_frames & 0xFFFFu);
}
