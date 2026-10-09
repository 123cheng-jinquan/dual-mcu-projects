/* =============================================================================
 *  modbus_master.cpp — Modbus RTU 主站实现（唯一总线持有者）
 * =============================================================================
 *  实现边界（哪些是"真代码"、哪些留 TODO）：
 *   [真] UART + DE 半双工收发、3.5T 帧静默、CRC16/MODBUS、FC03/04/06/10 组帧解析、
 *        异常码识别、超时/重试、统计计数、在线判定、
 *        慢车道 500ms / 快车道 2s（含 docs/02 §7 的序号一致性校验）、
 *        对着 docs/02 §3.1 与 §6 的逐字节自检。
 *   [TODO] v1.1 的 CAN 网关转发、多从机轮询（寄存器表已留从机地址，但 plan.md §10
 *          明确"先单从机做扎实"）。
 * =============================================================================
 */

#include "modbus_master.h"
#include "app_tasks.h"

#include <string.h>

/* 诊断：把"静默失败"和"收到垃圾字节"分开计数（见 mb_rx_frame 的注释）。
 *   silent  高 → 从机没上电 / 地址不对 / A-B 没接 / 它把我们静默丢弃了
 *   garbage 高 → 线路上有数据但不成帧 → 波特率不一致 / 干扰 / 半双工时序
 * 只有总线任务会改它们，读的时候容忍轻微撕裂（心跳日志用）。 */
volatile uint32_t s_diag_rx_silent  = 0;
volatile uint32_t s_diag_rx_garbage = 0;

/* 诊断：**字节级**对账（比"成功/失败次数"更硬的一条证据）。
 * 用途：把"信号在线路上丢了"和"收到了但解析对不上"分开 —— 这两种的修法天差地别。
 *   rx_bytes 远小于从机 tx_bytes  → 物理层丢字节（终端电阻/共地/线序/收发器）
 *   rx_bytes ≈ 从机 tx_bytes      → 字节都到了，问题在解析/时序/缓冲
 * 由总线任务在 mb_tx/mb_rx 里累加，心跳里打印差值。 */
volatile uint32_t s_diag_tx_bytes = 0;
volatile uint32_t s_diag_rx_bytes = 0;

/* -----------------------------------------------------------------------------
 * FC03/FC04 单帧寄存器数上限 125、FC10 上限 123（docs/02 §4）
 * -------------------------------------------------------------------------- */
#define MB_FC_READ_HOLDING   0x03
#define MB_FC_READ_INPUT     0x04
#define MB_FC_WRITE_SINGLE   0x06
#define MB_FC_WRITE_MULTI    0x10
#define MB_FC_EXCEPTION_MASK 0x80

#define MB_MAX_READ_REGS     125
#define MB_MAX_WRITE_REGS    123
#define MB_MAX_ADU           256

/* -----------------------------------------------------------------------------
 * 模块状态
 * -------------------------------------------------------------------------- */
QueueHandle_t g_mb_req_queue = NULL;

static HardwareSerial s_rs485(RS485_UART_NUM);
static bool           s_inited = false;

/* 收发缓冲做成 static：
 *   · 只有 task_modbus_bus 会碰它们（唯一总线持有者），天然无竞争；
 *   · 256 + 256 字节如果放栈上，TASK_STACK_BUS(4096) 一下子去掉 1/8。
 * 这是"用静态换栈"的典型取舍，前提就是前面那条硬约束成立。 */
static uint8_t s_tx_buf[MB_MAX_ADU];
static uint8_t s_rx_buf[MB_MAX_ADU];

static mb_stats_t       s_stats;
static portMUX_TYPE     s_stats_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool    s_online   = false;

/* =============================================================================
 * §1  CRC16 / MODBUS（docs/02 §3：多项式 0xA001，反射；发送先低字节后高字节）
 * ============================================================================= */
uint16_t mb_crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= *buf++;
        for (uint8_t i = 0; i < 8; i++) {
            crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
        }
    }
    return crc;
}

/* =============================================================================
 * §2  组帧
 * ============================================================================= */

static uint16_t put_crc(uint8_t *p, uint16_t body_len)
{
    /* body_len = 不含 CRC 的字节数；CRC 先低字节后高字节 */
    const uint16_t crc = mb_crc16(p, body_len);
    p[body_len]     = (uint8_t)(crc & 0xFF);
    p[body_len + 1] = (uint8_t)(crc >> 8);
    return (uint16_t)(body_len + 2);
}

/* 读请求： 地址 | 功能码 | 起始Hi | 起始Lo | 数量Hi | 数量Lo | CRC(2) = 8 字节 */
static uint16_t build_read_req(uint8_t *out, uint8_t func, uint16_t start, uint16_t count)
{
    out[0] = MODBUS_SLAVE_ADDR;
    out[1] = func;
    out[2] = (uint8_t)(start >> 8);
    out[3] = (uint8_t)(start & 0xFF);
    out[4] = (uint8_t)(count >> 8);
    out[5] = (uint8_t)(count & 0xFF);
    return put_crc(out, 6);
}

/* FC06：地址 | 06 | 寄存器Hi | Lo | 值Hi | Lo | CRC(2) = 8 字节 */
static uint16_t build_write_single_req(uint8_t *out, uint16_t reg, uint16_t value)
{
    out[0] = MODBUS_SLAVE_ADDR;
    out[1] = MB_FC_WRITE_SINGLE;
    out[2] = (uint8_t)(reg >> 8);
    out[3] = (uint8_t)(reg & 0xFF);
    out[4] = (uint8_t)(value >> 8);
    out[5] = (uint8_t)(value & 0xFF);
    return put_crc(out, 6);
}

/* FC10：地址 | 10 | 起始Hi | Lo | 数量Hi | Lo | 字节数 | data... | CRC(2) */
static uint16_t build_write_multi_req(uint8_t *out, uint16_t start, uint16_t count,
                                      const uint16_t *values)
{
    out[0] = MODBUS_SLAVE_ADDR;
    out[1] = MB_FC_WRITE_MULTI;
    out[2] = (uint8_t)(start >> 8);
    out[3] = (uint8_t)(start & 0xFF);
    out[4] = (uint8_t)(count >> 8);
    out[5] = (uint8_t)(count & 0xFF);
    out[6] = (uint8_t)(count * 2);
    for (uint16_t i = 0; i < count; i++) {
        out[7 + i * 2] = (uint8_t)(values[i] >> 8);
        out[8 + i * 2] = (uint8_t)(values[i] & 0xFF);
    }
    return put_crc(out, (uint16_t)(7 + count * 2));
}

/* =============================================================================
 * §3  物理层收发（半双工 + 3.5T 静默）
 * ============================================================================= */

static inline void de_set_tx(void)
{
#if RS485_USE_DE_PIN
    digitalWrite(RS485_DE_PIN, RS485_DE_ACTIVE_HIGH ? HIGH : LOW);
#endif
}

static inline void de_set_rx(void)
{
#if RS485_USE_DE_PIN
    digitalWrite(RS485_DE_PIN, RS485_DE_ACTIVE_HIGH ? LOW : HIGH);
#endif
}

/* 丢干净接收 FIFO 里的残留（上一次的回环、噪声）。
 * docs/02 §8 的硬要求："发送期间必须丢弃/忽略收到的字节（半双工回环会收到自己发的数据）"。
 * 主机侧同样适用 —— 485 收发器在 DE 还没落下时会把 TX 回灌到 RO。 */
static void rx_drain(void)
{
    while (s_rs485.available() > 0) {
        (void)s_rs485.read();
    }
}

/* 两次发送之间保证 ≥3.5 字符的静默（docs/02 §1）。
 * 不等够，从机会把上一帧尾巴和这一帧头粘成一条，判成长度错帧直接丢。 */
static void bus_idle_delay(void)
{
    delayMicroseconds(MODBUS_T35_US(MODBUS_BAUD) + MODBUS_INTER_FRAME_GUARD_US);
}

/* 一个字符的线上时间（8N1 = 11 bit）：用于 DE 落下前的保护延时 */
static inline uint32_t mb_char_us(void)
{
    return (uint32_t)(11000000UL / MODBUS_BAUD);
}

static void mb_tx(const uint8_t *buf, size_t len)
{
    bus_idle_delay();       /* 1) 保证帧间静默 */
    rx_drain();             /* 2) 清掉总线上的残留/回环 */

    de_set_tx();            /* 3) 切到发送 */
    s_rs485.write(buf, len);
    (void)__atomic_fetch_add(&s_diag_tx_bytes, (uint32_t)len, __ATOMIC_RELAXED);
    /* flush() 会阻塞到 TX FIFO + 移位寄存器全部发完（uart_wait_tx_done）。
     * 不用阻塞版是**从机侧**的硬要求（docs/02 §8），主机侧这里是唯一持有者，
     * 阻塞在这里不会影响别人。 */
    s_rs485.flush();

    /* ★DE 落下前的保护延时（一字符时间）★ —— 这一条是实测踩出来的：
     * ESP32 的 uart_wait_tx_done 只保证"FIFO 空了"，**不保证最后一个字节
     * 已经移出移位寄存器**。如果 DE 在这一瞬间落下，收发器会把最后一个字节
     * （CRC 高字节）截掉半个位 → 从机判 CRC 错 → **静默丢弃不回帧** →
     * 主机看到的就是"偶尔超时、偶尔正常"（实测成功率只有 ~27%，
     * 现象酷似波特率不一致或 A/B 干扰，非常难查）。
     * 补 11 bit 的时间（115200 → 96µs）就够，小于 3.5T 帧间隔，不会破坏时序。 */
    delayMicroseconds(mb_char_us());

    /* ★DE 切回接收之后**再清一次**残留★（2026-10）
     * 为什么要在 de_set_rx() 之后清：DE 电平翻转的瞬间，收发器会有一小段
     * 不确定状态，把总线上的电平抖成几个假字节灌进 UART 的 RX FIFO。
     * 这些字节如果留在 FIFO 里，下一次 mb_rx_frame() 的 `readBytes(buf,2)`
     * 会**立刻"成功"读到两个垃圾字节**，然后按功能码去解析一个根本不存在的帧
     * → 校验失败 → 上层看到的是**超时/静默**（因为有效响应其实还没来）。
     *
     * 现象特征（完全吻合本项目实测）：
     *   · 失败全部记为"静默"，但 `s_diag_rx_silent` 统计的是"2 字节都没读到"，
     *     垃圾字节在更早的 rx_drain() 里被清掉了，所以看起来像"线上没字节"
     *   · **重试必然成功**：因为重试时 FIFO 已经干净
     *   · 首试成功率 ~50%，且与信号质量无关（CRC 错仅 0.07%）
     *
     * ⚠ 只在**发送之后**清，不要在接收之后清（那会把刚到的响应丢掉）。 */
    de_set_rx();            /* 4) 切回接收，准备收响应 */
    rx_drain();             /* 5) 清掉 DE 翻转产生的假字节 */
}

/* 收一帧。返回实际长度，0 = 超时/不完整。
 * 按功能码预判长度，避免"等到超时才认为收完" —— 那是 60ms 的浪费。
 * 异常响应：地址 | 功能码|0x80 | 异常码 | CRC(2)          → 5 字节
 * FC03/04 ：地址 | 功能码 | 字节数 | data... | CRC(2)      → 5 + 字节数
 * FC06/10 ：地址 | 功能码 | 起始(2) | 数量(2) | CRC(2)      → 8 字节（原样回显） */
static size_t mb_rx_frame(uint8_t *buf, size_t cap, uint32_t timeout_ms, size_t *got_out)
{
    if (got_out != NULL) {
        *got_out = 0;
    }
    s_rs485.setTimeout(timeout_ms);

    if (s_rs485.readBytes(buf, 2) != 2) {
        /* 一个字节都没来 = 超时。**这是"静默"失败**，和"收到了垃圾字节"要分开：
         *   silent  ：从机没上电/地址不对/没接 A-B/它把我们静默丢弃了
         *   got>0   ：线路上有东西，但帧不成形 → 波特率/干扰/半双工时序
         * 这两种的处置完全不同，所以把"收到几个字节"也报出去。 */
        if (got_out != NULL) {
            *got_out = (size_t)s_rs485.available();
        }
        return 0;
    }
    size_t n = 2;
    const uint8_t func = buf[1];

    if (func & MB_FC_EXCEPTION_MASK) {
        if (cap < 5) return 0;
        if (s_rs485.readBytes(buf + n, 3) != 3) return 0;
        n += 3;
    } else if (func == MB_FC_READ_HOLDING || func == MB_FC_READ_INPUT) {
        if (s_rs485.readBytes(buf + n, 1) != 1) return 0;   /* 字节数 */
        n += 1;
        const size_t byte_count = buf[2];
        if (n + byte_count + 2 > cap) return 0;             /* 超缓冲 = 坏帧，直接丢 */
        if (s_rs485.readBytes(buf + n, byte_count + 2) != byte_count + 2) return 0;
        n += byte_count + 2;
    } else if (func == MB_FC_WRITE_SINGLE || func == MB_FC_WRITE_MULTI) {
        if (cap < 8) return 0;
        if (s_rs485.readBytes(buf + n, 6) != 6) return 0;
        n += 6;
    } else {
        /* 主机不会主动发这些功能码，收到就是总线上的别人/噪声 */
        return 0;
    }

    /* ★事务后保护间隔★（见 config.h 的 MODBUS_TXN_GUARD_US）
     * 帧已经收全了，但**从机那边可能还没彻底松手**：它刚把 61~260 字节发完，
     * 收发器的驱动器还在释放过程中。此时主机立刻发下一个请求，会被顶在总线上
     * 弄坏 → 从机静默丢弃 → 主机超时、重试才成功。
     * 实测证据：`TX 55 RX 35 超时 20 重试 20`（重试数 == 超时数）。
     * 放在"收到完整帧之后"而不是"每次发送之前"，是因为只在**真正发生过一次
     * 收发**之后才需要等；空闲轮询之间不需要额外加这个 100ms。 */
    delayMicroseconds(MODBUS_TXN_GUARD_US);
    return n;
}

/* =============================================================================
 * §4  一次事务（发 + 收 + 校验），不含重试
 * ============================================================================= */

static void stats_on_ok(void)
{
    portENTER_CRITICAL(&s_stats_mux);
    s_stats.rx_count++;
    s_stats.consecutive_timeout = 0;
    s_stats.last_ok_tick = xTaskGetTickCount();
    s_stats.last_rx_ms   = millis();
    s_online = true;
    portEXIT_CRITICAL(&s_stats_mux);
}

static void stats_on_fail(mb_status_t st)
{
    portENTER_CRITICAL(&s_stats_mux);
    switch (st) {
        case MB_ERR_TIMEOUT:
            s_stats.timeout_count++;
            s_stats.consecutive_timeout++;
            if (s_stats.consecutive_timeout >= MODBUS_OFFLINE_TIMEOUTS) {
                s_online = false;   /* docs/02 §9：连续 3 次超时 → 节点离线 */
            }
            break;
        case MB_ERR_CRC:
            s_stats.crc_err_count++;
            break;
        case MB_ERR_EXCEPTION:
            s_stats.exception_count++;
            break;
        default:
            break;
    }
    portEXIT_CRITICAL(&s_stats_mux);
}

#if USE_SIM_SLAVE
static void dump_frame(const char *tag, const uint8_t *buf, size_t len)
{
    char line[3 * MB_MAX_ADU + 16];
    size_t off = 0;
    for (size_t i = 0; i < len && off + 4 < sizeof(line); i++) {
        off += (size_t)snprintf(line + off, sizeof(line) - off, "%02X ", buf[i]);
    }
    APP_LOGI("[%s] %s", tag, line);
}
#define MB_DUMP(tag, buf, len) dump_frame((tag), (buf), (len))
#else
#define MB_DUMP(tag, buf, len) do { (void)(buf); (void)(len); } while (0)
#endif

static mb_status_t mb_execute_once(const modbus_request_t *r)
{
    uint16_t req_len = 0;

    switch (r->kind) {
        case MB_REQ_READ_HOLDING:
            req_len = build_read_req(s_tx_buf, MB_FC_READ_HOLDING, r->start, r->count);
            break;
        case MB_REQ_READ_INPUT:
            req_len = build_read_req(s_tx_buf, MB_FC_READ_INPUT, r->start, r->count);
            break;
        case MB_REQ_WRITE_SINGLE:
            req_len = build_write_single_req(s_tx_buf, r->start, r->wdata[0]);
            break;
        case MB_REQ_WRITE_MULTI:
            req_len = build_write_multi_req(s_tx_buf, r->start, r->count, r->wdata);
            break;
        default:
            return MB_ERR_ARG;
    }

    portENTER_CRITICAL(&s_stats_mux);
    s_stats.tx_count++;
    s_stats.last_request_kind = (uint8_t)r->kind;
    portEXIT_CRITICAL(&s_stats_mux);

    MB_DUMP("TX", s_tx_buf, req_len);
    mb_tx(s_tx_buf, req_len);

    size_t got = 0;
    const size_t rx_len = mb_rx_frame(s_rx_buf, sizeof(s_rx_buf), r->timeout_ms, &got);
    if (rx_len > 0) {
        (void)__atomic_fetch_add(&s_diag_rx_bytes, (uint32_t)rx_len, __ATOMIC_RELAXED);
    } else if (got > 0) {
        (void)__atomic_fetch_add(&s_diag_rx_bytes, (uint32_t)got, __ATOMIC_RELAXED);
    }
    if (rx_len == 0) {
        if (got == 0) {
            s_diag_rx_silent++;     /* 线路上一片安静：从机没应答 */
        } else {
            s_diag_rx_garbage++;    /* 有字节但凑不成帧：波特率/干扰/半双工时序 */
        }
        return MB_ERR_TIMEOUT;
    }
    MB_DUMP("RX", s_rx_buf, rx_len);
    if (rx_len < 4) {
        return MB_ERR_BAD_FRAME;
    }
    /* docs/02 §3 的自检技巧：整帧（含 CRC）送进 crc16 结果应为 0，
     * 比"重算 CRC 再逐字节比"省一半时间。 */
    if (mb_crc16(s_rx_buf, (uint16_t)rx_len) != 0) {
        return MB_ERR_CRC;
    }
    if (s_rx_buf[1] & MB_FC_EXCEPTION_MASK) {
        portENTER_CRITICAL(&s_stats_mux);
        s_stats.last_exception_code = s_rx_buf[2];
        portEXIT_CRITICAL(&s_stats_mux);
        return MB_ERR_EXCEPTION;
    }
    /* 地址与功能码必须对得上（总线上可能有别的从机/别人的回声） */
    if (s_rx_buf[0] != MODBUS_SLAVE_ADDR || s_rx_buf[1] != s_tx_buf[1]) {
        return MB_ERR_BAD_FRAME;
    }

    /* 按功能码把数据取出来 */
    switch (r->kind) {
        case MB_REQ_READ_HOLDING:
        case MB_REQ_READ_INPUT: {
            const uint8_t byte_count = s_rx_buf[2];
            if (byte_count != r->count * 2) {
                return MB_ERR_BAD_FRAME;    /* 从机回的个数和我们请求的不一致 */
            }
            if (r->buf == NULL) {
                return MB_ERR_ARG;
            }
            /* Modbus 寄存器本身就是大端（docs/02 §5-D 的 NOTE），手工拼 */
            for (uint16_t i = 0; i < r->count; i++) {
                r->buf[i] = (uint16_t)((uint16_t)s_rx_buf[3 + i * 2] << 8 |
                                        (uint16_t)s_rx_buf[4 + i * 2]);
            }
            break;
        }
        case MB_REQ_WRITE_SINGLE:
        case MB_REQ_WRITE_MULTI:
            /* FC06/FC10 的正常响应 = 原样回显前 6 字节（docs/02 §6 ④），
             * 不比对的话"写了但没写进去"这种情况查不出来。 */
            if (memcmp(s_rx_buf, s_tx_buf, 6) != 0) {
                return MB_ERR_BAD_FRAME;
            }
            break;
        default:
            break;
    }
    return MB_OK;
}

/* 带重试的事务 */
static mb_status_t mb_execute(const modbus_request_t *r)
{
    const uint8_t attempts = (uint8_t)(r->retries + 1);
    mb_status_t st = MB_ERR_TIMEOUT;

    for (uint8_t attempt = 0; attempt < attempts; attempt++) {
        if (attempt > 0) {
            portENTER_CRITICAL(&s_stats_mux);
            s_stats.retry_count++;
            portEXIT_CRITICAL(&s_stats_mux);
        }
        st = mb_execute_once(r);
        if (st == MB_OK) {
            stats_on_ok();
            return MB_OK;
        }
        /* 从机明确回了异常帧：这是"请求本身不合法"（地址/数量/值），
         * 重试一百次也是同样的异常，立刻放弃，别浪费总线（docs/02 §4）。 */
        if (st == MB_ERR_EXCEPTION) {
            stats_on_fail(st);
            return st;
        }
        stats_on_fail(st);
    }
    return st;
}

/* =============================================================================
 * §5  初始化
 * ============================================================================= */

void mb_init(void)
{
    if (s_inited) {
        return;
    }

#if RS485_USE_DE_PIN
    pinMode(RS485_DE_PIN, OUTPUT);
    de_set_rx();            /* 上电默认接收态（docs/01 §6 检查项） */
#endif

    s_rs485.begin(MODBUS_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
    s_rs485.setTimeout(MODBUS_TIMEOUT_MS_SLOW);
    rx_drain();

    if (g_mb_req_queue == NULL) {
        g_mb_req_queue = xQueueCreate(MODBUS_REQ_QUEUE_LEN, sizeof(modbus_request_t));
    }

    memset(&s_stats, 0, sizeof(s_stats));
    s_online = false;
    s_inited = (g_mb_req_queue != NULL);

    /* 帧格式自检：docs/02 §3.1 的 CRC 向量 + §6 的全部示例帧。
     * 放在启动时跑，是为了把"CRC 高低字节写反""起始地址算错"这类错误
     * 在第一次上电就暴露，而不是等到联调时数波形。 */
    if (!mb_selftest()) {
        APP_LOGE("Modbus 自检失败：帧构造与 docs/02 §3.1/§6 不一致，先修这个再联机");
    } else {
        APP_LOGI("Modbus 自检通过（CRC 向量 + §6 示例帧逐字节一致）");
    }
    /* DE 的描述在预处理期拼好（不能把 #if 塞进 log_i 的参数表里，那是 UB） */
#if RS485_USE_DE_PIN
    const char *de_desc = "GPIO" STR(RS485_DE_PIN);
#else
    const char *de_desc = "auto-direction(no DE pin)";
#endif
    APP_LOGI("RS485: UART%d baud=%d TX=%d RX=%d DE=%s",
          RS485_UART_NUM, MODBUS_BAUD, RS485_TX_PIN, RS485_RX_PIN, de_desc);
}

/* =============================================================================
 * §6  同步 / 异步 API
 * ============================================================================= */

static mb_status_t mb_call(modbus_request_t *r)
{
    if (!s_inited) {
        return MB_ERR_NOT_INIT;
    }
    mb_status_t st = MB_ERR_TIMEOUT;
    r->waiter     = xTaskGetCurrentTaskHandle();
    r->status_out = &st;

    /* 清掉可能残留在本任务通知里的旧值（上一次调用超时后持有者任务才回通知
     * 的极端情况），否则这一次会立刻"假成功"返回。 */
    (void)ulTaskNotifyTake(pdTRUE, 0);

    /* ★等待预算★（这里曾经算错，是"总线看起来没在轮询 / 队列像没人处理"的根因之一）
     * ---------------------------------------------------------------------
     * 总线是**串行**的：持有者任务一次只处理一个请求，其余在队列里排队。
     * 所以一个请求的完成时间 ≈ （排在它前面的请求数 + 1） × 单请求最坏耗时。
     * 旧代码只算了自己那一份：
     *       budget = timeout_ms × (retries+1) + 30ms   →  慢车道只有 180ms
     * 而实测单个请求（3 次尝试全超时）要 153~183ms，再加上"排在快车道那 7 个
     * 请求后面"的排队时间，慢车道几乎每次都等不到通知 → 被误判成超时。
     * 现在改成用 MODBUS_MAX_SINGLE_REQ_MS（实测最坏值）乘排队深度，见 config.h。
     *
     * 队列深度只在入队瞬间取一次：这是保守估计 —— 之后队列只会更深，
     * 而"更深"意味着等得更久，所以用入队时的值 + 上限封顶是安全的方向。 */
    const uint32_t depth = (uint32_t)uxQueueMessagesWaiting(g_mb_req_queue);
    uint32_t budget = (depth + 1u) * (uint32_t)MODBUS_MAX_SINGLE_REQ_MS
                    + (uint32_t)MODBUS_NOTIFY_SLACK_MS;
    if (budget > (uint32_t)MODBUS_NOTIFY_BUDGET_MAX_MS) {
        budget = (uint32_t)MODBUS_NOTIFY_BUDGET_MAX_MS;
    }

    if (xQueueSend(g_mb_req_queue, r, pdMS_TO_TICKS(MODBUS_REQ_QUEUE_WAIT_MS)) != pdTRUE) {
        APP_LOGW("总线请求队列满：持有者任务可能被卡住（看总线心跳里『等待处理』是否一直大于 0）");
        return MB_ERR_QUEUE_FULL;
    }

    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(budget)) == 0) {
        /* 走到这里说明持有者任务在预算内**没有回通知** —— 这是"总线任务卡死"
         * 的判定点（正常超时是持有者按 timeout_ms 结束后回通知，不会走这一支）。
         * 必须出声，否则上层只能看到一个笼统的 TIMEOUT。 */
        APP_LOGW("等待总线完成通知超时（%u ms，入队时前面还有 %u 个请求）："
                 "持有者任务可能卡在 UART 上（kind=%d）",
                 (unsigned)budget, (unsigned)depth, (int)r->kind);
        return MB_ERR_TIMEOUT;
    }
    return st;
}

bool mb_enqueue(modbus_request_t *req, TickType_t wait_ticks)
{
    if (!s_inited || req == NULL || req->status_out == NULL) {
        return false;
    }
    req->waiter = NULL;             /* 异步：不通知任何人 */
    return xQueueSend(g_mb_req_queue, req, wait_ticks) == pdTRUE;
}

mb_status_t mb_read_holding(uint16_t start, uint16_t count, uint16_t *out_buf,
                            uint32_t timeout_ms, uint8_t retries)
{
    /* 越界在本地拦掉（docs/02 §4：数量 1~125），不要浪费一次总线往返 */
    if (out_buf == NULL || count == 0 || count > MB_MAX_READ_REGS) {
        return MB_ERR_ARG;
    }
    modbus_request_t r;
    memset(&r, 0, sizeof(r));
    r.kind       = MB_REQ_READ_HOLDING;
    r.start      = start;
    r.count      = count;
    r.buf        = out_buf;
    r.timeout_ms = timeout_ms;
    r.retries    = retries;
    return mb_call(&r);
}

mb_status_t mb_read_input(uint16_t start, uint16_t count, uint16_t *out_buf,
                          uint32_t timeout_ms, uint8_t retries)
{
    if (out_buf == NULL || count == 0 || count > MB_MAX_READ_REGS) {
        return MB_ERR_ARG;
    }
    modbus_request_t r;
    memset(&r, 0, sizeof(r));
    r.kind       = MB_REQ_READ_INPUT;
    r.start      = start;
    r.count      = count;
    r.buf        = out_buf;
    r.timeout_ms = timeout_ms;
    r.retries    = retries;
    return mb_call(&r);
}

mb_status_t mb_write_single(uint16_t reg, uint16_t value,
                            uint32_t timeout_ms, uint8_t retries)
{
    const uint16_t v = value;
    modbus_request_t r;
    memset(&r, 0, sizeof(r));
    r.kind       = MB_REQ_WRITE_SINGLE;
    r.start      = reg;
    r.count      = 1;
    r.wdata      = &v;
    r.timeout_ms = timeout_ms;
    r.retries    = retries;
    return mb_call(&r);
}

mb_status_t mb_write_multiple(uint16_t start, uint16_t count, const uint16_t *values,
                              uint32_t timeout_ms, uint8_t retries)
{
    /* docs/02 §4：FC10 数量 1~123 */
    if (values == NULL || count == 0 || count > MB_MAX_WRITE_REGS) {
        return MB_ERR_ARG;
    }
    modbus_request_t r;
    memset(&r, 0, sizeof(r));
    r.kind       = MB_REQ_WRITE_MULTI;
    r.start      = start;
    r.count      = count;
    r.wdata      = values;
    r.timeout_ms = timeout_ms;
    r.retries    = retries;
    return mb_call(&r);
}

/* =============================================================================
 * §7  状态查询
 * ============================================================================= */

bool mb_is_online(void)
{
    return s_online;
}

void mb_get_stats(mb_stats_t *out)
{
    if (out == NULL) {
        return;
    }
    portENTER_CRITICAL(&s_stats_mux);
    *out = s_stats;
    out->online = s_online;
    portEXIT_CRITICAL(&s_stats_mux);
}

/* =============================================================================
 * §8  自检：CRC 向量 + 组帧逐字节比对
 * ============================================================================= */

typedef struct {
    const uint8_t *bytes;
    uint8_t        len;
} crc_vec_t;

/* docs/02 §3.1 的 10 组向量。整帧（报文 + 线上序 CRC）喂进 crc16 应得 0x0000。 */
static const uint8_t kV1[]  = { 0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A };
static const uint8_t kV2[]  = { 0x01, 0x03, 0x00, 0x00, 0x00, 0x0E, 0xC4, 0x0E };
static const uint8_t kV3[]  = { 0x01, 0x03, 0x00, 0x00, 0x00, 0x1C, 0x44, 0x03 };
static const uint8_t kV4[]  = { 0x01, 0x03, 0x01, 0x04, 0x00, 0x64, 0x04, 0x1C };
static const uint8_t kV5[]  = { 0x01, 0x06, 0x00, 0x20, 0x00, 0x01, 0x49, 0xC0 };
static const uint8_t kV6[]  = { 0x01, 0x06, 0x00, 0x21, 0x03, 0xE8, 0xD9, 0x7E };
static const uint8_t kV7[]  = { 0x02, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x38 };
static const uint8_t kV8[]  = { 0x01, 0x03, 0x00, 0x4B, 0x02, 0x00, 0x34, 0xBC };
static const uint8_t kV9[]  = { 0x01, 0x83, 0x02, 0xC0, 0xF1 };
static const uint8_t kV10[] = { 0x01, 0x83, 0x03, 0x01, 0x31 };
static const crc_vec_t kCrcVectors[] = {
    { kV1,  sizeof(kV1)  }, { kV2,  sizeof(kV2)  }, { kV3,  sizeof(kV3)  },
    { kV4,  sizeof(kV4)  }, { kV5,  sizeof(kV5)  }, { kV6,  sizeof(kV6)  },
    { kV7,  sizeof(kV7)  }, { kV8,  sizeof(kV8)  }, { kV9,  sizeof(kV9)  },
    { kV10, sizeof(kV10) },
};

/* docs/02 §6 表② 的 5 个分块请求帧（含 CRC）。
 * ⚠️ 这些常量必须与文档一致，且 CRC 已用「整帧残差为 0」复核过。
 *    本项目真的踩过一次：§6 示例② 第 1 行曾误写 0x84 0x17（正解 0xC5 0xD6），
 *    照抄进这里会让 mb_selftest() 在开机时报错、mb_init() 直接失败。
 *    F411 侧的 test/test_modbus.c::test_doc_frames() 现在也复算这些帧，可提前拦住。 */
static const uint8_t kChunk1[] = { 0x01, 0x03, 0x01, 0x04, 0x00, 0x7D, 0xC5, 0xD6 };
static const uint8_t kChunk2[] = { 0x01, 0x03, 0x01, 0x81, 0x00, 0x7D, 0xD4, 0x3F };
static const uint8_t kChunk3[] = { 0x01, 0x03, 0x01, 0xFE, 0x00, 0x7D, 0xE5, 0xE7 };
static const uint8_t kChunk4[] = { 0x01, 0x03, 0x02, 0x7B, 0x00, 0x7D, 0xF4, 0x4A };
static const uint8_t kChunk5[] = { 0x01, 0x03, 0x02, 0xF8, 0x00, 0x64, 0xC4, 0x68 };
/* docs/02 §6 ① 慢车道一帧读 28 个 */
static const uint8_t kSlowReq[]  = { 0x01, 0x03, 0x00, 0x00, 0x00, 0x1C, 0x44, 0x03 };
/* docs/02 §6 ④⑤ 写单寄存器 */
static const uint8_t kBuzzerReq[] = { 0x01, 0x06, 0x00, 0x20, 0x00, 0x01, 0x49, 0xC0 };
static const uint8_t kFanReq[]    = { 0x01, 0x06, 0x00, 0x21, 0x03, 0xE8, 0xD9, 0x7E };
/* docs/02 §6 ⑥ FC10 连写 控制字 + 风扇PWM */
static const uint8_t kMultiReq[]  = { 0x01, 0x10, 0x00, 0x20, 0x00, 0x02, 0x04,
                                      0x00, 0x01, 0x03, 0xE8, 0xA0, 0xC9 };

bool mb_selftest(void)
{
    uint8_t buf[MB_MAX_ADU];
    bool ok = true;

    /* (1) CRC 向量 */
    for (size_t i = 0; i < sizeof(kCrcVectors) / sizeof(kCrcVectors[0]); i++) {
        if (mb_crc16(kCrcVectors[i].bytes, kCrcVectors[i].len) != 0x0000) {
            APP_LOGE("selftest: CRC 向量 #%u 失败", (unsigned)(i + 1));
            ok = false;
        }
    }

    /* (2) 慢车道帧 */
    if (build_read_req(buf, MB_FC_READ_HOLDING, MODBUS_SLOW_START, MODBUS_SLOW_REG_COUNT) != sizeof(kSlowReq) ||
        memcmp(buf, kSlowReq, sizeof(kSlowReq)) != 0) {
        APP_LOGE("selftest: 慢车道帧与 docs/02 §6① 不一致");
        ok = false;
    }

    /* (3) 快车道 5 个分块帧：起始地址必须由「基址 + 已有块长之和」推出，
     *     绝不允许在代码里手写 0x0181/0x01FE/... 这些魔数（写错就静默错位）。 */
    static const uint16_t kChunkLens[MODBUS_WIN_CHUNK_COUNT] = { 125, 125, 125, 125, 100 };
    static const uint8_t *const kChunkExpect[MODBUS_WIN_CHUNK_COUNT] = {
        kChunk1, kChunk2, kChunk3, kChunk4, kChunk5
    };
    uint16_t off = 0;
    for (int i = 0; i < MODBUS_WIN_CHUNK_COUNT; i++) {
        const uint16_t len = build_read_req(buf, MB_FC_READ_HOLDING,
                                            (uint16_t)(MODBUS_WIN_DATA_START + off),
                                            kChunkLens[i]);
        if (len != 8 || memcmp(buf, kChunkExpect[i], 8) != 0) {
            APP_LOGE("selftest: 分块 #%d 帧与 docs/02 §6② 不一致", i + 1);
            ok = false;
        }
        off = (uint16_t)(off + kChunkLens[i]);
    }
    if (off != MODBUS_WIN_DATA_REGS) {
        APP_LOGE("selftest: 分块总长 %u != 600（docs/02 §5-D）", (unsigned)off);
        ok = false;
    }

    /* (4) 写帧 */
    if (build_write_single_req(buf, REG_CTRL1, CTRL1_BEEP) != sizeof(kBuzzerReq) ||
        memcmp(buf, kBuzzerReq, sizeof(kBuzzerReq)) != 0) {
        APP_LOGE("selftest: FC06 蜂鸣器帧与 docs/02 §6④ 不一致");
        ok = false;
    }
    if (build_write_single_req(buf, REG_FAN_PWM, 1000) != sizeof(kFanReq) ||
        memcmp(buf, kFanReq, sizeof(kFanReq)) != 0) {
        APP_LOGE("selftest: FC06 风扇 PWM 帧与 docs/02 §6⑤ 不一致");
        ok = false;
    }
    {
        const uint16_t vals[2] = { CTRL1_BEEP, 1000 };
        if (build_write_multi_req(buf, REG_CTRL1, 2, vals) != sizeof(kMultiReq) ||
            memcmp(buf, kMultiReq, sizeof(kMultiReq)) != 0) {
            APP_LOGE("selftest: FC10 连写帧与 docs/02 §6⑥ 不一致");
            ok = false;
        }
    }
    return ok;
}

/* =============================================================================
 * §9  任务
 * ============================================================================= */

/* ---- 唯一总线持有者 -------------------------------------------------------- */

/* 总线结果的短名（打日志用）。没有它，日志里只有 "-> 1" 这种数字，排错时要回来翻枚举。 */
static const char *mb_status_name(mb_status_t st)
{
    switch (st) {
        case MB_OK:            return "OK";
        case MB_ERR_TIMEOUT:   return "TIMEOUT";
        case MB_ERR_CRC:       return "CRC_ERR";
        case MB_ERR_EXCEPTION: return "EXCEPTION";
        case MB_ERR_BAD_FRAME: return "BAD_FRAME";
        case MB_ERR_ARG:       return "ARG";
        case MB_ERR_QUEUE_FULL:return "QUEUE_FULL";
        case MB_ERR_NOT_INIT:  return "NOT_INIT";
        default:               return "?";
    }
}

/* ── 为什么要有周期"心跳"日志（这一条是踩坑换来的）────────────────────────
 * 症状：上电后除启动横幅外串口一行都没有，看起来像"总线任务根本没在轮询"。
 * 真相：轮询一直在跑（队列里每秒都有请求），但**成功路径一行日志都不打**，
 *       而"没有从机应答"时只会走 silent 分支（连 ui_post 都因为 GUI 未就绪被丢弃），
 *       于是整个系统在串口上表现为"死了"，实际 CPU 跑得好好的。
 * 结论：一个只有失败才出声的系统，等于没有观测能力。
 * 所以这里固定每 MB_HEARTBEAT_PERIOD_MS 打一行心跳：
 *   · tx 在涨        → 总线任务活着，在发帧（从机没接就是 timeout 在涨）
 *   · tx 不涨、q 排队 → 持有者任务被卡住了，去看它是不是阻塞在 UART 上
 *   · 队列空、tx 不涨 → 生产端（mb_slow/mb_fast）没进来，看它们的高水位日志
 * 这一行能一眼把"总线死 / 从机死 / 生产端死"三种情况分开。 */
#define MB_HEARTBEAT_PERIOD_MS  5000

void task_modbus_bus(void *arg)
{
    (void)arg;
    modbus_request_t req;
    app_wdt_register();
    APP_LOGI("task_modbus_bus 已启动：等待 g_mb_req_queue（本任务独占 485 与 DE）");

    uint32_t last_beat_ms = millis();
    bool     last_online  = false;

    for (;;) {
        app_wdt_feed();

        /* 用 500ms 超时而不是 portMAX_DELAY：看门狗登记之后，任务必须周期回来喂狗，
         * 而"总线空闲"和"任务卡死"从看门狗眼里看是一样的 —— 所以空闲时也要醒。
         * 500ms 的相对开销可以忽略（一次队列查询），但换来"卡死必复位"。 */
        if (xQueueReceive(g_mb_req_queue, &req, pdMS_TO_TICKS(500)) == pdTRUE) {
            const mb_status_t st = mb_execute(&req);

            /* 把结果写回调用方的变量，再唤醒它（顺序不能反：先写结果，后通知） */
            if (req.status_out != NULL) {
                *(req.status_out) = st;
            }
            if (req.waiter != NULL) {
                xTaskNotifyGive(req.waiter);
            }
            /* 可选的完成回调。放在通知之后，且必须是"短活"——
             * 现在还没有用到它的地方，留接口是为了让异步链式下发不用多起一个任务。 */
            if (req.on_done != NULL) {
                req.on_done(&req);
            }
        }

        /* ---- 在线状态翻转：打日志 + 通知 GUI 灰显/恢复 ----------------
         * 顺序很重要：**先打日志**。这样即使 GUI 那条链路（队列满/未初始化）
         * 出问题，串口上也一定能看到"节点掉线了"，不会出现"总线出了事、
         * 而唯一能看到这件事的地方是那块还没点亮的屏"。 */
        const bool online_now = mb_is_online();
        if (online_now != last_online) {
            last_online = online_now;
            if (online_now) {
                APP_LOGI("485 节点上线：从机 0x%02X 开始正常响应", MODBUS_SLAVE_ADDR);
            } else {
                APP_LOGW("485 节点离线：连续 %d 次超时（检查 A/B 是否接反、从机是否上电、"
                         "波特率是否一致）", MODBUS_OFFLINE_TIMEOUTS);
            }
            /* GUI 未启动/队列满时 ui_post 会返回 false —— 只提示一次，不刷屏 */
            ui_msg_t m;
            memset(&m, 0, sizeof(m));
            m.kind     = UI_MSG_ONLINE;
            m.u.online = online_now ? 1 : 0;
            static bool warned_ui_drop = false;
            if (!ui_post(&m, 0) && !warned_ui_drop) {
                warned_ui_drop = true;
                APP_LOGW("ui_queue 满：在线状态消息没能送到 GUI（界面未就绪时属正常）");
            }
        }

        /* ---- 周期心跳（见上面那段注释） ---- */
        const uint32_t now_ms = millis();
        if ((now_ms - last_beat_ms) >= MB_HEARTBEAT_PERIOD_MS) {
            last_beat_ms = now_ms;
            mb_stats_t st;
            mb_get_stats(&st);
            APP_LOGI("总线心跳：TX %u  RX %u  超时 %u  重试 %u  异常 %u  CRC错 %u  %s  "
                     "等待处理 %u",
                     (unsigned)st.tx_count, (unsigned)st.rx_count,
                     (unsigned)st.timeout_count, (unsigned)st.retry_count,
                     (unsigned)st.exception_count, (unsigned)st.crc_err_count,
                     st.online ? "在线" : "离线",
                     (unsigned)uxQueueMessagesWaiting(g_mb_req_queue));

            /* ---- 失败定性：静默 vs 垃圾字节（这一行的用途见 README §9.1）----
             * 从机没接/没上电 → silent 涨；波特率不一致/干扰 → garbage 涨。
             * 两者都是"读不到数据"，但修法完全相反，所以必须分开看。 */
            {
                const uint32_t sil = s_diag_rx_silent;
                const uint32_t gar = s_diag_rx_garbage;
                static uint32_t sil_shown = 0, gar_shown = 0;
                if (sil != sil_shown || gar != gar_shown) {
                    sil_shown = sil;
                    gar_shown = gar;
                    APP_LOGW("[诊断] 收帧失败定性：静默(线上没字节) %u 次 / 有字节但不成帧 %u 次%s",
                             (unsigned)sil, (unsigned)gar,
                             (gar > sil / 4u) ? "  ← 以垃圾字节为主：查波特率是否一致、有无干扰" : "");
                }
            }

            /* ---- 字节级对账：和 F411 的 [mb] tx_bytes 一比就知道是不是物理层丢字节 ----
             * 判读（两侧同时看）：
             *   本机 rx_bytes ≈ F411 的 tx_bytes   → 字节都到了，问题在解析/时序
             *   本机 rx_bytes << F411 的 tx_bytes  → **物理层丢字节**：
             *        查 120Ω 终端电阻 / A-B 是否双绞 / 两块板是否共地 / 线是否过长
             *   本机 tx_bytes >> F411 的 rx_bytes  → 主站这边发出去就没了（同一类原因） */
            {
                const uint32_t txb = s_diag_tx_bytes;
                const uint32_t rxb = s_diag_rx_bytes;
                static uint32_t txb_shown = 0;
                if (txb - txb_shown >= 2000u) {     /* 每累计 2KB 报一次，别刷屏 */
                    txb_shown = txb;
                    APP_LOGI("[诊断] 字节累计：已发 %u 字节 / 已收 %u 字节"
                             "（与 F411 的 [mb] tx=…字节 对账）", (unsigned)txb, (unsigned)rxb);
                }
            }
        }
    }
}

/* ---- 慢车道：500ms 一次读 A+B 区 ------------------------------------------ */
void task_modbus_slow(void *arg)
{
    (void)arg;
    static slow_regs_t snap;            /* static：60 字节不占任务栈 */
    static uint16_t    last_seq = 0xFFFF;
    TickType_t last_wake = xTaskGetTickCount();
    app_wdt_register();
    APP_LOGI("task_modbus_slow 已启动：每 %dms 读 0x%04X 起 %d 个寄存器（A+B 区）",
             MODBUS_SLOW_PERIOD_MS, MODBUS_SLOW_START, MODBUS_SLOW_REG_COUNT);

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MODBUS_SLOW_PERIOD_MS));
        app_wdt_feed();

        const mb_status_t st = mb_read_holding(MODBUS_SLOW_START, MODBUS_SLOW_REG_COUNT,
                                               snap.regs,
                                               MODBUS_TIMEOUT_MS_SLOW, MODBUS_RETRY_READ);
        if (st != MB_OK) {
            /* 失败不清空上次的快照：GUI 继续显示最后一次已知值 + 置灰，
             * 比显示一堆 0 更不容易误判（"传感器全 0"和"通信断了"是两回事）。 */
            /* 从机不在线时这条会一直失败，所以**节流**打印（每 10 次一次 ≈ 5s）。
             * 为什么不静默：静默会让串口看起来像"固件死了"，而真相只是"从机没接" ——
             * 这两种情况必须能从日志上分开。 */
            static uint16_t fail_n = 0;
            if ((++fail_n % 10u) == 1u) {
                APP_LOGW("慢车道读失败（第 %u 次）：%s。若持续如此，检查 485 接线 / 从机供电 / 波特率",
                         (unsigned)fail_n, mb_status_name(st));
            }
            continue;
        }
        /* 第一次读成功时打一条，作为"链路真的通了"的锚点 */
        {
            static bool ok_logged = false;
            if (!ok_logged) {
                ok_logged = true;
                APP_LOGI("慢车道首次读成功：0x%04X~0x%04X（链路已通）",
                         MODBUS_SLOW_START,
                         (unsigned)(MODBUS_SLOW_START + MODBUS_SLOW_REG_COUNT - 1));
            }
        }

        snap.tick = xTaskGetTickCount();

        /* A 区 0x000F = 采样序号，用来判"从机还在采新数据吗"。
         * 序号长时间不变说明 F411 的采集任务卡了（不是通信问题）—— 只记日志，
         * 真正的告警仲裁在 GUI/报警页（TODO: v0.4 做报警联动）。 */
        const uint16_t seq = slow_u16(&snap, REG_SAMPLE_SEQ);
        if (seq == last_seq) {
            APP_LOGW("慢车道：0x000F 采样序号未变化（F411 采集任务可能卡住）");
        }
        last_seq = seq;

        slow_publish(&snap);

        ui_msg_t m;
        memset(&m, 0, sizeof(m));
        m.kind   = UI_MSG_ENV;
        m.u.env  = snap;
        (void)ui_post(&m, 0);
    }
}

/* ---- 快车道：2s 一次按 docs/02 §7 的顺序拉窗口 + 特征 ---------------------- */

/* 读一个完整窗口。返回 true = 数据可信（序号没变过）。
 * 顺序严格按 docs/02 §7 末段：头 → 5 块 → 再读 0x0101 校验。
 * 为什么"再读 0x0101"是防跨窗口拼接的关键：
 *   F411 的锁定法（§7）会在收到第一个 D 区读请求时上锁，读到 0x0101 才解锁；
 *   但锁定法本身有 500ms 兜底超时，且如果主机上一次请求中途崩了/被重试打断，
 *   锁可能已经释放、窗口可能已经切换。只有"读前记 seq0、读后记 seq1、两者相等"
 *   才能在主机侧独立地证明这 6 帧来自同一个窗口、没有拼出缝合怪样本。 */
/* ---- 快车道窗口读的诊断（2026-10）--------------------------------------
 * 为什么需要：窗口读失败时**原本一声不响**，界面上只有"等待窗口..."，
 * 分不清是"从机没就绪"（正常，等它采满）还是"通信失败"（异常）。
 * 这两种处置完全不同，所以把最后一次失败的原因记下来。 */
enum win_fail_t {
    WIN_FAIL_NONE = 0,
    WIN_FAIL_HEAD_IO,     /* 读窗口头就失败 → 通信问题 */
    WIN_FAIL_NOT_READY,   /* 头读到了但 bit15=0 → 从机还没产出有效窗口（**不是错误**）*/
    WIN_FAIL_CHUNK_IO,    /* 分块读失败 */
    WIN_FAIL_SEQ_MISMATCH /* 序号校验不过（这 6 帧跨了窗口切换）*/
};
static volatile uint8_t  s_win_last_fail = WIN_FAIL_NONE;
static volatile uint32_t s_win_ok   = 0;
static volatile uint32_t s_win_fail = 0;
static volatile uint8_t  s_win_last_status = 0;   /* 最近一次 mb_read_holding 的状态码 */

static const char *window_fail_reason_text(void)
{
    switch (s_win_last_fail) {
        case WIN_FAIL_HEAD_IO:      return "读窗口头失败 → 通信问题（查总线）";
        case WIN_FAIL_NOT_READY:    return "从机还没产出有效窗口(bit15=0) → **正常**，等它采满";
        case WIN_FAIL_CHUNK_IO:     return "分块读失败 → 通信问题（查总线）";
        case WIN_FAIL_SEQ_MISMATCH: return "序号校验不过（跨窗口切换）→ 重读即可";
        default:                    return "未知";
    }
}

static bool window_read_once(acq_window_t *w, uint32_t *mismatch_count)
{
    static uint16_t head[MODBUS_WIN_HEAD_COUNT];
    static uint16_t chunk[MODBUS_WIN_CHUNK_MAX];
    static const uint16_t kChunkLens[MODBUS_WIN_CHUNK_COUNT] = { 125, 125, 125, 125, 100 };

    /* ① 窗口头 0x0100~0x0103 */
    app_wdt_feed();
    {
        const mb_status_t hst = mb_read_holding(MODBUS_WIN_HEAD_START, MODBUS_WIN_HEAD_COUNT,
                                                head, MODBUS_TIMEOUT_MS_FAST, MODBUS_RETRY_READ);
        if (hst != MB_OK) {
            s_win_last_fail = WIN_FAIL_HEAD_IO;
            /* ★把**具体状态码**记下来★ —— 只写"失败"是查不出原因的：
             *   TIMEOUT   → 从机没应答（通信）
             *   EXCEPTION → 从机回了异常帧（地址/数量不合法）→ 查从机寄存器映射
             *   BAD_FRAME → 收到了但不是我们要的（地址/功能码/长度不对）
             *   CRC       → 帧被干扰
             * 这四种的处置完全不同。 */
            s_win_last_status = (uint8_t)hst;
            return false;
        }
    }
    if ((head[0] & WINDOW_HEAD_VALID_BIT) == 0) {
        /* bit15 = 0：从机还没准备好有效窗口（刚上电/F411 正在重采）。
         * 这不是错误，下一周期再来。 */
        s_win_last_fail = WIN_FAIL_NOT_READY;    /* 从机没就绪 —— **不是错误** */
        return false;
    }
    w->win_head  = head[0];
    w->win_seq   = head[1];
    w->rate_x10  = head[2];
    w->ts_10ms   = head[3];

    /* ② 5 次分块读 600 个寄存器（125/125/125/125/100，docs/02 §6 表②） */
    uint16_t off = 0;
    for (int i = 0; i < MODBUS_WIN_CHUNK_COUNT; i++) {
        const uint16_t cnt = kChunkLens[i];
        /* ★每帧之间必须喂狗★
         * 一次窗口读是 7 个**串行**请求；慢波特率下每个请求最坏
         *   MODBUS_TIMEOUT_MS_FAST × (retries+1) = 500×3 = 1.5s，
         * 七个连起来 8.4s —— 远超 5s 看门狗 ✗（2026-10 在 9600 下实测被复位：
         * `task_wdt: - mb_fast (CPU 0)`）。
         * 115200 时单个请求只要 180ms，所以这个问题**降速前不会暴露**。
         * 调用方在 ulTaskNotifyTake() 里阻塞期间没法喂狗，只能在每次请求之间补。 */
        app_wdt_feed();
        if (mb_read_holding((uint16_t)(MODBUS_WIN_DATA_START + off), cnt, chunk,
                            MODBUS_TIMEOUT_MS_FAST, MODBUS_RETRY_READ) != MB_OK) {
            s_win_last_fail = WIN_FAIL_CHUNK_IO;
            return false;
        }
        /* 原样搬进窗口缓冲；行优先的顺序由"地址连续 + 顺序拼接"天然保证，
         * 上层按 samples[i*6 + axis] 取就是对 docs/02 §5-D 的填充顺序。 */
        for (uint16_t k = 0; k < cnt; k++) {
            w->samples[off + k] = (int16_t)chunk[k];
        }
        off = (uint16_t)(off + cnt);
    }
    w->tick = xTaskGetTickCount();

    /* ③ 再读 0x0101：序号变了 → 这 6 帧跨了窗口切换，整窗作废 */
    static uint16_t seq1[1];
    app_wdt_feed();
    if (mb_read_holding(MODBUS_WIN_SEQ_ADDR, 1, seq1,
                        MODBUS_TIMEOUT_MS_FAST, MODBUS_RETRY_READ) != MB_OK) {
        return false;
    }
    if (seq1[0] != w->win_seq) {
        s_win_last_fail = WIN_FAIL_SEQ_MISMATCH;
        if (mismatch_count != NULL) {
            (*mismatch_count)++;
        }
        portENTER_CRITICAL(&s_stats_mux);
        s_stats.seq_mismatch_count++;
        portEXIT_CRITICAL(&s_stats_mux);
        APP_LOGW("窗口序号失配：读前 %u，读后 %u → 丢弃本次窗口重读",
              (unsigned)w->win_seq, (unsigned)seq1[0]);
        return false;
    }
    return true;
}

void task_modbus_fast(void *arg)
{
    (void)arg;
    static uint16_t feat_regs[MODBUS_FEAT_COUNT];
    TickType_t last_wake = xTaskGetTickCount();
    app_wdt_register();
    APP_LOGI("task_modbus_fast 已启动：每 %dms 读窗口 0x%04X、%d 块、E 区 %d 维特征",
             MODBUS_FAST_PERIOD_MS, MODBUS_WIN_HEAD_START,
             MODBUS_WIN_CHUNK_COUNT, MODBUS_FEAT_COUNT);

    /* ★★ "长周期任务"必须切片睡眠，否则会被看门狗复位 ★★
     * 踩过的坑（2026-10 实测）：`vTaskDelayUntil(..., MODBUS_FAST_PERIOD_MS)` 一次睡
     * 10 秒，而任务已经用 app_wdt_register() 登记过 —— 登记过的任务**两次喂狗间隔
     * 超过 WDT_TIMEOUT_S(5s) 就 panic 复位**，表现是刚过 10 秒就：
     *     task_wdt: Task watchdog got triggered. - mb_fast (CPU 0)
     *     rst:0xc (RTC_SW_CPU_RST)
     * 而且**任务自己的第一行日志都来不及打**（它还在 delay 里），
     * 看起来像"任务卡死"，其实是"睡太久没喂狗"。
     * 为什么以前没事：115200 时代快车道周期是 2s（< 5s），碰巧不触发。
     * 结论：**任何周期 > WDT_TIMEOUT_S/2 的任务都不能一口气睡完**，
     *       必须像下面这样按 BUS_WDT_FEED_PERIOD_MS 切片，每片都喂狗。 */
    TickType_t elapsed = 0;

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(BUS_WDT_FEED_PERIOD_MS));
        app_wdt_feed();

        elapsed += BUS_WDT_FEED_PERIOD_MS;
        if (elapsed < (TickType_t)MODBUS_FAST_PERIOD_MS) {
            continue;                       /* 还没到快车道的周期，继续睡下一片 */
        }
        elapsed = 0;

        /* ---- 窗口通道（原始 600 点，给 1D-CNN / 画曲线） ---- */
        acq_window_t *w = window_pool_next();
        if (w != NULL) {
            uint32_t mism = 0;
            bool got = false;
            bool last_ready_bit = false;
            for (uint8_t try_i = 0; try_i <= MODBUS_WIN_SEQ_RETRY; try_i++) {
                if (window_read_once(w, &mism)) {
                    got = true;
                    break;
                }
                app_wdt_feed();
            }

            if (got) {
                s_win_ok++;
                w->seq_mismatch_count = mism;

                acq_item_t it;
                memset(&it, 0, sizeof(it));
                it.kind    = ACQ_ITEM_WINDOW;
                it.win     = w;
                it.win_seq = w->win_seq;
                it.tick    = w->tick;
                if (xQueueSend(g_acq_queue, &it, 0) != pdTRUE) {
                    APP_LOGW("acq_queue 满，丢弃一窗（推理任务跟不上？）");
                }

                ui_msg_t m;
                memset(&m, 0, sizeof(m));
                m.kind  = UI_MSG_WINDOW;
                m.u.win = w;
                (void)ui_post(&m, 0);
            } else {
                s_win_fail++;
                /* ★限频诊断日志（每 10 次失败打一条）★
                 * 为什么必须加：快车道失败时**原本一声不响**，界面上只看到
                 * "等待窗口..."，根本分不清是
                 *   ① 从机还没产出有效窗口（g_ready=0，bit15=0）——不是错误
                 *   ② 请求失败/超时
                 *   ③ 序号校验不过（跨窗口切换）
                 * 这三种的处置完全不同，所以把状态打出来。 */
                if ((s_win_fail % 10u) == 1u) {
                    APP_LOGW("[快车道] 窗口读失败（累计 成功 %u / 失败 %u）：%s（状态码 %u）",
                             (unsigned)s_win_ok, (unsigned)s_win_fail,
                             window_fail_reason_text(), (unsigned)s_win_last_status);
                }
                (void)last_ready_bit;
            }
        }

        /* ---- 特征通道（E 区 36 个寄存器，给 MLP + 带宽对比实验） ----
         * plan.md §5.3 第 4 条的"双通道上抛"：同周期把 F411 侧算好的特征也拉回来，
         * 带宽只有原始通道的 1/17。两条通道都送进 acq_queue，由推理任务分别处理。 */
        app_wdt_feed();     /* 与窗口读同理：慢波特率下这个请求也可能等满预算 */
        if (mb_read_holding(MODBUS_FEAT_START, MODBUS_FEAT_COUNT, feat_regs,
                            MODBUS_TIMEOUT_MS_FAST, MODBUS_RETRY_READ) == MB_OK) {
            acq_item_t it;
            memset(&it, 0, sizeof(it));
            it.kind = ACQ_ITEM_FEATURES;
            for (int i = 0; i < MODBUS_FEAT_COUNT; i++) {
                it.features[i] = (int16_t)feat_regs[i];
            }
            it.win_seq = 0;
            it.tick    = xTaskGetTickCount();
            if (xQueueSend(g_acq_queue, &it, 0) != pdTRUE) {
                APP_LOGW("acq_queue 满，丢弃一帧特征向量");
            }
        }
    }
}
