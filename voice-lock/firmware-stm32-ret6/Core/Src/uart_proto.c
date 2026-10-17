/* =============================================================================
 *  uart_proto.c — 双片通信协议实现（组帧 / CRC / 打包）
 * =============================================================================
 *  见 uart_proto.h 顶部的帧格式说明。
 * ========================================================================== */

#include "uart_proto.h"

/* =============================================================================
 *  CRC16-Modbus
 * =============================================================================
 *  多项式 0xA001（0x8005 的反转形式），初值 0xFFFF，不异或输出。
 *  为什么用这个：和项目1 的 Modbus RTU 用的是**同一个算法**，
 *  已经被"4 小时长跑 47510 次事务、CRC 错 0 次"验证过（见项目1 文档）。
 *  两片之间用同一个校验算法，出问题时排查思路也能复用。
 *
 *  实现用"查表法"而不是逐位循环：
 *    逐位：每字节 8 次循环 → 512 字节要 4096 次迭代
 *    查表：每字节 1 次查表 → 512 字节只要 512 次
 *  在 72MHz 无 FPU 的 F103 上，传输 1 秒音频（16KB = 32 块）时，
 *  查表法能省下可观的 CPU 时间。
 * ========================================================================== */
static uint16_t s_crc_table[256];
static bool     s_crc_table_ready = false;

static void crc_table_init(void)
{
    for (uint32_t i = 0; i < 256U; i++) {
        uint16_t crc = (uint16_t)i;
        for (uint32_t bit = 0; bit < 8U; bit++) {
            if (crc & 0x0001U) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001U);
            } else {
                crc = (uint16_t)(crc >> 1);
            }
        }
        s_crc_table[i] = crc;
    }
    s_crc_table_ready = true;
}

uint16_t uart_crc16(const uint8_t *data, uint32_t len)
{
    if (!s_crc_table_ready) {
        crc_table_init();
    }

    uint16_t crc = 0xFFFFU;
    for (uint32_t i = 0; i < len; i++) {
        const uint8_t idx = (uint8_t)((crc ^ data[i]) & 0xFFU);
        crc = (uint16_t)((crc >> 8) ^ s_crc_table[idx]);
    }
    return crc;
}

/* =============================================================================
 *  组帧
 * ========================================================================== */
static uint8_t s_tx_seq = 0;   /* 发送序号，0~255 循环 */

uint8_t uart_next_seq(void)
{
    return s_tx_seq;
}

uint32_t uart_build_frame(uint8_t *out, uint8_t cmd, uint8_t seq,
                          const uint8_t *payload, uint16_t payload_len)
{
    if (out == 0) {
        return 0;
    }
    if (payload_len > UART_MAX_PAYLOAD) {
        return 0;                      /* 超长：拒绝，避免越界写 */
    }
    if (payload_len > 0U && payload == 0) {
        return 0;                      /* 声明有载荷但指针为空 */
    }

    out[0] = UART_SYNC_BYTE;
    out[1] = cmd;
    out[2] = seq;
    out[3] = (uint8_t)(payload_len & 0xFFU);          /* 小端低字节 */
    out[4] = (uint8_t)((payload_len >> 8) & 0xFFU);   /* 小端高字节 */

    for (uint16_t i = 0; i < payload_len; i++) {
        out[UART_HEADER_LEN + i] = payload[i];
    }

    /* CRC 覆盖 CMD ~ PAYLOAD（即从 offset 1 开始，长度 = 4 + payload_len）。
     * 不含 SYNC，理由见头文件说明。 */
    const uint32_t crc_span = 4U + (uint32_t)payload_len;
    const uint16_t crc = uart_crc16(&out[1], crc_span);
    out[UART_HEADER_LEN + payload_len]     = (uint8_t)(crc & 0xFFU);
    out[UART_HEADER_LEN + payload_len + 1] = (uint8_t)((crc >> 8) & 0xFFU);

    s_tx_seq++;      /* 下一帧用新序号（调用方通常直接传 uart_next_seq()）*/
    return (uint32_t)UART_OVERHEAD + (uint32_t)payload_len;
}

/* =============================================================================
 *  样本打包（16 位 → 小端字节流）
 * =============================================================================
 *  ⚠ 为什么统一用**小端**：
 *    STM32（Cortex-M3）和 ESP32（Xtensa）都是小端，直接 memcpy 也对。
 *    但显式写成"低字节在前"更安全 —— 万一以后换成大端平台（比如某些 DSP），
 *    协议不用改。
 * ========================================================================== */
uint32_t uart_pack_samples(uint8_t *out, const uint16_t *samples, uint32_t count)
{
    if (out == 0 || samples == 0) {
        return 0;
    }
    for (uint32_t i = 0; i < count; i++) {
        out[i * 2U]      = (uint8_t)(samples[i] & 0xFFU);
        out[i * 2U + 1U] = (uint8_t)((samples[i] >> 8) & 0xFFU);
    }
    return count * 2U;
}

/* =============================================================================
 *  接收侧状态机
 * =============================================================================
 *  和 ESP32 侧（uart_link.cpp）用的是**同一套逻辑**：
 *    ① 滑动找 0xAA 帧头
 *    ② 长度合法性校验（>1024 就说明这个 0xAA 是假帧头，跳过）
 *    ③ CRC16 校验（覆盖 CMD~PAYLOAD，不含 SYNC）
 *  两侧行为一致的好处：出问题时排查思路能复用。
 * ========================================================================== */
typedef enum {
    RXS_SYNC = 0,
    RXS_HEADER,
    RXS_PAYLOAD,
    RXS_CRC
} RxState;

static RxState  s_rxs = RXS_SYNC;
static uint8_t  s_rxHdr[4];
static uint8_t  s_rxHdrIdx = 0;
static uint16_t s_rxLen = 0;
static uint16_t s_rxIdx = 0;
static uint8_t  s_rxPayload[UART_MAX_PAYLOAD];
static uint8_t  s_rxCrc[2];
static uint8_t  s_rxCrcIdx = 0;

static volatile uint8_t s_resultReady = 0;
static UartResult       s_result;
static uint32_t         s_rxFrames = 0;
static uint32_t         s_rxCrcErr = 0;
static uint32_t         s_rxResync = 0;

void uartRxFeed(uint8_t b)
{
    switch (s_rxs) {

    case RXS_SYNC:
        if (b == UART_SYNC_BYTE) {
            s_rxHdrIdx = 0;
            s_rxs = RXS_HEADER;
        }
        return;

    case RXS_HEADER:
        s_rxHdr[s_rxHdrIdx++] = b;
        if (s_rxHdrIdx >= 4U) {
            s_rxLen = (uint16_t)(s_rxHdr[2] | ((uint16_t)s_rxHdr[3] << 8));
            if (s_rxLen > UART_MAX_PAYLOAD) {
                /* 长度不合理 ⇒ 这个 0xAA 是数据里偶然出现的，继续找帧头 */
                s_rxResync++;
                s_rxs = RXS_SYNC;
                s_rxHdrIdx = 0;
                return;
            }
            s_rxIdx = 0;
            s_rxs = (s_rxLen == 0U) ? RXS_CRC : RXS_PAYLOAD;
        }
        return;

    case RXS_PAYLOAD:
        s_rxPayload[s_rxIdx++] = b;
        if (s_rxIdx >= s_rxLen) {
            s_rxCrcIdx = 0;
            s_rxs = RXS_CRC;
        }
        return;

    case RXS_CRC:
        s_rxCrc[s_rxCrcIdx++] = b;
        if (s_rxCrcIdx >= 2U) {
            s_rxs = RXS_SYNC;      /* 无论成败都回到找帧头 */

            const uint16_t rxCrc = (uint16_t)(s_rxCrc[0]
                                   | ((uint16_t)s_rxCrc[1] << 8));

            /* CRC 覆盖 CMD~PAYLOAD（4 + len 字节）*/
            uint8_t span[4 + UART_MAX_PAYLOAD];
            span[0] = s_rxHdr[0];
            span[1] = s_rxHdr[1];
            span[2] = s_rxHdr[2];
            span[3] = s_rxHdr[3];
            for (uint16_t i = 0; i < s_rxLen; i++) {
                span[4 + i] = s_rxPayload[i];
            }
            if (uart_crc16(span, 4U + (uint32_t)s_rxLen) != rxCrc) {
                s_rxCrcErr++;
                return;
            }

            s_rxFrames++;

            /* 只关心判定结果帧 */
            if (s_rxHdr[0] == UART_CMD_RESULT && s_rxLen >= UART_RESULT_LEN) {
                s_result.verdict    = s_rxPayload[0];
                s_result.confidence = s_rxPayload[1];
                s_result.similarity = (uint16_t)(s_rxPayload[2]
                                      | ((uint16_t)s_rxPayload[3] << 8));
                s_result.elapsedMs  = (uint32_t)s_rxPayload[4]
                                    | ((uint32_t)s_rxPayload[5] << 8)
                                    | ((uint32_t)s_rxPayload[6] << 16)
                                    | ((uint32_t)s_rxPayload[7] << 24);
                s_result.valid      = 1U;
                s_resultReady       = 1U;     /* 通知主循环 */
            }
        }
        return;
    }
}

uint8_t uartResultReady(void)
{
    return s_resultReady;
}

void uartGetResult(UartResult *out)
{
    if (out == 0) { return; }
    *out = s_result;
    s_resultReady = 0;      /* 取走即清除，避免重复处理同一帧 */
}

void uartRxStats(uint32_t *frames, uint32_t *crcErr, uint32_t *resync)
{
    if (frames) { *frames = s_rxFrames; }
    if (crcErr) { *crcErr = s_rxCrcErr; }
    if (resync) { *resync = s_rxResync; }
}
