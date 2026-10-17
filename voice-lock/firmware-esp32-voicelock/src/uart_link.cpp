/* =============================================================================
 *  uart_link.cpp — 音频链路接收侧实现
 * =============================================================================
 *  见 uart_link.h 的协议说明。
 *
 *  接收状态机设计要点
 *  ---------------------------------------------------------------------------
 *  ① **滑动式重新同步**
 *     如果当前字节不是 0xAA，就丢掉它并继续找。这样即使中间丢了几个字节
 *     （电平干扰 / 上电时序），也能在下一个 0xAA 处重新对齐，而不是
 *     整个缓冲区错位到底。
 *
 *  ② **两级校验**
 *     找到 0xAA 后先读长度：如果长度 > UART_MAX_PAYLOAD，说明这个 0xAA
 *     其实不是真的帧头（是数据里恰好出现的），直接跳过继续找。
 *     这样能大幅降低"把数据误当帧头"的概率。
 *     然后再用 CRC 做最终确认。
 *
 *  ③ **超时丢弃**
 *     如果收到一半就没了（对端重启/线松），用时间戳判断并复位状态机，
 *     否则会一直卡在"等剩余字节"的状态里。
 *
 *  ⚠ 为什么不直接用 Serial2.readBytes() 阻塞式收整帧：
 *    那样一旦对端只发了一半就断开，会把整个任务卡死（门禁是实时交互，
 *    必须能及时恢复）。这里用非阻塞状态机，每轮只处理已有数据。
 * ========================================================================== */

#include "uart_link.h"

// ---- 接收状态机 ------------------------------------------------------------
enum RxState : uint8_t {
    RX_WAIT_SYNC = 0,   // 找 0xAA
    RX_HEADER,          // 收 CMD/SEQ/LEN（4 字节）
    RX_PAYLOAD,         // 收载荷
    RX_CRC              // 收 CRC（2 字节）
};

static HardwareSerial  s_f103(2);          // UART2
static RxState         s_state = RX_WAIT_SYNC;
static uint8_t         s_hdr[4];           // CMD, SEQ, LEN_L, LEN_H
static uint8_t         s_hdrIdx = 0;
static uint16_t        s_payloadLen = 0;
static uint16_t        s_payloadIdx = 0;
static uint8_t         s_payload[UART_MAX_PAYLOAD];
static uint8_t         s_crcBuf[2];
static uint8_t         s_crcIdx = 0;
static uint32_t        s_lastByteMs = 0;

static LinkStats       s_stats = {0, 0, 0, 0};

/* 原始字节计数（诊断用）。
 * 为什么单独统计"裸字节数"而不是只看"收帧数"：
 *   如果 RX 线上一个字节都没有 → 接线问题（TX/RX 接反/没接/没共地）
 *   如果有字节但都是乱的     → 波特率不对 或 电平不匹配
 *   如果有字节且成帧         → 链路正常
 *   三种情况的排查方向完全不同，所以要把"裸字节是否有"单独暴露出来。 */
static uint32_t        s_rawBytes = 0;
static uint32_t        s_rawBytesLast = 0;
static uint32_t        s_rawRateMs = 0;
static bool            s_hexDump = false;

/* =============================================================================
 *  多串口监听（诊断工具）
 * =============================================================================
 *  为什么需要"两个串口同时挂在同一对引脚上"：
 *    ESP32-S3 通过 GPIO 矩阵把 UART 信号路由到任意引脚。如果 UART2 收不到，
 *    可能是①引脚接错 ②UART2 控制器/路由有问题 ③电平问题。
 *    让 UART1 也挂到同样的引脚上，就能把②区分出来：
 *        UART1 收到 + UART2 收不到  ⇒ UART2 这条路有问题
 *        两个都收不到               ⇒ 引脚上根本没有信号（①或③）
 *        两个都收到                 ⇒ 之前的配置写错了（而不是硬件）
 *    这是"用对照实验代替猜测"的典型做法。
 *
 *  ⚠ 两个 UART 同时读同一对引脚在 ESP32-S3 上是允许的（都是输入，不冲突），
 *    实际使用时只会启用一个（见 uartLinkSetActiveUart）。
 * ========================================================================== */
static HardwareSerial  s_alt(1);          /* UART1，用于对照实验 */
static uint8_t         s_activeUart = F103_UART_NUM;   /* 当前用哪个收 */
static uint32_t        s_altBytes = 0;    /* UART1 对照实验收到的字节数 */
static uint32_t        s_altLast = 0;
static bool            s_altEnabled = false;

// ---- CRC16-Modbus（查表法，与 F103 侧同算法）-------------------------------
static uint16_t s_crcTable[256];
static bool     s_crcReady = false;

static void crcTableInit(void)
{
    for (uint32_t i = 0; i < 256; i++) {
        uint16_t crc = (uint16_t)i;
        for (uint32_t bit = 0; bit < 8; bit++) {
            if (crc & 1) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001);
            } else {
                crc = (uint16_t)(crc >> 1);
            }
        }
        s_crcTable[i] = crc;
    }
    s_crcReady = true;
}

static uint16_t crc16(const uint8_t *data, uint32_t len)
{
    if (!s_crcReady) { crcTableInit(); }
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; i++) {
        const uint8_t idx = (uint8_t)((crc ^ data[i]) & 0xFF);
        crc = (uint16_t)((crc >> 8) ^ s_crcTable[idx]);
    }
    return crc;
}

// ---- 初始化 ----------------------------------------------------------------
void uartLinkBegin(void)
{
    s_f103.begin(F103_BAUD, SERIAL_8N1, F103_PIN_RX, F103_PIN_TX);
    s_f103.setTimeout(50);
    s_state = RX_WAIT_SYNC;
    s_hdrIdx = 0;
    s_payloadIdx = 0;
    s_crcIdx = 0;
    s_lastByteMs = millis();

    Serial.printf("[LINK] UART%d ready: TX=GPIO%d RX=GPIO%d @ %d\n",
                  F103_UART_NUM, F103_PIN_TX, F103_PIN_RX, F103_BAUD);
}

// ---- 组帧发送 --------------------------------------------------------------
static uint8_t s_txSeq = 0;

static void sendFrame(uint8_t cmd, const uint8_t *payload, uint16_t len)
{
    if (len > UART_MAX_PAYLOAD) { return; }

    uint8_t frame[UART_MAX_FRAME];
    frame[0] = UART_SYNC_BYTE;
    frame[1] = cmd;
    frame[2] = s_txSeq++;
    frame[3] = (uint8_t)(len & 0xFF);
    frame[4] = (uint8_t)((len >> 8) & 0xFF);

    for (uint16_t i = 0; i < len; i++) {
        frame[UART_HEADER_LEN + i] = payload[i];
    }

    // CRC 覆盖 CMD~PAYLOAD（offset 1 起，长度 4+len）
    const uint16_t crc = crc16(&frame[1], (uint32_t)4 + len);
    frame[UART_HEADER_LEN + len]     = (uint8_t)(crc & 0xFF);
    frame[UART_HEADER_LEN + len + 1] = (uint8_t)((crc >> 8) & 0xFF);

    s_f103.write(frame, (size_t)(UART_OVERHEAD + len));
}

void uartLinkSendResult(uint8_t verdict, uint8_t confidence,
                        uint16_t similarity, uint32_t elapsedMs)
{
    uint8_t p[UART_RESULT_LEN];
    p[0] = verdict;
    p[1] = confidence;
    p[2] = (uint8_t)(similarity & 0xFF);
    p[3] = (uint8_t)((similarity >> 8) & 0xFF);
    p[4] = (uint8_t)(elapsedMs & 0xFF);
    p[5] = (uint8_t)((elapsedMs >> 8) & 0xFF);
    p[6] = (uint8_t)((elapsedMs >> 16) & 0xFF);
    p[7] = (uint8_t)((elapsedMs >> 24) & 0xFF);
    sendFrame(UART_CMD_RESULT, p, UART_RESULT_LEN);
}

void uartLinkSendStatus(const char *text)
{
    if (text == nullptr) { return; }
    const uint16_t len = (uint16_t)strnlen(text, UART_MAX_PAYLOAD);
    sendFrame(UART_CMD_STATUS, (const uint8_t *)text, len);
}

const LinkStats &uartLinkStats(void)
{
    return s_stats;
}

// ---- 处理一帧（校验通过后）-------------------------------------------------
static void handleFrame(const uint8_t *cmdSeq, const uint8_t *payload,
                        uint16_t len, AudioMetaCb onMeta, AudioDataCb onData)
{
    const uint8_t cmd = cmdSeq[0];

    switch (cmd) {
    case UART_CMD_AUDIO_META:
        if (onMeta && len >= UART_META_LEN) {
            AudioMeta m;
            m.sampleRate  = (uint32_t)payload[0]
                          | ((uint32_t)payload[1] << 8)
                          | ((uint32_t)payload[2] << 16)
                          | ((uint32_t)payload[3] << 24);
            m.sampleCount = (uint32_t)payload[4]
                          | ((uint32_t)payload[5] << 8)
                          | ((uint32_t)payload[6] << 16)
                          | ((uint32_t)payload[7] << 24);
            m.chunkCount  = (uint16_t)(payload[8] | (payload[9] << 8));
            m.rawBits     = payload[10];
            m.txBits      = payload[11];
            m.valid       = true;
            onMeta(m);
        }
        break;

    case UART_CMD_AUDIO_DATA:
        if (onData && len > 0) {
            onData(payload, len);
        }
        break;

    case UART_CMD_PING:
        sendFrame(UART_CMD_PING, nullptr, 0);
        break;

    default:
        break;
    }
}

// ---- 主轮询（非阻塞状态机）-------------------------------------------------
/** 处理单个字节，返回"本字节是否完成了一帧"。 */
static int rxByte(uint8_t b, AudioMetaCb onMeta, AudioDataCb onData)
{
    switch (s_state) {

    case RX_WAIT_SYNC:
        if (b == UART_SYNC_BYTE) {
            s_hdrIdx = 0;
            s_state = RX_HEADER;
        } else {
            s_stats.bytesDropped++;
        }
        return 0;

    case RX_HEADER:
        s_hdr[s_hdrIdx++] = b;
        if (s_hdrIdx >= 4) {
            s_payloadLen = (uint16_t)(s_hdr[2] | (s_hdr[3] << 8));
            // 两级校验的第一级：长度合法性。不合理就说明 0xAA 是假帧头。
            if (s_payloadLen > UART_MAX_PAYLOAD) {
                s_stats.resyncs++;
                s_state = RX_WAIT_SYNC;
                s_hdrIdx = 0;
                return 0;
            }
            s_payloadIdx = 0;
            s_state = (s_payloadLen == 0) ? RX_CRC : RX_PAYLOAD;
        }
        return 0;

    case RX_PAYLOAD:
        s_payload[s_payloadIdx++] = b;
        if (s_payloadIdx >= s_payloadLen) {
            s_crcIdx = 0;
            s_state = RX_CRC;
        }
        return 0;

    case RX_CRC:
        s_crcBuf[s_crcIdx++] = b;
        if (s_crcIdx >= 2) {
            const uint16_t rx = (uint16_t)(s_crcBuf[0] | (s_crcBuf[1] << 8));

            /* 第二级校验：CRC。覆盖 CMD~PAYLOAD（4 + len 字节）。
             * ⚠ 这里用一个**静态**缓冲而不是局部大数组：
             *   局部数组会在栈上占 4+1024 字节，而这个函数每次收字节都调用，
             *   没必要反复分配；静态缓冲只在单线程的 loop 里用，安全。 */
            static uint8_t span[4 + UART_MAX_PAYLOAD];
            span[0] = s_hdr[0];
            span[1] = s_hdr[1];
            span[2] = s_hdr[2];
            span[3] = s_hdr[3];
            for (uint16_t i = 0; i < s_payloadLen; i++) {
                span[4 + i] = s_payload[i];
            }
            const uint16_t calc = crc16(span, (uint32_t)4 + s_payloadLen);

            s_state = RX_WAIT_SYNC;      /* 无论成败都回到找帧头 */
            if (calc == rx) {
                s_stats.framesOk++;
                handleFrame(s_hdr, s_payload, s_payloadLen, onMeta, onData);
                return 1;
            }
            s_stats.crcErrors++;
            return 0;
        }
        return 0;
    }

    s_state = RX_WAIT_SYNC;
    return 0;
}

int uartLinkPoll(AudioMetaCb onMeta, AudioDataCb onData)
{
    int handled = 0;

    while (s_f103.available() > 0) {
        const uint8_t b = (uint8_t)s_f103.read();
        s_rawBytes++;                      /* 裸字节计数（不管成不成帧）*/
        if (s_hexDump) { Serial.printf("%02X ", b); }
        handled += rxByte(b, onMeta, onData);
        s_lastByteMs = millis();
    }
    if (s_hexDump && handled > 0) { Serial.println(); }

    // 半帧超时复位：避免对端只发了一半就跑掉时永远卡住
    if (s_state != RX_WAIT_SYNC && (millis() - s_lastByteMs) > 300) {
        s_state = RX_WAIT_SYNC;
        s_hdrIdx = 0;
        s_payloadIdx = 0;
        s_crcIdx = 0;
        s_stats.resyncs++;
    }

    return handled;
}

/* =============================================================================
 *  诊断接口
 * ========================================================================== */

uint32_t uartLinkRawBytes(void)
{
    return s_rawBytes;
}

uint32_t uartLinkLastSecondBytes(void)
{
    /* 每秒更新一次"最近 1 秒收到多少裸字节"。
     * 正常工作时：F103 发送期间应该接近 11520 字节/秒
     *            （115200 baud ÷ 10 bit/byte = 11520 字节/秒）。
     * 如果一直是 0 ⇒ RX 线上根本没有数据（接线 / 没共地）。
     * 如果有值但全是乱码 ⇒ 波特率不对 或 两端没共地导致电平漂移。 */
    const uint32_t now = millis();
    if ((now - s_rawRateMs) >= 1000) {
        s_rawRateMs    = now;
        s_rawBytesLast = s_rawBytes;
        s_rawBytes     = 0;
    }
    return s_rawBytesLast;
}

void uartLinkSetHexDump(bool on)
{
    s_hexDump = on;
    Serial.printf("[LINK] hex dump %s\n", on ? "ON" : "OFF");
}

/* =============================================================================
 *  对照实验：让 UART1 也挂到 IO5/IO4 上一起听
 * ========================================================================== */
void uartLinkAltBegin(void)
{
    if (s_altEnabled) { return; }
    s_alt.begin(F103_BAUD, SERIAL_8N1, F103_PIN_RX, F103_PIN_TX);
    s_altEnabled = true;
    s_altBytes = 0;
    s_altLast = 0;
    Serial.printf("[DIAG] UART1 also listening on RX=IO%d TX=IO%d @ %d\n",
                  F103_PIN_RX, F103_PIN_TX, F103_BAUD);
    Serial.println("[DIAG] now: on F103 press 'u' and speak, then check both"
                   " counters (command 'i')");
}

void uartLinkAltEnd(void)
{
    if (!s_altEnabled) { return; }
    s_alt.end();
    s_altEnabled = false;
    Serial.println("[DIAG] UART1 stopped");
}

uint32_t uartLinkAltBytes(void)
{
    return s_altLast;
}

void uartLinkPollAlt(void)
{
    if (!s_altEnabled) { return; }
    while (s_alt.available() > 0) {
        (void)s_alt.read();
        s_altBytes++;
    }
    /* 每秒结算一次，和主 UART 的口径一致 */
    static uint32_t lastMs = 0;
    const uint32_t now = millis();
    if ((now - lastMs) >= 1000) {
        lastMs = now;
        s_altLast = s_altBytes;
        s_altBytes = 0;
    }
}

uint8_t uartLinkActiveUart(void)
{
    return s_activeUart;
}

/* =============================================================================
 *  ESP32 侧 UART 硬件诊断
 * =============================================================================
 *  背景：F103 侧已经用寄存器证明"数据确实发出去了"（TC=1、PB10=AF_PP、引脚可驱动），
 *        但 ESP32 在 IO5/IO6 上都只看到 ~1 B/s 的杂波（= 引脚悬空时的噪声底）。
 *        ⇒ 要在 ESP32 侧确认：线到底通不通、信号有没有真的进到 UART 外设。
 *
 *  关键手段：**直接读引脚的输入电平**
 *    UART 空闲态是**高电平**。所以 F103 不发送时，它接在 ESP32 RX 上的那根线
 *    应该是 **高**。这一条比看 available() 准得多：
 *        RX 读到 1  ⇒ 线上确实挂着一个空闲的 UART TX（线通了）
 *        RX 读到 0  ⇒ 没接东西，或对端 TX 在拉低
 *        一直跳变    ⇒ 悬空受干扰
 * ========================================================================== */
void uartLinkDiag(void)
{
    Serial.println("---- ESP32 UART DIAG ----");
    Serial.printf("  UART num=%u  RX=IO%d  TX=IO%d  baud=%d\n",
                  (unsigned)s_activeUart, F103_PIN_RX, F103_PIN_TX, F103_BAUD);

    /* ① 引脚当前电平（判断线是否真的接上）*/
    const int rxLevel = digitalRead(F103_PIN_RX);
    const int txLevel = digitalRead(F103_PIN_TX);
    Serial.printf("  pin level: RX(IO%d)=%d  TX(IO%d)=%d\n",
                  F103_PIN_RX, rxLevel, F103_PIN_TX, txLevel);
    if (rxLevel == 1) {
        Serial.println("  RX HIGH -> an idle UART TX seems attached (good sign)");
    } else {
        Serial.println("  RX LOW  -> nothing attached, or peer TX is low");
    }

    /* ② 多处采样：悬空引脚的电平会乱跳，接好的会稳定 */
    int ones = 0;
    for (int i = 0; i < 200; i++) {
        if (digitalRead(F103_PIN_RX) == 1) { ones++; }
        delayMicroseconds(50);
    }
    Serial.printf("  RX high ratio = %d/200 (stable wire should be 200/200 "
                  "when idle; floating will be erratic)\n", ones);

    /* ③ 上层统计 */
    const LinkStats &st = uartLinkStats();
    Serial.printf("  rawBytes=%lu lastSec=%lu B/s frames=%lu crcErr=%lu "
                  "resync=%lu dropped=%lu\n",
                  (unsigned long)uartLinkRawBytes(),
                  (unsigned long)uartLinkLastSecondBytes(),
                  (unsigned long)st.framesOk, (unsigned long)st.crcErrors,
                  (unsigned long)st.resyncs, (unsigned long)st.bytesDropped);

    if (s_altEnabled) {
        Serial.printf("  alt UART1 on same pins: %lu B/s\n",
                      (unsigned long)s_altLast);
    }
    Serial.println("-------------------------");
}

/* =============================================================================
 *  UART 回环测试（loopback）——判断"ESP32 的 UART 到底能不能收发"
 * =============================================================================
 *  为什么需要这个：
 *    现状很矛盾 —— F103 的寄存器证明"数据发出去了"，ESP32 的引脚电平证明
 *    "线是通的（RX 稳定高）"，但 ESP32 就是收不到任何字节。
 *    ⇒ 必须单独验证 **ESP32 自己的 UART 外设**能不能收，
 *       否则会一直在"接线 / 对端"上打转。
 *
 *  做法（最干净的自证方式）：
 *    让用户拿一根杜邦线把 **IO6(TX) 直接接到 IO7(RX)**（自己接自己），
 *    然后本函数发一串已知数据、再从 RX 读回来。
 *      收到且内容一致 ⇒ UART 收发通路 + 两个引脚都正常
 *      收不到         ⇒ ESP32 这边（UART 驱动/引脚）有问题
 *
 *  ⚠ 做这个测试时必须先把 F103 那两根线拔掉，否则会互相干扰。
 * ========================================================================== */
void uartLinkLoopbackTest(void)
{
    static const char *pattern = "LOOPBACK-TEST-0123456789";
    const uint16_t plen = (uint16_t)strlen(pattern);

    Serial.println("---- UART LOOPBACK TEST ----");
    Serial.printf("  STEP 1: connect a jumper: IO%d (TX) --> IO%d (RX)\n",
                  F103_PIN_TX, F103_PIN_RX);
    Serial.println("  (unplug the F103 wires first!)");
    Serial.println("  starting in 5 s ...");
    for (int i = 5; i > 0; i--) {
        Serial.printf("  %d ...\n", i);
        delay(1000);
    }

    /* 清空接收缓冲，避免把之前残留的杂波当结果 */
    while (s_f103.available() > 0) { (void)s_f103.read(); }

    Serial.printf("  TX %u bytes: %s\n", (unsigned)plen, pattern);
    s_f103.write((const uint8_t *)pattern, plen);
    s_f103.flush();          /* 等数据真正发完（硬件移位寄存器空）*/

    /* 等回环数据回来（回环是即时的，给 300ms 足够）*/
    delay(300);

    uint8_t got[64];
    uint16_t n = 0;
    while (s_f103.available() > 0 && n < sizeof(got)) {
        got[n++] = (uint8_t)s_f103.read();
    }

    Serial.printf("  RX got %u bytes\n", (unsigned)n);
    if (n == 0) {
        Serial.println("  => LOOPBACK FAILED: nothing came back.");
        Serial.println("     check: is the jumper really IO" + String(F103_PIN_TX)
                       + " <-> IO" + String(F103_PIN_RX) + " ?");
        Serial.println("     if yes -> the ESP32 UART on these pins is not working.");
        return;
    }

    Serial.print("  RX data: ");
    for (uint16_t i = 0; i < n; i++) {
        const char c = (char)got[i];
        Serial.print((c >= 32 && c < 127) ? c : '.');
    }
    Serial.println();

    /* 比对（只比收到的长度，避免尾部噪声干扰判断）*/
    const uint16_t cmp = (n < plen) ? n : plen;
    if (cmp == plen && memcmp(got, pattern, plen) == 0) {
        Serial.println("  => LOOPBACK OK: ESP32 UART TX+RX both work.");
    } else {
        Serial.println("  => LOOPBACK PARTIAL: got data but it does not match.");
        Serial.println("     => baud/framing problem inside the ESP32 UART.");
    }
    Serial.println("----------------------------");
}
