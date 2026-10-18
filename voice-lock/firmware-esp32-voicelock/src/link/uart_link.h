/* =============================================================================
 *  uart_link.h — ESP32-S3 ↔ STM32F103RET6 音频链路（接收侧）
 * =============================================================================
 *  协议定义**必须和 F103 侧完全一致**（见 firmware-stm32-ret6/Core/Inc/uart_proto.h）
 *
 *  帧格式（小端）
 *  ---------------------------------------------------------------------------
 *    偏移  长度  字段      说明
 *    0     1     SYNC      固定 0xAA
 *    1     1     CMD       命令字
 *    2     1     SEQ       帧序号（0~255 循环）
 *    3     2     LEN       PAYLOAD 长度（小端）
 *    5     N     PAYLOAD   数据
 *    5+N   2     CRC16     CRC16-Modbus，覆盖 CMD~PAYLOAD（**不含 SYNC**）
 *
 *  ⚠ CRC 不含 SYNC 的原因（和 F103 侧一致）：
 *    SYNC 只是"找帧头"的标记。如果它参与 CRC，一旦 SYNC 被干扰破坏，
 *    接收方会因为"CRC 不过"而丢弃整帧；不含 SYNC 时接收方可以
 *    滑动式地重找帧头，鲁棒性更好。
 * ========================================================================== */

#ifndef __UART_LINK_H
#define __UART_LINK_H

#include <Arduino.h>

/* ---- 引脚定义 ---------------------------------------------------------- */
#define F103_UART_NUM       2       /* 用 UART2（UART0 是调试口）*/

/* ★ 引脚可以随时换（排查虚焊/坏脚用）★
 *   背景：F103 侧 USART3 的寄存器诊断显示 TE=1 / TC=1（芯片确实把数据推出去了），
 *   但 ESP32 在 IO5 上一个字节都收不到（UART1/UART2 都试过，裸字节 0）。
 *   ⇒ 只能从"片外"找原因：F103 排针虚焊、杜邦线、或 ESP32 的某个脚坏了。
 *   换脚是**零成本**的排查手段（改两行 + 换根线），比补焊快。
 *
 *   已验证过的组合：
 *     IO4 / IO5   → 收不到（裸字节 0）
 *   候选组合（按推荐顺序）：
 *     IO6 / IO7   → 先试这个（和 IO4/IO5 同排，好接）
 *     IO47/IO48   → 再试这个（另一侧，几乎不可能和其它外设冲突）
 */
#define F103_PIN_TX         6       /* ESP32 TX → F103 PB11 (RX) */
#define F103_PIN_RX         7       /* ESP32 RX ← F103 PB10 (TX) */
#define F103_BAUD           115200

/* ---- 协议常量（必须与 F103 侧一致）------------------------------------- */
#define UART_SYNC_BYTE      0xAA
#define UART_HEADER_LEN     5
#define UART_CRC_LEN        2
#define UART_OVERHEAD       (UART_HEADER_LEN + UART_CRC_LEN)   /* = 7 */
#define UART_MAX_PAYLOAD    1024
#define UART_MAX_FRAME      (UART_MAX_PAYLOAD + UART_OVERHEAD)

#define UART_CMD_AUDIO_META 0x02
#define UART_CMD_AUDIO_DATA 0x01
#define UART_CMD_RESULT     0x10
#define UART_CMD_STATUS     0x11
#define UART_CMD_PING       0x20

#define UART_VERDICT_FAIL   0x00
#define UART_VERDICT_PASS   0x01

#define UART_META_LEN       12
#define UART_RESULT_LEN     8

/* ---- 音频元信息（收到 META 帧后填充）----------------------------------- */
struct AudioMeta {
    uint32_t sampleRate;      /* 采样率 Hz */
    uint32_t sampleCount;     /* 总样本数 */
    uint16_t chunkCount;      /* 总块数（用于丢块检测）*/
    uint8_t  rawBits;         /* 原始位深（通常 16，表示 ADC 12bit 按 16 描述）*/
    uint8_t  txBits;          /* 传输位深（8 = 已做 8bit 量化）*/
    bool     valid;
};

/* ---- API --------------------------------------------------------------- */

/** 初始化 UART2（接 F103）。 */
void uartLinkBegin(void);

/** 轮询接收。收到完整帧时通过回调交给上层。
 *  参数 onAudioMeta：收到 META 帧时调用
 *  参数 onAudioData：收到音频块时调用（data 指向 8bit 量化样本，len 为字节数）
 *  返回：本次处理期间收到的帧数（0 表示没收到东西）。 */
typedef void (*AudioMetaCb)(const AudioMeta &meta);
typedef void (*AudioDataCb)(const uint8_t *data, uint16_t len);

int uartLinkPoll(AudioMetaCb onMeta, AudioDataCb onData);

/** 发送判定结果回 F103（PASS/FAIL + 置信度 + 相似度 + 耗时）。 */
void uartLinkSendResult(uint8_t verdict, uint8_t confidence,
                        uint16_t similarity, uint32_t elapsedMs);

/** 发送一条状态/调试文本回 F103（可选，方便 F103 侧打印）。 */
void uartLinkSendStatus(const char *text);

/** 统计信息（排查丢帧用）。 */
struct LinkStats {
    uint32_t framesOk;        /* 校验通过的帧数 */
    uint32_t crcErrors;       /* CRC 错误次数 */
    uint32_t resyncs;         /* 重新同步次数（说明有丢字节/干扰）*/
    uint32_t bytesDropped;    /* 因缓冲满或找不到帧头丢弃的字节数 */
};
const LinkStats &uartLinkStats(void);

/* ---- 诊断接口（排查"收不到数据"用）------------------------------------- */

/** RX 线上的**裸字节计数**（不要求成帧）。
 *  ⚠ 这个值和"收帧数"要分开看，因为故障方向完全不同：
 *     裸字节 = 0            ⇒ RX 线上没数据（接线 / 没共地）
 *     裸字节在涨但收帧 = 0  ⇒ 波特率不对 / 电平不匹配 / 干扰
 *     裸字节在涨且收帧在涨  ⇒ 链路正常 */
uint32_t uartLinkRawBytes(void);

/** 最近 1 秒收到的裸字节数。
 *  正常（F103 正在发送时）应接近 **11520 字节/秒**
 *  （115200 baud ÷ 10 bit/byte）。 */
uint32_t uartLinkLastSecondBytes(void);

/** 打开/关闭原始字节的十六进制打印（看线上到底是什么）。
 *  ⚠ 只在调试短时间打开 —— 每字节都打印会严重拖慢接收。 */
void uartLinkSetHexDump(bool on);

/* ---- 对照实验接口（排查"UART2 收不到"到底卡在哪一层）------------------- */

/** 让 UART1 也挂到同一对引脚（IO5/IO4）上一起监听。
 *  用途：区分"引脚上没信号"和"UART2 这条路有问题"（详见 .cpp 里的说明）。 */
void uartLinkAltBegin(void);
void uartLinkAltEnd(void);

/** 从对照 UART1 上收到的字节速率（每秒）。 */
uint32_t uartLinkAltBytes(void);

/** 必须每轮 loop 调用一次（在对照实验开启时才会真正读数据）。 */
void uartLinkPollAlt(void);

/** 当前主用哪个 UART（2 = UART2）。 */
uint8_t uartLinkActiveUart(void);

/** ESP32 侧 UART 硬件诊断：打印 RX/TX 引脚的**实际电平**和线路统计。
 *  UART 空闲态是高电平 ⇒ 若 RX 读到稳定高，说明线上挂着空闲的 TX（线通了）。
 *  这是判断"线到底接没接上"最直接的手段。 */
void uartLinkDiag(void);

/** UART 回环测试：把 IO(TX) 用一根线接到 IO(RX) 后运行，
 *  发一串数据再从 RX 读回，验证 ESP32 自己的 UART 收发通路。
 *  ⚠ 运行时先拔掉 F103 的线。函数内部会等 5 秒让你接线。 */
void uartLinkLoopbackTest(void);

#endif /* __UART_LINK_H */
