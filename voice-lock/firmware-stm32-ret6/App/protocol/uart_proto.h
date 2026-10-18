/* =============================================================================
 *  uart_proto.h — F103RET6 ↔ ESP32-S3 双片通信协议
 * =============================================================================
 *  为什么需要自定义协议（不用现成的 AT/Modbus）：
 *    这里传的是**音频数据**（1 秒 8kHz = 8000 点），对"完整性"要求很高：
 *    错一个字节就可能让 MFCC 特征算偏、推理结果翻转。所以必须有
 *    **帧同步 + 长度 + CRC 校验**，而且要能检测丢帧。
 *
 *  和项目1（Modbus 振动监测）的关系：
 *    两个项目都是"节点采集 + 网关处理"的两片架构，但总线层不同：
 *      · 项目1 用 **标准 Modbus RTU**（因为对接的是工业从机，必须标准）
 *      · 项目2 用 **自定义二进制帧**（因为传的是音频流，Modbus 寄存器模型不合适）
 *    面试时这是很好的对照点：**什么时候该用标准协议、什么时候该自定义**。
 *
 *  帧格式（小端）
 *  ---------------------------------------------------------------------------
 *    偏移  长度  字段      说明
 *    0     1     SYNC      固定 0xAA（帧头，用于乱码后重新对齐）
 *    1     1     CMD       命令字（见下面 UART_CMD_*）
 *    2     1     SEQ       帧序号，0~255 循环（用于丢帧检测）
 *    3     2     LEN       PAYLOAD 长度（小端，0 ~ UART_MAX_PAYLOAD）
 *    5     N     PAYLOAD   数据
 *    5+N   2     CRC16     CRC16-Modbus（覆盖 CMD~PAYLOAD，即 offset 1..5+N-1）
 *
 *  总长度 = 5 + LEN + 2 = LEN + 7
 *
 *  ⚠ CRC 覆盖范围为什么**不含 SYNC**：
 *    SYNC 只是"找帧头"的标记，如果它参与 CRC，一旦 SYNC 被干扰破坏，
 *    接收方会因为"CRC 不过"而丢弃整帧 —— 但其实换个位置还能重新同步。
 *    不含 SYNC 时，接收方可以"滑动窗口"式地重找帧头，鲁棒性更好。
 * ========================================================================== */

#ifndef __UART_PROTO_H
#define __UART_PROTO_H

#include <stdint.h>
#include <stdbool.h>

/* ---- 帧结构常量 -------------------------------------------------------- */
#define UART_SYNC_BYTE       0xAAU
#define UART_HEADER_LEN      5U      /* SYNC + CMD + SEQ + LEN(2) */
#define UART_CRC_LEN         2U
#define UART_OVERHEAD        (UART_HEADER_LEN + UART_CRC_LEN)   /* = 7 */
#define UART_MAX_PAYLOAD     1024U   /* 单帧最大载荷（够放 512 个 16bit 样本）*/
#define UART_MAX_FRAME       (UART_MAX_PAYLOAD + UART_OVERHEAD)

/* ---- 命令字 ------------------------------------------------------------ */
#define UART_CMD_AUDIO_META  0x02U   /* F103→ESP32: 音频元信息（采样率/点数）*/
#define UART_CMD_AUDIO_DATA  0x01U   /* F103→ESP32: 音频数据块 */
#define UART_CMD_RESULT      0x10U   /* ESP32→F103: 判定结果 */
#define UART_CMD_STATUS      0x11U   /* ESP32→F103: 状态/错误码 */
#define UART_CMD_PING        0x20U   /* 双向: 心跳 */

/* ---- 判定结果 ---------------------------------------------------------- */
#define UART_VERDICT_FAIL    0x00U
#define UART_VERDICT_PASS    0x01U

/* ---- 音频元信息的载荷布局（UART_CMD_AUDIO_META）------------------------
 *   偏移 长度 字段
 *   0    4    采样率 (Hz, 小端)
 *   4    4    总样本数 (小端)
 *   8    2    总块数   (小端)  —— 接收方可据此判断有没有丢块
 *   10   1    位深 (16 = uint16)
 *   11   1    保留
 * 共 12 字节。 */
#define UART_META_LEN        12U

/* ---- 判定结果的载荷布局（UART_CMD_RESULT）------------------------------
 *   偏移 长度 字段
 *   0    1    判定 (UART_VERDICT_*)
 *   1    1    置信度 (0~100)
 *   2    2    相似度 (0~1000, 千分比, 小端)
 *   4    4    推理耗时 (ms, 小端)
 * 共 8 字节。 */
#define UART_RESULT_LEN      8U

/* ---- API --------------------------------------------------------------- */

/** 计算 CRC16-Modbus（多项式 0xA001，初值 0xFFFF）。
 *  和项目1 的 Modbus CRC 是同一个算法 —— 可以直接复用验证过的实现。 */
uint16_t uart_crc16(const uint8_t *data, uint32_t len);

/** 组一帧到 out 缓冲。返回整帧长度（LEN+7），失败返回 0。
 *  out 缓冲必须 >= (payload_len + UART_OVERHEAD)。 */
uint32_t uart_build_frame(uint8_t *out, uint8_t cmd, uint8_t seq,
                          const uint8_t *payload, uint16_t payload_len);

/** 当前发送序号（每次 build 后自增，方便调用方不关心）*/
uint8_t uart_next_seq(void);

/** 把 16 位样本数组按小端写入字节缓冲（用于音频块打包）。
 *  返回写出的字节数（= count * 2）。 */
uint32_t uart_pack_samples(uint8_t *out, const uint16_t *samples, uint32_t count);

/* =============================================================================
 *  接收侧：解析 ESP32 回传的判定结果帧
 * =============================================================================
 *  为什么用"逐字节喂入的状态机"而不是"等一整帧再解析"：
 *    F103 这边接收是**中断驱动、一个字节一个字节来的**
 *    （HAL_UART_Receive_IT 每次收 1 字节）。如果去"等整帧"，
 *    就要在中断里凑缓冲、还要处理超时，容易写乱。
 *    状态机方式：中断里把每个字节喂进来，凑齐一帧就置个标志，
 *    主循环去查标志 —— 分工清晰，也不会阻塞。
 *
 *  用法：
 *      // 在 HAL_UART_RxCpltCallback(USART3) 里：
 *      uartRxFeed(byte);
 *
 *      // 在主循环里：
 *      if (uartResultReady()) { uartGetResult(&r); ... }
 * ========================================================================== */

/** 判定结果（解析出来之后）*/
typedef struct {
    uint8_t  verdict;      /* 0 = FAIL, 1 = PASS */
    uint8_t  confidence;   /* 0~100 */
    uint16_t similarity;   /* 0~1000（千分比）*/
    uint32_t elapsedMs;    /* ESP32 侧推理耗时 */
    uint8_t  valid;        /* 1 = 这一帧校验通过 */
} UartResult;

/** 喂一个接收到的字节进解析状态机（在串口中断回调里调用，必须快）。 */
void uartRxFeed(uint8_t b);

/** 是否已解析出一个完整的结果帧（主循环轮询用）。 */
uint8_t uartResultReady(void);

/** 取出结果并清除"就绪"标志。 */
void uartGetResult(UartResult *out);

/** 取接收统计（排查用）：收到的帧数 / CRC 错误数 / 重同步次数 */
void uartRxStats(uint32_t *frames, uint32_t *crcErr, uint32_t *resync);

#endif /* __UART_PROTO_H */
