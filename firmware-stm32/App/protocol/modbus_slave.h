/**
 * @file    modbus_slave.h
 * @brief   手写 Modbus RTU 从机协议栈（纯逻辑，不依赖 HAL / FreeRTOS）
 *
 * 设计意图：把"协议"与"硬件"彻底分开。
 *   - 本模块只做：帧校验 → 地址过滤 → 功能码分派 → 寄存器区检查 → 组帧（含 CRC）
 *   - 寄存器存取通过 mb_slave_t 里的函数指针注入，应用层决定数据放在哪、如何加临界区
 * 好处：整个协议栈可以在 PC 上用 gcc 直接编译并跑单元测试（见 test/test_modbus.c），
 *       不必烧板子、不必接 485 就能把边界与异常路径全部覆盖。
 *
 * 规格来源：docs/02-modbus-protocol.md
 */
#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 协议常量 ---------------------------------------------------------- */
#define MB_ADDR_BROADCAST   0x00u   /**< 广播地址：执行但不响应 */
#define MB_ADDR_MIN         0x01u
#define MB_ADDR_MAX         0xF7u   /**< 247 */

#define MB_MAX_ADU          256u    /**< RTU 最大 ADU */
#define MB_MAX_READ_REGS    125u    /**< FC03/04 单次读取上限（0x7D） */
#define MB_MAX_WRITE_REGS   123u    /**< FC10 单次写入上限（0x7B） */

#define MB_FC_READ_HOLDING  0x03u
#define MB_FC_READ_INPUT    0x04u
#define MB_FC_WRITE_SINGLE  0x06u
#define MB_FC_WRITE_MULTI   0x10u

/** 异常码 */
typedef enum {
    MB_EXC_NONE  = 0x00,   /**< 无异常 */
    MB_EXC_FUNC  = 0x01,   /**< 非法功能 */
    MB_EXC_ADDR  = 0x02,   /**< 非法数据地址 */
    MB_EXC_VALUE = 0x03    /**< 非法数据值 */
} mb_exc_t;

/** 寄存器区描述（连续的一段逻辑地址） */
typedef struct {
    uint16_t start;      /**< 起始地址 */
    uint16_t count;      /**< 寄存器个数 */
    bool     writable;   /**< 是否可写（FC06/FC10） */
} mb_area_t;

/** 从机实例：地址 + 区表 + 三个应用层钩子 */
typedef struct {
    uint16_t         slave_addr;   /**< 本机从机地址 */
    const mb_area_t *areas;        /**< 区表 */
    uint8_t          area_count;   /**< 区表项数 */

    /**
     * @brief 读一个寄存器（应用层实现，返回该地址当前值的快照）
     * @note  对 D 区（采样窗口）应返回"当前有效缓冲"的值，不要边采边读同一块内存
     */
    uint16_t (*get)(uint16_t addr);

    /**
     * @brief 写一个寄存器（应用层实现）
     * @return 异常码；返回 MB_EXC_NONE 表示写入成功
     * @note   建议实现为"先校验后写入"，失败不留副作用
     */
    mb_exc_t (*set)(uint16_t addr, uint16_t value);

    /**
     * @brief 写前置校验（可选，可为 NULL）。FC10 会先对全部写入点调用一遍，
     *        避免"写到一半失败"造成的部分写入。
     */
    mb_exc_t (*check)(uint16_t addr, uint16_t value);

    /**
     * @brief 块读通知（可选）。用于 D 区"读取期间锁定窗口缓冲"，见 docs/02 §7。
     * @param start 起始地址  @param count 寄存器个数
     */
    void (*on_block_read)(uint16_t start, uint16_t count);
} mb_slave_t;

/** 从机统计（可直接映射到 B 区 0x0013~0x0015） */
typedef struct {
    uint32_t ok_frames;    /**< 正常响应帧数 */
    uint32_t crc_errors;   /**< CRC 错帧数（静默丢弃） */
    uint32_t exc_frames;   /**< 返回异常码的帧数 */
    uint32_t silent_drops; /**< 地址不匹配 / 长度非法（静默丢弃） */
} mb_stats_t;

extern mb_stats_t g_mb_stats;

/**
 * @brief 处理一帧请求，生成响应
 * @param s         从机实例
 * @param req       收到的完整请求帧（含 2 字节 CRC）
 * @param req_len   请求帧长度
 * @param resp      响应缓冲区
 * @param resp_cap  响应缓冲区容量（建议 >= MB_MAX_ADU）
 * @return 响应帧长度；**返回 0 表示本帧不应答**（CRC 错 / 地址不匹配 / 广播 / 长度非法）
 *
 * 注意：返回 0 时调用方**不要**发送任何字节，也不要翻转 DE——多从机总线上
 *       错误地址应答会造成总线冲突。
 */
uint16_t mb_slave_process(const mb_slave_t *s,
                          const uint8_t *req, uint16_t req_len,
                          uint8_t *resp, uint16_t resp_cap);

/** 清空统计（对应控制字 0x0020 的 bit6） */
void mb_slave_clear_stats(void);

#ifdef __cplusplus
}
#endif
#endif /* MODBUS_SLAVE_H */
