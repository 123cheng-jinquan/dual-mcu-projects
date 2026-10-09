/**
 * @file    modbus_crc.h
 * @brief   Modbus RTU CRC16（多项式 0xA001，反射）——平台无关实现
 *
 * 关键约定：CRC 在总线上**先发低字节，再发高字节**。
 * 自检技巧：把整帧（含收到的 2 字节 CRC）送进 mb_crc16()，结果应为 0x0000。
 */
#ifndef MODBUS_CRC_H
#define MODBUS_CRC_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 计算 Modbus CRC16（初值 0xFFFF，多项式 0xA001 反射） */
uint16_t mb_crc16(const uint8_t *buf, uint16_t len);

/** 校验一整帧（含末尾 2 字节 CRC）。长度至少 4 字节；返回 true 表示 CRC 正确。 */
bool mb_crc_check_frame(const uint8_t *frame, uint16_t len);

/** CRC 低字节（先发） */
static inline uint8_t mb_crc_lo(uint16_t crc) { return (uint8_t)(crc & 0xFFu); }
/** CRC 高字节（后发） */
static inline uint8_t mb_crc_hi(uint16_t crc) { return (uint8_t)((crc >> 8) & 0xFFu); }

/** 把 CRC 追加到帧尾（先低后高），返回追加后的总长度 */
uint16_t mb_crc_append(uint8_t *frame, uint16_t len, uint16_t cap);

#ifdef __cplusplus
}
#endif
#endif /* MODBUS_CRC_H */
