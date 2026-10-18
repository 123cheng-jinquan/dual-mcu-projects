/**
 * @file    modbus_crc.c
 * @brief   Modbus CRC16 实现（逐位版，无需 512 字节查表；F411 上单帧 <10µs）
 */
#include "modbus_crc.h"

uint16_t mb_crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFFu;

    if (buf == 0) {
        return crc;
    }
    while (len-- > 0u) {
        crc ^= (uint16_t)(*buf++);
        for (uint8_t i = 0u; i < 8u; i++) {
            if ((crc & 0x0001u) != 0u) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            } else {
                crc = (uint16_t)(crc >> 1);
            }
        }
    }
    return crc;
}

bool mb_crc_check_frame(const uint8_t *frame, uint16_t len)
{
    if ((frame == 0) || (len < 4u)) {
        return false;
    }
    /* 整帧残差为 0 即为正确——比"重算再逐字节比较"更省事也更不容易写反 */
    return (mb_crc16(frame, len) == 0x0000u);
}

uint16_t mb_crc_append(uint8_t *frame, uint16_t len, uint16_t cap)
{
    uint16_t crc;

    if ((frame == 0) || (len + 2u > cap)) {
        return len; /* 容量不足，原样返回；调用方应保证 cap 足够 */
    }
    crc = mb_crc16(frame, len);
    frame[len]     = mb_crc_lo(crc);
    frame[len + 1u] = mb_crc_hi(crc);
    return (uint16_t)(len + 2u);
}
