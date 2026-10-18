#ifndef __OLED_H__
#define __OLED_H__

#include "main.h"

/* ★ 把 I2C 从机地址暴露到头文件 ★
 * 为什么要暴露：诊断代码（main.c 的 I2C_Diag）需要在总线上**单独探测这个地址**，
 * 看 OLED 到底有没有应答。原来这个宏藏在 oled.c 里，外部看不到。
 * 注意 HAL 用的是**8 位**地址（0x78），对应 7 位地址 0x3C。 */
#define OLED_I2C_ADDR        0x78

void OLED_Init(void);
void OLED_Clear(void);
void OLED_SetCursor(uint8_t page, uint8_t column);
void OLED_ShowChar(uint8_t page, uint8_t column, char chr);
void OLED_ShowString(uint8_t page, uint8_t column, const char *str);
void OLED_ShowNum(uint8_t page, uint8_t column, uint32_t num, uint8_t len);
void OLED_Refresh(void);
void SSD1306_Reinit(void);   /* 显示看门狗：周期性重发开显示命令 */

/* ★ I2C 通信统计 ★
 * 用来诊断"屏幕黑了"到底是代码问题还是总线/供电问题：
 *   ErrCount 持续增长 ⇒ 总线在出错（供电跌落 / 线接触不良）
 *   ErrCount 为 0 但屏幕黑 ⇒ 数据发出去了但屏幕没显示（供电或屏幕坏）*/
uint32_t OLED_I2C_OkCount(void);
uint32_t OLED_I2C_ErrCount(void);

#endif
