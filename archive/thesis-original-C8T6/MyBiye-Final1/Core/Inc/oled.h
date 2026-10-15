#ifndef __OLED_H__
#define __OLED_H__

#include "main.h"

void OLED_Init(void);
void OLED_Clear(void);
void OLED_SetCursor(uint8_t page, uint8_t column);
void OLED_ShowChar(uint8_t page, uint8_t column, char chr);
void OLED_ShowString(uint8_t page, uint8_t column, const char *str);
void OLED_ShowNum(uint8_t page, uint8_t column, uint32_t num, uint8_t len);
void OLED_Refresh(void);

#endif
