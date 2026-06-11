#ifndef __OLED_H
#define __OLED_H

#include "main.h"
#include <string.h>

/* ────────── I2C 地址 ────────── */
#define OLED_I2C_ADDR  0x78    /* 0x3C << 1 (8-bit HAL format) */
#define OLED_CMD       0x00    /* 写命令前缀 */
#define OLED_DATA      0x40    /* 写数据前缀 */

/* ────────── API ────────── */
void OLED_Init(void);
uint8_t OLED_IsOK(void);   /* 返回 1=OLED 正常, 0=故障 */
void OLED_Clear(void);
void OLED_ClearBuffer(void);   /* 只清缓冲不刷屏 */
void OLED_Refresh(void);
void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);

/* 绘图 */
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t);

/* 文字显示 (ASCII) */
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size1, uint8_t mode);
void OLED_ShowString(uint8_t x, uint8_t y, const uint8_t *chr, uint8_t size1, uint8_t mode);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size1, uint8_t mode);

/* 中文显示 (16×16) */
void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t index);

/* 浮点数显示 (2 位小数) */
void OLED_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t size1, uint8_t mode);

#endif /* __OLED_H */
