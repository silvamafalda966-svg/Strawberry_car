/**
 * @file    oled.c
 * @brief   SSD1315 0.96" OLED 驱动 (I2C2, HAL 硬件 I2C)
 * @note    基于厂家参考代码改写：bit-bang → HAL I2C
 *          I2C 地址: 0x78 (0x3C<<1), PB10(SCL)/PB11(SDA)
 */

#define OLEDFONT_DEFINE  /* 定义字库数据（非 extern 声明） */
#include "oled.h"
#include "oledfont.h"
#include "i2c.h"
#include "cmsis_os.h"

/* ────────── 显存帧缓冲 (page-major: 8 pages × 128 columns) ────────── */
static uint8_t OLED_GRAM[8][128];
static uint8_t oled_ok = 0;  /* I2C 通信状态: 1=正常, 0=故障 */

/* ────────── 写命令（带超时保护） ────────── */
static HAL_StatusTypeDef OLED_WriteCmd(uint8_t cmd)
{
    uint8_t buf[2] = {OLED_CMD, cmd};
    return HAL_I2C_Master_Transmit(&hi2c2, OLED_I2C_ADDR, buf, 2, 5);
}

/* ────────── 获取 OLED 状态 ────────── */
uint8_t OLED_IsOK(void) { return oled_ok; }

/* ────────── 开关显示 ────────── */
void OLED_DisPlay_On(void)
{
    OLED_WriteCmd(0x8D);  /* 电荷泵使能 */
    OLED_WriteCmd(0x14);  /* 打开电荷泵 */
    OLED_WriteCmd(0xAF);  /* 打开屏幕 */
}

void OLED_DisPlay_Off(void)
{
    OLED_WriteCmd(0x8D);
    OLED_WriteCmd(0x10);  /* 关闭电荷泵 */
    OLED_WriteCmd(0xAE);  /* 关闭屏幕 */
}

/* ────────── 刷新显存到屏幕 ────────── */
void OLED_Refresh(void)
{
    if (!oled_ok) return;  /* OLED 故障，跳过 I2C */

    HAL_StatusTypeDef rc;
    for (uint8_t page = 0; page < 8; page++)
    {
        rc = OLED_WriteCmd(0xB0 + page);
        if (rc != HAL_OK) goto i2c_err;
        rc = OLED_WriteCmd(0x00);
        if (rc != HAL_OK) goto i2c_err;
        rc = OLED_WriteCmd(0x10);
        if (rc != HAL_OK) goto i2c_err;

        uint8_t buf[129];
        buf[0] = OLED_DATA;
        memcpy(buf + 1, OLED_GRAM[page], 128);
        rc = HAL_I2C_Master_Transmit(&hi2c2, OLED_I2C_ADDR, buf, 129, 50);
        if (rc != HAL_OK) goto i2c_err;
    }
    return;

i2c_err:
    oled_ok = 0;  /* 标记故障，后续周期跳过 I2C */
}

/* ────────── 清缓冲（不刷屏） ────────── */
void OLED_ClearBuffer(void)
{
    memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
}

/* ────────── 清屏（清缓冲+刷屏） ────────── */
void OLED_Clear(void)
{
    memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
    OLED_Refresh();
}

/* ────────── 画点 ────────── */
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t)
{
    if (x >= 128 || y >= 64) return;
    uint8_t page = y / 8;
    uint8_t bit  = y % 8;
    if (t)  OLED_GRAM[page][x] |=  (1 << bit);
    else    OLED_GRAM[page][x] &= ~(1 << bit);
}

/* ────────── 显示 ASCII 字符 ────────── */
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size1, uint8_t mode)
{
    if (x >= 128 || y >= 64) return;
    if (chr < ' ' || chr > '~') chr = ' ';  /* 非ASCII显示空格 */

    uint8_t x0 = x, y0 = y;
    uint8_t chr1 = chr - ' ';  /* 偏移到字库索引 */

    uint8_t cols, rows;
    const uint8_t *pData = NULL;

    if (size1 == 8) {
        cols = 6; rows = 1;  /* 6×8 单页 */
        pData = asc2_0806[chr1];
    } else {  /* size1 == 16 */
        cols = 8; rows = 2;  /* 8×16 双页 */
        pData = asc2_1608[chr1];
    }

    for (uint8_t i = 0; i < cols * rows; i++) {
        uint8_t temp = pData[i];
        for (uint8_t m = 0; m < 8; m++) {
            if (temp & 0x01) OLED_DrawPoint(x, y, mode);
            else             OLED_DrawPoint(x, y, !mode);
            temp >>= 1;
            y++;
        }
        x++;
        if (size1 != 8 && (x - x0) == cols) {
            x = x0;
            y0 += 8;
        }
        y = y0;
    }
}

/* ────────── 显示字符串 ────────── */
void OLED_ShowString(uint8_t x, uint8_t y, const uint8_t *chr, uint8_t size1, uint8_t mode)
{
    while (*chr) {
        if (*chr >= ' ' && *chr <= '~') {
            OLED_ShowChar(x, y, *chr, size1, mode);
            x += (size1 == 8) ? 6 : (size1 / 2);
        }
        if (x >= 128) break;
        chr++;
    }
}

/* ────────── 显示数字 ────────── */
static uint32_t OLED_Pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--) result *= m;
    return result;
}

void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size1, uint8_t mode)
{
    uint8_t m = (size1 == 8) ? 2 : 0;
    for (uint8_t t = 0; t < len; t++) {
        uint8_t temp = (num / OLED_Pow(10, len - t - 1)) % 10;
        uint8_t step = (size1 == 8) ? 6 : (size1 / 2);
        OLED_ShowChar(x + (step + m) * t, y, temp + '0', size1, mode);
    }
}

/* ────────── 显示浮点数 (保留 2 位小数) ────────── */
void OLED_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t size1, uint8_t mode)
{
    uint8_t step = (size1 == 8) ? 6 : (size1 / 2);
    /* 处理符号 */
    float abs_num = num;
    if (num < 0.0f) {
        abs_num = -num;
        OLED_ShowChar(x, y, '-', size1, mode);
        x += step;
    }
    /* 整数部分 */
    int32_t int_part = (int32_t)abs_num;
    if (int_part >= 100) {
        OLED_ShowChar(x, y, '0' + (int_part / 100) % 10, size1, mode); x += step;
    }
    if (int_part >= 10) {
        OLED_ShowChar(x, y, '0' + (int_part / 10) % 10, size1, mode); x += step;
    }
    OLED_ShowChar(x, y, '0' + (int_part % 10), size1, mode); x += step;
    OLED_ShowChar(x, y, '.', size1, mode); x += step;
    /* 小数部分 */
    float frac_part = abs_num - (float)int_part;
    uint32_t frac = (uint32_t)(frac_part * 100.0f + 0.5f);
    if (frac >= 100) frac = 99;
    OLED_ShowChar(x, y, '0' + (frac / 10) % 10, size1, mode); x += step;
    OLED_ShowChar(x, y, '0' + (frac % 10), size1, mode);
}

/* ────────── 显示中文 (16×16, 1 像素反色) ────────── */
void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t index)
{
    if (x >= 128 || y >= 64) return;
    if (index >= HZK16_COUNT) return;

    uint8_t x0 = x, y0 = y;
    const uint8_t *pData = Hzk16[index];

    for (uint8_t i = 0; i < 32; i++) {
        uint8_t temp = pData[i];
        for (uint8_t m = 0; m < 8; m++) {
            if (temp & 0x01) OLED_DrawPoint(x, y, 1);
            else             OLED_DrawPoint(x, y, 0);
            temp >>= 1;
            y++;
        }
        x++;
        if ((x - x0) == 16) {
            x = x0;
            y0 += 8;
        }
        y = y0;
    }
}

/* ────────── 初始化 ────────── */
void OLED_Init(void)
{
    oled_ok = 0;
    osDelay(50);

    HAL_StatusTypeDef rc;

    /* 初始化序列 (SSD1315, 兼容 SSD1306) */
    #define CHECK(cmd) do { rc = OLED_WriteCmd(cmd); if (rc != HAL_OK) return; } while(0)

    CHECK(0xAE);  /* 关闭显示 */
    CHECK(0x00);  /* 低列地址 */
    CHECK(0x10);  /* 高列地址 */
    CHECK(0x40);  /* 起始行地址 */
    CHECK(0x81);  /* 对比度 */
    CHECK(0xCF);
    CHECK(0xA1);  /* 段重映射 */
    CHECK(0xC8);  /* COM 扫描方向 */
    CHECK(0xA6);  /* 正常显示 */
    CHECK(0xA8);  /* 多路复用 */
    CHECK(0x3F);  /* 1/64 duty */
    CHECK(0xD3);  /* 显示偏移 */
    CHECK(0x00);
    CHECK(0xD5);  /* 时钟分频 */
    CHECK(0x80);
    CHECK(0xD9);  /* 预充电周期 */
    CHECK(0xF1);
    CHECK(0xDA);  /* COM 引脚配置 */
    CHECK(0x12);
    CHECK(0xDB);  /* VCOM 反压 */
    CHECK(0x40);
    CHECK(0x20);  /* 页寻址模式 */
    CHECK(0x02);
    CHECK(0x8D);  /* 电荷泵 */
    CHECK(0x14);  /* 启用 */
    CHECK(0xA4);  /* 全屏点亮: 关闭 */
    CHECK(0xA6);  /* 反色: 关闭 */

    #undef CHECK

    oled_ok = 1;  /* 初始化全部成功 */
    OLED_Clear();
    OLED_WriteCmd(0xAF);  /* 打开显示 */
}
