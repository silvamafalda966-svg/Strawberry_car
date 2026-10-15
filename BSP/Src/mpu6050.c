/**
 * @file    mpu6050.c
 * @brief   MPU6050 六轴 IMU 驱动（GY-521, I2C）
 * @note    核心改进：
 *          1. FIFO 零偏校准 + 写入硬件偏移寄存器（芯片自动减）
 *          2. 自检（Self-Test）验证芯片健康
 *          3. 软件 EMA 低通滤波（抑制电机振动）
 *          4. 灵敏度修正系数（补偿个体差异 ±3%）
 *          5. DLPF 从 21Hz 提升到 42Hz（降低延迟）
 *
 * Reference:
 *   - Kris Winer MPU6050 library (calibrateMPU6050 / SelfTest)
 *   - empty-main 项目 (GYRO_SCALE_CORR / 静止检测)
 *   - InvenSense Register Map 4.2
 */

#include "mpu6050.h"
#include "cmsis_os.h"
#include <math.h>

/* ────────── 全局状态 ────────── */
static float g_gz_bias_dps = 0.0f;       /* 零偏 (°/s)，供外部参考 */
static float g_scale_corr   = GYRO_SCALE_CORR_DEFAULT;  /* 灵敏度修正 */
static float g_gz_filtered  = 0.0f;      /* EMA 低通滤波后的值 */

/* ────────── 内部 I2C 读写 ────────── */

static HAL_StatusTypeDef MPU6050_WriteReg(I2C_HandleTypeDef *hi2c,
                                           uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return HAL_I2C_Master_Transmit(hi2c, MPU6050_I2C_ADDR << 1, buf, 2, 3);
}

static HAL_StatusTypeDef MPU6050_ReadRegs(I2C_HandleTypeDef *hi2c,
                                           uint8_t reg, uint8_t *data, uint8_t len)
{
    return HAL_I2C_Mem_Read(hi2c, MPU6050_I2C_ADDR << 1, reg,
                            I2C_MEMADD_SIZE_8BIT, data, len, 3);
}

static HAL_StatusTypeDef MPU6050_ReadReg(I2C_HandleTypeDef *hi2c,
                                          uint8_t reg, uint8_t *val)
{
    return MPU6050_ReadRegs(hi2c, reg, val, 1);
}

/* ────────── 初始化 ────────── */

uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t whoami = 0;

    /* 1. 复位设备 */
    MPU6050_WriteReg(hi2c, MPU6050_REG_PWR_MGMT_1, 0x80);
    osDelay(100);

    /* 2. 唤醒 + 时钟源 = PLL with X gyro */
    MPU6050_WriteReg(hi2c, MPU6050_REG_PWR_MGMT_1, 0x01);
    MPU6050_WriteReg(hi2c, MPU6050_REG_PWR_MGMT_2, 0x00);

    /* 3. 检查 WHO_AM_I */
    if (MPU6050_ReadReg(hi2c, MPU6050_REG_WHO_AM_I, &whoami) != HAL_OK)
        return 0;
    if (whoami != 0x68)
        return 0;

    /* 4. DLPF=42Hz, 采样率=1kHz 内部 */
    MPU6050_WriteReg(hi2c, MPU6050_REG_CONFIG, DLPF_CFG_42HZ);

    /* 5. 采样率分频: 1kHz/(1+9)=100Hz */
    MPU6050_WriteReg(hi2c, MPU6050_REG_SMPLRT_DIV, 9);

    /* 6. 陀螺仪量程 ±250°/s（最大灵敏度） */
    MPU6050_WriteReg(hi2c, MPU6050_REG_GYRO_CONFIG, GYRO_FS_SEL_250);

    /* 7. 禁用中断 + FIFO（等校准时才开 FIFO） */
    MPU6050_WriteReg(hi2c, MPU6050_REG_FIFO_EN, 0x00);
    MPU6050_WriteReg(hi2c, MPU6050_REG_USER_CTRL, 0x00);

    /* 8. 初始化软件滤波器状态 */
    g_gz_filtered = 0.0f;

    return 1;
}

/* ────────── 自检 ────────── */

uint8_t MPU6050_SelfTest(I2C_HandleTypeDef *hi2c, float result[6])
{
    uint8_t raw[4], st[6];
    float ft[6];

    /* 启用自检：量程±250dps gyro, ±8g accel */
    MPU6050_WriteReg(hi2c, MPU6050_REG_GYRO_CONFIG,  0xE0);
    MPU6050_WriteReg(hi2c, MPU6050_REG_ACCEL_CONFIG, 0xF0);
    osDelay(250);  /* 等自检完成 */

    MPU6050_ReadReg(hi2c, MPU6050_REG_SELF_TEST_X, &raw[0]);
    MPU6050_ReadReg(hi2c, MPU6050_REG_SELF_TEST_Y, &raw[1]);
    MPU6050_ReadReg(hi2c, MPU6050_REG_SELF_TEST_Z, &raw[2]);
    MPU6050_ReadReg(hi2c, MPU6050_REG_SELF_TEST_A, &raw[3]);

    /* 解析加速度计自检值 (5 bits each) */
    st[0] = (raw[0] >> 3) | ((raw[3] & 0x30) >> 4);
    st[1] = (raw[1] >> 3) | ((raw[3] & 0x0C) >> 2);
    st[2] = (raw[2] >> 3) | ((raw[3] & 0x03) >> 0);

    /* 解析陀螺仪自检值 (5 bits each) */
    st[3] = raw[0] & 0x1F;
    st[4] = raw[1] & 0x1F;
    st[5] = raw[2] & 0x1F;

    /* 计算加速度计出厂 trim 值 */
    for (int i = 0; i < 3; i++) {
        if (st[i] == 0)
            ft[i] = 0.0f;
        else
            ft[i] = 4096.0f * 0.34f * powf(0.92f / 0.34f,
                     ((float)st[i] - 1.0f) / 30.0f);
    }

    /* 计算陀螺仪出厂 trim 值 (X/Z=+25, Y=-25) */
    ft[3] =  25.0f * 131.0f * powf(1.046f, (float)st[3] - 1.0f);
    ft[4] = -25.0f * 131.0f * powf(1.046f, (float)st[4] - 1.0f);
    ft[5] =  25.0f * 131.0f * powf(1.046f, (float)st[5] - 1.0f);

    /* 偏差百分比 = 100 + 100*(st - ft)/ft */
    uint8_t pass = 1;
    for (int i = 0; i < 6; i++) {
        if (ft[i] != 0.0f) {
            result[i] = 100.0f + 100.0f * ((float)st[i] - ft[i]) / ft[i];
        } else {
            result[i] = 0.0f;
        }
        /* ±14% 以内为通过 */
        if (result[i] < 86.0f || result[i] > 114.0f) pass = 0;
    }

    /* 恢复陀螺仪量程 */
    MPU6050_WriteReg(hi2c, MPU6050_REG_GYRO_CONFIG,  GYRO_FS_SEL_250);
    MPU6050_WriteReg(hi2c, MPU6050_REG_ACCEL_CONFIG, 0x00);

    return pass;
}

/* ────────── FIFO 零偏校准 ────────── */

/**
 * @brief  用 FIFO 模式采集零偏样本并写入硬件偏移寄存器
 * @note   参考 Kris Winer calibrateMPU6050():
 *         1. 临时设 DLPF=188Hz, 采样率=1kHz
 *         2. 开 FIFO，连续采集 ~80ms
 *         3. 读取 FIFO 取平均
 *         4. 写入硬件偏移寄存器（格式: 32.9 LSB/dps, 取反）
 *         5. 恢复 DLPF=42Hz, 采样率=100Hz
 *
 *         硬件偏移寄存器:
 *           XG_OFFS_USR (0x13-0x14), YG_OFFS_USR (0x15-0x16),
 *           ZG_OFFS_USR (0x17-0x18)
 *           分辨率: 32.9 LSB/(°/s) → 原始偏置 ÷ 4
 *           寄存器的偏置值会被芯片加到输出上，所以要取反
 */
float MPU6050_Calibrate(I2C_HandleTypeDef *hi2c, uint16_t samples)
{
    uint8_t  data[12];
    int32_t  gyro_bias[3] = {0, 0, 0};
    uint16_t fifo_count, packet_count;

    /* ── 1. 临时配置：高采样率 + 高带宽 ── */
    MPU6050_WriteReg(hi2c, MPU6050_REG_PWR_MGMT_1, 0x01);
    MPU6050_WriteReg(hi2c, MPU6050_REG_PWR_MGMT_2, 0x00);
    osDelay(200);  /* 等 PLL 稳定 */

    MPU6050_WriteReg(hi2c, MPU6050_REG_FIFO_EN, 0x00);   /* 关 FIFO */
    MPU6050_WriteReg(hi2c, MPU6050_REG_USER_CTRL, 0x00); /* 关 I2C master */
    MPU6050_WriteReg(hi2c, MPU6050_REG_USER_CTRL, 0x0C); /* 复位 FIFO + DMP */
    osDelay(15);

    MPU6050_WriteReg(hi2c, MPU6050_REG_CONFIG, DLPF_CFG_188HZ);    /* 188Hz */
    MPU6050_WriteReg(hi2c, MPU6050_REG_SMPLRT_DIV, 0x00);          /* 1kHz */
    MPU6050_WriteReg(hi2c, MPU6050_REG_GYRO_CONFIG, GYRO_FS_SEL_250);

    /* ── 2. 开 FIFO，收集样本 ── */
    MPU6050_WriteReg(hi2c, MPU6050_REG_USER_CTRL, 0x40);  /* 使能 FIFO */
    MPU6050_WriteReg(hi2c, MPU6050_REG_FIFO_EN, 0x78);    /* gyro+accel → FIFO */
    osDelay(80);  /* 80ms = 约 80 组完整样本 (gyro+accel 各 3 轴 = 12 字节/组) */

    /* ── 3. 关 FIFO，读取 ── */
    MPU6050_WriteReg(hi2c, MPU6050_REG_FIFO_EN, 0x00);

    uint8_t count_buf[2];
    MPU6050_ReadRegs(hi2c, MPU6050_REG_FIFO_COUNTH, count_buf, 2);
    fifo_count = ((uint16_t)count_buf[0] << 8) | count_buf[1];
    packet_count = fifo_count / 12;  /* 每组 12 字节 */

    if (packet_count > samples) packet_count = samples;
    if (packet_count == 0) {
        /* fallback: 用旧方法 */
        goto fallback_cal;
    }

    for (uint16_t i = 0; i < packet_count; i++) {
        int16_t gyro_temp[3];
        MPU6050_ReadRegs(hi2c, MPU6050_REG_FIFO_R_W, data, 12);
        gyro_temp[0] = (int16_t)(((uint16_t)data[6]  << 8) | data[7]);
        gyro_temp[1] = (int16_t)(((uint16_t)data[8]  << 8) | data[9]);
        gyro_temp[2] = (int16_t)(((uint16_t)data[10] << 8) | data[11]);

        gyro_bias[0] += (int32_t)gyro_temp[0];
        gyro_bias[1] += (int32_t)gyro_temp[1];
        gyro_bias[2] += (int32_t)gyro_temp[2];
    }

    gyro_bias[0] /= (int32_t)packet_count;
    gyro_bias[1] /= (int32_t)packet_count;
    gyro_bias[2] /= (int32_t)packet_count;

    /* ── 4. 写入硬件偏移寄存器 ──
     * 格式: 32.9 LSB/dps → 原值 ÷ 4
     * 寄存器是加性的 → 取反
     */
    data[0] = (uint8_t)((-gyro_bias[0] / 4) >> 8) & 0xFF;
    data[1] = (uint8_t)((-gyro_bias[0] / 4)       & 0xFF);
    data[2] = (uint8_t)((-gyro_bias[1] / 4) >> 8) & 0xFF;
    data[3] = (uint8_t)((-gyro_bias[1] / 4)       & 0xFF);
    data[4] = (uint8_t)((-gyro_bias[2] / 4) >> 8) & 0xFF;
    data[5] = (uint8_t)((-gyro_bias[2] / 4)       & 0xFF);

    MPU6050_WriteReg(hi2c, MPU6050_REG_XG_OFFS_USRH, data[0]);
    MPU6050_WriteReg(hi2c, MPU6050_REG_XG_OFFS_USRL, data[1]);
    MPU6050_WriteReg(hi2c, MPU6050_REG_YG_OFFS_USRH, data[2]);
    MPU6050_WriteReg(hi2c, MPU6050_REG_YG_OFFS_USRL, data[3]);
    MPU6050_WriteReg(hi2c, MPU6050_REG_ZG_OFFS_USRH, data[4]);
    MPU6050_WriteReg(hi2c, MPU6050_REG_ZG_OFFS_USRL, data[5]);

    g_gz_bias_dps = (float)gyro_bias[2] / GYRO_LSB_PER_DPS;

    /* ── 5. 恢复运行参数 ── */
    MPU6050_WriteReg(hi2c, MPU6050_REG_CONFIG, DLPF_CFG_42HZ);
    MPU6050_WriteReg(hi2c, MPU6050_REG_SMPLRT_DIV, 9);  /* 100Hz */
    MPU6050_WriteReg(hi2c, MPU6050_REG_FIFO_EN, 0x00);
    MPU6050_WriteReg(hi2c, MPU6050_REG_USER_CTRL, 0x00);

    /* 重置软件滤波器 */
    g_gz_filtered = 0.0f;

    return g_gz_bias_dps;

fallback_cal:
    /* FIFO 失败 → 回退到简单平均 */
    MPU6050_WriteReg(hi2c, MPU6050_REG_CONFIG, DLPF_CFG_42HZ);
    MPU6050_WriteReg(hi2c, MPU6050_REG_SMPLRT_DIV, 9);
    MPU6050_WriteReg(hi2c, MPU6050_REG_FIFO_EN, 0x00);
    MPU6050_WriteReg(hi2c, MPU6050_REG_USER_CTRL, 0x00);

    float sum = 0.0f;
    for (uint16_t i = 0; i < 20; i++) {
        uint8_t buf[2];
        MPU6050_ReadRegs(hi2c, MPU6050_REG_GYRO_ZOUT_H, buf, 2);
        osDelay(2);
    }
    for (uint16_t i = 0; i < 500; i++) {
        uint8_t buf[2];
        MPU6050_ReadRegs(hi2c, MPU6050_REG_GYRO_ZOUT_H, buf, 2);
        int16_t raw = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
        sum += (float)raw / GYRO_LSB_PER_DPS;
        osDelay(2);
    }
    g_gz_bias_dps = sum / 500.0f;

    /* 同样尝试写入硬件寄存器 */
    int16_t bias_lsb = (int16_t)(g_gz_bias_dps * GYRO_LSB_PER_DPS);
    data[4] = (uint8_t)((-bias_lsb / 4) >> 8) & 0xFF;
    data[5] = (uint8_t)((-bias_lsb / 4)       & 0xFF);
    MPU6050_WriteReg(hi2c, MPU6050_REG_ZG_OFFS_USRH, data[4]);
    MPU6050_WriteReg(hi2c, MPU6050_REG_ZG_OFFS_USRL, data[5]);

    g_gz_filtered = 0.0f;
    return g_gz_bias_dps;
}

/* ────────── 读取（已减硬件零偏 + 软件低通 + 灵敏度修正）── */

float MPU6050_ReadGyroZ(I2C_HandleTypeDef *hi2c, uint8_t *ok)
{
    uint8_t buf[2];
    if (MPU6050_ReadRegs(hi2c, MPU6050_REG_GYRO_ZOUT_H, buf, 2) != HAL_OK) {
        if (ok) *ok = 0;
        return 0.0f;
    }
    if (ok) *ok = 1;

    int16_t raw = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);

    /* 硬件偏移寄存器已经自动减零偏 → raw 就是零偏校准后的值 */
    float gz = (float)raw / GYRO_LSB_PER_DPS;

    /* 灵敏度修正（补偿芯片个体差异 ±3%） */
    gz *= g_scale_corr;

    /* 软件 EMA 低通滤波（抑制电机振动） */
    g_gz_filtered += GYRO_LPF_ALPHA * (gz - g_gz_filtered);

    return g_gz_filtered;
}

/* ────────── 灵敏度修正 ────────── */

void MPU6050_SetScaleCorr(float corr)
{
    g_scale_corr = corr;
}

/* ────────── 温度 ────────── */

float MPU6050_ReadTemp(I2C_HandleTypeDef *hi2c)
{
    uint8_t buf[2];
    if (MPU6050_ReadRegs(hi2c, 0x41, buf, 2) != HAL_OK)  /* TEMP_OUT_H */
        return 0.0f;
    int16_t raw = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    return 36.53f + (float)raw / 340.0f;
}
