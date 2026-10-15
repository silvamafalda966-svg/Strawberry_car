/**
 * @file    mpu6050.h
 * @brief   MPU6050 6-axis IMU driver (GY-521, I2C)
 * @note    Supports hardware offset registers, FIFO calibration, self-test,
 *          software low-pass filter, and sensitivity correction.
 *          Shares I2C bus with OLED (different address: 0x68 vs 0x78).
 *
 * Reference: Kris Winer MPU6050 library, InvenSense register map 4.2
 */

#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"
#include "i2c.h"

/* ────────── I2C 地址 ────────── */
#define MPU6050_I2C_ADDR    0x68   /* AD0=GND → 7-bit addr 0x68 */

/* ────────── 寄存器地址 ────────── */
#define MPU6050_REG_SELF_TEST_X    0x0D   /* 自检 X: bits 7-5=XA_TEST, 4-0=XG_TEST */
#define MPU6050_REG_SELF_TEST_Y    0x0E
#define MPU6050_REG_SELF_TEST_Z    0x0F
#define MPU6050_REG_SELF_TEST_A    0x10   /* bits 5-4=XA_TEST低2位, 3-2=YA, 1-0=ZA */
#define MPU6050_REG_XG_OFFS_USRH   0x13   /* 陀螺仪 X 偏移高字节 */
#define MPU6050_REG_XG_OFFS_USRL   0x14   /* 陀螺仪 X 偏移低字节 */
#define MPU6050_REG_YG_OFFS_USRH   0x15
#define MPU6050_REG_YG_OFFS_USRL   0x16
#define MPU6050_REG_ZG_OFFS_USRH   0x17
#define MPU6050_REG_ZG_OFFS_USRL   0x18
#define MPU6050_REG_SMPLRT_DIV     0x19   /* 采样率分频 */
#define MPU6050_REG_CONFIG         0x1A   /* DLPF 配置 */
#define MPU6050_REG_GYRO_CONFIG    0x1B   /* 陀螺仪量程 */
#define MPU6050_REG_ACCEL_CONFIG   0x1C   /* 加速度计量程 */
#define MPU6050_REG_FIFO_EN        0x23   /* FIFO 使能 */
#define MPU6050_REG_USER_CTRL      0x6A   /* 用户控制: bit6=FIFO_EN, bit2=FIFO_RESET */
#define MPU6050_REG_PWR_MGMT_1     0x6B   /* 电源管理: bit7=复位, bit6=睡眠 */
#define MPU6050_REG_PWR_MGMT_2     0x6C   /* 电源管理2: 待机控制 */
#define MPU6050_REG_FIFO_COUNTH    0x72   /* FIFO 计数高字节 */
#define MPU6050_REG_FIFO_COUNTL    0x73
#define MPU6050_REG_FIFO_R_W       0x74   /* FIFO 读写 */
#define MPU6050_REG_WHO_AM_I       0x75   /* 复位值 0x68 */
#define MPU6050_REG_ACCEL_XOUT_H   0x3B   /* 加速度 X 高字节 */
#define MPU6050_REG_GYRO_XOUT_H    0x43   /* 陀螺仪 X 高字节 */
#define MPU6050_REG_GYRO_ZOUT_H    0x47   /* 陀螺仪 Z 轴高字节 */
#define MPU6050_REG_GYRO_ZOUT_L    0x48

/* ────────── 量程 ────────── */
#define GYRO_FS_SEL_250   0x00
#define GYRO_LSB_PER_DPS  131.0f  /* ±250°/s: 131 LSB/°/s */

/* ────────── DLPF 配置 ──────────
 * DLPF_CFG | 带宽(Hz) | 延迟(ms) | 采样率(kHz)
 *     0    |   260    |  0.98    |    8
 *     1    |   184    |  2.9     |    1
 *     2    |    94    |  3.9     |    1
 *     3    |    44    |  5.8     |    1
 *     4    |    21    |  8.5     |    1
 *     5    |    10    | 13.4     |    1
 *     6    |     5    | 18.6     |    1
 */
#define DLPF_CFG_188HZ   0x01   /* 校准用：188Hz 带宽，2.9ms 延迟 */
#define DLPF_CFG_42HZ    0x03   /* 运行用：44Hz 带宽，5.8ms 延迟 */
#define DLPF_CFG_21HZ    0x04   /* 旧版：21Hz，已废弃 */

/* ────────── 灵敏度修正 ──────────
 * 每个 MPU6050 芯片的灵敏度有 ±3% 个体差异。
 * 用自检响应的出厂值与实测值对比算出修正系数。
 * 默认 1.0（不修正），运行时可调。
 */
#define GYRO_SCALE_CORR_DEFAULT  1.0f

/* ────────── 软件低通滤波 ──────────
 * EMA 系数：alpha 越大响应越快但噪声越多。
 * 0.20 → 时间常数 ~5 个周期@100Hz = 50ms
 */
#define GYRO_LPF_ALPHA  0.20f

/* ────────── API ────────── */

/**
 * @brief  初始化 MPU6050（配置时钟、量程、DLPF=42Hz、采样率=100Hz）
 * @return 1=成功, 0=失败
 */
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief  自检：验证陀螺仪+加速度计在出厂范围内
 * @param  result[6]  输出：0-2=加速度计偏差%, 3-5=陀螺仪偏差%（应 < ±14%）
 * @return 1=通过, 0=失败
 */
uint8_t MPU6050_SelfTest(I2C_HandleTypeDef *hi2c, float result[6]);

/**
 * @brief  FIFO 零偏校准（参考 Kris Winer calibrateMPU6050）
 * @param  hi2c      I2C 句柄
 * @param  samples   采样点数（建议 100~200，FIFO 自动采集）
 * @return 零偏值 (°/s)
 * @note   内部把零偏写入硬件偏移寄存器（XG/YG/ZG_OFFS_USRH/L），
 *         之后芯片自动减零偏。同时返回 float 值供软件参考。
 *         调用前车必须静止！
 */
float MPU6050_Calibrate(I2C_HandleTypeDef *hi2c, uint16_t samples);

/**
 * @brief  读取陀螺仪 Z 轴 (°/s)，已减硬件零偏 + 软件低通滤波
 * @param  ok  输出：1=通信成功, 0=失败
 */
float MPU6050_ReadGyroZ(I2C_HandleTypeDef *hi2c, uint8_t *ok);

/**
 * @brief  设置灵敏度修正系数（默认 1.0）
 */
void MPU6050_SetScaleCorr(float corr);

/**
 * @brief  读取温度 (°C)
 */
float MPU6050_ReadTemp(I2C_HandleTypeDef *hi2c);

#endif /* __MPU6050_H */
