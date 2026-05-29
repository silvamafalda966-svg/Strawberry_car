#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include "main.h" // 引入底层类型定义

/* ------------------- 车轮速度传感（编码器读数） ------------------- */
extern volatile float speed_FL_mps; // 左前 (Front Left) 速度，单位 m/s
extern volatile float speed_FR_mps; // 右前 (Front Right) 速度，单位 m/s
extern volatile float speed_BL_mps; // 左后 (Back Left) 速度，单位 m/s
extern volatile float speed_BR_mps; // 右后 (Back Right) 速度，单位 m/s

/* ------------------- 战区统领 API ------------------- */
// 电机系统初始化（开机时调用一次，用来发射 PWM 信号）
void Motor_Init(void);

void Motor_FL_SetSpeed(int speed);
void Motor_FR_SetSpeed(int speed);
void Motor_BL_SetSpeed(int speed);
void Motor_BR_SetSpeed(int speed);

// 基础差速转向控制（直接传入基础速度和差速值）
void Chassis_Drive(int base_speed, int turn_diff);
/* 在 bsp_motor.h 中添加 */


#endif
