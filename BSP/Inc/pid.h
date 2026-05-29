#ifndef __PID_H
#define __PID_H

#include "main.h" // 包含底层类型定义

// 纯粹的增量式 PID 结构体定义
typedef struct 
{
    float Target;       // 外部设置的目标速度 (m/s)
    float ramp_target;  // 斜坡平滑后的实际目标 (m/s)
    float Actual;       // 实际测得速度 (m/s)
    
    float err;          // 当前时刻的误差 e(k)
    float err_last;     // 上一时刻的误差 e(k-1)
    float err_prev2;    // 上上时刻的误差 e(k-2)
    
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    
    int8_t   direction;  // 方向校正: 1=正向, -1=反向
    uint16_t stall_cnt;  // 堵转计时器 (10ms/tick, >50=500ms 则切断)
    float out_PWM;      // 最终累加输出的实际 PWM 值
    float max_out_PWM;  // 输出限幅 (防止超出定时器 ARR 烧毁硬件)
    float min_out_PWM;  // 输出下限 (通常与最大限幅对称，比如 -999)
    
} PID_TypeDef;

// 按队伍规范声明四个轮子的控制器实例 (外部变量，供 bsp_motor.c 和 main.c 使用)
// 左侧组
extern PID_TypeDef pid_FL; // 左前
extern PID_TypeDef pid_BL; // 左后
// 右侧组
extern PID_TypeDef pid_FR; // 右前
extern PID_TypeDef pid_BR; // 右后

// 提供给外部调用的函数接口
void PID_Init(void); // 在主循环或 RTOS 任务开始前调用，初始化参数
float PID_Incremental_Calc(PID_TypeDef *pid, float actual_speed); // 每次计算调用此核心函数

#endif // __PID_H
