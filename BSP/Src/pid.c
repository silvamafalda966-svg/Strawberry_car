#include "pid.h"

// 1. 实例化这四个全局变量 (在 .c 里真正分配内存)
PID_TypeDef pid_FL;
PID_TypeDef pid_BL;
PID_TypeDef pid_FR;
PID_TypeDef pid_BR;

// 2. 初始化函数：清空历史状态，注入初始调参值
void PID_Init(void)
{
    // ================= 左侧分组 =================
    // 左前轮 (FL) 初始化
    pid_FL.Target = 0.5f;       // 默认目标 0.5 m/s
    pid_FL.ramp_target = 0.0f;  // 斜坡初始 0
    pid_FL.Actual = 0.0f;
    pid_FL.err = 0.0f;
    pid_FL.err_last = 0.0f;
    pid_FL.err_prev2 = 0.0f;
    pid_FL.direction = 1;       // 左前: 正向
    pid_FL.stall_cnt = 0;
    pid_FL.Kp = 0.25f;          // V2 自动调参: Kp=0.25
    pid_FL.Ki = 0.05f;          // Ki=0.05
    pid_FL.Kd = 0.06f;          // Kd=0.06
    pid_FL.out_PWM = 0.0f;
    pid_FL.max_out_PWM = 999.0f;   // 必须小于或等于你的 TIM1 ARR (如 999)
    pid_FL.min_out_PWM = -999.0f;  // 必须支持负数进行反接制动和后退

    // 左后轮 (BL) 初始化 (复制 FL 配置)
    pid_BL.Target = 0.5f;       
    pid_BL.Actual = 0.0f;
    pid_BL.err = 0.0f;
    pid_BL.err_last = 0.0f;
    pid_BL.err_prev2 = 0.0f;
    pid_BL.direction = 1;       // 左后: 正向
    pid_BL.stall_cnt = 0;
    pid_BL.Kp = 0.25f;
    pid_BL.Ki = 0.05f;
    pid_BL.Kd = 0.06f;
    pid_BL.out_PWM = 0.0f;
    pid_BL.max_out_PWM = 999.0f;   
    pid_BL.min_out_PWM = -999.0f;  

    // ================= 右侧分组 =================
    // 右前轮 (FR) 初始化
    pid_FR.Target = 0.5f;       
    pid_FR.Actual = 0.0f;
    pid_FR.err = 0.0f;
    pid_FR.err_last = 0.0f;
    pid_FR.err_prev2 = 0.0f;
    pid_FR.direction = 1;       // 右前: 正向
    pid_FR.stall_cnt = 0;
    pid_FR.Kp = 0.25f;
    pid_FR.Ki = 0.05f;
    pid_FR.Kd = 0.06f;
    pid_FR.out_PWM = 0.0f;
    pid_FR.max_out_PWM = 999.0f;   
    pid_FR.min_out_PWM = -999.0f;  

    // 右后轮 (BR) 初始化
    pid_BR.Target = 0.5f;       
    pid_BR.Actual = 0.0f;
    pid_BR.err = 0.0f;
    pid_BR.err_last = 0.0f;
    pid_BR.err_prev2 = 0.0f;
    pid_BR.direction = 1;       // 右后: 正向
    pid_BR.stall_cnt = 0;
    pid_BR.Kp = 0.25f;          // V2 四轮统一: Kp=0.25 (自动调参)
    pid_BR.Ki = 0.05f;          // Ki=0.05
    pid_BR.Kd = 0.06f;          // Kd=0.06
    pid_BR.out_PWM = 0.0f;
    pid_BR.max_out_PWM = 999.0f;   
    pid_BR.min_out_PWM = -999.0f;  
}

// 3. 核心算法：标准的增量式 PID 公式实现
float PID_Incremental_Calc(PID_TypeDef *pid, float actual_speed)
{
    float increment_val; // 本次计算出的 PWM 增量值

    pid->Actual = actual_speed; // 接收传感器传来的当前速度
    pid->err = pid->Target - pid->Actual; // 计算出当下的偏差

    // 增量式 PID 核心数学公式：
    // ΔU(k) = Kp*[e(k) - e(k-1)] + Ki*e(k) + Kd*[e(k) - 2e(k-1) + e(k-2)]
    float p_term = pid->Kp * (pid->err - pid->err_last);
    float i_term = pid->Ki * pid->err;
    float d_term = pid->Kd * (pid->err - 2.0f * pid->err_last + pid->err_prev2);

    // === 积分抗饱和：输出已饱和时，抑制同向积分蓄力 ===
    if ((pid->out_PWM >= pid->max_out_PWM && i_term > 0.0f) ||
        (pid->out_PWM <= pid->min_out_PWM && i_term < 0.0f)) {
        i_term = 0.0f;  // 饱和方向不蓄积分，防止 windup
    }

    increment_val = p_term + i_term + d_term;

    // 将计算出的增量值累加到历史 PWM 输出上
    pid->out_PWM += increment_val;

    // 更新历史误差记录，把当下变成过去，等待下一次循环
    pid->err_prev2 = pid->err_last;
    pid->err_last = pid->err;

    // === 极限保护：PWM 输出防超限 ===
    if (pid->out_PWM > pid->max_out_PWM)
    {
        pid->out_PWM = pid->max_out_PWM; // 最高不超满速
    }
    else if (pid->out_PWM < pid->min_out_PWM)
    {
        pid->out_PWM = pid->min_out_PWM; // 最低不超满速倒车
    }

    // 返回经过保护后、准备下发给电机的最终 PWM 占空比
    return pid->out_PWM;
}

