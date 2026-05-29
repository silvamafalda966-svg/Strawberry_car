#include "bsp_motor.h"
#include "tim.h"   // 调用 PWM 定时器
#include "gpio.h"  // 调用引脚高低电平控制
#include <stdlib.h> // 调用 abs() 绝对值函数
#include "pid.h"

// 假设你的 TIM1 ARR(重装载值) 设的是 999，那么 PWM 的最大有效范围就是 0~1000
// 队长注意：如果你的 ARR 不是 999，请把这个宏定义改成你的 ARR 值！
#define MAX_PWM  999
// 在 bsp_motor.c 的顶部定义它们

volatile float speed_FL_mps = 0.0f;
volatile float speed_FR_mps = 0.0f;
volatile float speed_BL_mps = 0.0f;
volatile float speed_BR_mps = 0.0f;

/**
  * @brief  电机子系统初始化
  * @note   必须在 main 函数的 USER CODE BEGIN 2 里调用一次
  */
void Motor_Init(void) {
    // 扣动扳机：开启 TIM1 的四个通道，开始向 TB6612 发送 PWM 波
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // FL
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // FR
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // BL
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // BR
}

/**
  * @brief  左路战区速度设置 (统管 FL 和 BL)
  * @param  speed: 目标速度。正数前进，负数后退，范围 -MAX_PWM 到 +MAX_PWM
  */
/**
  * @brief  左前轮 (FL) 独立速度设置
  */
void Motor_FL_SetSpeed(int speed) {
    if(speed > MAX_PWM) speed = MAX_PWM;
    if(speed < -MAX_PWM) speed = -MAX_PWM;
    uint16_t pwm_val = abs(speed);

    if(speed > 0) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
    } else if(speed < 0) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
    }
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm_val);
}

/**
  * @brief  右前轮 (FR) 独立速度设置
  */
void Motor_FR_SetSpeed(int speed) {
    if(speed > MAX_PWM) speed = MAX_PWM;
    if(speed < -MAX_PWM) speed = -MAX_PWM;
    uint16_t pwm_val = abs(speed);

    if(speed > 0) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    } else if(speed < 0) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    }
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwm_val);
}

/**
  * @brief  左后轮 (BL) 独立速度设置
  */
void Motor_BL_SetSpeed(int speed) {
    if(speed > MAX_PWM) speed = MAX_PWM;
    if(speed < -MAX_PWM) speed = -MAX_PWM;
    uint16_t pwm_val = abs(speed);

    if(speed > 0) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
    } else if(speed < 0) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
    }
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwm_val);
}

/**
  * @brief  右后轮 (BR) 独立速度设置
  */
void Motor_BR_SetSpeed(int speed) {
    if(speed > MAX_PWM) speed = MAX_PWM;
    if(speed < -MAX_PWM) speed = -MAX_PWM;
    uint16_t pwm_val = abs(speed);

    if(speed > 0) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
    } else if(speed < 0) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
    }
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, pwm_val);
}

/**
  * @brief  底盘差速运动高级接口 (已升级为兼容四轮独立底层)
  * @param  base_speed: 基础向前速度
  * @param  turn_diff:  差速值 (正数右转，负数左转)
  */
void Chassis_Drive(int base_speed, int turn_diff) {
    // 1. 左侧战区：左前和左后同步接收差速指令
    Motor_FL_SetSpeed(base_speed + turn_diff);
    Motor_BL_SetSpeed(base_speed + turn_diff);
    
    // 2. 右侧战区：右前和右后同步接收差速指令
    Motor_FR_SetSpeed(base_speed - turn_diff);
    Motor_BR_SetSpeed(base_speed - turn_diff);
}
