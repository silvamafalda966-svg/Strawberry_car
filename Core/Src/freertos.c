/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "bsp_motor.h" // 假设你的底层电机控制和测速在这个文件里
#include "tim.h"  // 必须包含这个，才能让 freertos.c 认识 htim5, htim2 等
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t rx_buf[32];          // 接收缓存
uint8_t rx_index = 0;        // 缓存索引
volatile uint8_t rx_flag = 0;// 指令到达标志
uint8_t rx_byte;             // 单字节接收位
// K210 8字节帧解析状态机
uint8_t  k210_buf[8];        // K210 帧缓存
uint8_t  k210_idx = 0;       // 当前接收索引
uint8_t  k210_sync = 0;      // 0=找0xAA头, 1=收帧中
volatile uint8_t k210_ready = 0; // 完整帧到达标志
// 为四个轮子单独分配 PID 大脑，并直接赋初始值（防死锁）
extern PID_TypeDef pid_FL;
extern PID_TypeDef pid_FR;
extern PID_TypeDef pid_BL;
extern PID_TypeDef pid_BR;
/* USER CODE END Variables */
osThreadId Task_PID_CtrlHandle;
osThreadId Task_VisionHandle;
osThreadId Task_SensorHandle;
osThreadId Task_OLEDHandle;
osMessageQId Queue_VisionDataHandle;
osMutexId Mutex_SensorDataHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static float CalcOneWheel(PID_TypeDef *pid, float speed_mps);
/* USER CODE END FunctionPrototypes */

void StartTaskChassis(void const * argument);
void StartTaskVision(void const * argument);
void StartTaskSensor(void const * argument);
void StartTaskOLED(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of Mutex_SensorData */
  osMutexDef(Mutex_SensorData);
  Mutex_SensorDataHandle = osMutexCreate(osMutex(Mutex_SensorData));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of Queue_VisionData */
  osMessageQDef(Queue_VisionData, 2, 12);
  Queue_VisionDataHandle = osMessageCreate(osMessageQ(Queue_VisionData), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Task_PID_Ctrl */
  osThreadDef(Task_PID_Ctrl, StartTaskChassis, osPriorityAboveNormal, 0, 512);
  Task_PID_CtrlHandle = osThreadCreate(osThread(Task_PID_Ctrl), NULL);

  /* definition and creation of Task_Vision */
  osThreadDef(Task_Vision, StartTaskVision, osPriorityNormal, 0, 512);
  Task_VisionHandle = osThreadCreate(osThread(Task_Vision), NULL);

  /* definition and creation of Task_Sensor */
  osThreadDef(Task_Sensor, StartTaskSensor, osPriorityHigh, 0, 256);
  Task_SensorHandle = osThreadCreate(osThread(Task_Sensor), NULL);

  /* definition and creation of Task_OLED */
  osThreadDef(Task_OLED, StartTaskOLED, osPriorityLow, 0, 256);
  Task_OLEDHandle = osThreadCreate(osThread(Task_OLED), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartTaskChassis */
/**
  * @brief  Function implementing the Task_PID_Ctrl thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTaskChassis */
void StartTaskChassis(void const * argument)
{
  /* USER CODE BEGIN StartTaskChassis */
  // 1. 给四个轮子赋初始 PID 参数（防死锁）
  pid_FL.direction = 1;  pid_FL.Kp = 0.35f; pid_FL.Ki = 0.06f; pid_FL.Kd = 0.04f; pid_FL.Target = 0.0f;
  pid_BL.direction = 1;  pid_BL.Kp = 0.35f; pid_BL.Ki = 0.06f; pid_BL.Kd = 0.04f; pid_BL.Target = 0.0f;
  pid_FR.direction = 1;  pid_FR.Kp = 0.35f; pid_FR.Ki = 0.06f; pid_FR.Kd = 0.04f; pid_FR.Target = 0.0f;
  pid_BR.direction = 1;  pid_BR.Kp = 0.35f; pid_BR.Ki = 0.06f; pid_BR.Kd = 0.04f; pid_BR.Target = 0.0f;

  // 2. 确保 USART1 VOFA+ 中断开启
  extern UART_HandleTypeDef huart1;
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

  static uint8_t cur_wheel = 4; // 当前选中轮: 0=FL,1=FR,2=BL,3=BR,4=ALL（默认全选）

  for(;;)
  {
      // A.【参数解析区】VOFA+ 命令
      if (rx_flag == 1)
      {
          if (rx_buf[0] == 'W') {
              cur_wheel = (rx_buf[1] == 'A') ? 4 : (rx_buf[1] - '0');
              // 切换轮子时，清零非选中轮 Target
              for (int i = 0; i < 4; i++) {
                  PID_TypeDef *p = (i==0)?&pid_FL:(i==1)?&pid_FR:(i==2)?&pid_BL:&pid_BR;
                  if (cur_wheel < 4 && i != (int)cur_wheel) p->Target = 0.0f;
              }
          } else {
              float val = atof((char*)&rx_buf[1]);
              if (cur_wheel < 4) {
                  PID_TypeDef *pid = (cur_wheel==0)?&pid_FL:(cur_wheel==1)?&pid_FR:(cur_wheel==2)?&pid_BL:&pid_BR;
                  switch (rx_buf[0]) {
                      case 'P': pid->Kp = val; break;
                      case 'I': pid->Ki = val; break;
                      case 'D': pid->Kd = val; break;
                      case 'T': pid->Target = val; break;
                  }
              } else {
                  switch (rx_buf[0]) {
                      case 'P': pid_FL.Kp = pid_FR.Kp = pid_BL.Kp = pid_BR.Kp = val; break;
                      case 'I': pid_FL.Ki = pid_FR.Ki = pid_BL.Ki = pid_BR.Ki = val; break;
                      case 'D': pid_FL.Kd = pid_FR.Kd = pid_BL.Kd = pid_BR.Kd = val; break;
                      case 'T': pid_FL.Target = pid_FR.Target = pid_BL.Target = pid_BR.Target = val; break;
                  }
              }
          }
          rx_flag = 0;
      }

      // B.【编码器采集】四路独立读数
      short dFL = (short)__HAL_TIM_GET_COUNTER(&htim5);
      __HAL_TIM_SET_COUNTER(&htim5, 0);
      speed_FL_mps = (float)dFL * 0.0003694f;

      short dBL = (short)__HAL_TIM_GET_COUNTER(&htim4);
      __HAL_TIM_SET_COUNTER(&htim4, 0);
      speed_BL_mps = (float)dBL * 0.0003694f;

      short dFR = (short)__HAL_TIM_GET_COUNTER(&htim2);
      __HAL_TIM_SET_COUNTER(&htim2, 0);
      speed_FR_mps = (float)dFR * 0.0003694f;

      short dBR = (short)__HAL_TIM_GET_COUNTER(&htim8);
      __HAL_TIM_SET_COUNTER(&htim8, 0);
      speed_BR_mps = (float)dBR * 0.0003694f;

      // C.【PID 计算 + 电机下发】只给选中轮子输出
      float pwm_FL = (cur_wheel==0||cur_wheel==4) ? CalcOneWheel(&pid_FL, speed_FL_mps) : 0.0f;
      float pwm_FR = (cur_wheel==1||cur_wheel==4) ? CalcOneWheel(&pid_FR, speed_FR_mps) : 0.0f;
      float pwm_BL = (cur_wheel==2||cur_wheel==4) ? CalcOneWheel(&pid_BL, speed_BL_mps) : 0.0f;
      float pwm_BR = (cur_wheel==3||cur_wheel==4) ? CalcOneWheel(&pid_BR, speed_BR_mps) : 0.0f;

      Motor_FL_SetSpeed((int)pwm_FL);
      Motor_FR_SetSpeed((int)pwm_FR);
      Motor_BL_SetSpeed((int)pwm_BL);
      Motor_BR_SetSpeed((int)pwm_BR);

      // D.【监控上传区】VOFA+ 四路速度波形
      float vofa_buf[5];
      vofa_buf[0] = speed_FL_mps;
      vofa_buf[1] = speed_FR_mps;
      vofa_buf[2] = speed_BL_mps;
      vofa_buf[3] = speed_BR_mps;
      uint32_t tail = 0x7F800000;
      vofa_buf[4] = *(float*)&tail;
      HAL_UART_Transmit(&huart1, (uint8_t*)vofa_buf, sizeof(vofa_buf), 10);

      // E.【系统节奏】
      osDelay(10);
  }
  /* USER CODE END StartTaskChassis */
}

/* USER CODE BEGIN Header_StartTaskVision */
/**
* @brief Function implementing the Task_Vision thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskVision */
void StartTaskVision(void const * argument)
{
  /* USER CODE BEGIN StartTaskVision */
    extern UART_HandleTypeDef huart2;
    extern osMessageQId Queue_VisionDataHandle;
    extern volatile uint8_t k210_ready;
    extern uint8_t k210_buf[8];

    // 启动 USART2 中断接收，用于 K210 视觉数据
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

    for(;;)
    {
      if (k210_ready) {
          k210_ready = 0;

          // 校验 8 字节帧: 0xAA + Class + X + Y + Area + Reserved + Checksum + 0x55
          if (k210_buf[0] == 0xAA && k210_buf[7] == 0x55) {
              uint8_t checksum = k210_buf[1] ^ k210_buf[2] ^ k210_buf[3]
                               ^ k210_buf[4] ^ k210_buf[5];
              if (checksum == k210_buf[6]) {
                  // 帧有效，推入消息队列给 Task_PID_Ctrl 消费
                  osMessagePut(Queue_VisionDataHandle, *(uint32_t*)&k210_buf, 0);
              }
          }
      }
      osDelay(10); // 100Hz 检查频率
    }
  /* USER CODE END StartTaskVision */
}

/* USER CODE BEGIN Header_StartTaskSensor */
/**
* @brief Function implementing the Task_Sensor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskSensor */
void StartTaskSensor(void const * argument)
{
  /* USER CODE BEGIN StartTaskSensor */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskSensor */
}

/* USER CODE BEGIN Header_StartTaskOLED */
/**
* @brief Function implementing the Task_OLED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskOLED */
void StartTaskOLED(void const * argument)
{
  /* USER CODE BEGIN StartTaskOLED */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskOLED */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // 定义外部变量，让函数认识它们
    extern uint8_t rx_buf[32];
    extern uint8_t rx_index;
    extern volatile uint8_t rx_flag;
    extern uint8_t rx_byte;
    extern uint8_t k210_buf[8];
    extern uint8_t k210_idx;
    extern uint8_t k210_sync;
    extern volatile uint8_t k210_ready;

    // ── USART1: VOFA+ 文本命令行（'\n' 换行结束） ──
    if (huart->Instance == USART1) {
        if (rx_byte == '\n') {
            if (rx_index > 0) {
                rx_buf[rx_index] = '\0';
                rx_flag = 1;
                rx_index = 0;
            }
        } else if (rx_byte != '\r') {
            if (rx_index < 31) {
                rx_buf[rx_index++] = rx_byte;
            }
        }
        HAL_UART_Receive_IT(huart, &rx_byte, 1);
    }
    // ── USART2: K210 8字节二进制帧（0xAA...0x55） ──
    else if (huart->Instance == USART2) {
        if (k210_sync == 0) {
            // 等待帧头 0xAA
            if (rx_byte == 0xAA) {
                k210_sync = 1;
                k210_idx = 0;
                k210_buf[k210_idx++] = rx_byte; // 存 0xAA
            }
        } else {
            // 收字节 1~7
            k210_buf[k210_idx++] = rx_byte;
            if (k210_idx >= 8) {
                k210_sync = 0;
                if (k210_buf[7] == 0x55) {
                    k210_ready = 1; // 帧尾正确，通知 Task_Vision
                }
            }
        }
        HAL_UART_Receive_IT(huart, &rx_byte, 1);
    }
}
/**
  * @brief  单轮 PID 计算 + 停车锁死策略（四轮共用）
  * @note   目标单位为 m/s，内部映射到 PWM 量纲
  */
static float CalcOneWheel(PID_TypeDef *pid, float speed_mps)
{
    // ── 堵转保护：PWM>300 且速度≈0 持续 500ms → 切断输出 ──
    if (pid->out_PWM > 300.0f || pid->out_PWM < -300.0f) {
        if (speed_mps < 0.01f && speed_mps > -0.01f) {
            pid->stall_cnt++;
        } else {
            pid->stall_cnt = 0;
        }
    } else {
        pid->stall_cnt = 0;
    }
    if (pid->stall_cnt > 50) {
        pid->out_PWM = 0.0f;
        pid->err = 0.0f;
        pid->err_last = 0.0f;
        pid->err_prev2 = 0.0f;
        pid->stall_cnt = 0;
        return 0.0f;
    }

    // ── 目标速度斜坡：限制加速度，防止启停过猛 ──
    #define RAMP_STEP 0.03f  // 每10ms最多变化0.03m/s ≈ 3m/s²
    if (pid->Target > pid->ramp_target) {
        pid->ramp_target += RAMP_STEP;
        if (pid->ramp_target > pid->Target) pid->ramp_target = pid->Target;
    } else if (pid->Target < pid->ramp_target) {
        pid->ramp_target -= RAMP_STEP;
        if (pid->ramp_target < pid->Target) pid->ramp_target = pid->Target;
    }

    // ── 停车锁死策略：ramp_target=0 且接近静止 → 清零 ──
    if (pid->ramp_target == 0.0f && speed_mps < 0.03f && speed_mps > -0.03f) {
        pid->out_PWM = 0.0f;
        pid->err = 0.0f;
        pid->err_last = 0.0f;
        pid->err_prev2 = 0.0f;
        return 0.0f;
    }
    float speed_scaled = speed_mps * 2000.0f;
    float saved_target  = pid->Target;
    pid->Target = pid->ramp_target * 2000.0f;  // 用斜坡值替代原始目标
    float out = PID_Incremental_Calc(pid, speed_scaled);
    pid->Target = saved_target;
    return out * pid->direction;
}
/* USER CODE END Application */
