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
#include "bsp_motor.h"
#include "chassis.h"
#include "tim.h"
#include "oled.h"
#include "oledfont.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
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
uint8_t rx_buf[32];
uint8_t rx_index = 0;
volatile uint8_t rx_flag = 0;
uint8_t rx_byte;
uint8_t rx_byte_bt;  // 蓝牙 UART4 接收字节
// K210 DMA 定长 24B 接收缓冲区
uint8_t  k210_dma_buf[24];
volatile uint8_t k210_frame_ready = 0;  // IDLE 中断置位，PID任务消费
// 传感器数据（Task_Sensor 写，Task_PID_Ctrl 读）
static SensorData_t g_sensor_data = {{1,1,1,1,1,1,1,1}, 300.0f};
/* USER CODE END Variables */
osThreadId Task_PID_CtrlHandle;
osThreadId Task_VisionHandle;
osThreadId Task_SensorHandle;
osThreadId Task_OLEDHandle;
QueueHandle_t xQueue_VisionData;   // 24B 定长帧队列（FreeRTOS 原生）
osMutexId Mutex_SensorDataHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static float CalcOneWheel(PID_TypeDef *pid, float speed_mps);
static void HC_SR04_Trigger(void);

// BR 摩擦前馈 + EMA 滤波系数（运行时可调，F/L 命令）
float g_br_feedforward   = 40.0f;    // F 命令调整（最优值 40）
float g_speed_filt_alpha = 0.3f;     // L 命令调整（最优值 0.3）
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
  /* K210 24B 定长帧队列（深度 2） */
  xQueue_VisionData = xQueueCreate(2, 24);

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

/* ── HC-SR04 超声波测距（TIM3 CH4 输入捕获中断，零阻塞）── */
static volatile float g_ultrasonic_cm = 300.0f; // ISR 写入，PID 任务读取
static uint32_t ic_val1 = 0;                    // 上升沿捕获值
static uint8_t  ic_phase = 0;                   // 0=待上升沿, 1=待下降沿

static void HC_SR04_Trigger(void)
{
    ic_phase = 0;
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    for (volatile int i = 0; i < 280; i++) { __NOP(); } // ~10µs @168MHz
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3 || htim->Channel != HAL_TIM_ACTIVE_CHANNEL_4) return;

    if (ic_phase == 0) {
        ic_val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);
        ic_phase = 1;
    } else {
        uint32_t ic_val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
        uint32_t pulse_us = (ic_val2 >= ic_val1) ? (ic_val2 - ic_val1)
                                                  : (65535 - ic_val1 + ic_val2 + 1);
        g_ultrasonic_cm = (float)pulse_us / 58.0f;
        HAL_TIM_IC_Stop_IT(htim, TIM_CHANNEL_4);
        ic_phase = 0;
    }
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
  // 1. 四轮初始 PID 参数（防死锁）
  // Systematically tuned 2026-06-13 (Kp→Ki→Kd full sweep)
  // FL/FR/BL: score=8, rise=96ms, overshoot=0.5%
  pid_FL.direction = 1; pid_FL.Kp = 0.20f; pid_FL.Ki = 0.08f; pid_FL.Kd = 0.02f; pid_FL.Target = 0.0f;
  pid_BL.direction = 1; pid_BL.Kp = 0.20f; pid_BL.Ki = 0.08f; pid_BL.Kd = 0.02f; pid_BL.Target = 0.0f;
  pid_FR.direction = 1; pid_FR.Kp = 0.20f; pid_FR.Ki = 0.08f; pid_FR.Kd = 0.02f; pid_FR.Target = 0.0f;
  // BR: score=26, rise=107ms, over=1.4%, plateau=95ms (mechanical friction dead zone)
  pid_BR.direction = 1; pid_BR.Kp = 0.35f; pid_BR.Ki = 0.08f; pid_BR.Kd = 0.10f; pid_BR.Target = 0.0f;

  // 2. 引用 USART1/UART4/UART5 句柄（中断已在 main.c 启动）
  extern UART_HandleTypeDef huart1;
  extern UART_HandleTypeDef huart4;
  extern UART_HandleTypeDef huart5;

  // 3. 状态机初始化
  Chassis_Init();

  // EMA 滤波状态（编码器速度低通滤波，降噪）
  float filt_FL = 0.0f, filt_FR = 0.0f, filt_BL = 0.0f, filt_BR = 0.0f;

  for(;;)
  {
      // ── A. VOFA+ 命令解析 ──
      if (rx_flag == 1) {
          Chassis_VOFA_Parse(rx_buf);
          rx_flag = 0;
      }

      // ── B. 编码器采集（四路独立） ──
      short dFL = -(short)__HAL_TIM_GET_COUNTER(&htim5);
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
      g_avg_speed_mps = (fabsf(speed_FL_mps) + fabsf(speed_FR_mps)
                      +  fabsf(speed_BL_mps) + fabsf(speed_BR_mps)) / 4.0f;

      // ── EMA 低通滤波：平滑编码器量化噪声 ──
      float a = g_speed_filt_alpha;
      filt_FL = a * filt_FL + (1.0f - a) * speed_FL_mps;
      filt_FR = a * filt_FR + (1.0f - a) * speed_FR_mps;
      filt_BL = a * filt_BL + (1.0f - a) * speed_BL_mps;
      filt_BR = a * filt_BR + (1.0f - a) * speed_BR_mps;

      // ── 累积编码器里程（四轮均值，有符号） ──
      g_enc_total += (int32_t)(dFL + dFR + dBL + dBR) / 4;
      // ── 四轮编码器绝对值累加（用于精算转角，不受方向影响） ──
      g_turn_enc += (int32_t)(abs(dFL) + abs(dFR) + abs(dBL) + abs(dBR));

      // ── C. 视觉队列检查（仅需 K210 的状态才取队列） ──
      VisionFrame_t vis_frame;
      memset(&vis_frame, 0, sizeof(vis_frame));
      uint8_t has_vision = 0;
      if (g_chassis_state == STATE_LINE_FOLLOW ||
              g_chassis_state == STATE_BRANCH_JUDGE) {
          uint8_t raw[24];
          if (xQueueReceive(xQueue_VisionData, raw, 0) == pdTRUE) {
              // 解析定长 24B 帧 → VisionFrame_t
              vis_frame.count = raw[1];
              for (int i = 0; i < 5; i++) {
                  vis_frame.t[i].cls  = raw[2 + i*4 + 0];
                  vis_frame.t[i].x    = raw[2 + i*4 + 1];
                  vis_frame.t[i].y    = raw[2 + i*4 + 2];
                  vis_frame.t[i].area = raw[2 + i*4 + 3];
              }
              has_vision = 1;
          }
      }

      // ── D. 传感器数据获取（互斥锁） ──
      SensorData_t local_sensor;
      memset(&local_sensor, 0, sizeof(local_sensor));
      local_sensor.distance_cm = 300.0f;
      if (osMutexWait(Mutex_SensorDataHandle, 0) == osOK) {
          local_sensor = g_sensor_data;
          osMutexRelease(Mutex_SensorDataHandle);
      }

      // ── D2. 超声波测距（中断驱动，零阻塞；20Hz 触发一次） ──
#if ENABLE_ULTRASONIC
      static uint8_t ultra_tick = 0;
      if (++ultra_tick >= 5) {
          ultra_tick = 0;
          HC_SR04_Trigger();
      }
      local_sensor.distance_cm = g_ultrasonic_cm; // ISR 异步更新
      // 急停标志位：仅在 LINE_FOLLOW 中生效
      if (g_chassis_state == STATE_LINE_FOLLOW) {
          if (local_sensor.distance_cm < 15.0f) {
              g_emergency_stop = 1;
          } else {
              g_emergency_stop = 0;
          }
      }
#endif

      // ── E. 紧急刹车执行（仅 LINE_FOLLOW 巡线中触发） ──
      if (g_emergency_stop && g_chassis_state == STATE_LINE_FOLLOW) {
          Chassis_Stop();
          g_chassis_state = STATE_OBSTACLE;
          if (speed_FL_mps < 0.03f && speed_FL_mps > -0.03f &&
              speed_FR_mps < 0.03f && speed_FR_mps > -0.03f &&
              speed_BL_mps < 0.03f && speed_BL_mps > -0.03f &&
              speed_BR_mps < 0.03f && speed_BR_mps > -0.03f) {
              g_emergency_stop = 0;
          }
      }

      // ── F. 状态机运行 ──
      static ChassisState_t prev_chassis_state = STATE_COUNT;
      Chassis_StateMachine_Run(&local_sensor, has_vision, &vis_frame,
                               (int32_t)dFL, (int32_t)dFR,
                               (int32_t)dBL, (int32_t)dBR);

      // ── F2. K210 决策文本行上报（BRANCH_JUDGE → BRANCH_REVERSE 时触发一次） ──
      if (prev_chassis_state == STATE_BRANCH_JUDGE
          && g_chassis_state == STATE_BRANCH_REVERSE) {
          char d_line[32];
          int len = snprintf(d_line, sizeof(d_line),
                             "D:%d,%d,%d,%d,%d\n",
                             g_branch_decision.pick,
                             g_branch_decision.cls,
                             g_branch_decision.area,
                             g_branch_decision.total_red,
                             g_branch_decision.total_green);
          if (len > 0 && len < (int)sizeof(d_line)) {
              if (huart5.gState == HAL_UART_STATE_READY) {
                  HAL_UART_Transmit(&huart5, (uint8_t*)d_line, len, 2);
              }
              HAL_UART_Transmit(&huart1, (uint8_t*)d_line, len, 10);
          }
      }

      // ── 区域完成累计统计（BRANCH_REVERSE → LINE_FOLLOW 时上报） ──
      if (prev_chassis_state == STATE_BRANCH_REVERSE
          && (g_chassis_state == STATE_LINE_FOLLOW
              || g_chassis_state == STATE_ZONE_RETURN)) {
          char c_line[24];
          int clen = snprintf(c_line, sizeof(c_line),
                              "C:%d,%d,%d\n",
                              g_branch_zone,
                              g_red_count,
                              g_green_count);
          if (clen > 0 && clen < (int)sizeof(c_line)) {
              if (huart5.gState == HAL_UART_STATE_READY) {
                  HAL_UART_Transmit(&huart5, (uint8_t*)c_line, clen, 2);
              }
              HAL_UART_Transmit(&huart1, (uint8_t*)c_line, clen, 10);
          }
      }
      prev_chassis_state = g_chassis_state;

      // ── F3. 位置标记检测（3~4路IR黑 = 横向胶带，6路全黑 = 岔路口） ──
      {
          static uint8_t  marker_cnt     = 0;
          static uint32_t marker_cooldown = 0;
          int mk_black = 0;
          for (int i = 0; i < IR_SENSOR_COUNT; i++) {
              if (local_sensor.ir[i] == 0) mk_black++;
          }

          if (mk_black == 3 || mk_black == 4) {
              marker_cnt++;
              if (marker_cnt >= 3 && marker_cooldown == 0) {
                  char k_line[16];
                  int klen = snprintf(k_line, sizeof(k_line), "K:%d\n", (int)g_enc_total);
                  if (klen > 0 && klen < (int)sizeof(k_line)) {
                      if (huart5.gState == HAL_UART_STATE_READY) {
                          HAL_UART_Transmit(&huart5, (uint8_t*)k_line, klen, 2);
                      }
                      HAL_UART_Transmit(&huart1, (uint8_t*)k_line, klen, 10);
                  }
                  marker_cnt = 0;
                  marker_cooldown = 50;  // 500ms 冷却，防重复触发
              }
          } else {
              marker_cnt = 0;
          }
          if (marker_cooldown > 0) marker_cooldown--;
      }

      // ── G. PID 计算 + 电机下发（四轮统一，使用滤波速度） ──
      float pwm_FL = CalcOneWheel(&pid_FL, filt_FL);
      float pwm_FR = CalcOneWheel(&pid_FR, filt_FR);
      float pwm_BL = CalcOneWheel(&pid_BL, filt_BL);
      float pwm_BR = CalcOneWheel(&pid_BR, filt_BR);

      // BR 摩擦前馈：克服静摩擦死区，按 ramp_target 方向加固定推力
      if (pid_BR.ramp_target > 0.01f) {
          pwm_BR += g_br_feedforward;
      } else if (pid_BR.ramp_target < -0.01f) {
          pwm_BR -= g_br_feedforward;
      }

      Motor_FL_SetSpeed((int)pwm_FL);
      Motor_FR_SetSpeed((int)pwm_FR);
      Motor_BL_SetSpeed((int)pwm_BL);
      Motor_BR_SetSpeed((int)pwm_BR);

      // ── H. VOFA+ 上传（8 通道：4轮速度 + IR字节 + 超声波距离 + 编码器里程 + 帧尾） ──
      float vofa_buf[8];
      vofa_buf[0] = filt_FL;
      vofa_buf[1] = filt_FR;
      vofa_buf[2] = filt_BL;
      vofa_buf[3] = filt_BR;
      uint8_t ir_byte = 0;
      for (int i = 0; i < IR_SENSOR_COUNT; i++) {
          if (local_sensor.ir[i]) ir_byte |= (1 << i);
      }
      vofa_buf[4] = (float)ir_byte;
      vofa_buf[5] = local_sensor.distance_cm;
      vofa_buf[6] = (float)g_enc_total;
      uint32_t tail = 0x7F800000;     // 保持纯 +Inf 帧尾，兼容 VOFA+
      vofa_buf[7] = *(float*)&tail;

      // 状态变更时发文本行（car-studio 用，VOFA+ 忽略）
      static ChassisState_t last_reported_state = STATE_COUNT;
      if (g_chassis_state != last_reported_state) {
          char s_line[8];
          int slen = snprintf(s_line, sizeof(s_line), "S:%d\n", (int)g_chassis_state);
          if (slen > 0 && slen < (int)sizeof(s_line)) {
              if (huart5.gState == HAL_UART_STATE_READY) {
                  HAL_UART_Transmit(&huart5, (uint8_t*)s_line, slen, 2);
              }
              HAL_UART_Transmit(&huart1, (uint8_t*)s_line, slen, 10);
          }
          last_reported_state = g_chassis_state;
      }

      // 蓝牙 UART5 可能因 HC-06 异常而阻塞 → 先发 USB，蓝牙仅 ready 时发
      if (huart5.gState == HAL_UART_STATE_READY) {
          HAL_UART_Transmit(&huart5, (uint8_t*)vofa_buf, sizeof(vofa_buf), 2);
      }
      HAL_UART_Transmit(&huart1, (uint8_t*)vofa_buf, sizeof(vofa_buf), 10);

      // ── I. 全局滴答 + 系统节奏 100Hz ──
      g_sys_tick++;
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

    // 启动 USART2 DMA + IDLE 接收（定长 24B 帧）
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, k210_dma_buf, 24);
    // ⚠️ 关键：关闭半传输中断，防止 12 字节时误触发
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);

    for(;;)
    {
      // k210_frame_ready 由 IDLE 中断回调置位
      // 此处仅做看门狗/心跳，实际数据在 IDLE 回调中已入队
      osDelay(10);
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
  for(;;)
  {
      // 读 6 路 TCRT5000 DO 口 PE0~PE5
      SensorData_t sensor;
      uint32_t idr = GPIOE->IDR;
      // 🔧 赛道实测：TCRT5000+LM393 模块输出极性反了
      // PIN HIGH = 黑线, PIN LOW = 白底 → 翻转映射为 ir=0=黑, ir=1=白
      sensor.ir[0] = (idr & GPIO_PIN_0) ? 0 : 1;
      sensor.ir[1] = (idr & GPIO_PIN_1) ? 0 : 1;
      sensor.ir[2] = (idr & GPIO_PIN_2) ? 0 : 1;
      sensor.ir[3] = (idr & GPIO_PIN_3) ? 0 : 1;
      sensor.ir[4] = (idr & GPIO_PIN_4) ? 0 : 1;
      sensor.ir[5] = (idr & GPIO_PIN_5) ? 0 : 1;
      sensor.ir[6] = 1;  // 未接，恒为白
      sensor.ir[7] = 1;
      sensor.distance_cm = 300.0f;

      // 互斥锁写入
      if (osMutexWait(Mutex_SensorDataHandle, osWaitForever) == osOK) {
          g_sensor_data = sensor;
          osMutexRelease(Mutex_SensorDataHandle);
      }

      osDelay(20); // 50Hz
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
  OLED_Init();

  /* ── 状态名 ↔ 中文字库索引映射 ──
     注：新增 STATE_SCANNING(扫描) 暂用 ASCII "SC" 显示，
     待扩充字库后改为中文。索引已重新映射。 */
  static const uint8_t state_cn_idx[STATE_COUNT][2] = {
      [STATE_DEBUG]           = {HZK_00, HZK_01},  /* 调试 */
      [STATE_IDLE]            = {HZK_02, HZK_03},  /* 待机 */
      [STATE_LINE_FOLLOW]     = {HZK_04, HZK_05},  /* 巡线 */
      [STATE_OBSTACLE]        = {HZK_06, HZK_07},  /* 避障 */
      [STATE_FINISH]          = {HZK_17, HZK_18},  /* 完成 */
      [STATE_BRANCH_ENTER]    = {0xFF, 0xFF},       /* BR-E */
      [STATE_BRANCH_FOLLOW]   = {0xFF, 0xFF},       /* BR-F */
      [STATE_BRANCH_JUDGE]    = {HZK_31, HZK_32},   /* 区域 */
      [STATE_BRANCH_REVERSE]  = {0xFF, 0xFF},       /* BR-REV */
      [STATE_ZONE_RETURN]    = {0xFF, 0xFF},       /* Z-RET */
  };

  uint8_t heartbeat = 0;
  uint8_t retry_tick = 0;

  for(;;)
  {
      if (OLED_IsOK())
      {
          /* ─── OLED 正常：显示全部信息 ─── */
          OLED_ClearBuffer();

          // 状态名（中文）
          if (state_cn_idx[g_chassis_state][0] != 0xFF) {
              OLED_ShowChinese(0, 0, state_cn_idx[g_chassis_state][0]);
              OLED_ShowChinese(18, 0, state_cn_idx[g_chassis_state][1]);
          } else {
              // ASCII fallback for states without Chinese names
              if (g_chassis_state == STATE_BRANCH_ENTER) {
                  OLED_ShowString(0, 0, (uint8_t*)"BR-ENT", 8, 1);
              } else if (g_chassis_state == STATE_BRANCH_FOLLOW) {
                  OLED_ShowString(0, 0, (uint8_t*)"BR-FLW", 8, 1);
              } else if (g_chassis_state == STATE_BRANCH_REVERSE) {
                  OLED_ShowString(0, 0, (uint8_t*)"BR-REV", 8, 1);
              } else if (g_chassis_state == STATE_ZONE_RETURN) {
                  OLED_ShowString(0, 0, (uint8_t*)"Z-RET", 8, 1);
              } else {
                  OLED_ShowString(0, 0, (uint8_t*)"SCAN", 16, 1);
              }
          }

                    // Zone info for BRANCH states
          if (g_chassis_state == STATE_BRANCH_JUDGE ||
              g_chassis_state == STATE_BRANCH_REVERSE ||
              g_chassis_state == STATE_ZONE_RETURN) {
              OLED_ShowChinese(36, 18, HZK_31);   /* qu */
              OLED_ShowChinese(54, 18, HZK_32);   /* yu */
              OLED_ShowNum(72, 18, g_branch_zone, 1, 16, 1);

              if (g_chassis_state == STATE_BRANCH_JUDGE) {
                  OLED_ShowChinese(0, 36, HZK_21);   /* hong */
                  OLED_ShowString(16, 36, (uint8_t*)":", 16, 1);
                  OLED_ShowNum(24, 36, g_branch_decision.total_red, 2, 16, 1);
                  OLED_ShowChinese(56, 36, HZK_08);   /* lv */
                  OLED_ShowString(72, 36, (uint8_t*)":", 16, 1);
                  OLED_ShowNum(80, 36, g_branch_decision.total_green, 2, 16, 1);
                  if (g_branch_decision.pick) {
                      OLED_ShowString(0, 54, (uint8_t*)"Pick:ripe ", 8, 1);
                  } else if (g_branch_decision.detected) {
                      OLED_ShowString(0, 54, (uint8_t*)"Skip:unripe", 8, 1);
                  } else {
                      OLED_ShowString(0, 54, (uint8_t*)"No target  ", 8, 1);
                  }
              } else {
                  OLED_ShowString(0, 36, (uint8_t*)"Total R:", 8, 1);
                  OLED_ShowNum(56, 36, g_red_count, 3, 16, 1);
                  OLED_ShowString(0, 54, (uint8_t*)"Total G:", 8, 1);
                  OLED_ShowNum(56, 54, g_green_count, 3, 16, 1);
              }
          } else {
              OLED_ShowString(0, 18, (uint8_t*)"R:", 16, 1);
              OLED_ShowNum(24, 18, g_red_count, 3, 16, 1);
              OLED_ShowString(64, 18, (uint8_t*)"G:", 16, 1);
              OLED_ShowNum(88, 18, g_green_count, 3, 16, 1);

              OLED_ShowString(0, 36, (uint8_t*)"V:", 16, 1);
              OLED_ShowFloat(24, 36, g_avg_speed_mps, 16, 1);
              OLED_ShowString(96, 36, (uint8_t*)"m/s", 16, 1);

              if (g_emergency_stop) {
                  OLED_ShowString(0, 54, (uint8_t*)"VOFA:OK !STOP!", 8, 1);
              } else {
                  OLED_ShowString(0, 54, (uint8_t*)"VOFA:OK", 8, 1);
              }
              OLED_ShowChar(120, 54, (heartbeat & 1) ? '*' : '.', 8, 1);
          }

          OLED_Refresh();

          /* 心跳蜂鸣 5Hz */
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
          osDelay(20);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
          osDelay(180);  /* 5Hz */
      }
      else
      {
          /* ─── OLED 故障：降级为蜂鸣器心跳 + 定期重试 ─── */
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
          osDelay(50);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
          osDelay(150);

          /* 每 ~3 秒重试初始化 */
          if (++retry_tick >= 15) {
              retry_tick = 0;
              OLED_Init();
          }
      }

      heartbeat++;
  }
  /* USER CODE END StartTaskOLED */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    extern uint8_t rx_buf[32];
    extern uint8_t rx_index;
    extern volatile uint8_t rx_flag;
    extern uint8_t rx_byte;
    extern uint8_t rx_byte_bt;

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
    // ── UART4: 蓝牙文本命令行（与 USART1 共用 rx_buf） ──
    else if (huart->Instance == UART4) {
        if (rx_byte_bt == '\n') {
            if (rx_index > 0) {
                rx_buf[rx_index] = '\0';
                rx_flag = 1;
                rx_index = 0;
            }
        } else if (rx_byte_bt != '\r') {
            if (rx_index < 31) {
                rx_buf[rx_index++] = rx_byte_bt;
            }
        }
        HAL_UART_Receive_IT(huart, &rx_byte_bt, 1);
    }
    // USART2 已改用 DMA + IDLE，不再走逐字节中断
}

/* ── USART2 DMA + IDLE 接收完成回调 ── */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART2) return;

    if (Size == 24) {
        // 校验帧头帧尾
        if (k210_dma_buf[0] == 0xAA && k210_dma_buf[23] == 0x55) {
            // XOR 校验：Count + 20B Target = 21 字节
            uint8_t x = 0;
            for (int i = 1; i <= 21; i++) x ^= k210_dma_buf[i];
            if (x == k210_dma_buf[22]) {
                // 校验通过 → 24B 完整帧入队
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                xQueueSendFromISR(xQueue_VisionData, k210_dma_buf,
                                  &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
    }

    // 重启 IDLE 接收
    HAL_UARTEx_ReceiveToIdle_DMA(huart, k210_dma_buf, 24);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
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
