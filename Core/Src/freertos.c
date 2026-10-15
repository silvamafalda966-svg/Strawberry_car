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
#include "mpu6050.h"

/* USART1 (USB/CH340) 调试开关：1=开启, 0=关闭省 CPU */
#define UART1_DEBUG 1  // 🔧 陀螺仪调试：USB 输出
#define USE_MPU6050 0  // 🔧 陀螺仪开关：1=启用, 0=禁用（不用陀螺仪设0，避免I2C超时拖慢PID）
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
volatile uint32_t k210_frame_count = 0; // 调试：收到的有效帧总数
// 传感器数据（Task_Sensor 写，Task_PID_Ctrl 读）
static SensorData_t g_sensor_data = {{1,1,1,1,1,1,1,1}, 300.0f};
/* USER CODE END Variables */
osThreadId Task_PID_CtrlHandle;
osThreadId Task_VisionHandle;
osThreadId Task_SensorHandle;
osThreadId Task_OLEDHandle;
QueueHandle_t xQueue_VisionData;   // 24B 定长帧队列（FreeRTOS 原生）
osMutexId Mutex_SensorDataHandle;
osMutexId Mutex_I2C2Handle;        // I2C2 总线互斥锁（OLED 专用，MPU6050 已迁 I2C1）

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

  /* I2C2 总线互斥锁（OLED + MPU6050 共享） */
  osMutexDef(Mutex_I2C2);
  Mutex_I2C2Handle = osMutexCreate(osMutex(Mutex_I2C2));

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
  osThreadDef(Task_Sensor, StartTaskSensor, osPriorityBelowNormal, 0, 256);
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
  // 1. 四轮初始 PID 参数（防死锁，与 PID_Init() 保持一致）
  // V2 自动调参 2026-07-17: 四轮统一 Kp=0.25 Ki=0.05 Kd=0.06
  pid_FL.direction = 1; pid_FL.Kp = 0.25f; pid_FL.Ki = 0.05f; pid_FL.Kd = 0.06f; pid_FL.Target = 0.0f;
  pid_BL.direction = 1; pid_BL.Kp = 0.25f; pid_BL.Ki = 0.05f; pid_BL.Kd = 0.06f; pid_BL.Target = 0.0f;
  pid_FR.direction = 1; pid_FR.Kp = 0.25f; pid_FR.Ki = 0.05f; pid_FR.Kd = 0.06f; pid_FR.Target = 0.0f;
  pid_BR.direction = 1; pid_BR.Kp = 0.25f; pid_BR.Ki = 0.05f; pid_BR.Kd = 0.06f; pid_BR.Target = 0.0f;

  // 2. 引用 USART1/UART4/UART5 句柄（中断已在 main.c 启动）
  extern UART_HandleTypeDef huart1;
  extern UART_HandleTypeDef huart4;
  extern UART_HandleTypeDef huart5;
  extern I2C_HandleTypeDef hi2c1;

  // 3. 状态机初始化
  Chassis_Init();

  // 4. MPU6050 陀螺仪初始化（已禁用 — USE_MPU6050=0）
#if USE_MPU6050
  int mpu_ok = 0;
  float self_test_result[6] = {0};
  float gyro_bias_dps = 0.0f;

  for (int retry = 0; retry < 3; retry++) {
      mpu_ok = MPU6050_Init(&hi2c1);
      if (mpu_ok) break;
      osDelay(50);
  }
  if (mpu_ok) {
      // Step A: 自检（验证芯片健康）
      uint8_t st_pass = MPU6050_SelfTest(&hi2c1, self_test_result);

      // Step B: 预热等待（温度稳定后自动校准）
      osDelay(500);  // 等自检后芯片恢复
      float last_temp = MPU6050_ReadTemp(&hi2c1);
      // 如果温度读失败（返回 0），说明 I2C 还没稳，重试
      if (last_temp < 10.0f) {
          osDelay(1000);
          last_temp = MPU6050_ReadTemp(&hi2c1);
      }
      int warmup_sec = 0, stable_cnt = 0;
      const int WARMUP_MAX_SEC = 120;
      char w_line[32];

      while (warmup_sec < WARMUP_MAX_SEC) {
          osDelay(5000);
          warmup_sec += 5;
          float cur_temp = MPU6050_ReadTemp(&hi2c1);
          if (cur_temp < 10.0f) cur_temp = last_temp;  // 读失败保持上次值
          float delta = (cur_temp > last_temp) ? (cur_temp - last_temp)
                                               : (last_temp - cur_temp);
          last_temp = cur_temp;

          if (delta < 0.2f) {
              if (++stable_cnt >= 3) break;
          } else {
              stable_cnt = 0;
          }

          int pct = (warmup_sec * 100) / WARMUP_MAX_SEC;
          int wlen = snprintf(w_line, sizeof(w_line), "W:%d,%d\n",
                              pct, (int)(cur_temp * 10.0f));
          if (wlen > 0 && wlen < (int)sizeof(w_line)) {
              HAL_UART_Transmit(&huart1, (uint8_t*)w_line, wlen, 10);
          }
      }

      // Step C: FIFO 零偏校准（写入硬件偏移寄存器）
      gyro_bias_dps = MPU6050_Calibrate(&hi2c1, 80);
      g_mpu6050_alive = 1;  // 校准成功，标记陀螺仪可用

      (void)st_pass;
      (void)gyro_bias_dps;
  }
  float yaw_deg = g_mpu6050_yaw;  // yaw 积分变量（从上次保留值继续）

  // 调试：M: 行（包含更多信息）
  {
      char dbg[64];
      int dlen = snprintf(dbg, sizeof(dbg), "M:%d,%d,%d,%d\n",
                          mpu_ok,
                          (int)(self_test_result[5]),  // 陀螺 Z 自检%
                          (int)(gyro_bias_dps * 100.0f),  // 零偏 (厘度)
                          (int)(g_mpu6050_alive));
      if (dlen > 0 && dlen < (int)sizeof(dbg)) {
          HAL_UART_Transmit(&huart1, (uint8_t*)dbg, dlen, 10);
      }
  }
#else
  int mpu_ok = 0;
  (void)mpu_ok;
#endif /* USE_MPU6050 */

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
      // 🔒 fallback 用上次成功值，不用 memset(0)！
      //    memset(0) → ir全黑 → 假岔路口 → 误拐出赛道
      static SensorData_t last_local_sensor = {{1,1,1,1,1,1,1,1}, 300.0f};
      SensorData_t local_sensor;
      if (osMutexWait(Mutex_SensorDataHandle, 0) == osOK) {
          local_sensor = g_sensor_data;
          last_local_sensor = g_sensor_data;  // 存档，供下次 fallback
          osMutexRelease(Mutex_SensorDataHandle);
      } else {
          local_sensor = last_local_sensor;   // 锁不到用上次值
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
          char d_line[48];
          int len = snprintf(d_line, sizeof(d_line),
                             "D:%d,%d,%d,%d,%d,%d,%d\n",
                             g_branch_decision.pick,
                             g_branch_decision.cls,
                             g_branch_decision.area,
                             g_branch_decision.total_red,
                             g_branch_decision.total_green,
                             g_branch_decision.frames_in,
                             g_branch_decision.frames_ok);
          if (len > 0 && len < (int)sizeof(d_line)) {
              if (huart5.gState == HAL_UART_STATE_READY) {
                  HAL_UART_Transmit(&huart5, (uint8_t*)d_line, len, 5);
              }
#if UART1_DEBUG
              HAL_UART_Transmit(&huart1, (uint8_t*)d_line, len, 10);
#endif
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
                  HAL_UART_Transmit(&huart5, (uint8_t*)c_line, clen, 5);
              }
#if UART1_DEBUG
              HAL_UART_Transmit(&huart1, (uint8_t*)c_line, clen, 10);
#endif
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
                          HAL_UART_Transmit(&huart5, (uint8_t*)k_line, klen, 5);
                      }
#if UART1_DEBUG
                      HAL_UART_Transmit(&huart1, (uint8_t*)k_line, klen, 10);
#endif
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
      vofa_buf[6] = g_mpu6050_yaw;  // 陀螺仪 yaw 角度 (°)，替换编码器
      uint32_t tail = 0x7F800000;     // 保持纯 +Inf 帧尾，兼容 car-studio 帧同步
      vofa_buf[7] = *(float*)&tail;

      // 状态变更时发文本行（car-studio 用，VOFA+ 忽略）
      static ChassisState_t last_reported_state = STATE_COUNT;
      if (g_chassis_state != last_reported_state) {
          char s_line[8];
          int slen = snprintf(s_line, sizeof(s_line), "S:%d\n", (int)g_chassis_state);
          if (slen > 0 && slen < (int)sizeof(s_line)) {
              if (huart5.gState == HAL_UART_STATE_READY) {
                  HAL_UART_Transmit(&huart5, (uint8_t*)s_line, slen, 5);
              }
#if UART1_DEBUG
              HAL_UART_Transmit(&huart1, (uint8_t*)s_line, slen, 10);
#endif
          }
          last_reported_state = g_chassis_state;
      }

      // ── VOFA+ 帧发送 ──
      if (huart5.gState == HAL_UART_STATE_READY) {
          HAL_UART_Transmit(&huart5, (uint8_t*)vofa_buf, sizeof(vofa_buf), 5);
      }
#if UART1_DEBUG
      HAL_UART_Transmit(&huart1, (uint8_t*)vofa_buf, sizeof(vofa_buf), 10);
#endif

      // ── 调试：V: K210帧计数器（1Hz） ──
      static uint8_t vdbg_tick = 0;
      if (++vdbg_tick >= 100) {
          vdbg_tick = 0;
          extern volatile uint32_t k210_frame_count;
          char v_line[16];
          int vlen = snprintf(v_line, sizeof(v_line), "V:%d\n", (int)k210_frame_count);
          if (vlen > 0 && vlen < (int)sizeof(v_line)) {
#if UART1_DEBUG
              HAL_UART_Transmit(&huart1, (uint8_t*)v_line, vlen, 10);
#endif
          }
      }

      // ── H2. MPU6050 陀螺仪读取 + yaw 积分 ──
#if USE_MPU6050
      if (mpu_ok) {
          uint8_t gz_ok = 0;
          float gz = MPU6050_ReadGyroZ(&hi2c1, &gz_ok);
          if (gz_ok) {
              g_mpu6050_alive = 1;

              // 真实 dt（FreeRTOS tick，毫秒级精度）
              static TickType_t last_tick = 0;
              TickType_t now = xTaskGetTickCount();
              float dt = (float)(now - last_tick) / 1000.0f;
              last_tick = now;
              if (dt > 0.1f || dt <= 0.0f) dt = 0.01f;

              // 静止检测：陀螺仪方差法（连续 N 帧波动 < 阈值 = 真静止）
              static float gz_last = 0.0f;
              static uint8_t still_cnt = 0;
              float gz_var = (gz > gz_last) ? (gz - gz_last) : (gz_last - gz);
              gz_last = gz;
              if (gz_var < 0.15f) {  // < 0.15°/s 波动 = 静止
                  if (still_cnt < 255) still_cnt++;
              } else {
                  still_cnt = 0;
              }

              // 静止时小死区（滤除零偏残余和噪声）
              if (still_cnt >= 10) {
                  if (gz > -0.08f && gz < 0.08f) gz = 0.0f;
              }

              // 积分（硬件零偏已自动扣除 + 软件低通已平滑 + 灵敏度已修正）
              yaw_deg += gz * dt;
              // 回卷到 [0, 360)，方便 VOFA+ 观看
              while (yaw_deg >=  360.0f) yaw_deg -= 360.0f;
              while (yaw_deg <     0.0f) yaw_deg += 360.0f;
              g_mpu6050_yaw = yaw_deg;
          } else {
              g_mpu6050_alive = 0;
          }
      }
#endif /* USE_MPU6050 */

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
  extern UART_HandleTypeDef huart1;
  extern UART_HandleTypeDef huart5;
  /* MPU6050 moved to PID task (100Hz). Sensor task: IR + Y: report only */
  for(;;)
  {
      SensorData_t sensor;
      uint32_t idr = GPIOE->IDR;
      sensor.ir[0] = (idr & GPIO_PIN_0) ? 0 : 1;
      sensor.ir[1] = (idr & GPIO_PIN_1) ? 0 : 1;
      sensor.ir[2] = (idr & GPIO_PIN_2) ? 0 : 1;
      sensor.ir[3] = (idr & GPIO_PIN_3) ? 0 : 1;
      sensor.ir[4] = (idr & GPIO_PIN_4) ? 0 : 1;
      sensor.ir[5] = (idr & GPIO_PIN_5) ? 0 : 1;
      sensor.ir[6] = 1;
      sensor.ir[7] = 1;
      sensor.distance_cm = 300.0f;

      /* Y: yaw report (10Hz, USB + Bluetooth) */
      static uint8_t yaw_tick = 0;
      if (++yaw_tick >= 5) {
          yaw_tick = 0;
          char y_line[32];
          int ylen = snprintf(y_line, sizeof(y_line), "Y:%d,%d\n",
                              (int)(g_mpu6050_yaw * 100.0f),
                              g_mpu6050_alive);
          if (ylen > 0 && ylen < (int)sizeof(y_line)) {
              HAL_UART_Transmit(&huart1, (uint8_t*)y_line, ylen, 5);
              if (huart5.gState == HAL_UART_STATE_READY) {
                  HAL_UART_Transmit(&huart5, (uint8_t*)y_line, ylen, 5);
              }
          }
      }

      if (osMutexWait(Mutex_SensorDataHandle, osWaitForever) == osOK) {
          g_sensor_data = sensor;
          osMutexRelease(Mutex_SensorDataHandle);
      }

      osDelay(20);
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
  /* I2C2 互斥保护 OLED 初始化 */
  if (osMutexWait(Mutex_I2C2Handle, 50) == osOK) {
      OLED_Init();
      osMutexRelease(Mutex_I2C2Handle);
  }

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

          /* I2C2 互斥：OLED 刷屏期间阻止 MPU6050 访问 */
          if (osMutexWait(Mutex_I2C2Handle, 10) == osOK) {
              OLED_Refresh();
              osMutexRelease(Mutex_I2C2Handle);
          }

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
              if (osMutexWait(Mutex_I2C2Handle, 10) == osOK) {
                  OLED_Init();
                  osMutexRelease(Mutex_I2C2Handle);
              }
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
                k210_frame_count++;  /* 调试计数器 */
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
    // ── 堵转保护：PWM>500 且速度≈0 持续 500ms → 切断输出 ──
    if (pid->out_PWM > 500.0f || pid->out_PWM < -500.0f) {
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
