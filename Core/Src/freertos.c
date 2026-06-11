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
// K210 DMA 定长 24B 接收缓冲区
uint8_t  k210_dma_buf[24];
volatile uint8_t k210_frame_ready = 0;  // IDLE 中断置位，PID任务消费
// 传感器数据（Task_Sensor 写，Task_PID_Ctrl 读）
static SensorData_t g_sensor_data = {{0}, 300.0f};
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
  pid_FL.direction = 1; pid_FL.Kp = 0.35f; pid_FL.Ki = 0.06f; pid_FL.Kd = 0.04f; pid_FL.Target = 0.0f;
  pid_BL.direction = 1; pid_BL.Kp = 0.35f; pid_BL.Ki = 0.06f; pid_BL.Kd = 0.04f; pid_BL.Target = 0.0f;
  pid_FR.direction = 1; pid_FR.Kp = 0.35f; pid_FR.Ki = 0.06f; pid_FR.Kd = 0.04f; pid_FR.Target = 0.0f;
  pid_BR.direction = 1; pid_BR.Kp = 0.50f; pid_BR.Ki = 0.04f; pid_BR.Kd = 0.04f; pid_BR.Target = 0.0f;

  // 2. 启动 USART1 VOFA+ 中断
  extern UART_HandleTypeDef huart1;
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

  // 3. 状态机初始化
  Chassis_Init();

  for(;;)
  {
      // ── A. VOFA+ 命令解析 ──
      if (rx_flag == 1) {
          Chassis_VOFA_Parse(rx_buf);
          rx_flag = 0;
      }

      // ── B. 编码器采集（四路独立） ──
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
      g_avg_speed_mps = (fabsf(speed_FL_mps) + fabsf(speed_FR_mps)
                      +  fabsf(speed_BL_mps) + fabsf(speed_BR_mps)) / 4.0f;

      // ── 累积编码器里程（四轮均值，有符号） ──
      g_enc_total += (int32_t)(dFL + dFR + dBL + dBR) / 4;
      // ── 四轮编码器绝对值累加（用于精算转角，不受方向影响） ──
      g_turn_enc += (int32_t)(abs(dFL) + abs(dFR) + abs(dBL) + abs(dBR));

      // ── C. 视觉队列检查（仅需 K210 的状态才取队列） ──
      VisionFrame_t vis_frame;
      memset(&vis_frame, 0, sizeof(vis_frame));
      uint8_t has_vision = 0;
      if (g_chassis_state == STATE_LINE_FOLLOW ||
          g_chassis_state == STATE_BREAKAWAY ||
          g_chassis_state == STATE_VISUAL_APPROACH ||
          g_chassis_state == STATE_SCANNING) {
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
      Chassis_StateMachine_Run(&local_sensor, has_vision, &vis_frame,
                               (int32_t)dFL, (int32_t)dFR,
                               (int32_t)dBL, (int32_t)dBR);

      // ── G. PID 计算 + 电机下发（四轮统一） ──
      float pwm_FL = CalcOneWheel(&pid_FL, speed_FL_mps);
      float pwm_FR = CalcOneWheel(&pid_FR, speed_FR_mps);
      float pwm_BL = CalcOneWheel(&pid_BL, speed_BL_mps);
      float pwm_BR = CalcOneWheel(&pid_BR, speed_BR_mps);

      Motor_FL_SetSpeed((int)pwm_FL);
      Motor_FR_SetSpeed((int)pwm_FR);
      Motor_BL_SetSpeed((int)pwm_BL);
      Motor_BR_SetSpeed((int)pwm_BR);

      // ── H. VOFA+ 上传（8 通道：4轮速度 + IR字节 + 超声波距离 + 编码器里程 + 帧尾） ──
      float vofa_buf[8];
      vofa_buf[0] = speed_FL_mps;
      vofa_buf[1] = speed_FR_mps;
      vofa_buf[2] = speed_BL_mps;
      vofa_buf[3] = speed_BR_mps;
      uint8_t ir_byte = 0;
      for (int i = 0; i < 8; i++) {
          if (local_sensor.ir[i]) ir_byte |= (1 << i);
      }
      vofa_buf[4] = (float)ir_byte;
      vofa_buf[5] = local_sensor.distance_cm;
      vofa_buf[6] = (float)g_enc_total;
      uint32_t tail = 0x7F800000;
      vofa_buf[7] = *(float*)&tail;
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
      // 读 8 路红外 PE0~PE7（不读超声波，避免阻塞抢占 PID 任务）
      SensorData_t sensor;
      uint32_t idr = GPIOE->IDR;
      sensor.ir[0] = (idr & GPIO_PIN_0) ? 1 : 0;
      sensor.ir[1] = (idr & GPIO_PIN_1) ? 1 : 0;
      sensor.ir[2] = (idr & GPIO_PIN_2) ? 1 : 0;
      sensor.ir[3] = (idr & GPIO_PIN_3) ? 1 : 0;
      sensor.ir[4] = (idr & GPIO_PIN_4) ? 1 : 0;
      sensor.ir[5] = (idr & GPIO_PIN_5) ? 1 : 0;
      sensor.ir[6] = (idr & GPIO_PIN_6) ? 1 : 0;
      sensor.ir[7] = (idr & GPIO_PIN_7) ? 1 : 0;
      sensor.distance_cm = 300.0f; // 由 PID 任务实测，此处占位

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
      [STATE_GREEN_FILTER]    = {HZK_08, HZK_09},  /* 绿滤 */
      [STATE_BREAKAWAY]       = {HZK_10, HZK_11},  /* 脱离 */
      [STATE_VISUAL_APPROACH] = {HZK_12, HZK_13},  /* 接近 */
      [STATE_SCANNING]        = {HZK_29, HZK_30},  /* 扫描 */
      [STATE_DOCKING]         = {HZK_14, HZK_15},  /* 对接 */
      [STATE_BLIND_RETURN]    = {HZK_16, HZK_17},  /* 盲返 */
      [STATE_FINISH]          = {HZK_18, HZK_19},  /* 完成 */
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
              // STATE_SCANNING ASCII fallback
              OLED_ShowString(0, 0, (uint8_t*)"SCAN", 16, 1);
          }

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

          OLED_Refresh();

          /* 蜂鸣器：DOCKING 时由 chassis.c run_docking 直接控制；
             其他状态保持心跳蜂鸣 */
          if (g_chassis_state != STATE_DOCKING) {
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
              osDelay(20);
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
              osDelay(180);  /* 5Hz */
          } else {
              osDelay(200);  /* DOCKING 时 OLED 跳过蜂鸣，仅刷新 */
          }
      }
      else
      {
          /* ─── OLED 故障：降级为蜂鸣器心跳 + 定期重试 ─── */
          if (g_chassis_state != STATE_DOCKING) {
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
              osDelay(50);
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
              osDelay(150);
          } else {
              osDelay(200);  /* DOCKING 时不碰蜂鸣 */
          }

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
