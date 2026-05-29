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

/* USER CODE END Variables */
osThreadId Task_PID_CtrlHandle;
osThreadId Task_VisionHandle;
osThreadId Task_SensorHandle;
osThreadId Task_OLEDHandle;
osMessageQId Queue_VisionDataHandle;
osMutexId Mutex_SensorDataHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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

/* USER CODE END Application */
