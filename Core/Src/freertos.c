/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "main_DXL.h"
#include "main_MRS.h"
#include "main_ZeroErr.h"
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MRS_Task */
osThreadId_t MRS_TaskHandle;
const osThreadAttr_t MRS_Task_attributes = {
  .name = "MRS_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh6,
};
/* Definitions for zeroErrTask */
osThreadId_t zeroErrTaskHandle;
const osThreadAttr_t zeroErrTask_attributes = {
  .name = "zeroErrTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for DXL_Task */
osThreadId_t DXL_TaskHandle;
const osThreadAttr_t DXL_Task_attributes = {
  .name = "DXL_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for zerPosi */
osMessageQueueId_t zerPosiHandle;
const osMessageQueueAttr_t zerPosi_attributes = {
  .name = "zerPosi"
};
/* Definitions for dxlPosi */
osMessageQueueId_t dxlPosiHandle;
const osMessageQueueAttr_t dxlPosi_attributes = {
  .name = "dxlPosi"
};
/* Definitions for zerCmd_rx */
osMessageQueueId_t zerCmd_rxHandle;
const osMessageQueueAttr_t zerCmd_rx_attributes = {
  .name = "zerCmd_rx"
};
/* Definitions for zerCmd_tx */
osMessageQueueId_t zerCmd_txHandle;
const osMessageQueueAttr_t zerCmd_tx_attributes = {
  .name = "zerCmd_tx"
};
/* Definitions for dxlCmd_rx */
osMessageQueueId_t dxlCmd_rxHandle;
const osMessageQueueAttr_t dxlCmd_rx_attributes = {
  .name = "dxlCmd_rx"
};
/* Definitions for dxlCmd_tx */
osMessageQueueId_t dxlCmd_txHandle;
const osMessageQueueAttr_t dxlCmd_tx_attributes = {
  .name = "dxlCmd_tx"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
extern void main_MRS(void *argument);
extern void main_ZeroErr(void *argument);
extern void main_DXL(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

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
  /* creation of zerPosi */
  zerPosiHandle = osMessageQueueNew (40, sizeof(MotionPacket_TypeDef), &zerPosi_attributes);

  /* creation of dxlPosi */
  dxlPosiHandle = osMessageQueueNew (40, sizeof(MotionPacket_TypeDef), &dxlPosi_attributes);

  /* creation of zerCmd_rx */
  zerCmd_rxHandle = osMessageQueueNew (40, sizeof(BypassPacket_TypeDef), &zerCmd_rx_attributes);

  /* creation of zerCmd_tx */
  zerCmd_txHandle = osMessageQueueNew (40, sizeof(BypassPacket_TypeDef), &zerCmd_tx_attributes);

  /* creation of dxlCmd_rx */
  dxlCmd_rxHandle = osMessageQueueNew (40, sizeof(BypassPacket_TypeDef), &dxlCmd_rx_attributes);

  /* creation of dxlCmd_tx */
  dxlCmd_txHandle = osMessageQueueNew (40, sizeof(BypassPacket_TypeDef), &dxlCmd_tx_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of MRS_Task */
  MRS_TaskHandle = osThreadNew(main_MRS, NULL, &MRS_Task_attributes);

  /* creation of zeroErrTask */
  zeroErrTaskHandle = osThreadNew(main_ZeroErr, NULL, &zeroErrTask_attributes);

  /* creation of DXL_Task */
  DXL_TaskHandle = osThreadNew(main_DXL, NULL, &DXL_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

