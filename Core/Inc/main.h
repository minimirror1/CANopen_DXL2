/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LD_RUN_Pin GPIO_PIN_13
#define LD_RUN_GPIO_Port GPIOC
#define LD_ARM_Pin GPIO_PIN_14
#define LD_ARM_GPIO_Port GPIOC
#define LD_MRS_RX_Pin GPIO_PIN_2
#define LD_MRS_RX_GPIO_Port GPIOC
#define LD_MRS_TX_Pin GPIO_PIN_3
#define LD_MRS_TX_GPIO_Port GPIOC
#define USART2_EN_Pin GPIO_PIN_1
#define USART2_EN_GPIO_Port GPIOA
#define DXL_ID_01_Pin GPIO_PIN_5
#define DXL_ID_01_GPIO_Port GPIOA
#define DXL_ID_02_Pin GPIO_PIN_6
#define DXL_ID_02_GPIO_Port GPIOA
#define DXL_ID_04_Pin GPIO_PIN_7
#define DXL_ID_04_GPIO_Port GPIOA
#define DXL_ID_08_Pin GPIO_PIN_4
#define DXL_ID_08_GPIO_Port GPIOC
#define ZER_ID_01_Pin GPIO_PIN_5
#define ZER_ID_01_GPIO_Port GPIOC
#define ZER_ID_02_Pin GPIO_PIN_0
#define ZER_ID_02_GPIO_Port GPIOB
#define ZER_ID_04_Pin GPIO_PIN_1
#define ZER_ID_04_GPIO_Port GPIOB
#define ZER_ID_08_Pin GPIO_PIN_2
#define ZER_ID_08_GPIO_Port GPIOB
#define LD_DXL2_ERR_Pin GPIO_PIN_14
#define LD_DXL2_ERR_GPIO_Port GPIOB
#define LD_DXL2_RX_Pin GPIO_PIN_15
#define LD_DXL2_RX_GPIO_Port GPIOB
#define LD_DXL2_TX_Pin GPIO_PIN_6
#define LD_DXL2_TX_GPIO_Port GPIOC
#define LD_DXL1_ERR_Pin GPIO_PIN_7
#define LD_DXL1_ERR_GPIO_Port GPIOC
#define LD_DXL1_RX_Pin GPIO_PIN_8
#define LD_DXL1_RX_GPIO_Port GPIOC
#define LD_DXL1_TX_Pin GPIO_PIN_9
#define LD_DXL1_TX_GPIO_Port GPIOC
#define USART1_EN_Pin GPIO_PIN_8
#define USART1_EN_GPIO_Port GPIOA
#define LD_ZER_RX_Pin GPIO_PIN_15
#define LD_ZER_RX_GPIO_Port GPIOA
#define LD_ZER_TX_Pin GPIO_PIN_10
#define LD_ZER_TX_GPIO_Port GPIOC
#define LD_ZER_ERR_Pin GPIO_PIN_11
#define LD_ZER_ERR_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
#define MASTER_CAN_ID 0

/*----------------------------------------------------------------------------*/
#define CAN_Q_BUFF_SIZE 	512   //  ((id 4 Byte + data 8 Byte) x 512(CAN_Q_BUFF_SIZE)) x 2(rx,tx) = 12,288 Byte
// CAN 1개일 경우
#define CAN_1	0 //
#define CAN_CNT 1

void HAL_CAN_RxFifo0MsgPendingCallback1(CAN_HandleTypeDef *hcan);
#include "dl_can.h"
#include "net_phd_pid.h"

//#define CANOPEN_PP_MODE

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
