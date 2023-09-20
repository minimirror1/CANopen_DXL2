/*
 * main_MRS.cpp
 *
 *  Created on: Sep 20, 2023
 *      Author: minim
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* RTOS ----------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

/* User Task -----------------------------------------------------------------*/
#include "main_MRS.h"

/* Component -----------------------------------------------------------------*/
#include "cpp_tick.h"

/* MRS -----------------------------------------------------------------------*/
#include "app_pid_motion_cmd.h"

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void CAN_FilterConfig(CAN_HandleTypeDef *hcan);
uint8_t idRead(
		GPIO_TypeDef* GPIO_01, uint16_t Pin_01,
		GPIO_TypeDef* GPIO_02, uint16_t Pin_02,
		GPIO_TypeDef* GPIO_04, uint16_t Pin_04,
		GPIO_TypeDef* GPIO_08, uint16_t Pin_08,
		GPIO_PinState setState);
uint8_t MRS_DXL_id = 0;
uint8_t MRS_ZER_id = 0;

void main_MRS(void *argument){

	/* MRS protocol */
	can_init_data_save(&hcan1);
	gm_motion_TX_LED_init(LD_MRS_TX_GPIO_Port, LD_MRS_TX_Pin, GPIO_PIN_RESET);
	gm_motion_RX_LED_init(LD_MRS_RX_GPIO_Port, LD_MRS_RX_Pin, GPIO_PIN_RESET);

	CAN_FilterConfig(&hcan1);

	MRS_DXL_id = idRead(
			DXL_ID_01_GPIO_Port, DXL_ID_01_Pin,
			DXL_ID_02_GPIO_Port, DXL_ID_02_Pin,
			DXL_ID_04_GPIO_Port, DXL_ID_04_Pin,
			DXL_ID_08_GPIO_Port, DXL_ID_08_Pin,
			GPIO_PIN_RESET);
	MRS_ZER_id = idRead(
			ZER_ID_01_GPIO_Port, ZER_ID_01_Pin,
			ZER_ID_02_GPIO_Port, ZER_ID_02_Pin,
			ZER_ID_04_GPIO_Port, ZER_ID_04_Pin,
			ZER_ID_08_GPIO_Port, ZER_ID_08_Pin,
			GPIO_PIN_RESET);

	//myCanid = idRead();

	//set_my_can_id(myCanid);
	add_my_can_sub_id(1, 28);

	while(1){
		osDelay(1);
		/* MRS Protocol */
		proc_can_rx();
		proc_can_tx();
	}
}



void CAN_FilterConfig(CAN_HandleTypeDef *hcan)
{
	CAN_FilterTypeDef  sFilterConfig;

	/*##-2- Configure the CAN Filter ###########################################*/
#ifdef CAN1
	if(hcan->Instance == CAN1)
#else
	if(hcan->Instance == CAN)
#endif
	{
		sFilterConfig.FilterBank = 0;
		sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
		sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
		sFilterConfig.FilterIdHigh = 0x0000;
		sFilterConfig.FilterIdLow = 0x0000;
		sFilterConfig.FilterMaskIdHigh = 0x0000;
		sFilterConfig.FilterMaskIdLow = 0x0000;
		sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
		sFilterConfig.FilterActivation = ENABLE;
		sFilterConfig.SlaveStartFilterBank = 14;
	}
	else
	{
		sFilterConfig.FilterBank = 14;
		sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
		sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
		sFilterConfig.FilterIdHigh = 0x0000;
		sFilterConfig.FilterIdLow = 0x0000;
		sFilterConfig.FilterMaskIdHigh = 0x0000;
		sFilterConfig.FilterMaskIdLow = 0x0000;
		sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
		sFilterConfig.FilterActivation = ENABLE;
		sFilterConfig.SlaveStartFilterBank = 14;
	}

	if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
	{
		/* Filter configuration Error */
		Error_Handler();
	}

	/*##-3- Start the CAN peripheral ###########################################*/
	if (HAL_CAN_Start(hcan) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}

	/*##-4- Activate CAN RX notification #######################################*/
	if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		/* Notification Error */
		Error_Handler();
	}
}

uint8_t idRead(
		GPIO_TypeDef* GPIO_01, uint16_t Pin_01,
		GPIO_TypeDef* GPIO_02, uint16_t Pin_02,
		GPIO_TypeDef* GPIO_04, uint16_t Pin_04,
		GPIO_TypeDef* GPIO_08, uint16_t Pin_08,
		GPIO_PinState setState){
	uint8_t ret = 0;
	ret += (HAL_GPIO_ReadPin(GPIO_01, Pin_01) == setState)? 1:0;
	ret += (HAL_GPIO_ReadPin(GPIO_02, Pin_02) == setState)? 2:0;
	ret += (HAL_GPIO_ReadPin(GPIO_04, Pin_04) == setState)? 4:0;
	ret += (HAL_GPIO_ReadPin(GPIO_08, Pin_08) == setState)? 8:0;

	return ret;
}

