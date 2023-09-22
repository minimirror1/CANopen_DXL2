/*
 * main_DXL.cpp
 *
 *  Created on: Sep 19, 2023
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
#include "main_DXL.h"
#include "main_MRS.h"

/* Component -----------------------------------------------------------------*/
#include "cpp_serial.h"
#include "cpp_tick.h"

/* Robotis SDK ---------------------------------------------------------------*/
#include "DynamixelSDK.h"

/* Robotis Class --------------------------------------------------------------*/
#include "motor_DXL.h"

/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart1;	//DXL 1 = ARM
extern UART_HandleTypeDef huart2;	//DXL 2 = NACK
Serial serial1;	//huart1
Serial serial2;	//huart2

extern osMessageQId dxlPosiHandle;

/* Private function prototypes -----------------------------------------------*/
void Serial_1_Init(void);
void Serial_2_Init(void);


void main_DXL(void *argument){

	MotionPacket_TypeDef motionMsg;

	Serial_1_Init();
	Serial_2_Init();

	motorDXL dxl_1(&serial1, PROTOCOL_VERSION2, 1, 10);
	motorDXL dxl_2(&serial2, PROTOCOL_VERSION2, 1, 10);

	dxl_1.add_motor(1, DXL_ROT_CCW, 180, 3071, 2048);
	dxl_1.add_motor(2, DXL_ROT_CW, 90, 1535, 2048);
	dxl_1.add_motor(3, DXL_ROT_CW, 90, 1535, 2048);
	dxl_1.add_motor(4, DXL_ROT_CCW, 90, 2559, 2048);

	dxl_2.add_motor(5, DXL_ROT_CCW, 280, 3640, 2048);
	dxl_2.add_motor(6, DXL_ROT_CCW, 120, 2047, 2048);
	dxl_2.add_motor(7, DXL_ROT_CCW, 272, 3594, 2048);
	dxl_2.add_motor(8, DXL_ROT_CW, 280, 455, 2048);
	dxl_2.add_motor(9, DXL_ROT_CW, 120, 2047, 2048);
	dxl_2.add_motor(10, DXL_ROT_CW, 272, 500, 2048);


	dxl_1.init();
	dxl_2.init();

	/* Infinite loop */
	for (;;) {

		osStatus_t status;
		do{
			status = osMessageQueueGet(dxlPosiHandle, &motionMsg, NULL, 0U); // wait for message
			if (status == osOK) {
				dxl_1.setPosition(motionMsg.sid, motionMsg.posi);
				dxl_2.setPosition(motionMsg.sid, motionMsg.posi);
			}
		}while(status == osOK);

		osDelay(20);
		serial1.rxLed_Check();
		serial2.rxLed_Check();
	}
}

/* HAL Driver Callback */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	serial1.TxCpltCallback(huart);
	serial2.TxCpltCallback(huart);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	serial1.RxCpltCallback(huart);
	serial2.RxCpltCallback(huart);
}



/**
  * @brief Serial 1 Init
  * @retval None
  */
void Serial_1_Init(void){
	serial1.init(&huart1, USART1_IRQn);
	serial1.init_txLed(LD_DXL1_TX_GPIO_Port, LD_DXL1_TX_Pin, GPIO_PIN_RESET);
	serial1.init_rxLed(LD_DXL1_RX_GPIO_Port, LD_DXL1_RX_Pin, GPIO_PIN_RESET);
	serial1.init_rs485(USART1_EN_GPIO_Port, USART1_EN_Pin);
	HAL_GPIO_WritePin(LD_DXL1_ERR_GPIO_Port, LD_DXL1_ERR_Pin, GPIO_PIN_SET);
}
/**
  * @brief Serial 2 Init
  * @retval None
  */
void Serial_2_Init(void){
	serial2.init(&huart2, USART2_IRQn);
	serial2.init_txLed(LD_DXL2_TX_GPIO_Port, LD_DXL2_TX_Pin, GPIO_PIN_RESET);
	serial2.init_rxLed(LD_DXL2_RX_GPIO_Port, LD_DXL2_RX_Pin, GPIO_PIN_RESET);
	serial2.init_rs485(USART2_EN_GPIO_Port, USART2_EN_Pin);
	HAL_GPIO_WritePin(LD_DXL2_ERR_GPIO_Port, LD_DXL2_ERR_Pin, GPIO_PIN_SET);
}
