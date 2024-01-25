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
#include <string.h>

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

motorDXL dxl_1;
motorDXL dxl_2;

Init_TypeDef Dxl_All_init_flag = INIT_NONE;//0:none, 1:start init, 2:init ok, 3:init fail

extern osMessageQId dxlPosiHandle;
extern osMessageQueueId_t dxlCmd_rxHandle;
extern osMessageQueueId_t dxlCmd_txHandle;

DxlSetting_TypeDef dxlSetting[20] = {0,};

/* Private function prototypes -----------------------------------------------*/
void mrs_dxlrx_cmd_process(BypassPacket_TypeDef *cmd_rx);

void Serial_1_Init(void);
void Serial_2_Init(void);


void main_DXL(void *argument){

	Tick t_dxl1_run_led;
	Tick t_dxl2_run_led;

	MotionPacket_TypeDef motionMsg;
	BypassPacket_TypeDef cmd_rx;

	Serial_1_Init();
	Serial_2_Init();

	dxl_1.motorDXL_assign(&serial2, PROTOCOL_VERSION2, 1, 10);
	dxl_2.motorDXL_assign(&serial1, PROTOCOL_VERSION2, 1, 10);


#if 0//중간
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
#endif
#if 0//최종
	dxl_1.add_motor(1, DXL_ROT_CCW, 180, 3071, 2048);
	dxl_1.add_motor(2, DXL_ROT_CW, 90, 1535, 2048);
	dxl_1.add_motor(3, DXL_ROT_CW, 90, 1535, 2048);
	dxl_1.add_motor(4, DXL_ROT_CCW, 90, 2559, 2048);

	dxl_2.add_motor(5, DXL_ROT_CCW, 280, 3640, 2158);//
	dxl_2.add_motor(6, DXL_ROT_CCW, 120, 2047, 0);//
	dxl_2.add_motor(7, DXL_ROT_CCW, 272, 3594, 2048);
	dxl_2.add_motor(8, DXL_ROT_CW, 280, 455, 2054);//
	dxl_2.add_motor(9, DXL_ROT_CW, 120, 2047, 0);
	dxl_2.add_motor(10, DXL_ROT_CW, 272, 500, 2048);
#endif

/*	dxl_1.init();
	dxl_2.init();*/

	/* Infinite loop */
	for (;;) {

		osStatus_t status;

		status = osMessageQueueGet(dxlCmd_rxHandle, &cmd_rx, NULL, 0U); // wait for message
		if (status == osOK) {
			mrs_dxlrx_cmd_process(&cmd_rx);
		}

		if(Dxl_All_init_flag == INIT_INFO_DEFAULT_POSI_START){
			dxl_1.init();
			dxl_2.init();
			Dxl_All_init_flag = INIT_OK;		//init ok
		}
		else if(Dxl_All_init_flag == INIT_OK){
			do{
				status = osMessageQueueGet(dxlPosiHandle, &motionMsg, NULL, 0U); // wait for message
				if (status == osOK) {
					dxl_1.setPosition(motionMsg.sid, motionMsg.posi);
					dxl_2.setPosition(motionMsg.sid, motionMsg.posi);
				}
			}while(status == osOK);

			serial1.rxLed_Check();
			serial2.rxLed_Check();
		}
		osDelay(20);
		if(t_dxl1_run_led.delay(500))
			HAL_GPIO_TogglePin(LD_DXL1_ERR_GPIO_Port, LD_DXL1_ERR_Pin);
		if(t_dxl2_run_led.delay(500))
			HAL_GPIO_TogglePin(LD_DXL2_ERR_GPIO_Port, LD_DXL2_ERR_Pin);
	}
}

/* */

void mrs_dxlrx_cmd_process(BypassPacket_TypeDef *cmd_rx) {

	if (cmd_rx->sid > 12)
		return;

	switch (cmd_rx->cmd) {
	case MRS_RX_DATA1: {
		prtc_data_ctl_init_driver_data1_t *pData = (prtc_data_ctl_init_driver_data1_t*) cmd_rx->data;

		dxlSetting[cmd_rx->sid].f_data1 = true;
		dxlSetting[cmd_rx->sid].rot_dir = (pData->direction == 0 ? DXL_ROT_CW : DXL_ROT_CCW);
		dxlSetting[cmd_rx->sid].angle = (float) pData->angle / 100;
		dxlSetting[cmd_rx->sid].defult_posi = pData->init_position;

		BypassPacket_TypeDef msg;
		msg.gid = cmd_rx->gid;
		msg.sid = cmd_rx->sid;
		msg.cmd = MRS_TX_DATA1_ACK;
		memcpy(msg.data, (uint8_t *)pData, 8);
		osMessageQueuePut(dxlCmd_txHandle, &msg, 0U, 0U);

		break;
	}

//	case MRS_RX_DATA2: {
//		if (dxlSetting[cmd_rx->sid].f_data1 != true)
//			return;
//		dxlSetting[cmd_rx->sid].f_data1 = false; //f_data1 가 먼저 수신된 후 data2가 수신되었을때만 유효하다.
//
//		prtc_data_ctl_init_driver_data2_t *pData = (prtc_data_ctl_init_driver_data2_t*) cmd_rx->data;
//		dxlSetting[cmd_rx->sid].home_cnt = pData->count;
//
//		if( 1 <= cmd_rx->sid && cmd_rx->sid <= 4)
//			dxl_1.add_motor(
//					cmd_rx->sid,
//					dxlSetting[cmd_rx->sid].rot_dir,
//					dxlSetting[cmd_rx->sid].angle,
//					dxlSetting[cmd_rx->sid].home_cnt,
//					dxlSetting[cmd_rx->sid].defult_posi);
//		else if( 5 <= cmd_rx->sid && cmd_rx->sid <= 10)
//			dxl_2.add_motor(
//					cmd_rx->sid,
//					dxlSetting[cmd_rx->sid].rot_dir,
//					dxlSetting[cmd_rx->sid].angle,
//					dxlSetting[cmd_rx->sid].home_cnt,
//					dxlSetting[cmd_rx->sid].defult_posi);
//
//		BypassPacket_TypeDef msg;
//		msg.gid = cmd_rx->gid;
//		msg.sid = cmd_rx->sid;
//		msg.cmd = MRS_TX_DATA2_ACK;
//		memcpy(msg.data, (uint8_t *)pData, 8);
//		osMessageQueuePut(dxlCmd_txHandle, &msg, 0U, 0U);
//		break;
//	}
	case MRS_RX_DATA_OP : {
		if (dxlSetting[cmd_rx->sid].f_data1 != true)
			return;


		prtc_data_ctl_init_driver_data_op_dxl_t *pData = (prtc_data_ctl_init_driver_data_op_dxl_t*) cmd_rx->data;
		dxlSetting[cmd_rx->sid].home_cnt = pData->home_cnt;

		if( 1 <= cmd_rx->sid && cmd_rx->sid <= 4)
			dxl_1.add_motor(
					cmd_rx->sid,
					dxlSetting[cmd_rx->sid].rot_dir,
					dxlSetting[cmd_rx->sid].angle,
					dxlSetting[cmd_rx->sid].home_cnt,
					dxlSetting[cmd_rx->sid].defult_posi);
		else if( 5 <= cmd_rx->sid && cmd_rx->sid <= 10)
			dxl_2.add_motor(
					cmd_rx->sid,
					dxlSetting[cmd_rx->sid].rot_dir,
					dxlSetting[cmd_rx->sid].angle,
					dxlSetting[cmd_rx->sid].home_cnt,
					dxlSetting[cmd_rx->sid].defult_posi);

		BypassPacket_TypeDef msg;
		msg.gid = cmd_rx->gid;
		msg.sid = cmd_rx->sid;
		msg.cmd = MRS_TX_DATA_OP_ACK;
		memcpy(msg.data, (uint8_t *)pData, 8);
		osMessageQueuePut(dxlCmd_txHandle, &msg, 0U, 0U);

		break;
	}

	case MRS_RX_MOVE_DEFAULT_POSI: {

		if(cmd_rx->sid == 10){
			Dxl_All_init_flag = INIT_INFO_DEFAULT_POSI_START;
			dxlSetting[cmd_rx->sid].f_data1 = false; //f_data1 가 먼저 수신된 후 data2가 수신되었을때만 유효하다.
		}
		break;
	}

	case MRS_RX_MOVE_DEFAULT_POSI_CHECK: {
		if(Dxl_All_init_flag == INIT_OK){

			bool assign = false;
			if( 1 <= cmd_rx->sid && cmd_rx->sid <= 4)
				assign = dxl_1.getAssign(cmd_rx->sid);
			else if( 5 <= cmd_rx->sid && cmd_rx->sid <= 10)
				assign = dxl_2.getAssign(cmd_rx->sid);

			//모터가 연결되어야만 응답을 반환한다.
			if(assign == true){
				BypassPacket_TypeDef msg = {0,};
				msg.gid = cmd_rx->gid;
				msg.sid = cmd_rx->sid;
				msg.cmd = MRS_TX_MOVE_DEFAULT_POSI_CHECK;
				osMessageQueuePut(dxlCmd_txHandle, &msg, 0U, 0U);
			}
		}
		break;
	}
	default:
		break;
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
