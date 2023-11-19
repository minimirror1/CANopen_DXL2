/*
 * main_ZeroErr.cpp
 *
 *  Created on: Sep 21, 2023
 *      Author: minim
 */


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* RTOS ----------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

/* User Task -----------------------------------------------------------------*/
#include "main_ZeroErr.h"
#include "main_MRS.h"

/* Component -----------------------------------------------------------------*/
#include "cpp_serial.h"
#include "cpp_tick.h"

/* CANopen SDK ---------------------------------------------------------------*/
#include "../CANopenNode_STM32/CO_app_STM32.h"
#include "OD.h"
#include "sdo.h"
#include "pdo.h"

/* Motor Class ---------------------------------------------------------------*/
#include "CANopen_Motor.h"

/* Private variables ---------------------------------------------------------*/
extern CO_t* CO;
Motors motors;

uint8_t All_init_flag = 0;//0:none, 1:start init, 2:init ok, 3:init fail

ZerSetting_TypeDef zerSetting[20] = {0,};

extern osMessageQueueId_t zerPosiHandle;
extern osMessageQueueId_t zerCmd_rxHandle;
extern osMessageQueueId_t zerCmd_txHandle;

/* Private function prototypes -----------------------------------------------*/
void CANopenNode_Init(void);
void mrs_rx_cmd_process(BypassPacket_TypeDef *cmd_rx);

void main_ZeroErr(void *argument){


	osDelay(10000);
	MotionPacket_TypeDef motionMsg;
	BypassPacket_TypeDef cmd_rx;

	/* CANopen Init */
	//CANopenNode_Init();

	CANopenNodeSTM32 canOpenNodeSTM32;
	canOpenNodeSTM32.CANHandle = &hcan2;
	canOpenNodeSTM32.HWInitFunction = MX_CAN2_Init;
	canOpenNodeSTM32.timerHandle = &htim14;
	canOpenNodeSTM32.desiredNodeID = 60;	//0x01;
	canOpenNodeSTM32.baudrate = 1000;		//1Mbps
	canopen_app_init(&canOpenNodeSTM32);
	TPDO1_rx_init();

	CO_NMT_t *NMTmaster = CO->NMT;
	NMTmaster->internalCommand = CO_NMT_ENTER_OPERATIONAL;

//	motors.motorsInit(CO, 1, 12);
	motors.motorsInit(CO, 10, 10);
	motors.init_status_led(LD_ZER_ERR_GPIO_Port, LD_ZER_ERR_Pin, GPIO_PIN_RESET);

#if 0
	motors.add_motor(1 , ROT_CW  ,90  ,262144, 262144);
	motors.add_motor(2 , ROT_CCW ,90  ,174763, 174763);
	motors.add_motor(3 , ROT_CW  ,60  ,174763, 174763);
	motors.add_motor(4 , ROT_CW  ,130 ,174763, 174763);

	motors.add_motor(5 , ROT_CCW ,210 ,262144, 262144);
	motors.add_motor(6 , ROT_CW  ,160 ,262144, 262144);
	motors.add_motor(7 , ROT_CCW ,60  ,262144, 262144);
	motors.add_motor(8 , ROT_CCW ,40  ,262144, 262144);

	motors.add_motor(9 , ROT_CW  ,210 ,262144, 262144);
	motors.add_motor(10, ROT_CCW ,160 ,262144, 262144);
	motors.add_motor(11, ROT_CW  ,60  ,262144, 262144);
	motors.add_motor(12, ROT_CW  ,40  ,262144, 262144);
#endif
#if 1
#ifdef CANOPEN_MODE
	//조종기 초기위치
	//motors.add_motor(10, ROT_CCW ,160 ,10000, 5566, 4095);

#else
	//조종기 초기위치
//	motors.add_motor(1 , ROT_CW  ,90  ,262144, 262144, 2048);
//	motors.add_motor(2 , ROT_CCW ,90  ,174763, 174763, 1020);
//	motors.add_motor(3 , ROT_CW  ,60  ,174763, 174763, 2048);
//	motors.add_motor(4 , ROT_CW  ,130 ,174763, 174763, 936);
////
//	motors.add_motor(5 , ROT_CCW ,210 ,262144, 262144, 624);
//	motors.add_motor(6 , ROT_CW  ,160 ,262144, 262144, 12);
//	motors.add_motor(7 , ROT_CCW ,60  ,262144, 262144, 2041);
//	motors.add_motor(8 , ROT_CCW ,40  ,262144, 262144, 2048);
////
//	motors.add_motor(9 , ROT_CW  ,210 ,262144, 262144, 626);
	motors.add_motor(10, ROT_CCW ,160 ,262144, 262144, 48);
//	motors.add_motor(11, ROT_CW  ,60  ,262144, 262144, 2170);
//	motors.add_motor(12, ROT_CW  ,40  ,262144, 262144, 2048);
#endif
#endif
    //motors.init();

   //osDelay(1000);
    //motors.setPosition(10,4094);

	while(1){

#ifdef CANOPEN_MODE

		osStatus_t status = osMessageQueueGet(zerPosiHandle, &motionMsg, NULL, 0U); // wait for message
		if (status == osOK) {
			motors.setPosition(motionMsg.sid, motionMsg.posi);
		}


		status = osMessageQueueGet(zerCmd_rxHandle, &cmd_rx, NULL, 0U); // wait for message
		if (status == osOK) {
			mrs_rx_cmd_process(&cmd_rx);
		}

		if(All_init_flag == 1){
			uint8_t init_result = motors.init();
			if(init_result == 1)		//성공
				All_init_flag = 2;		//init ok
			else if(init_result == 0)	//실패
				All_init_flag = 3;		//init fail
		}
		else if(All_init_flag == 2){
			motors.movePosition();
			send_RPDO_BuffSend(CO);
			osDelay(1);
			send_sync(CO);
		}
		osDelay(9);



//		osDelay(5);
//		motors.setAllControlbit();
//		osDelay(5);
//		send_sync(CO);
#else
		osStatus_t status = osMessageQueueGet(zerPosiHandle, &motionMsg, NULL, 0U); // wait for message
		if (status == osOK) {
			motors.setPosition(motionMsg.sid, motionMsg.posi);
		}
		motors.movePosition();

		send_RPDO_1_BuffSend(CO);

		send_sync(CO);
		osDelay(10);
#endif


		HAL_GPIO_TogglePin(LD_ZER_TX_GPIO_Port, LD_ZER_TX_Pin);
	}
}

void mrs_rx_cmd_process(BypassPacket_TypeDef *cmd_rx) {

	if (cmd_rx->sid > 12)
		return;

	switch (cmd_rx->cmd) {
	case MRS_RX_DATA1: {
		prtc_data_ctl_init_driver_data1_t *pData = (prtc_data_ctl_init_driver_data1_t*) cmd_rx->data;

		zerSetting[cmd_rx->sid].rot_dir = (pData->direction == 0 ? ROT_CW : ROT_CCW);
		zerSetting[cmd_rx->sid].angle = (float) pData->angle / 100;
		zerSetting[cmd_rx->sid].defult_posi = pData->init_position;
		zerSetting[cmd_rx->sid].f_data1 = true;


		BypassPacket_TypeDef msg;
		msg.gid = cmd_rx->gid;
		msg.sid = cmd_rx->sid;
		msg.cmd = MRS_TX_DATA1_ACK;
		memcpy(msg.data, (uint8_t *)pData, 7);
		osMessageQueuePut(zerCmd_txHandle, &msg, 0U, 0U);

		break;
	}

	case MRS_RX_DATA2: {
		if (zerSetting[cmd_rx->sid].f_data1 != true)
			return;
		zerSetting[cmd_rx->sid].f_data1 = false; //f_data1 가 먼저 수신된 후 data2가 수신되었을때만 유효하다.

		prtc_data_ctl_init_driver_data2_t *pData = (prtc_data_ctl_init_driver_data2_t*) cmd_rx->data;
		zerSetting[cmd_rx->sid].tar_speed = pData->count;
		zerSetting[cmd_rx->sid].tar_acc = pData->rpm * 100;

		motors.add_motor(
				cmd_rx->sid,
				zerSetting[cmd_rx->sid].rot_dir ,
				zerSetting[cmd_rx->sid].angle ,
				zerSetting[cmd_rx->sid].tar_speed,
				zerSetting[cmd_rx->sid].tar_acc,
				zerSetting[cmd_rx->sid].defult_posi);

		BypassPacket_TypeDef msg;
		msg.gid = cmd_rx->gid;
		msg.sid = cmd_rx->sid;
		msg.cmd = MRS_TX_DATA2_ACK;
		memcpy(msg.data, (uint8_t *)pData, 7);
		osMessageQueuePut(zerCmd_txHandle, &msg, 0U, 0U);

		break;
	}

	case MRS_RX_MOVE_DEFAULT_POSI: {

		//if(cmd_rx->sid == 12)
		All_init_flag = 1;

		break;
	}

	case MRS_RX_MOVE_DEFAULT_POSI_CHECK: {
		if(All_init_flag == 2){
			BypassPacket_TypeDef msg = {0,};
			msg.gid = cmd_rx->gid;
			msg.sid = cmd_rx->sid;
			msg.cmd = MRS_TX_MOVE_DEFAULT_POSI_CHECK;
			osMessageQueuePut(zerCmd_txHandle, &msg, 0U, 0U);
		}
		break;
	}
	default:
		break;
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback_ByPass(CAN_HandleTypeDef *hcan)//231110 shs
{
	if(hcan->Instance == CAN2)
		HAL_CAN_RxFifo0MsgPendingCallback_CO(hcan);
}


void CANopenNode_Init(void){

	CANopenNodeSTM32 canOpenNodeSTM32;
	canOpenNodeSTM32.CANHandle = &hcan2;
	canOpenNodeSTM32.HWInitFunction = MX_CAN2_Init;
	canOpenNodeSTM32.timerHandle = &htim14;
	canOpenNodeSTM32.desiredNodeID = 60;	//0x01;
	canOpenNodeSTM32.baudrate = 1000;		//1Mbps
	canopen_app_init(&canOpenNodeSTM32);
}
