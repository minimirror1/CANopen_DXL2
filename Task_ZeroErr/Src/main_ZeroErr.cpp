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
#include "gpio.h"
#include "string.h"
/* RTOS ----------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

/* User Task -----------------------------------------------------------------*/
#include "main_ZeroErr.h"
#include "main_MRS.h"

/* Component -----------------------------------------------------------------*/
#include "cpp_tick.h"

/* CANopen SDK ---------------------------------------------------------------*/
#include "../CANopenNode_STM32/CO_app_STM32.h"
#include "OD.h"
#include "sdo.h"
#include "pdo.h"

/* Motor Class ---------------------------------------------------------------*/
#include "CANopen_Motor.h"

/* MRS Communication -----------------------------------------------------------*/
#include "MRS_Communication.h"

/* Private variables ---------------------------------------------------------*/
/* CANopen 관련 변수 */
extern CO_t* CO;
CANopenNodeSTM32 canOpenNodeSTM32; // 전역 변수로 선언

/* 모터 및 통신 관련 변수 */
Motors motors;
MRS_Communication mrsComm;

/* 상태 관련 변수 */
Init_TypeDef Zer_All_init_flag = INIT_NONE;//0:none, 1:start init, 2:init ok, 3:init fail
ZerSetting_TypeDef zerSetting[20] = {0,};

/* 큐 핸들 */
extern osMessageQueueId_t zerPosiHandle;
extern osMessageQueueId_t zerCmd_rxHandle;
extern osMessageQueueId_t zerCmd_txHandle;

/* 에러 체크 관련 변수 - 240305 추가 */
uint8_t zergid = 0;
uint8_t zerErrorTxCnt = 30;
Tick t_statusFault[13];
uint32_t zerRxMotion = 0;
Tick t_ZerRxMotion;

/* 카운터 변수 */
uint32_t os_rx_cnt = 0;

/* Private function prototypes -----------------------------------------------*/
/* 초기화 및 프로세스 관련 함수 */
void ZER_Init_Process(void);
void CANopenNode_Init(void);

/* 통신 처리 관련 함수 */
void mrs_zerrx_cmd_process(BypassPacket_TypeDef *cmd_rx);
void zer_statusFaultCheck(void);




void CANopenNode_Init(void){
    canOpenNodeSTM32.CANHandle = &hcan2;
    canOpenNodeSTM32.HWInitFunction = MX_CAN2_Init;
    canOpenNodeSTM32.timerHandle = &htim14;
    canOpenNodeSTM32.desiredNodeID = 60;
    canOpenNodeSTM32.baudrate = 1000;
    canopen_app_init(&canOpenNodeSTM32);
}

void main_ZeroErr(void *argument){
    /* CANopen Init */
    //CANopenNode_Init();

	CANopenNode_Init();
    TPDO1_rx_init();

    CO_NMT_t *NMTmaster = CO->NMT;
    NMTmaster->internalCommand = CO_NMT_ENTER_OPERATIONAL;

    motors.motorsInit(CO, 1, 30);

    //motors.init_status_led(LD_ZER_ERR_GPIO_Port, LD_ZER_ERR_Pin, GPIO_PIN_RESET);
//    motors.add_motor(
//        3 ,
//        ROT_CW  ,
//        359  ,
//        174763,
//        174763,
//        0);
//
//    motors.add_motor(
//        4 ,
//        ROT_CW  ,
//        359  ,
//        174763,
//        174763,
//        0);
//
//    motors.init_motor(3);
//    motors.init_motor(4);
//
//	motors.init_default_posi(3);
//	motors.init_default_posi(4);

    //Zer_All_init_flag = INIT_INFO_DEFAULT_POSI_START;

    while(1){
#ifdef CANOPEN_MODE
        mrsComm.processCommandQueue();
        mrsComm.processPositionQueue();
        mrsComm.checkCommunicationStatus();
        mrsComm.processMovePosition();
        //ZER_Init_Process();

        motors.default_posi_check_process();
#endif
    }
}

void ZER_Init_Process(void){

	if(Zer_All_init_flag == INIT_INFO_DEFAULT_POSI_START){
		uint8_t init_result = motors.init();
		if(init_result == 1)				//성공
			Zer_All_init_flag = INIT_OK;	//init ok
		else if(init_result == 0){			//실패
			Zer_All_init_flag = INIT_FAIL;	//init fail
			osDelay(1);
			send_sync(CO);
			zer_statusFaultCheck();
		}
	}
	else if(Zer_All_init_flag == INIT_OK){
		motors.movePosition();
		send_RPDO_BuffSend(CO);
		osDelay(1);
		send_sync(CO);
		//zer_statusFaultCheck();
	}
	else if(Zer_All_init_flag == INIT_DEFAULT_POSI_START){
		osDelay(3000);// 이후 초기위치 이동은 3초 후 시작한다.
		uint8_t def_result = motors.default_posi();
		if(def_result == 1)				//성공
			Zer_All_init_flag = INIT_OK;	//init ok
		else if(def_result == 0)			//실패
			Zer_All_init_flag = INIT_FAIL;	//init fail
	}
}

void zer_statusFaultCheck(void){

	for(int i = 1; i <= 12; i++){
		ZER_StatusCheck_TypeDef status = motors.getStatusFaultCheck(i);
		if(status != 999 && status != ZER_STATUS_NONE) {

			if(t_statusFault[i].delay(500) && zerErrorTxCnt != 0){
				char str[10] = {0,};
				if(status == ZER_STATUS_FAULT){
					strcpy(str, "001");	//fault
				}
				else if(status == ZER_STATUS_WARNING){
					strcpy(str, "002");//warning
				}
				else if(status == ZER_STATUS_TIMEOUT){
					strcpy(str, "003");//timeout
				}


				BypassPacket_TypeDef msg = {0,};
				msg.gid = zergid;
				msg.sid = i;
				msg.cmd = MRS_TX_ERROR_MSG;
				memcpy(msg.data, (uint8_t *)str, 8);
				osMessageQueuePut(zerCmd_txHandle, &msg, 0U, 0U);
				Zer_All_init_flag = INIT_FAIL;
				zerErrorTxCnt--;
			}
		}
	}
}

void mrs_zerrx_cmd_process(BypassPacket_TypeDef *cmd_rx) {

	if (cmd_rx->sid > 12)
		return;

	zergid = cmd_rx->gid;//240305;

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
		memcpy(msg.data, (uint8_t *)pData, 8);
		osMessageQueuePut(zerCmd_txHandle, &msg, 0U, 0U);

		break;
	}

	case MRS_RX_DATA_OP: {
			if (zerSetting[cmd_rx->sid].f_data1 != true)
				return;
			prtc_data_ctl_init_driver_data_op_zero_err_t *pData = (prtc_data_ctl_init_driver_data_op_zero_err_t*) cmd_rx->data;
			zerSetting[cmd_rx->sid].tar_speed = pData->profile_target_speed;
			zerSetting[cmd_rx->sid].tar_acc = pData->profile_acc_cnt;

			motors.add_motor(
					cmd_rx->sid,
					zerSetting[cmd_rx->sid].rot_dir ,
					zerSetting[cmd_rx->sid].angle ,
					zerSetting[cmd_rx->sid].tar_speed,
					zerSetting[cmd_rx->sid].tar_acc,
					zerSetting[cmd_rx->sid].defult_posi);

			if( motors.init_motor(cmd_rx->sid) == 1){
				Zer_All_init_flag = INIT_OK;
			}

			BypassPacket_TypeDef msg;
			msg.gid = cmd_rx->gid;
			msg.sid = cmd_rx->sid;
			msg.cmd = MRS_TX_DATA_OP_ACK;
			memcpy(msg.data, (uint8_t *)pData, 8);
			osMessageQueuePut(zerCmd_txHandle, &msg, 0U, 0U);

			break;
		}


	case MRS_RX_MOVE_DEFAULT_POSI: {

		if(cmd_rx->sid == 12){
			//new data
			if(zerSetting[cmd_rx->sid].f_data1 == true){
				Zer_All_init_flag = INIT_INFO_DEFAULT_POSI_START;
				zerSetting[cmd_rx->sid].f_data1 = false; //f_data1 가 먼저 수신된 후 data2가 수신되었을때만 유효하다.
			}
			else{ //default posi only
				Zer_All_init_flag = INIT_DEFAULT_POSI_START;
			}
		}
		break;
	}

	case MRS_RX_MOVE_DEFAULT_POSI_CHECK: {
		if(Zer_All_init_flag == INIT_OK){
			BypassPacket_TypeDef msg = {0,};
			msg.gid = cmd_rx->gid;
			msg.sid = cmd_rx->sid;
			msg.cmd = MRS_TX_MOVE_DEFAULT_POSI_CHECK;
			osMessageQueuePut(zerCmd_txHandle, &msg, 0U, 0U);
		}
		break;
	}
	case MRS_TX_MOTOR_STATUS_CHECK :{
		//240729

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



