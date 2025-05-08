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
	CANopenNode_Init();
    TPDO1_rx_init();

    CO_NMT_t *NMTmaster = CO->NMT;
    NMTmaster->internalCommand = CO_NMT_ENTER_OPERATIONAL;

    motors.motorsInit(CO, 1, 30);

    while(1){
#ifdef CANOPEN_MODE
        mrsComm.processCommandQueue();
        mrsComm.processPositionQueue();
        mrsComm.checkCommunicationStatus();
        mrsComm.processMovePosition();

        motors.default_posi_check_process();
#endif
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback_ByPass(CAN_HandleTypeDef *hcan)//231110 shs
{
	if(hcan->Instance == CAN2)
		HAL_CAN_RxFifo0MsgPendingCallback_CO(hcan);
}



