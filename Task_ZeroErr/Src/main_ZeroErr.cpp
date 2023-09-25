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

extern osMessageQId zerPosiHandle;

/* Private function prototypes -----------------------------------------------*/
void CANopenNode_Init(void);


void main_ZeroErr(void *argument){


	osDelay(10000);
	MotionPacket_TypeDef motionMsg;

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

	motors.motorsInit(CO, 1, 12);
	motors.init_status_led(LD_ZER_ERR_GPIO_Port, LD_ZER_ERR_Pin, GPIO_PIN_RESET);

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

    motors.init();

	while(1){
		osStatus_t status = osMessageQueueGet(zerPosiHandle, &motionMsg, NULL, 0U); // wait for message
		if (status == osOK) {
			motors.setPosition(motionMsg.sid, motionMsg.posi);
		}
		motors.movePosition();
		send_RPDO_1_BuffSend(CO);
		send_sync(CO);
		osDelay(10);

		HAL_GPIO_TogglePin(LD_ZER_TX_GPIO_Port, LD_ZER_TX_Pin);
	}
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
