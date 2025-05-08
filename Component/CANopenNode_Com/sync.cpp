/*
 * pdo.cpp
 *
 *  Created on: Jul 21, 2023
 *      Author: minim
 */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
//#include "canopen_main.h"
#include "main.h"

#include "stdio.h"
#include "string.h"

#include "sync.h"

/* CANopen SDK */
#include "../CANopenNode_STM32/CO_app_STM32.h"
#include "OD.h"


/* Private includes ----------------------------------------------------------*/


/* client -- RPDO --> server*/
CO_SDO_abortCode_t send_sync(CO_t *co)
{
	CO_CANtx_t CANtxBuff;
	CANtxBuff.ident = CO_CAN_ID_SYNC + 0;
	CANtxBuff.DLC = 0;
	CANtxBuff.bufferFull = false;
	CANtxBuff.syncFlag = false;

	CO_CANsend(co->CANmodule, &CANtxBuff);

	return CO_SDO_AB_NONE;
}
