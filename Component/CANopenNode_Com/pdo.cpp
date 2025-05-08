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

#include "pdo.h"

/* CANopen SDK */
#include "../CANopenNode_STM32/CO_app_STM32.h"
#include "CO_driver_target.h"
#include "OD.h"

/* motor */
#include "CANopen_Motor.h"

/* Private includes ----------------------------------------------------------*/
extern Motors motors;
/* test */
extern int32_t actualPosition_1;
extern int32_t actualPosition_2;
extern int32_t actualPosition_3;

CO_CANrx_t TPDO_Rx_buffer;

typedef struct{
	uint16_t statusword;
	uint32_t actualPosition;
}TPDO1_rx_TypeDef;

const char* status_bits[16] = {
    "Ready to switch on",
    "Switched on",
    "Operation enabled",
    "Fault",
    "Voltage enabled",
    "Quick stop",
    "Switch on disabled",
    "Warning",
    "Manufacturer specific",
    "Remote",
    "Target reached",
    "Internal limit active",
    "Operation mode specific",
    "Operation mode specific",
    "Manufacturer specific",
    "Manufacturer specific"
};



void print_status(int id, int status_word) {

	static int id_9 = 100;
	static int id_10 = 100;

	if(id == 9){
		id_9--;
		if(id_9 != 0)
			return;
		id_9=100;
	}

	if(id == 10){
		id_10--;
		if(id_10 != 0)
			return;
		id_10=100;
	}


    // status_word의 각 비트를 확인하고 해당 상태를 출력합니다.
    for (int bit = 0; bit < 16; bit++) {
    	//if(bit == 3){
			if (status_word & (1 << bit)) {
				printf("[%d]status: %s\n", id, status_bits[bit]);
			}
    	//}
    }
}


bool first = true;

void TPDO1rx_callback(void* object, void* message){
	uint8_t id = CO_CANrxMsg_readIdent(message);
    uint8_t DLC = CO_CANrxMsg_readDLC(message);
    uint8_t *data = CO_CANrxMsg_readData(message);

    int32_t actualPosition = data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24);
    motors.setCurrentPosition(id - 0x80 , actualPosition);

    uint16_t statusWord = data[0] | (data[1] << 8);
    motors.setStatusWord(id - 0x80, statusWord);
//
//    print_status(id - 0x80 , data[0] | (data[1] << 8));
//    /* test */
//    if(actualPosition_1 == 0 && id == 0x89)
//		actualPosition_1 = data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24);
//    if(actualPosition_2 == 0 && id == 0x8A)
//		actualPosition_2 = data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24);
//    if(actualPosition_3 == 0 && id == 0x83)
//		actualPosition_3 = data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24);




    //print_status(data[0]);

#if 0
    printf("rx tpdo1 [%d]: ",id);
    for(size_t i = 0; i < DLC; i++) {
           printf("%02X ", data[i]);
    }
    printf("\n");
    printf("parsing %d\n", a);
#endif



}

void TPDO1_rx_init(){
	TPDO_Rx_buffer.ident = 0x181;
	TPDO_Rx_buffer.mask = 0x780;
	TPDO_Rx_buffer.object = NULL;
	TPDO_Rx_buffer.CANrx_callback = TPDO1rx_callback;
}

/* client -- RPDO --> server*/

CO_SDO_abortCode_t send_RPDO_1(CO_t *co, uint8_t nodeId, uint8_t *txData, uint8_t size)
{
	CO_CANtx_t CANtxBuff;
	CANtxBuff.ident = CO_CAN_ID_RPDO_1 + nodeId;
	CANtxBuff.DLC = size;

	for(int i = 0; i < size; i++)
	{
		CANtxBuff.data[i] = txData[i];
	}

	CANtxBuff.bufferFull = false;
	CANtxBuff.syncFlag = false;

	CO_CANsend(co->CANmodule, &CANtxBuff);

	return CO_SDO_AB_NONE;
}

CO_SDO_abortCode_t send_RPDO_2(CO_t *co, uint8_t nodeId, uint8_t *txData, uint8_t size)
{
	CO_CANtx_t CANtxBuff;
	CANtxBuff.ident = CO_CAN_ID_RPDO_2 + nodeId;
	CANtxBuff.DLC = size;

	for(int i = 0; i < size; i++)
	{
		CANtxBuff.data[i] = txData[i];
	}

	CANtxBuff.bufferFull = false;
	CANtxBuff.syncFlag = false;

	CO_CANsend(co->CANmodule, &CANtxBuff);

	return CO_SDO_AB_NONE;
}

#define RPDO_BUFF_SIZE	300
typedef struct _RPDO_Buff_TypeDef{
	CO_CANtx_t buff[RPDO_BUFF_SIZE];
	uint32_t front;
	uint32_t rear;
}RPDO_Buff_TypeDef;

RPDO_Buff_TypeDef RPDO_Buff;

CO_SDO_abortCode_t send_RPDO_1_Buff(CO_t *co, uint8_t nodeId, uint8_t *txData, uint8_t size)
{
	CO_CANtx_t CANtxBuff;
	CANtxBuff.ident = CO_CAN_ID_RPDO_1 + nodeId;
	CANtxBuff.DLC = size;

	for(int i = 0; i < size; i++)
	{
		CANtxBuff.data[i] = txData[i];
	}

	CANtxBuff.bufferFull = false;
	CANtxBuff.syncFlag = false;

	//CO_CANsend(co->CANmodule, &CANtxBuff);

	memcpy(&RPDO_Buff.buff[RPDO_Buff.front], &CANtxBuff, sizeof(CO_CANtx_t));
	if(++RPDO_Buff.front >= RPDO_BUFF_SIZE)
		RPDO_Buff.front = 0;

	return CO_SDO_AB_NONE;
}

CO_SDO_abortCode_t send_RPDO_2_Buff(CO_t *co, uint8_t nodeId, uint8_t *txData, uint8_t size)
{
	CO_CANtx_t CANtxBuff;
	CANtxBuff.ident = CO_CAN_ID_RPDO_2 + nodeId;
	CANtxBuff.DLC = size;

	for(int i = 0; i < size; i++)
	{
		CANtxBuff.data[i] = txData[i];
	}

	CANtxBuff.bufferFull = false;
	CANtxBuff.syncFlag = false;

	//CO_CANsend(co->CANmodule, &CANtxBuff);

	memcpy(&RPDO_Buff.buff[RPDO_Buff.front], &CANtxBuff, sizeof(CO_CANtx_t));
	if(++RPDO_Buff.front >= RPDO_BUFF_SIZE)
		RPDO_Buff.front = 0;

	return CO_SDO_AB_NONE;
}


void send_RPDO_BuffSend(CO_t *co){
	while(RPDO_Buff.front != RPDO_Buff.rear){
		if (HAL_CAN_GetTxMailboxesFreeLevel(((CANopenNodeSTM32*)co->CANmodule->CANptr)->CANHandle) > 0) {
			CO_CANsend(co->CANmodule, &RPDO_Buff.buff[RPDO_Buff.rear]);
			if(++RPDO_Buff.rear >= RPDO_BUFF_SIZE)
					RPDO_Buff.rear = 0;
		}
	}
}


