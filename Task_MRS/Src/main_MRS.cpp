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
#include "string.h"

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
#include "app_pid_init_cmd.h"

/* Private variables ---------------------------------------------------------*/
extern osMessageQueueId_t zerPosiHandle;
extern osMessageQueueId_t dxlPosiHandle;
extern osMessageQueueId_t zerCmd_rxHandle;
extern osMessageQueueId_t zerCmd_txHandle;
extern osMessageQueueId_t dxlCmd_rxHandle;
extern osMessageQueueId_t dxlCmd_txHandle;

/* Private function prototypes -----------------------------------------------*/
void CAN_FilterConfig(CAN_HandleTypeDef *hcan);
void mrs_tx_cmd_process(BypassPacket_TypeDef *cmd_tx);
uint8_t idRead(
		GPIO_TypeDef* GPIO_01, uint16_t Pin_01,
		GPIO_TypeDef* GPIO_02, uint16_t Pin_02,
		GPIO_TypeDef* GPIO_04, uint16_t Pin_04,
		GPIO_TypeDef* GPIO_08, uint16_t Pin_08,
		GPIO_PinState setState);
uint8_t MRS_DXL_id = 0;
uint8_t MRS_ZER_id = 0;

void main_MRS(void *argument){

	Tick runLED;
	BypassPacket_TypeDef cmd_tx;
	/* MRS protocol */
	can_init_data_save(&hcan1);
	gm_motion_TX_LED_init(LD_MRS_TX_GPIO_Port, LD_MRS_TX_Pin, GPIO_PIN_RESET);
	gm_motion_RX_LED_init(LD_MRS_RX_GPIO_Port, LD_MRS_RX_Pin, GPIO_PIN_RESET);

	CAN_FilterConfig(&hcan1);

	MRS_ZER_id = idRead(
			ZER_ID_01_GPIO_Port, ZER_ID_01_Pin,
			ZER_ID_02_GPIO_Port, ZER_ID_02_Pin,
			ZER_ID_04_GPIO_Port, ZER_ID_04_Pin,
			ZER_ID_08_GPIO_Port, ZER_ID_08_Pin,
			GPIO_PIN_RESET);

	MRS_DXL_id = idRead(
			DXL_ID_01_GPIO_Port, DXL_ID_01_Pin,
			DXL_ID_02_GPIO_Port, DXL_ID_02_Pin,
			DXL_ID_04_GPIO_Port, DXL_ID_04_Pin,
			DXL_ID_08_GPIO_Port, DXL_ID_08_Pin,
			GPIO_PIN_RESET);

	set_my_can_id(MRS_ZER_id);
	add_my_can_sub_id(1, 28);

	/* MRS Boot msg */
	app_tx_init_sub_pid_boot_ctl(
			0,
			0,
			MRS_ZER_id,
			MASTER_CAN_ID,
			1,
			0);

	while(1){
		osDelay(1);
		osStatus_t status;
		status = osMessageQueueGet(zerCmd_txHandle, &cmd_tx, NULL, 0U); // wait for message
		if (status == osOK) {
			mrs_tx_cmd_process(&cmd_tx);
		}

		status = osMessageQueueGet(dxlCmd_txHandle, &cmd_tx, NULL, 0U); // wait for message
		if (status == osOK) {
			mrs_tx_cmd_process(&cmd_tx);
		}


		/* MRS Protocol */
		proc_can_rx();
		proc_can_tx();

		if(runLED.delay(500)){
			HAL_GPIO_TogglePin(LD_RUN_GPIO_Port, LD_RUN_Pin);
		}
	}
}

//재정의
extern can_q_buff_t can_rx_ring_buff[CAN_CNT];
void proc_can_rx(void)
{
	for(int i = 0; i < can_init.cnt; i++){
		if(can_rx_ring_buff[i].head != can_rx_ring_buff[i].tail){
			prtc_header_t *pPh = (prtc_header_t *)&can_rx_ring_buff[i].can_header[can_rx_ring_buff[i].tail];
			if(pPh->cmd == CMD_LFS){

			}
			else{
				for(int j = 0; j < my_can_id_data.sub_id_cnt; j++){
					if((pPh->target_id == my_can_id_data.id + 1||pPh->target_id == my_can_id_data.id || pPh->target_id == CAN_ID_BROAD_CAST) && (pPh->target_sub_id == my_can_id_data.sub_id[j] || pPh->target_sub_id == CAN_SUB_ID_BROAD_CAST)){
					// 하드코딩 if((pPh->target_id == my_can_id_data.id || pPh->target_id == CAN_ID_BROAD_CAST) && (pPh->target_sub_id == my_can_id_data.sub_id[j] || pPh->target_sub_id == CAN_SUB_ID_BROAD_CAST)){
						gm_motion_RX_LED_ON(i);//210218 shs//210430kjh
						net_phd_pid(i, &can_rx_ring_buff[i].can_header[can_rx_ring_buff[i].tail], (uint8_t *)&can_rx_ring_buff[i].data[can_rx_ring_buff[i].tail]);
						osDelay(1);
						break;
					}
				}
			}
			proc_rx_ring_buff_tail_chk(i);
		}
	}
	gm_motion_RX_LED_OFF();//210218 shs
}


void mrs_tx_cmd_process(BypassPacket_TypeDef *cmd_tx){

	if (cmd_tx->sid > 12)
			return;

		switch (cmd_tx->cmd) {
		case MRS_TX_DATA1_ACK: {

			app_tx_init_sub_pid_driver_data1_rsp(
				0,
				0,
				cmd_tx->gid,
				MASTER_CAN_ID,
				cmd_tx->sid,
				0,
				0,
				0,
				0,
				0);
			break;
		}
		case MRS_TX_DATA2_ACK: {
			app_tx_init_sub_pid_driver_data2_rsp(
				0,
				0,
				cmd_tx->gid,
				MASTER_CAN_ID,
				cmd_tx->sid,
				0,
				0,
				0);
			break;
		}
		case MRS_TX_DATA_OP_ACK : {
			uint8_t data;
			app_tx_init_sub_pid_driver_data_op_rsp(
				0,
				0,
				cmd_tx->gid,
				MASTER_CAN_ID,
				cmd_tx->sid,
				0,
				&data);
			break;
		}
		case MRS_TX_MOVE_DEFAULT_POSI_CHECK: {
			app_tx_init_sub_pid_status_rsp(
				0,
				0,
				cmd_tx->gid,
				MASTER_CAN_ID,
				cmd_tx->sid,
				0,
				MOVE_INIT_POSITION,
				1);//status 1
			break;
		}

		default :
			break;
		}
}

uint32_t can_rx_cnt = 0;
void app_rx_motion_sub_pid_adc_ctl(uint8_t num, prtc_header_t *pPh, prtc_data_ctl_motion_adc_t *pData)
{
	uint8_t axleId = (uint8_t)pPh->target_sub_id;
	axleId -= 1;
	if (32 < axleId) return;

	MotionPacket_TypeDef motionMsg;

	motionMsg.gid = pPh->target_id;
	motionMsg.sid = pPh->target_sub_id;
	motionMsg.posi = pData->adc_val;

	if(pPh->target_id == MRS_ZER_id){
		osMessageQueuePut(zerPosiHandle, &motionMsg, 0U, 0U);
		can_rx_cnt++;
	}
	else if(pPh->target_id == MRS_DXL_id){
		osMessageQueuePut(dxlPosiHandle, &motionMsg, 0U, 0U);
	}

}

/* init sequnce */
void app_rx_init_sub_pid_driver_data1_ctl(uint8_t num, prtc_header_t *pPh, prtc_data_ctl_init_driver_data1_t *pData)
{
	if(pPh->target_id != MRS_ZER_id && pPh->target_id != MRS_DXL_id)
		return;
//prtc_data_ctl_init_driver_data1_t *temp = (prtc_data_ctl_init_driver_data1_t *)pData;

//	uint8_t SensorDirection = (uint8_t)temp->direction;
//	uint16_t OppositeLimit = (uint16_t)temp->angle;
//	uint16_t DefaultLocation = (uint16_t)temp->init_position;
//	uint8_t ReductionRatio = (uint8_t)temp->reducer_ratio;

	BypassPacket_TypeDef msg;
	msg.gid = pPh->target_id;
	msg.sid = pPh->target_sub_id;
	msg.cmd = MRS_RX_DATA1;
	memcpy(msg.data, (uint8_t *)pData, 8);

	if(pPh->target_id == MRS_ZER_id)
		osMessageQueuePut(zerCmd_rxHandle, &msg, 0U, 0U);
	else if(pPh->target_id == MRS_DXL_id)
		osMessageQueuePut(dxlCmd_rxHandle, &msg, 0U, 0U);

}
//op 로 변경
void app_rx_init_sub_pid_driver_data2_ctl(uint8_t num, prtc_header_t *pPh, prtc_data_ctl_init_driver_data2_t *pData)
{
	if(pPh->target_id != MRS_ZER_id && pPh->target_id != MRS_DXL_id)
		return;
//	prtc_data_ctl_init_driver_data2_t *temp = (prtc_data_ctl_init_driver_data2_t *)pData;
//	uint32_t target_speed = temp->count;
//	uint32_t target_acc = temp->rpm * 100;

	BypassPacket_TypeDef msg;
	msg.gid = pPh->target_id;
	msg.sid = pPh->target_sub_id;
	msg.cmd = MRS_RX_DATA2;
	memcpy(msg.data, (uint8_t *)pData, 8);

	if(pPh->target_id == MRS_ZER_id)
		osMessageQueuePut(zerCmd_rxHandle, &msg, 0U, 0U);
	else if(pPh->target_id == MRS_DXL_id)
		osMessageQueuePut(dxlCmd_rxHandle, &msg, 0U, 0U);
}

void app_rx_init_sub_pid_driver_data_op_ctl(uint8_t num, prtc_header_t *pPh, uint8_t *pData)
{
	if(pPh->target_id != MRS_ZER_id && pPh->target_id != MRS_DXL_id)
		return;

	BypassPacket_TypeDef msg;
	msg.gid = pPh->target_id;
	msg.sid = pPh->target_sub_id;
	msg.cmd = MRS_RX_DATA_OP;
	memcpy(msg.data, (uint8_t *)pData, 8);

	if(pPh->target_id == MRS_ZER_id)
		osMessageQueuePut(zerCmd_rxHandle, &msg, 0U, 0U);
	else if(pPh->target_id == MRS_DXL_id)
		osMessageQueuePut(dxlCmd_rxHandle, &msg, 0U, 0U);
}

void app_rx_init_sub_pid_move_sensor_ctl(uint8_t num, prtc_header_t *pPh, uint8_t *pData)
{
	if(pPh->target_id != MRS_ZER_id && pPh->target_id != MRS_DXL_id)
		return;

	app_tx_init_sub_pid_move_sensor_rsp(
		num,
		0,
		pPh->target_id,
		MASTER_CAN_ID,
		pPh->target_sub_id,
		0,
		0);
}

void app_rx_init_sub_pid_status_rqt(uint8_t num, prtc_header_t *pPh, prtc_data_rqt_init_status_t *pData)
{
	uint8_t status = 0;

	if(pPh->target_id != MRS_ZER_id && pPh->target_id != MRS_DXL_id)
		return;

	prtc_data_rqt_init_status_t *temp = (prtc_data_rqt_init_status_t *) pData;

	switch(temp->step)
	{
	//init step 1 : 1. vattery check
	case ABSOLUTE_BATTERY:
		app_tx_init_sub_pid_status_rsp(
			num,
			0,
			pPh->target_id,
			MASTER_CAN_ID,
			pPh->target_sub_id,
			0,
			ABSOLUTE_BATTERY,
			1); // ok;
		break;
	case DRIVER_DATA1:

		break;
	case DRIVER_DATA2:

		break;
	case MOVE_SENSOR:

		status = 1;//ok
		app_tx_init_sub_pid_status_rsp(
			num,
			0,
			pPh->target_id,
			MASTER_CAN_ID,
			pPh->target_sub_id,
			0,
			MOVE_SENSOR,
			status);

		break;
	case MOVE_INIT_POSITION:

		BypassPacket_TypeDef msg;
		msg.gid = pPh->target_id;
		msg.sid = pPh->target_sub_id;
		msg.cmd = MRS_RX_MOVE_DEFAULT_POSI_CHECK;
		memcpy(msg.data, (uint8_t *)pData, 8);

		//초기위치로 이동 완료하였는지 확인
		if(pPh->target_id == MRS_ZER_id)
			osMessageQueuePut(zerCmd_rxHandle, &msg, 0U, 0U);
		else if(pPh->target_id == MRS_DXL_id)
			osMessageQueuePut(dxlCmd_rxHandle, &msg, 0U, 0U);

		break;
	}
}
void app_rx_init_sub_pid_move_init_position_ctl(uint8_t num, prtc_header_t *pPh, uint8_t *pData)
{
	if(pPh->target_id != MRS_ZER_id && pPh->target_id != MRS_DXL_id)
		return;

	BypassPacket_TypeDef msg;
	msg.gid = pPh->target_id;
	msg.sid = pPh->target_sub_id;
	msg.cmd = MRS_RX_MOVE_DEFAULT_POSI;

	if(pPh->target_id == MRS_ZER_id)
		osMessageQueuePut(zerCmd_rxHandle, &msg, 0U, 0U);
	else if(pPh->target_id == MRS_DXL_id)
		osMessageQueuePut(dxlCmd_rxHandle, &msg, 0U, 0U);

 	app_tx_init_sub_pid_move_init_position_rsp(
		num,
		0,
		pPh->target_id,
		MASTER_CAN_ID,
		pPh->target_sub_id,
		0);
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

