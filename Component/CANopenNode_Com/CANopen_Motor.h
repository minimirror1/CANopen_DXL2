/*
 * CANopen_Motor.h
 *
 *  Created on: Jul 21, 2023
 *      Author: minim
 */

#ifndef INC_CANOPEN_MOTOR_H_
#define INC_CANOPEN_MOTOR_H_


#include "cmsis_os.h"

/* CANopen SDK */
#include "../CANopenNode_STM32/CO_app_STM32.h"
#include "OD.h"
#include "sdo.h"
#include "pdo.h"
#include "sync.h"
#include "cia402.h"


#include "stdio.h"
#include "string.h"
#include <array>

#include "math.h"

#include "cpp_tick.h"


//extern CO_t* CO;

#define DXL_COM_TIMEOUT 1000

#define MAX_ENCODER_VAL	524288


#define CURVE_TIME	10000
#define CURVE_TIME_QUANTUM 20

#define CAN_RETRY_DELAY 1  // 실패한 경우 재시도 전에 딜레이 시간(ms)

typedef enum _OperatingStatus_TypeDef{
	Op_STATUS_NONE = 0,
	Op_STATUS_INFO_OK,

	Op_STATUS_CANOPEN_INIT,
	Op_STATUS_CANOPEN_OK,


	Op_STATUS_MOVE_DEFAULT_POSI,
	Op_STATUS_OPERATING,

	Op_STATUS_CANOPEN_FAIL = 999	//240305 위치이동
}OperatingStatus_TypeDef;


typedef enum{
	ZER_STATUS_NONE = 0,
	ZER_STATUS_FAULT,
	ZER_STATUS_WARNING,
	ZER_STATUS_TIMEOUT,
	ZER_RES = 999
}ZER_StatusCheck_TypeDef;


class CANopen_Motor{
private :
	/* parameter */
	CO_t *co_;

	/* motor setting */



	OperatingStatus_TypeDef OpStatus;

	uint8_t id_;
	bool	rotation_;			//true : CW->CCW [-]->[+], false : CCW->CW[+]->[-]
	float 	range_angle_;		//range_angle
	int32_t range_cnt_;			//range 카운트 변환
	int32_t default_cnt_;		//초기위치 카운트 변환
	int32_t default_posi_;		//초기위치 모션 4095
	uint32_t velocity_;			//최대 속도
	int32_t acc_;				//최대 가속도
	int32_t deAcc_;				//최대 감속도
	int32_t sync_ms_;			//sync 주기 ms

	float max_sync_velocity;
	float max_sync_acc;
	float max_sync_dacc;

	/* velocity control */
	float current_Velocity;		//현재 속도
	float current_Acc;			//현재 가속도

	/* position control */
	int32_t commend_position;	//위치 명령
	uint8_t new_trigger;
	uint8_t rpdo1_val;
	int32_t old_position;
	float calc_velocity;
	bool f_current_position;	//첫 수신 true : 수신 성공, false : 수신 실패
	int32_t current_position;	//현재 모터 위치(모터 TxPDO 수신 값)	19bit max 524288   0~524287

	int32_t test_cmd;

	int32_t init_msg_cnt;
	int32_t init_tx_cnt;

	typedef union {
	    struct {
	        uint16_t bit0_readyToSwitchOn : 1;
	        uint16_t bit1_switchedOn : 1;
	        uint16_t bit2_operationEnabled : 1;
	        uint16_t bit3_fault : 1;
	        uint16_t bit4_voltageEnabled : 1;
	        uint16_t bit5_quickStop : 1;
	        uint16_t bit6_switchOnDisabled : 1;
	        uint16_t bit7_warning : 1;
	        uint16_t bit8_manufacturerSpecific1 : 1;
	        uint16_t bit9_remote : 1;
	        uint16_t bit10_targetReached : 1;
	        uint16_t bit11_internalLimitActive : 1;
	        uint16_t bit12_targetValueAcknowledge : 1;
	        uint16_t bit13_operationModeSpecific : 1;
	        uint16_t bit1415_manufacturerSpecific2 : 2;
	    } bits;
	    uint16_t all;
	} StatusWord;


	StatusWord statusWord_;
	Tick t_TxPDO;
	ZER_StatusCheck_TypeDef statusCheck;


	int32_t motion_target_posi;	//수신 모션 value 0~4095
	float	motion_target_volume;		//수신 모션 %
	int32_t target_position;	//현재 목표 위치(모터 RxPDO 송신 값)

	float distance_in_acceleration;//미사용

	/* */
	float error_position;

	float error_acc;

	int32_t last_cmd;


	/* motor status */
	CO_OperatingMode_t operatingMode;
	int32_t actualPosition;

	//-----------------------
	typedef struct __CurveTypeDef
	{
		unsigned char InitFlag;

		float a;
		float b;
		float c;
		float d;

		float offsetX,offsetY;

		int32_t targetY;

		int32_t TimeRange;//커브 시간
		int32_t TimeQuantum;//커브 키프레임
		int32_t TimeQuantumCnt;//키프레임 수

		int32_t TimeCnt;

	}CurveTypeDef;
	CurveTypeDef curve;

public :
	CANopen_Motor() : co_(nullptr), id_(0) {
		OpStatus = Op_STATUS_NONE;
	} // 디폴트 생성자 추가
	CANopen_Motor(CO_t *co, uint8_t id, bool rotation, float range_angle, uint32_t velocity = 262144, int32_t acc = 262144, int defaultPosi = 2048, int32_t deAcc = 262144, int32_t sync_ms = 10) :
					co_(co), id_(id), rotation_(rotation), range_angle_(range_angle), velocity_(velocity), acc_(acc), deAcc_(deAcc), sync_ms_(sync_ms) {

		OpStatus = Op_STATUS_INFO_OK;

		float temp_range_cnt = range_angle_/360 * MAX_ENCODER_VAL;
		range_cnt_ = (rotation == true)?temp_range_cnt : -temp_range_cnt;

		/*초기위치*/
		if(defaultPosi > 4095)
			defaultPosi = 4095;

		default_posi_ = defaultPosi;
		float temp_default_ratio = (float)defaultPosi/4095;
		default_cnt_ = temp_default_ratio * range_cnt_;

		/* velocity control */
		current_Velocity = 0;	//현재 속도	(plus/s)
		current_Acc = 0;		//현재 가속도	(plus/s^2)

		max_sync_velocity = (float)sync_ms_/1000 * velocity_;
		max_sync_acc = (float)sync_ms_/1000 * acc_;
		max_sync_dacc = (float)sync_ms_/1000 * deAcc_;

		max_sync_acc *= 0.6; //최대 가속도에서 60% 제한

		distance_in_acceleration = 0.5 * deAcc_ * (velocity_ / deAcc_) * (velocity_ / deAcc_);

		/* position control */
		commend_position = 0;	//위치 명령
		f_current_position = false;
		current_position = 0;	//현재 모터 위치(모터 TxPDO 수신 값)
		target_position = 0;	//현재 목표 위치(모터 RxPDO 송신 값)

		operatingMode = OPERATING_NONE;
		actualPosition = 0;
		statusCheck = ZER_STATUS_NONE;


		error_acc = 0;

		test_cmd = 0;

		new_trigger = 0;

		init_msg_cnt = 0;
		init_tx_cnt = 0;

	}
	~CANopen_Motor(){};

	/* NMT */
	void NMT_send_OP(){
		CO_NMT_sendCommand(co_->NMT, CO_NMT_ENTER_OPERATIONAL ,id_);
		osDelay(10);
	}
	void NMT_send_STOP(){
		CO_NMT_sendCommand(co_->NMT, CO_NMT_ENTER_STOPPED ,id_);
		osDelay(10);
	}
	void NMT_send_PRE_OP(){
		CO_NMT_sendCommand(co_->NMT, CO_NMT_ENTER_PRE_OPERATIONAL ,id_);
		osDelay(10);
	}
	void NMT_send_ResetNode(){
		CO_NMT_sendCommand(co_->NMT, CO_NMT_RESET_NODE ,id_);
		osDelay(10);
	}
	void NMT_send_ResetComm(){
		CO_NMT_sendCommand(co_->NMT, CO_NMT_RESET_COMMUNICATION ,id_);
		osDelay(10);
	}

	/* SDO */
	//3_set operation mode_CANopen
	//write CO_402_INDEX_MODE_OF_OPERATION				0x6060
	CO_SDO_abortCode_t SDO_write_ModesOfOperation(CO_OperatingMode_t operationMode) {
		uint8_t writebuff = operationMode;
		CO_SDO_abortCode_t abort = write_SDO(
				co_->SDOclient,
				id_,
				CO_402_INDEX_MODE_OF_OPERATION,
				CO_402_SUB_INDEX_MODE_OF_OPERATION,
				&writebuff,
				sizeof(writebuff)
				);
		printf("write node %d index 0x%X %s\n", id_, CO_402_INDEX_MODE_OF_OPERATION, (abort == CO_SDO_AB_NONE)?"success":"fail");
		return abort;
	}

	//read CO_402_INDEX_MODE_OF_OPERATION_DISPLAY		0x6061
	CO_SDO_abortCode_t SDO_read_ModesOfOperationDisplay(CO_OperatingMode_t *operatingMode) {
		uint8_t readbuff = 0;
		size_t readSize;
		CO_SDO_abortCode_t abort = read_SDO(
				co_->SDOclient,
				id_,
				CO_402_INDEX_MODE_OF_OPERATION_DISPLAY,
				CO_402_SUB_INDEX_MODE_OF_OPERATION_DISPLAY,
				&readbuff,
				sizeof(readbuff),
				&readSize
				);
		if(abort == CO_SDO_AB_NONE){
			*operatingMode = (CO_OperatingMode_t)readbuff;
			printOperatingMode((CO_OperatingMode_t)readbuff);
		}
		printf("read node %d index 0x%X %s\n", id_, CO_402_INDEX_MODE_OF_OPERATION_DISPLAY, (abort == CO_SDO_AB_NONE)?"success":"fail");
		return abort;
	}

	// max speed
	CO_SDO_abortCode_t SDO_write_MaxSpeed(uint32_t velocity) {
		uint32_t writebuff = velocity;
		CO_SDO_abortCode_t abort = write_SDO(
				co_->SDOclient,
				id_,
				CO_402_INDEX_MAX_SPEED,
				CO_402_SUB_INDEX_MAX_SPEED,
				(uint8_t *)&writebuff,
				sizeof(writebuff)
				);
		printf("write node %d index 0x%X %s\n", id_, CO_402_INDEX_MAX_SPEED, (abort == CO_SDO_AB_NONE)?"success":"fail");
		return abort;
	}

	//4_set speed_CANopen
	CO_SDO_abortCode_t SDO_write_ProfileVelocity(uint32_t velocity) {
		uint32_t writebuff = velocity;
		CO_SDO_abortCode_t abort = write_SDO(
				co_->SDOclient,
				id_,
				CO_402_INDEX_PROFILE_VELOCITY,
				CO_402_SUB_INDEX_PROFILE_VELOCITY,
				(uint8_t *)&writebuff,
				sizeof(writebuff)
				);
		printf("write node %d index 0x%X %s\n", id_, CO_402_INDEX_PROFILE_VELOCITY, (abort == CO_SDO_AB_NONE)?"success":"fail");
		return abort;
	}
	CO_SDO_abortCode_t SDO_write_ProfileAcceleration(uint32_t acceleration) {
		uint32_t writebuff = acceleration;
		CO_SDO_abortCode_t abort = write_SDO(
				co_->SDOclient,
				id_,
				CO_402_INDEX_PROFILE_ACCELERATION,
				CO_402_SUBINDEX_PROFILE_ACCELERATION,
				(uint8_t *)&writebuff,
				sizeof(writebuff)
				);
		printf("write node %d index 0x%X %s\n", id_, CO_402_INDEX_PROFILE_ACCELERATION, (abort == CO_SDO_AB_NONE)?"success":"fail");
		return abort;
	}
	CO_SDO_abortCode_t SDO_write_ProfileDeceleration(uint32_t deceleration) {
		uint32_t writebuff = deceleration;
		CO_SDO_abortCode_t abort = write_SDO(
				co_->SDOclient,
				id_,
				CO_402_INDEX_PROFILE_DECELERATION,
				CO_402_SUBINDEX_PROFILE_DECELERATION,
				(uint8_t *)&writebuff,
				sizeof(writebuff)
				);
		printf("write node %d index 0x%X %s\n", id_, CO_402_INDEX_PROFILE_DECELERATION, (abort == CO_SDO_AB_NONE)?"success":"fail");
		return abort;
	}

	CO_SDO_abortCode_t SDO_write_TargetVelocity(uint32_t targetVelocity) {
		uint32_t writebuff = targetVelocity;
		CO_SDO_abortCode_t abort = write_SDO(
				co_->SDOclient,
				id_,
				CO_402_INDEX_TARGET_VELOCITY,
				CO_402_SUBINDEX_TARGET_VELOCITY,
				(uint8_t *)&writebuff,
				sizeof(writebuff)
				);
		printf("write node %d index 0x%X %s\n", id_, CO_402_INDEX_TARGET_VELOCITY, (abort == CO_SDO_AB_NONE)?"success":"fail");
		return abort;
	}


	//5_set communication cycle_CANop 0x1005
	CO_SDO_abortCode_t SDO_write_COBID_SYNC_SetNotSyncCANRevA(void) {
		uint32_t writebuff = CO_301_VALUE_COB_ID_SYNC;//bit30:0, bit29:0, bit28-11:0 bit10-0:[COBID=0x80=128]
		CO_SDO_abortCode_t abort = write_SDO(
				co_->SDOclient,
				id_,
				CO_301_INDEX_COB_ID_SYNC,
				CO_301_SUBINDEX_COB_ID_SYNC,
				(uint8_t *)&writebuff,
				sizeof(writebuff)
				);
		printf("write node %d index 0x%X %s\n", id_, CO_301_INDEX_COB_ID_SYNC, (abort == CO_SDO_AB_NONE)?"success":"fail");
		return abort;
	}

	//6_set comm cycle_CANopen
	CO_SDO_abortCode_t SDO_write_SetSyncPeriod(uint32_t time_ms) {
		uint32_t writebuff = time_ms;	//	10,000us = 10ms
		CO_SDO_abortCode_t abort = write_SDO(
				co_->SDOclient,
				id_,
				CO_301_INDEX_SYNC_PERIOD,
				CO_301_SUBINDEX_SYNC_PERIOD,
				(uint8_t *)&writebuff,
				sizeof(writebuff)
				);
		printf("write node %d index 0x%X %s\n", id_, CO_301_INDEX_SYNC_PERIOD, (abort == CO_SDO_AB_NONE)?"success":"fail");
		return abort;
	}

	//7_TPDO mapping_CANopen
	CO_SDO_abortCode_t SDO_write_SetTPDO_Mapping(void) {
		uint32_t writebuff_4;
		uint8_t writebuff_1;
		CO_SDO_abortCode_t abort;

		//Disable TxPDO1  cob id
		writebuff_4 = CO_301_MASK_TPDO_COB_ID_DISABLE + id_;
		abort = write_SDO(
				co_->SDOclient,
				id_,
				CO_301_INDEX_TPDO_COB_ID,
				CO_301_SUBINDEX_TPDO_COB_ID,
				(uint8_t *)&writebuff_4,
				sizeof(writebuff_4)
				);
		printf("write node %d index 0x%X %s\n", id_, CO_301_INDEX_TPDO_COB_ID, (abort == CO_SDO_AB_NONE)?"success":"fail");

		//Defines the transmitssion type
		writebuff_1 = CO_301_VALUE_TPDO_TRANSMISSION_TYPE;
		abort = write_SDO(
				co_->SDOclient,
				id_,
				CO_301_INDEX_TPDO_TRANSMISSION_TYPE,
				CO_301_SUBINDEX_TPDO_TRANSMISSION_TYPE,
				(uint8_t *)&writebuff_1,
				sizeof(writebuff_1)
				);
		printf("write node %d index 0x%X %s\n", id_, CO_301_INDEX_TPDO_TRANSMISSION_TYPE, (abort == CO_SDO_AB_NONE)?"success":"fail");

		//Defines the number of valid entries in the mapping
		writebuff_1 = CO_301_VALUE_TPDO_NUMBER_OF_MAP_0;
		abort = write_SDO(
				co_->SDOclient,
				id_,
				CO_301_INDEX_TPDO_NUMBER_OF_MAP,
				CO_301_SUBINDEX_TPDO_NUMBER_OF_MAP,
				(uint8_t *)&writebuff_1,
				sizeof(writebuff_1)
				);
		printf("write node %d index 0x%X %s\n", id_, CO_301_INDEX_TPDO_NUMBER_OF_MAP, (abort == CO_SDO_AB_NONE)?"success":"fail");

		//0x6041(status word)sub(0x00)size(0x10:16bit)
		writebuff_4 = CO_301_VALUE_TPDO_MAPPING_ENTRY_1_IND + CO_301_VALUE_TPDO_MAPPING_ENTRY_1_SUBIND + CO_301_VALUE_TPDO_MAPPING_ENTRY_1_SIZE;
		abort = write_SDO(
				co_->SDOclient,
				id_,
				CO_301_INDEX_TPDO_MAPPING_ENTRY_1,
				CO_301_SUBINDEX_TPDO_MAPPING_ENTRY_1,
				(uint8_t *)&writebuff_4,
				sizeof(writebuff_4)
				);
		printf("write node %d index 0x%X %s\n", id_, CO_301_INDEX_TPDO_MAPPING_ENTRY_1, (abort == CO_SDO_AB_NONE)?"success":"fail");

		//0x6064(actual position)sub(0x00)size(0x20 32bit)
		writebuff_4 = CO_301_VALUE_TPDO_MAPPING_ENTRY_2_IND + CO_301_VALUE_TPDO_MAPPING_ENTRY_2_SUBIND + CO_301_VALUE_TPDO_MAPPING_ENTRY_2_SIZE;
		abort = write_SDO(
				co_->SDOclient,
				id_,
				CO_301_INDEX_TPDO_MAPPING_ENTRY_2,
				CO_301_SUBINDEX_TPDO_MAPPING_ENTRY_2,
				(uint8_t *)&writebuff_4,
				sizeof(writebuff_4)
				);
		printf("write node %d index 0x%X %s\n", id_, CO_301_INDEX_TPDO_MAPPING_ENTRY_2, (abort == CO_SDO_AB_NONE)?"success":"fail");

		//0x1A00:00h the number of valid entries in the mapping record:2
		writebuff_1 = CO_301_VALUE_TPDO_NUMBER_OF_MAP_2;
		abort = write_SDO(
				co_->SDOclient,
				id_,
				CO_301_INDEX_TPDO_NUMBER_OF_MAP,
				CO_301_SUBINDEX_TPDO_NUMBER_OF_MAP,
				(uint8_t *)&writebuff_1,
				sizeof(writebuff_1)
				);
		printf("write node %d index 0x%X %s\n", id_, CO_301_INDEX_TPDO_NUMBER_OF_MAP, (abort == CO_SDO_AB_NONE)?"success":"fail");

		//Enable TxPDO_1
		writebuff_4 = CO_301_MASK_TPDO_COB_ID_ENABLE + id_;
		abort = write_SDO(
				co_->SDOclient,
				id_,
				CO_301_INDEX_TPDO_COB_ID,
				CO_301_SUBINDEX_TPDO_COB_ID,
				(uint8_t *)&writebuff_4,
				sizeof(writebuff_4)
				);
		printf("write node %d index 0x%X %s\n", id_, CO_301_INDEX_TPDO_COB_ID, (abort == CO_SDO_AB_NONE)?"success":"fail");
		return abort;
	}

	//8_RPDO mapping_CANopen
	CO_SDO_abortCode_t SDO_write_SetRPDO_Mapping(void) {
		uint32_t writebuff_4;
		uint8_t writebuff_1;
		CO_SDO_abortCode_t abort;

		//Disable RxPDO_1
		// Receive PDO Communication 1 - COB ID used by PDO, expedited
		writebuff_4 = CO_301_MASK_RPDO_1_COB_ID_DISABLE + id_;
		abort = write_SDO(
				co_->SDOclient,
				id_,
				CO_301_INDEX_RPDO_1_COB_ID,
				CO_301_SUBINDEX_RPDO_1_COB_ID,
				(uint8_t *)&writebuff_4,
				sizeof(writebuff_4)
				);
		printf("write node %d index 0x%X %s\n", id_, CO_301_INDEX_RPDO_1_COB_ID, (abort == CO_SDO_AB_NONE)?"success":"fail");

		//Defines the transmission type
		//[0x1400,0x02] Receive PDO Communication 1 - Transmission Type, expedited
		writebuff_1 = CO_301_VALUE_RPDO_1_TRANSMISSION_TYPE;
		abort = write_SDO(
				co_->SDOclient,
				id_,
				CO_301_INDEX_RPDO_1_TRANSMISSION_TYPE,
				CO_301_SUBINDEX_RPDO_1_TRANSMISSION_TYPE,
				(uint8_t *)&writebuff_1,
				sizeof(writebuff_1)
				);
		printf("write node %d index 0x%X sub 0x%X %s\n", id_, CO_301_INDEX_RPDO_1_TRANSMISSION_TYPE, CO_301_SUBINDEX_RPDO_1_TRANSMISSION_TYPE, (abort == CO_SDO_AB_NONE)?"success":"fail");

		//Defines the number of valid entries in the mapping record
		writebuff_1 = CO_301_VALUE_RPDO_NUMBER_OF_MAP_0;
		abort = write_SDO(
				co_->SDOclient,
				id_,
				CO_301_INDEX_RPDO_1_NUMBER_OF_MAP,
				CO_301_SUBINDEX_RPDO_NUMBER_OF_MAP,
				(uint8_t *)&writebuff_1,
				sizeof(writebuff_1)
				);
		printf("write node %d index 0x%X %s\n", id_, CO_301_INDEX_RPDO_1_NUMBER_OF_MAP, (abort == CO_SDO_AB_NONE)?"success":"fail");

		//0x6040:00h(control word)size(0x10:16bit)
		writebuff_4 = CO_301_VALUE_RPDO_MAPPING_ENTRY_1_IND + CO_301_VALUE_RPDO_MAPPING_ENTRY_1_SUBIND + CO_301_VALUE_RPDO_MAPPING_ENTRY_1_SIZE;
		abort = write_SDO(
				co_->SDOclient,
				id_,
				CO_301_INDEX_RPDO_1_MAPPING_ENTRY_1,
				CO_301_SUBINDEX_RPDO_MAPPING_ENTRY_1,
				(uint8_t *)&writebuff_4,
				sizeof(writebuff_4)
				);
		printf("write node %d index 0x%X %s\n", id_, CO_301_INDEX_RPDO_1_MAPPING_ENTRY_1, (abort == CO_SDO_AB_NONE)?"success":"fail");

		//0x607A:00h(target position)size(0x20:32bit)
		writebuff_4 = CO_301_VALUE_RPDO_MAPPING_ENTRY_2_IND + CO_301_VALUE_RPDO_MAPPING_ENTRY_2_SUBIND + CO_301_VALUE_RPDO_MAPPING_ENTRY_2_SIZE;
		abort = write_SDO(
				co_->SDOclient,
				id_,
				CO_301_INDEX_RPDO_1_MAPPING_ENTRY_1,
				CO_301_SUBINDEX_RPDO_MAPPING_ENTRY_2,
				(uint8_t *)&writebuff_4,
				sizeof(writebuff_4)
				);
		printf("write node %d index 0x%X %s\n", id_, CO_301_INDEX_RPDO_1_MAPPING_ENTRY_1, (abort == CO_SDO_AB_NONE)?"success":"fail");

		//the number of valid entries in the mapping record : 2	
		writebuff_1 = CO_301_VALUE_RPDO_NUMBER_OF_MAP_2;
		abort = write_SDO(
				co_->SDOclient,
				id_,
				CO_301_INDEX_RPDO_1_NUMBER_OF_MAP,
				CO_301_SUBINDEX_RPDO_NUMBER_OF_MAP,
				(uint8_t *)&writebuff_1,
				sizeof(writebuff_1)
				);
		printf("write node %d index 0x%X %s\n", id_, CO_301_INDEX_RPDO_1_NUMBER_OF_MAP, (abort == CO_SDO_AB_NONE)?"success":"fail");

		//Enable RxPDO_1
		writebuff_4 = CO_301_MASK_RPDO_1_COB_ID_ENABLE + id_;
		abort = write_SDO(
				co_->SDOclient,
				id_,
				CO_301_INDEX_RPDO_1_COB_ID,
				CO_301_SUBINDEX_RPDO_1_COB_ID,
				(uint8_t *)&writebuff_4,
				sizeof(writebuff_4)
				);
		printf("write node %d index 0x%X %s\n", id_, CO_301_INDEX_RPDO_1_COB_ID, (abort == CO_SDO_AB_NONE)?"success":"fail");
		return abort;

#if 0
		//Disable RxPDO_1
		writebuff_4 = CO_301_MASK_RPDO_1_COB_ID_DISABLE + id_;
		abort = write_SDO(
				co_->SDOclient,
				id_,
				CO_301_INDEX_RPDO_1_COB_ID,
				CO_301_SUBINDEX_RPDO_1_COB_ID,
				(uint8_t *)&writebuff_4,
				sizeof(writebuff_4)
				);
		printf("write node %d index 0x%X %s\n", id_, CO_301_INDEX_RPDO_1_COB_ID, (abort == CO_SDO_AB_NONE)?"success":"fail");

		//Defines the transmission type
		writebuff_1 = CO_301_VALUE_RPDO_1_TRANSMISSION_TYPE;
		abort = write_SDO(
				co_->SDOclient,
				id_,
				CO_301_INDEX_RPDO_1_TRANSMISSION_TYPE,
				CO_301_SUBINDEX_RPDO_1_TRANSMISSION_TYPE,
				(uint8_t *)&writebuff_1,
				sizeof(writebuff_1)
				);
		printf("write node %d index 0x%X %s\n", id_, CO_301_INDEX_RPDO_1_TRANSMISSION_TYPE, (abort == CO_SDO_AB_NONE)?"success":"fail");

		//Defines the number of valid entries in the mapping record
		writebuff_1 = CO_301_VALUE_RPDO_NUMBER_OF_MAP_0;
		abort = write_SDO(
				co_->SDOclient,
				id_,
				CO_301_INDEX_RPDO_1_NUMBER_OF_MAP,
				CO_301_SUBINDEX_RPDO_NUMBER_OF_MAP,
				(uint8_t *)&writebuff_1,
				sizeof(writebuff_1)
				);
		printf("write node %d index 0x%X %s\n", id_, CO_301_INDEX_RPDO_1_NUMBER_OF_MAP, (abort == CO_SDO_AB_NONE)?"success":"fail");

		//0x6040:00h(control word)size(0x10:16bit)
		writebuff_4 = CO_301_VALUE_RPDO_MAPPING_ENTRY_1_IND + CO_301_VALUE_RPDO_MAPPING_ENTRY_1_SUBIND + CO_301_VALUE_RPDO_MAPPING_ENTRY_1_SIZE;
		abort = write_SDO(
				co_->SDOclient,
				id_,
				CO_301_INDEX_RPDO_1_MAPPING_ENTRY_1,
				CO_301_SUBINDEX_RPDO_MAPPING_ENTRY_1,
				(uint8_t *)&writebuff_4,
				sizeof(writebuff_4)
				);
		printf("write node %d index 0x%X %s\n", id_, CO_301_INDEX_RPDO_1_MAPPING_ENTRY_1, (abort == CO_SDO_AB_NONE)?"success":"fail");

		//0x607A:00h(target position)size(0x20:32bit)
		writebuff_4 = CO_301_VALUE_RPDO_MAPPING_ENTRY_2_IND + CO_301_VALUE_RPDO_MAPPING_ENTRY_2_SUBIND + CO_301_VALUE_RPDO_MAPPING_ENTRY_2_SIZE;
		abort = write_SDO(
				co_->SDOclient,
				id_,
				CO_301_INDEX_RPDO_MAPPING_ENTRY_2,
				CO_301_SUBINDEX_RPDO_MAPPING_ENTRY_2,
				(uint8_t *)&writebuff_4,
				sizeof(writebuff_4)
				);
		printf("write node %d index 0x%X %s\n", id_, CO_301_INDEX_RPDO_MAPPING_ENTRY_2, (abort == CO_SDO_AB_NONE)?"success":"fail");

		//the number of valid entries in the mapping record : 2
		writebuff_1 = CO_301_VALUE_RPDO_NUMBER_OF_MAP_2;
		abort = write_SDO(
				co_->SDOclient,
				id_,
				CO_301_INDEX_RPDO_1_NUMBER_OF_MAP,
				CO_301_SUBINDEX_RPDO_NUMBER_OF_MAP,
				(uint8_t *)&writebuff_1,
				sizeof(writebuff_1)
				);
		printf("write node %d index 0x%X %s\n", id_, CO_301_INDEX_RPDO_1_NUMBER_OF_MAP, (abort == CO_SDO_AB_NONE)?"success":"fail");

		//Enable RxPDO_1
		writebuff_4 = CO_301_MASK_RPDO_1_COB_ID_ENABLE + id_;
		abort = write_SDO(
				co_->SDOclient,
				id_,
				CO_301_INDEX_RPDO_1_COB_ID,
				CO_301_SUBINDEX_RPDO_1_COB_ID,
				(uint8_t *)&writebuff_4,
				sizeof(writebuff_4)
				);
		printf("write node %d index 0x%X %s\n", id_, CO_301_INDEX_RPDO_1_COB_ID, (abort == CO_SDO_AB_NONE)?"success":"fail");
		return abort;
#endif

	}

	CO_SDO_abortCode_t SDO_read_ActualPosition(void){
		int32_t readbuff = 0;
		size_t readSize;
		CO_SDO_abortCode_t abort = read_SDO(
				co_->SDOclient,
				id_,
				CO_402_INDEX_ACTUAL_POSITION_VALUE,
				CO_402_SUB_INDEX_ACTUAL_POSITION_VALUE,
				(uint8_t *)&readbuff,
				sizeof(readbuff),
				&readSize
				);

		f_current_position = false;
		if(abort == CO_SDO_AB_NONE){
			f_current_position = true;
			current_position = readbuff;
		}
		printf("read node %d index 0x%X %s position %d\n", id_, CO_402_INDEX_ACTUAL_POSITION_VALUE, (abort == CO_SDO_AB_NONE)?"success":"fail", (int)readbuff);
		return abort;
	}

	//9_start remote node_CANopen
	void StartRemoteNode(void) {		
		NMT_send_OP();		
	}

	//10_sync command_CANopen
	void Sync(void) {
		send_sync(co_);
		//printf("sync\n");
	}

	//11_enable_CANopen
	void send_RPDO_1_Control_Position(uint16_t controlword, uint32_t targetPosition){
		uint8_t buff[8] = {0,};
		memcpy(buff, &controlword, sizeof(controlword));
		memcpy(buff + sizeof(controlword), &targetPosition, sizeof(targetPosition));
		send_RPDO_1(co_, id_, buff, 6);
	}

	void send_RPDO_1_Control(uint16_t controlword){
		uint8_t buff[8] = {0,};
		memcpy(buff, &controlword, sizeof(controlword));
		//send_RPDO_1(co_, id_, buff, 2);
		send_RPDO_1_Buff(co_, id_, buff, 2);
	}

	void send_RPDO_2_TargetPosition(uint32_t targetPosition){
		uint8_t buff[8] = {0,};
		memcpy(buff, &targetPosition, sizeof(targetPosition));
		//send_RPDO_2(co_, id_, buff, 4);
		send_RPDO_2_Buff(co_, id_, buff, 4);
	}


	void send_RPDO_1_Control_Position_Buff(uint16_t controlword, uint32_t targetPosition){
		uint8_t buff[8] = {0,};
		memcpy(buff, &controlword, sizeof(controlword));
		memcpy(buff + sizeof(controlword), &targetPosition, sizeof(targetPosition));
		send_RPDO_1_Buff(co_, id_, buff, 6);
	}

	//_set controlword 0x6040
	CO_SDO_abortCode_t SDO_write_ControlWord(uint16_t controlWord) {
		uint16_t writebuff = controlWord;
		CO_SDO_abortCode_t abort = write_SDO(
				co_->SDOclient,
				id_,
				CO_402_INDEX_CONTROL_WORD,
				CO_402_SUB_INDEX_CONTROL_WORD,
				(uint8_t *)&writebuff,
				sizeof(writebuff)
				);
		printf("write node %d index 0x%X %s\n", id_, CO_402_INDEX_CONTROL_WORD, (abort == CO_SDO_AB_NONE)?"success":"fail");
		return abort;
	}
	CO_SDO_abortCode_t SDO_write_TargetPosition(uint32_t targetPosition) {
		uint32_t writebuff = targetPosition;
		CO_SDO_abortCode_t abort = write_SDO(
				co_->SDOclient,
				id_,
				CO_402_INDEX_TARGET_POSIOTION,
				CO_402_SUB_INDEX_TARGET_POSIOTION,
				(uint8_t *)&writebuff,
				sizeof(writebuff)
				);
		printf("write node %d index 0x%X %s\n", id_, CO_402_INDEX_TARGET_POSIOTION, (abort == CO_SDO_AB_NONE)?"success":"fail");
		return abort;
	}


	void EnableCANopen(void){
#if CANOPEN_MODE == PDO_CSP
		//1 [Fault reset]
		send_RPDO_1_Control_Position(0x80, 0x00, current_position);
		osDelay(1);
		//2 [Enable voltage], [Quick stop]
		send_RPDO_1_Control_Position(0x06, 0x00, current_position);
		osDelay(1);
		//3 [Switch on], Enable voltage, Quick stop, [operation enable]
		send_RPDO_1_Control_Position(0x07, 0x00, current_position);
		osDelay(1);
		//4 [ready to switch on], Switch on, Enable voltage, Quick stop, operation enable, [quick stop]
		send_RPDO_1_Control_Position(0x0F, 0x00, current_position);
		osDelay(1);
#elif CANOPEN_MODE == PDO_PP
		//1 [Fault reset]
//		send_RPDO_1_Control_Position(0x80, 0x00);
//		Sync();
//		osDelay(1);

#if 0
		//2 [Enable voltage], [Quick stop]
		send_RPDO_1_Control_Position(0x26, 0x00);
		Sync();
		osDelay(10);

		//3 [Switch on], Enable voltage, Quick stop, [operation enable]
		send_RPDO_1_Control_Position(0x27, 0x00);
		Sync();
		osDelay(10);

		//4 [ready to switch on], Switch on, Enable voltage, Quick stop, operation enable, [quick stop]
		send_RPDO_1_Control_Position(0x2F, 0x00);
		Sync();
		osDelay(10);
#endif
		//2 [Enable voltage], [Quick stop]
		send_RPDO_1_Control(0x26);
		send_RPDO_BuffSend(co_);
		Sync();
		osDelay(10);

		//3 [Switch on], Enable voltage, Quick stop, [operation enable]
		send_RPDO_1_Control(0x27);
		send_RPDO_BuffSend(co_);
		Sync();
		osDelay(10);

		//4 [ready to switch on], Switch on, Enable voltage, Quick stop, operation enable, [quick stop]
		send_RPDO_2_TargetPosition(1000);
		send_RPDO_BuffSend(co_);
		send_RPDO_1_Control(0x2F);
		Sync();
		osDelay(10);

#elif CANOPEN_MODE == SDO_PP

#endif

	}

	void DisableCANopen(void){
		// Enable voltage, Quick stop
		send_RPDO_1_Control_Position(0x06, 0x00);
	}


	int init(void)	{
		int abort = CO_SDO_AB_TIMEOUT;

		if(OpStatus == Op_STATUS_NONE)
			return abort;

		abort = CO_SDO_AB_NONE;

		init_msg_cnt = 0;
		init_tx_cnt = 0;

#if CANOPEN_MODE == PDO_CSP
		//1_Stop remote node_CANopen
		NMT_send_STOP();
		//2_reset comm_CANopen
		NMT_send_ResetComm();

		//3_set operation mode_CANopen

		if(SDO_write_ModesOfOperation(CYCLIC_SYNCHRONOUS_POSITION_MODE) != CO_SDO_AB_NONE){
			return; // 연결 안됨. 초기화 중단.
		}
		CO_OperatingMode_t opermode3;
		SDO_read_ModesOfOperationDisplay(&opermode3);

		//4_set speed_CANopen
		SDO_write_MaxSpeed(velocity_);
		SDO_write_ProfileVelocity(velocity_);
		SDO_write_ProfileAcceleration(acc_);//5566
		SDO_write_ProfileDeceleration(deAcc_);//5566

		//5_set communication cycle_CANop
		SDO_write_COBID_SYNC_SetNotSyncCANRevA();

		//6_set comm cycle_CANopen
		SDO_write_SetSyncPeriod(10000);

		//7_TPDO mapping_CANopen
		SDO_write_SetTPDO_Mapping();

		//8_RPDO mapping_CANopen
		SDO_write_SetRPDO_Mapping();

		SDO_read_ActualPosition();

#if 1
		//9_start remote node_CANopen
		StartRemoteNode();
		Sync();
		osDelay(10);
		//10
		EnableCANopen();
		Sync();
#endif

#elif CANOPEN_MODE == PDO_PP

		/* Stop the node and configure the relevant parameters */
		//1_Stop remote node_CANopen
		NMT_send_STOP();
		osDelay(500);
		//2_reset comm_CANopen
		
		NMT_send_ResetComm();
		//3_Set the profile position mode

		//3 [Switch on], Enable voltage, Quick stop, [operation enable]
		//send_RPDO_1_Control(0x27);
		//send_RPDO_BuffSend(co_);
		//Sync();
		init_msg_cnt++;
		printf("ControlWord : 0x27\n");
		for (int retryCount = 0; retryCount < 5; ++retryCount) {
			abort = SDO_write_ControlWord(0x27);
			init_tx_cnt++;
			if (abort == CO_SDO_AB_NONE) {
				break; // 성공하면 루프 탈출
			} else {
				osDelay(CAN_RETRY_DELAY); // 실패한 경우 재시도 전에 딜레이 추가 (필요에 따라 조정)
				if(retryCount >= 4){
					OpStatus = Op_STATUS_CANOPEN_FAIL;
					abort = CO_SDO_AB_TIMEOUT;
					return abort;
				}
			}
		}
		osDelay(10);

		//2 [Enable voltage], [Quick stop]
		//send_RPDO_1_Control(0x26);
		//send_RPDO_BuffSend(co_);
		//Sync();
		printf("ControlWord : 0x26\n");
		init_msg_cnt++;
		for (int retryCount = 0; retryCount < 5; ++retryCount) {
			abort = SDO_write_ControlWord(0x26);
			init_tx_cnt++;
			if (abort == CO_SDO_AB_NONE) {
				break; // 성공하면 루프 탈출
			} else {
				osDelay(CAN_RETRY_DELAY); // 실패한 경우 재시도 전에 딜레이 추가 (필요에 따라 조정)
				if(retryCount >= 4){
					OpStatus = Op_STATUS_CANOPEN_FAIL;
					abort = CO_SDO_AB_TIMEOUT;
					return abort;
				}
			}
		}
		osDelay(10);

		printf("ControlWord : 0x80\n");
		init_msg_cnt++;
		for (int retryCount = 0; retryCount < 5; ++retryCount) {
			abort = SDO_write_ControlWord(0x80);
			init_tx_cnt++;
			if (abort == CO_SDO_AB_NONE) {
				break; // 성공하면 루프 탈출
			} else {
				osDelay(CAN_RETRY_DELAY); // 실패한 경우 재시도 전에 딜레이 추가 (필요에 따라 조정)
				if(retryCount >= 4){
					OpStatus = Op_STATUS_CANOPEN_FAIL;
					abort = CO_SDO_AB_TIMEOUT;
					return abort;
				}
			}
		}
		osDelay(10);

		NMT_send_PRE_OP();
		printf("Error Reset complete\n");

		/* Stop the node and configure the relevant parameters */
		init_msg_cnt++;
		for (int retryCount = 0; retryCount < 5; ++retryCount) {
			abort = SDO_write_ModesOfOperation(PROFILE_POSITION_MODE);
			init_tx_cnt++;
			if (abort == CO_SDO_AB_NONE) {
				break; // 성공하면 루프 탈출
			} else {
				osDelay(CAN_RETRY_DELAY); // 실패한 경우 재시도 전에 딜레이 추가 (필요에 따라 조정)
				if(retryCount >= 4){
					OpStatus = Op_STATUS_CANOPEN_FAIL;
					abort = CO_SDO_AB_TIMEOUT;
					return abort;
				}
			}
		}

		//4_Check operationg mode to PP pattern
		CO_OperatingMode_t opermode3;
		init_msg_cnt++;
		for (int retryCount = 0; retryCount < 5; ++retryCount) {
			abort = SDO_read_ModesOfOperationDisplay(&opermode3);
			init_tx_cnt++;
			if (abort == CO_SDO_AB_NONE) {
				break; // 성공하면 루프 탈출
			} else {
				osDelay(CAN_RETRY_DELAY); // 실패한 경우 재시도 전에 딜레이 추가 (필요에 따라 조정)
				if(retryCount >= 4){
					OpStatus = Op_STATUS_CANOPEN_FAIL;
					abort = CO_SDO_AB_TIMEOUT;
					return abort;
				}
			}
		}

		//5_Set the profile velocity to 200,000 plus/s
//		SDO_write_TargetVelocity(262144);
//		SDO_write_ProfileVelocity(262144);
//		SDO_write_ProfileAcceleration(436000);
//		SDO_write_ProfileDeceleration(436000);


		init_msg_cnt++;
		for (int retryCount = 0; retryCount < 5; ++retryCount) {
			abort = SDO_write_ProfileVelocity(velocity_);
			init_tx_cnt++;
			if (abort == CO_SDO_AB_NONE) {
				break; // 성공하면 루프 탈출
			} else {
				osDelay(CAN_RETRY_DELAY); // 실패한 경우 재시도 전에 딜레이 추가 (필요에 따라 조정)
				if(retryCount >= 4){
					OpStatus = Op_STATUS_CANOPEN_FAIL;
					abort = CO_SDO_AB_TIMEOUT;
					return abort;
				}
			}
		}

		init_msg_cnt++;
		for (int retryCount = 0; retryCount < 5; ++retryCount) {
			abort = SDO_write_ProfileAcceleration(acc_);
			init_tx_cnt++;
			if (abort == CO_SDO_AB_NONE) {
				break; // 성공하면 루프 탈출
			} else {
				osDelay(CAN_RETRY_DELAY); // 실패한 경우 재시도 전에 딜레이 추가 (필요에 따라 조정)
				if(retryCount >= 4){
					OpStatus = Op_STATUS_CANOPEN_FAIL;
					abort = CO_SDO_AB_TIMEOUT;
					return abort;
				}
			}
		}

		init_msg_cnt++;
		for (int retryCount = 0; retryCount < 5; ++retryCount) {
			abort = SDO_write_ProfileDeceleration(acc_);
			init_tx_cnt++;
			if (abort == CO_SDO_AB_NONE) {
				break; // 성공하면 루프 탈출
			} else {
				osDelay(CAN_RETRY_DELAY); // 실패한 경우 재시도 전에 딜레이 추가 (필요에 따라 조정)
				if(retryCount >= 4){
					OpStatus = Op_STATUS_CANOPEN_FAIL;
					abort = CO_SDO_AB_TIMEOUT;
					return abort;
				}
			}
		}


		//7_TPDO mapping_CANopen
		init_msg_cnt++;
		for (int retryCount = 0; retryCount < 5; ++retryCount) {
			abort = SDO_write_SetTPDO_Mapping();
			init_tx_cnt++;
			if (abort == CO_SDO_AB_NONE) {
				break; // 성공하면 루프 탈출
			} else {
				osDelay(CAN_RETRY_DELAY); // 실패한 경우 재시도 전에 딜레이 추가 (필요에 따라 조정)
				if(retryCount >= 4){
					OpStatus = Op_STATUS_CANOPEN_FAIL;
					abort = CO_SDO_AB_TIMEOUT;
					return abort;
				}
			}
		}

		//8_RPDO mapping_CANopen
		init_msg_cnt++;
		for (int retryCount = 0; retryCount < 5; ++retryCount) {
			abort = SDO_write_SetRPDO_Mapping();
			init_tx_cnt++;
			if (abort == CO_SDO_AB_NONE) {
				break; // 성공하면 루프 탈출
			} else {
				osDelay(CAN_RETRY_DELAY); // 실패한 경우 재시도 전에 딜레이 추가 (필요에 따라 조정)
				if(retryCount >= 4){
					OpStatus = Op_STATUS_CANOPEN_FAIL;
					abort = CO_SDO_AB_TIMEOUT;
					return abort;
				}
			}
		}



		if(abort != CO_SDO_AB_NONE ){
			OpStatus = Op_STATUS_CANOPEN_FAIL;
			printf("CAN OPEN CONNECTION FAIL ID %d \n", id_);
			return abort;
		}

		//9_start remote node_CANopen
		StartRemoteNode();

		init_msg_cnt++;
		for (int retryCount = 0; retryCount < 5; ++retryCount) {
			abort = SDO_read_ActualPosition();
			init_tx_cnt++;
			if (abort == CO_SDO_AB_NONE) {
				break; // 성공하면 루프 탈출
			} else {
				osDelay(CAN_RETRY_DELAY); // 실패한 경우 재시도 전에 딜레이 추가 (필요에 따라 조정)
				if(retryCount >= 4){
					OpStatus = Op_STATUS_CANOPEN_FAIL;
					abort = CO_SDO_AB_TIMEOUT;
					return abort;
				}
			}
		}

		Sync();
		osDelay(10);





		

//		EnableCANopen();

		//현 위치를 읽지 못 할경우 초기화 실패
		if( f_current_position == false){
			OpStatus = Op_STATUS_CANOPEN_FAIL;
			printf("CAN OPEN CONNECTION FAIL ID %d \n", id_);
			abort = CO_SDO_AB_TIMEOUT;
			return abort;
		}

		if( 0 < range_cnt_ )//양수
		{
			if( -100 < current_position && current_position <= range_cnt_)
			{
				OpStatus = Op_STATUS_CANOPEN_OK;
			}
			else{
				OpStatus = Op_STATUS_CANOPEN_FAIL;
			}
		}
		else//음수
		{
			if( range_cnt_ <= current_position && current_position < 100)
			{
				OpStatus = Op_STATUS_CANOPEN_OK;
			}
			else{
				OpStatus = Op_STATUS_CANOPEN_FAIL;
			}
		}

		
		send_RPDO_1_Control(0x2F);
		send_RPDO_BuffSend(co_);

		send_RPDO_2_TargetPosition(current_position);
		send_RPDO_BuffSend(co_);

		send_RPDO_1_Control(0x103F);
		send_RPDO_BuffSend(co_);


#elif CANOPEN_MODE == SDO_PP
		if(SDO_write_ModesOfOperation(PROFILE_POSITION_MODE) != CO_SDO_AB_NONE){
			return; // 연결 안됨. 초기화 중단.
		}
		//4_Check operationg mode to PP pattern
		CO_OperatingMode_t opermode3;
		SDO_read_ModesOfOperationDisplay(&opermode3);
		//5_Set the profile velocity to 5566 plus/s
		SDO_write_ProfileVelocity(260000);
		SDO_write_ProfileAcceleration(6000);//5566
		SDO_write_ProfileDeceleration(6000);//5566

		SDO_write_ControlWord(0x80);

		//enable

		SDO_write_ControlWord(0x26);
		SDO_write_ControlWord(0x27);
		SDO_write_ControlWord(0x2F);


#endif

		return abort;
	}

	void init_RemoteNode(){
		StartRemoteNode();
	}
	void init_EnableCANopen(){
		EnableCANopen();
		osDelay(10);
		Sync();
	}

//=============================================================================

	void setCurrentPosition(int32_t cnt){
		f_current_position = true; //현 위치 수신 성공
		current_position = cnt;
	}

	void setStatusWord(uint16_t statusWord){

		statusWord_.bits.bit12_targetValueAcknowledge = 1;
		statusWord_.all = statusWord;
		t_TxPDO.tickUpdate();
	}


	//240305 status check
	ZER_StatusCheck_TypeDef getStatusWord_FaultCheck(void){
		if(statusWord_.bits.bit3_fault == 1)
			statusCheck = ZER_STATUS_FAULT;
		else if(statusWord_.bits.bit7_warning == 1)
			statusCheck = ZER_STATUS_WARNING;
		else if(t_TxPDO.delay(DXL_COM_TIMEOUT)== true)
			statusCheck = ZER_STATUS_TIMEOUT;
		else
			statusCheck = ZER_STATUS_NONE;

		return statusCheck;
	}


	void setPosition(int32_t cnt){

		if(OpStatus == Op_STATUS_NONE)
			return;

		if( (OpStatus != Op_STATUS_OPERATING) && (OpStatus != Op_STATUS_MOVE_DEFAULT_POSI))
			return;

//		send_RPDO_1_Control_Torque_Position(0x2F, 0x00, cnt);
//		send_RPDO_1_Control_Torque_Position(0x3F, 0x00, cnt);
//		send_RPDO_1_Control_Torque_Position(0x2F, 0x00, cnt);

//		send_RPDO_1_Control_Torque_Position(0x1F, 0x00, cnt);	//바로 속도 제어

		//target_position = cnt;

		motion_target_posi = cnt;
		motion_target_volume = (float)motion_target_posi/4095;
		target_position = motion_target_volume * range_cnt_;

	}

	int32_t getPosition(){
		return current_position;
	}



	float calculate_acceleration(int32_t target, int32_t current) {
		int32_t distance_to_target = target - current;

		// 절대값 사용을 위해 <math.h>가 필요합니다.
		if (abs(distance_to_target) <= deAcc_) {
			// 감속 구간
			if (distance_to_target > 0) {
				return -deAcc_; // 목표가 현재 위치보다 오른쪽에 있을 때 감속
			} else {
				return deAcc_;  // 목표가 현재 위치보다 왼쪽에 있을 때 감속
			}
		} else {
			// 가속 구간
			if (distance_to_target > 0) {
				return acc_;  // 목표가 현재 위치보다 오른쪽에 있을 때 가속
			} else {
				return -acc_; // 목표가 현재 위치보다 왼쪽에 있을 때 가속
			}
		}
	}
	// sync ms 마다 호출되어 target 위치로 이동량을 구함
	void movePosition(){
		if(OpStatus == Op_STATUS_NONE)
			return;
#if CANOPEN_MODE == PDO_CSP
		error_position = target_position - current_position;

		if(abs(error_position) < 5)
			return;

		error_position *= 0.03;

		if(error_position > 0){
			if(error_position > ( max_sync_acc*0.05 + error_acc))
				error_position = ( max_sync_acc*0.05 + error_acc);
		}
		else if(error_position < 0){
			if(error_position < (-max_sync_acc*0.05 + error_acc))
				error_position = (-max_sync_acc*0.05 + error_acc);
		}

		if(error_position > 0){
			if(error_position > max_sync_acc)
				error_position = max_sync_acc;
		}
		else if(error_position < 0){
			if(error_position < -max_sync_acc)
				error_position = -max_sync_acc;
		}
		error_acc = error_position;

		commend_position = current_position + error_position;


		send_RPDO_1_Control_Position_Buff(0x1F, commend_position);
#elif CANOPEN_MODE == PDO_PP

		if( (OpStatus != Op_STATUS_OPERATING) && (OpStatus != Op_STATUS_MOVE_DEFAULT_POSI))
			return;

		if(new_trigger == 1)
		{
			//send_RPDO_1_Control(0x2F);
			rpdo1_val = 0x2F;
			new_trigger = 0;
			return;
		}

		error_position = target_position - current_position;

		if(abs(error_position) < 5 && rpdo1_val == 0x3F){
			//send_RPDO_1_Control(0x2F);
			rpdo1_val = 0x2F;
			new_trigger = 0;
			return;
		}
		commend_position = current_position + error_position;
		if(old_position !=commend_position){
			send_RPDO_2_TargetPosition(commend_position);
			send_RPDO_BuffSend(co_);
			send_RPDO_1_Control(0x103F);
			send_RPDO_BuffSend(co_);
			rpdo1_val = 0x3F;
			new_trigger = 1;

			last_cmd = commend_position;
			old_position = commend_position;
		}

//		send_RPDO_2_TargetPosition(test_cmd);
//		osDelay(10);
//		Sync();
//
//		send_RPDO_1_Control(0x2F);
//		osDelay(10);
//		Sync();
//
//		send_RPDO_1_Control(0x3F);
//		osDelay(10);
//		Sync();

#elif CANOPEN_MODE == SDO_PP
		error_position = target_position - current_position;

		if(error_position > 0){
			if(error_position > max_sync_acc)
				error_position = max_sync_acc;
		}
		else if(error_position < 0){
			if(error_position < -max_sync_acc)
				error_position = -max_sync_acc;
		}

		SDO_write_TargetPosition(commend_position);
		SDO_write_ControlWord(0x3F);
		SDO_write_ControlWord(0x2F);
		current_position = commend_position;
		return;

		if(f_current_position)
#endif


#if 0

		int32_t err_cnt = commend_position - current_position;
		bool f_minus = (err_cnt < 0)? true : false;

		/* 음수 처리 */
		if(f_minus)
			err_cnt = -err_cnt;

		if(err_cnt > max_sync_acc)
			err_cnt = max_sync_acc;

		/* 음수 처리 */
		if(f_minus)
			err_cnt = -err_cnt;

		target_position = current_position + err_cnt;

		send_RPDO_1_Control_Position(0x1F, 0x00, target_position);	//바로 속도 제어
#endif

#if 0
		if(abs(target_position - current_position) < 50)
			return;

		current_Acc = calculate_acceleration(target_position, current_position);
		current_Velocity += current_Acc * 0.01;

		//commend_position = current_position + (current_Velocity * 0.01) + (0.5 * current_Acc * 0.01 * 0.01);
		commend_position = current_position + (current_Velocity * 0.01) ;


		send_RPDO_1_Control_Position(0x1F, 0x00, commend_position);

		calc_velocity = (commend_position - current_position) / 0.01;
		//current_position = commend_position;
#endif
	}

	void PP_mode_send(){

	}
	OperatingStatus_TypeDef getOperatingMode(){
		return OpStatus;
	}
	void setOperatingMode(OperatingStatus_TypeDef mode){
		if(OpStatus == Op_STATUS_NONE)
			return;
		OpStatus = mode;
	}
//-----------------------------------------------------------------------------------------------
	void Curve_Clear(void)
	{
		if(OpStatus == Op_STATUS_NONE)
			return;

		curve.InitFlag = SET;
		curve.a = 0;
		curve.b = 0;
		curve.c = 0;
		curve.d = 0;
		curve.offsetX = 0;
		curve.offsetY = 0;
		curve.targetY = 0;

		curve.TimeCnt = 0;
	}
	//y1 = 현재위치, y2 = 목표위치, d1 = 0, d2 = 0
	void Curve_Init(){//(float y1,float y2,float d1,float d2){
		if(OpStatus == Op_STATUS_NONE)
			return;

		float y1 = current_position;
		//float y2 = range_cnt_ / 2 ;
		float y2 = default_cnt_;
		float d1 = 0;
		float d2 = 0;
		if(curve.InitFlag == SET) //Clear 가 선행되고 나서 진입할수 있다.
		{
			curve.TimeRange = CURVE_TIME;
			curve.TimeQuantum = CURVE_TIME_QUANTUM;
			curve.TimeQuantumCnt = curve.TimeRange / curve.TimeQuantum;
			curve.TimeCnt = 0;
			Curve_Hermite3(y1,y2,d1,d2);
		}


		/* test  초기위치 동기화 */
		motion_target_posi = default_posi_;
		motion_target_volume = (float)motion_target_posi/4095;
		target_position = motion_target_volume * range_cnt_;
	}
	void Curve_Hermite3(float y1,float y2,float d1,float d2){
		float x1, x2;
		x1 = 0;
		x2 = curve.TimeRange;


		curve.targetY = y2;

		curve.offsetX=x1;
		curve.offsetY=y1;

	    float x = x2 - x1;
	    float y = y2 - y1;

	    curve.a = 0;
	    curve.b = d1;
	    float A = (curve.b + d2)*x - 2*y;
	    curve.d = A/(x*x*x);
	    curve.c = (d2-curve.b-3*curve.d*x*x)/(2*x);
	}

	void Move_Default_Posi(){
		if(OpStatus == Op_STATUS_MOVE_DEFAULT_POSI){
			int32_t calc_herm = Curve_CalcHermiteY();
#if	CANOPEN_MODE == PDO_CSP
			send_RPDO_1_Control_Position_Buff(0x1F, Curve_CalcHermiteY());
#elif CANOPEN_MODE == PDO_PP
			send_RPDO_2_TargetPosition(calc_herm);
			send_RPDO_BuffSend(co_);
			send_RPDO_1_Control(0x3F);
			send_RPDO_BuffSend(co_);
			last_cmd = calc_herm;
#elif CANOPEN_MODE == SDO_PP

#endif
		}

	}

	void set_Control_bit4(uint8_t bit4)
	{
		if(OpStatus == Op_STATUS_NONE)
			return;

		if(bit4 == 1){
			send_RPDO_1_Control(0x103F);
			send_RPDO_BuffSend(co_);
		}			
		//else
		//	send_RPDO_1_Control(0x2F);
	}

	int32_t Curve_CalcHermiteY(void)
	{
		int32_t temp;
	    float ret;
	    float x1 = (float)curve.TimeCnt;
	    float x2 = x1*x1;
	    float x3 = x2*x1;

		ret = curve.a;
		ret += (curve.b * x1);
		ret += (curve.c * x2);
		ret += (curve.d * x3);

	    ret += curve.offsetY;

	    temp = (int32_t) ret;
	    if(curve.TimeCnt < curve.TimeRange)
	    {
	    	curve.TimeCnt += (float)CURVE_TIME_QUANTUM;
	    }
	    else
	    {
	    	temp = curve.targetY;
	    }


	    return temp;
	}



//-------------------------------------------------------------------------------------------------
	void printOperatingMode(CO_OperatingMode_t operMode)
	{
		switch(operMode) {
		    case PROFILE_POSITION_MODE:
		        printf("node[%d] operating mode: PROFILE_POSITION_MODE\n", id_);
		        break;
		    case PROFILE_VELOCITY_MODE:
		        printf("node[%d] operating mode: PROFILE_VELOCITY_MODE\n", id_);
		        break;
		    case PROFILE_TORQUE_MODE:
		        printf("node[%d] operating mode: PROFILE_TORQUE_MODE\n", id_);
		        break;
		    case HOMING_MODE:
		        printf("node[%d] operating mode: HOMING_MODE\n", id_);
		        break;
		    case INTERPOLATED_POSITION_MODE:
		        printf("node[%d] operating mode: INTERPOLATED_POSITION_MODE\n", id_);
		        break;
		    case CYCLIC_SYNCHRONOUS_POSITION_MODE:
		        printf("node[%d] operating mode: CYCLIC_SYNCHRONOUS_POSITION_MODE\n", id_);
		        break;
		    case CYCLIC_SYNCHRONOUS_VELOCITY_MODE:
		        printf("node[%d] operating mode: CYCLIC_SYNCHRONOUS_VELOCITY_MODE\n", id_);
		        break;
		    case CYCLIC_SYNCHRONOUS_TORQUE_MODE:
		        printf("node[%d] operating mode: CYCLIC_SYNCHRONOUS_TORQUE_MODE\n", id_);
		        break;
		    default:
		        printf("node[%d] operating mode: UNKNOWN\n", id_);
		        break;
		}

	}

	// curve 상태를 외부에서 확인할 수 있는 public 메서드
	bool isCurveInitialized() const {
		return curve.InitFlag == SET;
	}
	
	bool isCurveCompleted() const {
		return curve.TimeCnt >= curve.TimeRange;
	}

};

#define MAX_CANOPEN_MOTORS 30+1

/* home 방향 표기 */
#define ROT_CW		true
#define ROT_CCW		false

class Motors {
public:

    Motors(){
    	LD_Status_Port_ = NULL;
		LD_Status_Pin_ = 0;
		
		// motors_ 배열의 모든 요소를 디폴트 생성자로 초기화
		for(int i = 0; i < MAX_CANOPEN_MOTORS; i++) {
		    // 디폴트 생성자가 호출되어 OpStatus가 Op_STATUS_NONE으로 설정됨
		    motors_[i] = CANopen_Motor();
		}
    }
    ~Motors(){}

    void motorsInit(CO_t* co, uint8_t idMin , uint8_t idMax ){
    	co_ = co;
    	idMin_ = idMin;
    	idMax_ = idMax;

#if 0
    	motors_[1]  = CANopen_Motor(co, 1,  ROT_CW  ,90  ,262144, 262144);
    	//motors_[2]  = CANopen_Motor(co, 2,  ROT_CCW ,60  ,174763, 174763);
    	motors_[2]  = CANopen_Motor(co, 2,  ROT_CCW ,90  ,174763, 174763);
    	motors_[3]  = CANopen_Motor(co, 3,  ROT_CW  ,60  ,174763, 174763);
    	motors_[4]  = CANopen_Motor(co, 4,  ROT_CW  ,130  ,174763, 174763); //뒤20 앞 110
    	//motors_[4]  = CANopen_Motor(co, 4,  ROT_CW  ,85  ,174763, 174763); 뒤20 앞 65

    	motors_[5]  = CANopen_Motor(co, 5,  ROT_CCW ,210 ,262144, 262144);
    	motors_[6]  = CANopen_Motor(co, 6,  ROT_CW  ,160 ,262144, 262144);
    	motors_[7]  = CANopen_Motor(co, 7,  ROT_CCW ,60  ,262144, 262144);
    	motors_[8]  = CANopen_Motor(co, 8,  ROT_CCW ,40  ,262144, 262144);

    	motors_[9]  = CANopen_Motor(co, 9,  ROT_CW  ,210 ,262144, 262144);
    	motors_[10] = CANopen_Motor(co, 10, ROT_CCW ,160 ,262144, 262144);
    	motors_[11] = CANopen_Motor(co, 11, ROT_CW  ,60  ,262144, 262144);
    	motors_[12] = CANopen_Motor(co, 12, ROT_CW  ,40  ,262144, 262144);
#endif
	}
    void init_status_led(GPIO_TypeDef *Port, uint16_t Pin, GPIO_PinState OnState){
    	LD_Status_Port_ = Port;
    	LD_Status_Pin_ = Pin;
    	LD_Status_On_ = OnState;
    }
    void status_led_on(){
    	if(LD_Status_Port_ == NULL)return;
    	HAL_GPIO_WritePin(LD_Status_Port_, LD_Status_Pin_, LD_Status_On_);
    }
    void status_led_off(){
    	if(LD_Status_Port_ == NULL)return;
    	HAL_GPIO_WritePin(LD_Status_Port_, LD_Status_Pin_, (LD_Status_On_==GPIO_PIN_SET)?GPIO_PIN_RESET:GPIO_PIN_SET);
    }
    void status_led_toggle(){
    	if(LD_Status_Port_ == NULL)return;
		HAL_GPIO_TogglePin(LD_Status_Port_, LD_Status_Pin_);
    }

    void add_motor(uint8_t id, bool rot, float angle, uint32_t velocity, int32_t acc, int defaultPosi){
		if( (id < idMin_) || (idMax_ < id))
			return;
		motors_[id] = CANopen_Motor(co_, id,  rot ,angle ,velocity, acc, defaultPosi);
	}

    uint8_t init(){
    	uint8_t ret = 1;//0:fail, 1 : ok
    	status_led_on();

    	/* CANopen init */
    	for(int i = idMin_; i <= idMax_; i++)
    	{
    		if(motors_[i].init() != CO_SDO_AB_NONE)
    			ret = 0;
    	}
		osDelay(10);

#if !(CANOPEN_MODE == SDO_PP)
		/* default posi curve init */
    	for(int i = idMin_; i <= idMax_; i++){
    		motors_[i].Curve_Clear();
    		motors_[i].Curve_Init();
    	}

    	int time_cnt = CURVE_TIME / CURVE_TIME_QUANTUM;
    	uint8_t ledCnt = 0;


    	while(1){

			for(int i = idMin_; i <= idMax_; i++){
				motors_[i].set_Control_bit4(0);
			}
			send_RPDO_BuffSend(co_);
			osDelay(1);
			send_sync(co_);
			osDelay(9);


    		for(int i = idMin_; i <= idMax_; i++){
				motors_[i].Move_Default_Posi();
			}

    		send_RPDO_BuffSend(co_);
    		osDelay(1);
    		send_sync(co_);
			osDelay(9);



			if(time_cnt % 100 == 0)
				printf("move default [%d]\n", time_cnt);

			time_cnt--;
			if(time_cnt == 0)
				break;
			if(++ledCnt >= 20){
				ledCnt = 0;
				status_led_toggle();
			}
    	}

    	/* operating mode change */
    	for(int i = idMin_; i <= idMax_; i++){
    		if(motors_[i].getOperatingMode() != Op_STATUS_CANOPEN_FAIL)
    			motors_[i].setOperatingMode(Op_STATUS_OPERATING);
		}
#endif
    	status_led_off();

    	return ret;

    }
    uint8_t default_posi(){
		uint8_t ret = 1;//0:fail, 1 : ok
		status_led_on();

#if !(CANOPEN_MODE == SDO_PP)
		/* default posi curve init */
		for(int i = idMin_; i <= idMax_; i++){
			motors_[i].Curve_Clear();
			motors_[i].Curve_Init();
		}

		int time_cnt = CURVE_TIME / CURVE_TIME_QUANTUM;
		uint8_t ledCnt = 0;


		while(1){

			for(int i = idMin_; i <= idMax_; i++){
				motors_[i].set_Control_bit4(0);
			}
			send_RPDO_BuffSend(co_);
			osDelay(1);
			send_sync(co_);
			osDelay(9);


			for(int i = idMin_; i <= idMax_; i++){
				motors_[i].Move_Default_Posi();
			}

			send_RPDO_BuffSend(co_);
			osDelay(1);
			send_sync(co_);
			osDelay(9);



			if(time_cnt % 100 == 0)
				printf("move default [%d]\n", time_cnt);

			time_cnt--;
			if(time_cnt == 0)
				break;
			if(++ledCnt >= 20){
				ledCnt = 0;
				status_led_toggle();
			}
		}

		/* operating mode change */
		for(int i = idMin_; i <= idMax_; i++){
			if(motors_[i].getOperatingMode() != Op_STATUS_CANOPEN_FAIL)
				motors_[i].setOperatingMode(Op_STATUS_OPERATING);
		}
#endif
		status_led_off();
		return ret;
	}


    void movePosition(){
    	for(int i = idMin_; i <= idMax_; i++)
		{
    		motors_[i].movePosition();
		}
    }

    void setAllControlbit(){
    	for(int i = idMin_; i <= idMax_; i++){
			motors_[i].set_Control_bit4(0);
		}
		send_RPDO_BuffSend(co_);
    }

    void PP_mode_send(){
    	for(int i = idMin_; i <= idMax_; i++)
		{
			motors_[i].PP_mode_send();
		}
    }

    void setPosition(uint8_t id, int32_t position){
    	if( (id < idMin_) || (idMax_ < id))
    		return;
    	motors_[id].setPosition(position);
    }

    int32_t getPosition(uint8_t id){
    	if( (id < idMin_) || (idMax_ < id))
			return 0;
    	return motors_[id].getPosition();
    }

    void setCurrentPosition(uint8_t id, int32_t position){
    	if( (id < idMin_) || (idMax_ < id))
			return;
		motors_[id].setCurrentPosition(position);
    }

    void setStatusWord(uint8_t id, uint16_t statusWord){
    	if( (id < idMin_) || (idMax_ < id))
			return;
    	motors_[id].setStatusWord(statusWord);
    }

    OperatingStatus_TypeDef getOperatingMode(uint8_t id){
      	if( (id < idMin_) || (idMax_ < id))
			return Op_STATUS_CANOPEN_FAIL;
		return motors_[id].getOperatingMode();
    }

    ZER_StatusCheck_TypeDef getStatusFaultCheck(uint8_t id){
    	if( (id < idMin_) || (idMax_ < id))
			return ZER_RES;
		return motors_[id].getStatusWord_FaultCheck();
    }

    uint8_t init_motor(uint8_t id){
        uint8_t ret = 1; // 0:fail, 1:ok
        
        if((id < idMin_) || (idMax_ < id))
            return 0;
        
        // 해당 ID의 모터가 이미 add_motor에 의해 할당되었는지 확인
        if(motors_[id].getOperatingMode() == Op_STATUS_NONE) {
            // 할당되지 않은 모터
            return 0;
        }
            
        status_led_on();
        
        // 해당 ID의 모터만 초기화
        if(motors_[id].init() != CO_SDO_AB_NONE)
            ret = 0;
            
        osDelay(10);
        


        status_led_off();
        return ret;
    }

	void default_posi_check_process(){

#if !(CANOPEN_MODE == SDO_PP)
        // 모든 모터에 대해 Op_STATUS_MOVE_DEFAULT_POSI 상태인지 확인하고 처리
        for(int i = idMin_; i <= idMax_; i++) {
            if(motors_[i].getOperatingMode() == Op_STATUS_MOVE_DEFAULT_POSI) {
                // 모터가 초기화되지 않은 경우 스킵
                if(motors_[i].getOperatingMode() == Op_STATUS_NONE) {
                    continue;
                }
                
                // 해당 모터의 curve 초기화 (아직 초기화되지 않았다면)
                if(!motors_[i].isCurveInitialized()) {
                    motors_[i].Curve_Clear();
                    motors_[i].Curve_Init();
                }
                
                // 현재 커브 진행 상황에 따라 이동 명령 수행
                motors_[i].set_Control_bit4(0);
                motors_[i].Move_Default_Posi();
                
                // 커브 이동이 완료되었는지 확인
                if(motors_[i].isCurveCompleted()) {
                    // 이동 완료, 상태 변경
                    motors_[i].setOperatingMode(Op_STATUS_OPERATING);
					printf("[%d]move default posi complete\n", i);
                }
            }
        }
        
        // 모든 모터의 명령을 모아서 한 번에 전송
        send_RPDO_BuffSend(co_);
        osDelay(1);
        send_sync(co_);
#endif
	}

	uint8_t init_default_posi(uint8_t id){
		uint8_t ret = 1; // 0:fail, 1:ok

		if((id < idMin_) || (idMax_ < id))
            return 0;
        
        // 해당 ID의 모터가 이미 add_motor에 의해 할당되었는지 확인
        if(motors_[id].getOperatingMode() == Op_STATUS_NONE) {
            // 할당되지 않은 모터
            return 0;
        }

        // 모터 상태를 초기 위치 이동 모드로 변경
		motors_[id].setOperatingMode(Op_STATUS_MOVE_DEFAULT_POSI);
        
        // 커브 상태 초기화
        motors_[id].Curve_Clear();
        motors_[id].Curve_Init();

		return ret;
	}

    void enableCANopen(uint8_t id){
        if((id < idMin_) || (idMax_ < id))
            return;
        motors_[id].EnableCANopen();
    }


private:
    CO_t* co_;
    uint8_t idMin_;
    uint8_t idMax_;

    GPIO_TypeDef *LD_Status_Port_;
    uint16_t LD_Status_Pin_;
    GPIO_PinState LD_Status_On_;

    CANopen_Motor motors_[MAX_CANOPEN_MOTORS];
};



#endif /* INC_CANOPEN_MOTOR_H_ */


