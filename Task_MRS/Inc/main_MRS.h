/*
 * main_MRS.h
 *
 *  Created on: Sep 20, 2023
 *      Author: minim
 */

#ifndef INC_MAIN_MRS_H_
#define INC_MAIN_MRS_H_


#ifdef __cplusplus
extern "C" {
#endif

void main_MRS(void *argument);



typedef enum{
	INIT_NONE = 0,
	INIT_NEW_DATA,					//new data-> info + default posi
	INIT_INFO_DEFAULT_POSI_START,
	INIT_OK,						//ok -> default posi
	INIT_FAIL,
	INIT_DEFAULT_POSI_START
}Init_TypeDef;

typedef struct _MotionPacket_TypeDef{
	uint8_t gid;
	uint8_t sid;
	uint32_t posi;
}MotionPacket_TypeDef;

/*bypass*/
typedef enum _BypassCmd_TypeDef{
	MRS_RX_DATA1 = 1,
	MRS_RX_DATA2,
	MRS_RX_DATA_OP,
	MRS_RX_MOVE_DEFAULT_POSI,
	MRS_RX_MOVE_DEFAULT_POSI_CHECK,
	MRS_TX_DATA1_ACK,
	MRS_TX_DATA2_ACK,
	MRS_TX_DATA_OP_ACK,
	MRS_TX_MOVE_DEFAULT_POSI_ACK,
	MRS_TX_MOVE_DEFAULT_POSI_CHECK
}BypassCmd_TypeDef;

typedef struct _BypassPacket_TypeDef{
	uint8_t gid;
	uint8_t sid;
	uint8_t cmd;
	uint8_t data[8];
}BypassPacket_TypeDef;

#ifdef __cplusplus
}
#endif


#endif /* INC_MAIN_MRS_H_ */


