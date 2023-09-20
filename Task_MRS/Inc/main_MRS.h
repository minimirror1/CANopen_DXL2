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


typedef struct _MotionPacket_TypeDef{
	uint8_t gid;
	uint8_t sid;
	uint32_t posi;
}MotionPacket_TypeDef;

#ifdef __cplusplus
}
#endif


#endif /* INC_MAIN_MRS_H_ */
