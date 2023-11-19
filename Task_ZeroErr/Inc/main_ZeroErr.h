/*
 * main_ZeroErr.h
 *
 *  Created on: Sep 21, 2023
 *      Author: minim
 */

#ifndef INC_MAIN_ZEROERR_H_
#define INC_MAIN_ZEROERR_H_


#ifdef __cplusplus
extern "C" {
#endif


typedef struct _ZerSetting_TypeDef{
	/*data1*/
	uint8_t f_data1;		//true : 새로운값, false : none
	uint8_t rot_dir;
	float angle;
	int defult_posi;
	/*data2*/
	uint32_t tar_speed;
	uint32_t tar_acc;
}ZerSetting_TypeDef;


void main_ZeroErr(void *argument);

#ifdef __cplusplus
}
#endif


#endif /* INC_MAIN_ZEROERR_H_ */
