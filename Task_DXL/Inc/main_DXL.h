/*
 * main_DXL.h
 *
 *  Created on: Sep 19, 2023
 *      Author: minim
 */

#ifndef INC_MAIN_DXL_H_
#define INC_MAIN_DXL_H_


#ifdef __cplusplus
extern "C" {
#endif

typedef struct _DxlSetting_TypeDef{
	/*data1*/
	uint8_t f_data1;		//true : 새로운값, false : none
	uint8_t rot_dir;
	float angle;
	int defult_posi;
	int home_cnt;
}DxlSetting_TypeDef;


void main_DXL(void *argument);

#ifdef __cplusplus
}
#endif


#endif /* INC_MAIN_DXL_H_ */
