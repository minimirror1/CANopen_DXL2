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

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/



void main_MRS(void *argument){

	/* MRS protocol */
	can_init_data_save(&hcan1);
	gm_motion_TX_LED_init(LD_MRS_TX_GPIO_Port, LD_MRS_TX_Pin, GPIO_PIN_RESET);
	gm_motion_RX_LED_init(LD_MRS_RX_GPIO_Port, LD_MRS_RX_Pin, GPIO_PIN_RESET);
	//CAN_FilterConfig(&hcan1);

	//myCanid = idRead();

	//set_my_can_id(myCanid);
	add_my_can_sub_id(1, 28);

	while(1){
		osDelay(1);
		/* MRS Protocol */
		proc_can_rx();
		proc_can_tx();
	}
}
