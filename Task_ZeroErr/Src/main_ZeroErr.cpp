/*
 * main_ZeroErr.cpp
 *
 *  Created on: Sep 21, 2023
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
#include "main_ZeroErr.h"

/* Component -----------------------------------------------------------------*/
#include "cpp_serial.h"
#include "cpp_tick.h"


void main_ZeroErr(void *argument){



	while(1){
		osDelay(10);
	}
}
