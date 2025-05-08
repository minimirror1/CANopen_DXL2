/*
 * sync.h
 *
 *  Created on: Jul 21, 2023
 *      Author: minim
 */

#ifndef INC_SYNC_H_
#define INC_SYNC_H_


/* CANopen SDK */
#include "../CANopenNode_STM32/CO_app_STM32.h"
#include "OD.h"

CO_SDO_abortCode_t send_sync(CO_t *co);


#endif /* INC_SYNC_H_ */
