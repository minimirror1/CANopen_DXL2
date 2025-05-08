/*
 * pdo.h
 *
 *  Created on: Jul 21, 2023
 *      Author: minim
 */

#ifndef INC_PDO_H_
#define INC_PDO_H_


/* CANopen SDK */
#include "../CANopenNode_STM32/CO_app_STM32.h"
#include "OD.h"

void TPDO1_rx_init();
CO_SDO_abortCode_t send_RPDO_1(CO_t *co, uint8_t nodeId, uint8_t *txData, uint8_t size);
CO_SDO_abortCode_t send_RPDO_2(CO_t *co, uint8_t nodeId, uint8_t *txData, uint8_t size);
CO_SDO_abortCode_t send_RPDO_1_Buff(CO_t *co, uint8_t nodeId, uint8_t *txData, uint8_t size);
CO_SDO_abortCode_t send_RPDO_2_Buff(CO_t *co, uint8_t nodeId, uint8_t *txData, uint8_t size);
void send_RPDO_BuffSend(CO_t *co);


#endif /* INC_PDO_H_ */
