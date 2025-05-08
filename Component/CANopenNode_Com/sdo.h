/*
 * sdo.h
 *
 *  Created on: Jul 21, 2023
 *      Author: minim
 */

#ifndef INC_SDO_H_
#define INC_SDO_H_


/* CANopen SDK */
#include "../CANopenNode_STM32/CO_app_STM32.h"
#include "OD.h"


CO_SDO_abortCode_t read_SDO(CO_SDOclient_t *SDO_C, uint8_t nodeId,
                            uint16_t index, uint8_t subIndex,
                            uint8_t *buf, size_t bufSize, size_t *readSize);

CO_SDO_abortCode_t write_SDO(CO_SDOclient_t *SDO_C, uint8_t nodeId,
                             uint16_t index, uint8_t subIndex,
                             uint8_t *data, size_t dataSize);

#endif /* INC_SDO_H_ */
