/*
 * sdo.cpp
 *
 *  Created on: Jul 21, 2023
 *      Author: minim
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
//#include "canopen_main.h"
#include "main.h"

#include "stdio.h"
#include "string.h"

#include "sdo.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

/* CANopen SDK */
#include "../CANopenNode_STM32/CO_app_STM32.h"
#include "OD.h"

// CAN 통신 재시도 딜레이 값 적용
#define CAN_RETRY_DELAY 10  // 실패한 경우 재시도 전에 딜레이 시간(ms)

/* Private includes ----------------------------------------------------------*/



CO_SDO_abortCode_t read_SDO(CO_SDOclient_t *SDO_C, uint8_t nodeId,
                            uint16_t index, uint8_t subIndex,
                            uint8_t *buf, size_t bufSize, size_t *readSize)
{
    CO_SDO_return_t SDO_ret;

    // setup client (this can be skipped, if remote device don't change)
    SDO_ret = CO_SDOclient_setup(SDO_C,
                                 CO_CAN_ID_SDO_CLI + nodeId,
                                 CO_CAN_ID_SDO_SRV + nodeId,
                                 nodeId);
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
        return CO_SDO_AB_GENERAL;
    }

    // initiate upload
    SDO_ret = CO_SDOclientUploadInitiate(SDO_C, index, subIndex, 1000, false);
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
        return CO_SDO_AB_GENERAL;
    }

    // upload data
    do {
    	uint32_t timeDifference_us = 10000;
        CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;

        SDO_ret = CO_SDOclientUpload(SDO_C,
                                     timeDifference_us,
                                     false,
                                     &abortCode,
                                     NULL, NULL, NULL);
        if (SDO_ret < 0) {
            return abortCode;
        }

        osDelay(timeDifference_us/1000);//1ms osdelay
    } while(SDO_ret > 0);

    // copy data to the user buffer (for long data function must be called
    // several times inside the loop)
    *readSize = CO_SDOclientUploadBufRead(SDO_C, buf, bufSize);

    return CO_SDO_AB_NONE;
}

CO_SDO_abortCode_t write_SDO(CO_SDOclient_t *SDO_C, uint8_t nodeId,
                             uint16_t index, uint8_t subIndex,
                             uint8_t *data, size_t dataSize)
{
    CO_SDO_return_t SDO_ret;
    bool_t bufferPartial = false;

    // setup client (this can be skipped, if remote device is the same)
    SDO_ret = CO_SDOclient_setup(SDO_C,
                                 CO_CAN_ID_SDO_CLI + nodeId,
                                 CO_CAN_ID_SDO_SRV + nodeId,
                                 nodeId);
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
        return CO_SDO_AB_GENERAL;
    }

    // initiate download
    SDO_ret = CO_SDOclientDownloadInitiate(SDO_C, index, subIndex,
                                           dataSize, 1000, false);
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
        return CO_SDO_AB_CMD;
    }

    // fill data
    size_t nWritten = CO_SDOclientDownloadBufWrite(SDO_C, data, dataSize);
    if (nWritten < dataSize) {
        bufferPartial = true;
        // If SDO Fifo buffer is too small, data can be refilled in the loop.
    }

    //download data
    do {
        uint32_t timeDifference_us = 10000;
        CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;

        SDO_ret = CO_SDOclientDownload(SDO_C,
                                       timeDifference_us,
                                       false,
                                       bufferPartial,
                                       &abortCode,
                                       NULL, NULL);
        if (SDO_ret < 0) {
            return abortCode;
        }

        //sleep_us(timeDifference_us);
        osDelay(timeDifference_us/1000);
    } while(SDO_ret > 0);

    return CO_SDO_AB_NONE;
}
