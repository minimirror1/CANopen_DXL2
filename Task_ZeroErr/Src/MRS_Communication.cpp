#include "MRS_Communication.h"
#include "string.h"
#include "main_MRS.h"

extern osMessageQueueId_t zerPosiHandle;
extern osMessageQueueId_t zerCmd_rxHandle;
extern osMessageQueueId_t zerCmd_txHandle;
extern Motors motors;
extern Init_TypeDef Zer_All_init_flag;

MRS_Communication::MRS_Communication() : 
    zerRxMotion(0),
    zergid(0),
    zerErrorTxCnt(30),
    os_rx_cnt(0)
{
}

MRS_Communication::~MRS_Communication()
{
}

void MRS_Communication::processCommandQueue()
{
    BypassPacket_TypeDef cmd_rx;
    osStatus_t status = osMessageQueueGet(zerCmd_rxHandle, &cmd_rx, NULL, 0U);
    
    if (status == osOK) {
        processCommandMessage(&cmd_rx);
        zerRxMotion = t_ZerRxMotion.getTickCount();
    }
}

void MRS_Communication::processPositionQueue()
{
    MotionPacket_TypeDef motionMsg;
    osStatus_t status;
    
    for(int time = 0; time < 9; time++) {
        do {
            status = osMessageQueueGet(zerPosiHandle, &motionMsg, NULL, 0U);
            
            if (status == osOK) {
                processPositionMessage(&motionMsg);
                os_rx_cnt++;
                zerRxMotion = t_ZerRxMotion.getTickCount();
            }
        } while (status == osOK);
        osDelay(1);
    }
}

void MRS_Communication::checkCommunicationStatus()
{
    if(t_ZerRxMotion.elapsed(zerRxMotion) >= 37000) {
        for(int i = 1; i <= 12; i++) {
            ZER_StatusCheck_TypeDef status = motors.getStatusFaultCheck(i);
            if(status != 999 && status != ZER_STATUS_NONE) {
                handleErrorStatus(i, status);
            }
        }
        zerRxMotion = t_ZerRxMotion.getTickCount();
    }
}

void MRS_Communication::handleErrorStatus(uint8_t sid, ZER_StatusCheck_TypeDef status)
{
    if(t_ZerRxMotion.delay(500) && zerErrorTxCnt != 0) {
        const char* errorCode;
        switch(status) {
            case ZER_STATUS_FAULT:
                errorCode = "001";
                break;
            case ZER_STATUS_WARNING:
                errorCode = "002";
                break;
            case ZER_STATUS_TIMEOUT:
                errorCode = "003";
                break;
            default:
                return;
        }
        sendErrorMessage(sid, errorCode);
        zerErrorTxCnt--;
    }
}

void MRS_Communication::sendErrorMessage(uint8_t sid, const char* errorCode)
{
    BypassPacket_TypeDef msg = {0,};
    msg.gid = zergid;
    msg.sid = sid;
    msg.cmd = MRS_TX_ERROR_MSG;
    memcpy(msg.data, (uint8_t *)errorCode, 8);
    osMessageQueuePut(zerCmd_txHandle, &msg, 0U, 0U);
}

void MRS_Communication::sendAckMessage(uint8_t sid, uint8_t cmd, void* data)
{
    BypassPacket_TypeDef msg = {0,};
    msg.gid = zergid;
    msg.sid = sid;
    msg.cmd = cmd;
    memcpy(msg.data, (uint8_t *)data, 8);
    osMessageQueuePut(zerCmd_txHandle, &msg, 0U, 0U);
}

void MRS_Communication::processCommandMessage(BypassPacket_TypeDef* cmd_rx)
{
    if (cmd_rx->sid > 12)
        return;

    zergid = cmd_rx->gid;

    switch (cmd_rx->cmd) {
        case MRS_RX_DATA1: {
            prtc_data_ctl_init_driver_data1_t *pData = (prtc_data_ctl_init_driver_data1_t*) cmd_rx->data;

            zerSetting[cmd_rx->sid].rot_dir = (pData->direction == 0 ? ROT_CW : ROT_CCW);
            zerSetting[cmd_rx->sid].angle = (float) pData->angle / 100;
            zerSetting[cmd_rx->sid].defult_posi = pData->init_position;
            zerSetting[cmd_rx->sid].f_data1 = true;

            sendAckMessage(cmd_rx->sid, MRS_TX_DATA1_ACK, pData);
            break;
        }

        case MRS_RX_DATA_OP: {
            if (zerSetting[cmd_rx->sid].f_data1 != true)
                return;

            prtc_data_ctl_init_driver_data_op_zero_err_t *pData = (prtc_data_ctl_init_driver_data_op_zero_err_t*) cmd_rx->data;
            zerSetting[cmd_rx->sid].tar_speed = pData->profile_target_speed;
            zerSetting[cmd_rx->sid].tar_acc = pData->profile_acc_cnt;

            motors.add_motor(
                cmd_rx->sid,
                zerSetting[cmd_rx->sid].rot_dir,
                zerSetting[cmd_rx->sid].angle,
                zerSetting[cmd_rx->sid].tar_speed,
                zerSetting[cmd_rx->sid].tar_acc,
                zerSetting[cmd_rx->sid].defult_posi);

            sendAckMessage(cmd_rx->sid, MRS_TX_DATA_OP_ACK, pData);
            break;
        }

        case MRS_RX_MOVE_DEFAULT_POSI: {
            if(cmd_rx->sid == 12) {
                if(zerSetting[cmd_rx->sid].f_data1 == true) {
                    Zer_All_init_flag = INIT_INFO_DEFAULT_POSI_START;
                    zerSetting[cmd_rx->sid].f_data1 = false;
                } else {
                    Zer_All_init_flag = INIT_DEFAULT_POSI_START;
                }
            }
            break;
        }

        case MRS_RX_MOVE_DEFAULT_POSI_CHECK: {
            if(Zer_All_init_flag == INIT_OK) {
                BypassPacket_TypeDef msg = {0,};
                msg.gid = cmd_rx->gid;
                msg.sid = cmd_rx->sid;
                msg.cmd = MRS_TX_MOVE_DEFAULT_POSI_CHECK;
                osMessageQueuePut(zerCmd_txHandle, &msg, 0U, 0U);
            }
            break;
        }

        case MRS_TX_MOTOR_STATUS_CHECK:
            break;

        default:
            break;
    }
}

void MRS_Communication::processPositionMessage(MotionPacket_TypeDef* motionMsg)
{
    motors.setPosition(motionMsg->sid, motionMsg->posi);
} 