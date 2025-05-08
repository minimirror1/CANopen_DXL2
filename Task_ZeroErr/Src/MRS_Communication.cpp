#include "MRS_Communication.h"
#include <cstring>
#include "main_MRS.h"

extern osMessageQueueId_t zerPosiHandle;
extern osMessageQueueId_t zerCmd_rxHandle;
extern osMessageQueueId_t zerCmd_txHandle;
extern Motors motors;
extern Init_TypeDef Zer_All_init_flag;
extern CO_t* CO;

MRS_Communication::MRS_Communication() : 
    zerRxMotion(0),
    zergid(0),
    zerErrorTxCnt(30),
    os_rx_cnt(0)
{
    // 초기화 시 모든 모터의 설정 변경 플래그를 false로 설정
    for (int i = 0; i <= 12; i++) {
        zerSetting[i].settings_changed = false;
    }
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

void MRS_Communication::processMovePosition()
{
    motors.movePosition();
	send_RPDO_BuffSend(CO);
	osDelay(1);
	send_sync(CO);
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



// sendAckMessage 함수 사용 예시:
//
// 1. 기본 사용법 (데이터 없이 응답만 보내기)
// sendAckMessage(1, MRS_TX_DATA1_ACK, nullptr);
//
// 2. 데이터와 함께 응답 보내기
// prtc_data_ctl_init_driver_data1_t responseData;
// responseData.direction = 1;
// responseData.angle = 90;
// sendAckMessage(1, MRS_TX_DATA1_ACK, &responseData);
//
// 3. 에러 응답 보내기
// sendAckMessage(1, MRS_TX_DATA_OP_FAIL, nullptr);
//
// 매개변수 설명:
// - sid: 대상 모터 ID (1~12)
// - cmd: 응답 명령어 타입 (MRS_TX_로 시작하는 매크로)
// - data: 전송할 데이터 포인터 (최대 8바이트, nullptr 가능)

void MRS_Communication::sendAckMessage(uint8_t sid, uint8_t cmd, void* data)
{
    BypassPacket_TypeDef msg = {0,};
    msg.gid = zergid;
    msg.sid = sid;
    msg.cmd = cmd;
    if (data != nullptr) {
        memcpy(msg.data, (uint8_t *)data, 8);
    }
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
            
            zerSetting[cmd_rx->sid].settings_changed = false;
            
            if (zerSetting[cmd_rx->sid].rot_dir != (pData->direction == 0 ? ROT_CW : ROT_CCW) ||
                zerSetting[cmd_rx->sid].angle != (float) pData->angle / 100 ||
                zerSetting[cmd_rx->sid].defult_posi != pData->init_position ||
                zerSetting[cmd_rx->sid].f_data1 != true) {
                
                zerSetting[cmd_rx->sid].rot_dir = (pData->direction == 0 ? ROT_CW : ROT_CCW);
                zerSetting[cmd_rx->sid].angle = (float) pData->angle / 100;
                zerSetting[cmd_rx->sid].defult_posi = pData->init_position;
                zerSetting[cmd_rx->sid].f_data1 = true;
                zerSetting[cmd_rx->sid].settings_changed = true;
            }

            sendAckMessage(cmd_rx->sid, MRS_TX_DATA1_ACK, pData);
            break;
        }

        case MRS_RX_DATA_OP: {
            if (zerSetting[cmd_rx->sid].f_data1 != true)
                return;

            prtc_data_ctl_init_driver_data_op_zero_err_t *pData = (prtc_data_ctl_init_driver_data_op_zero_err_t*) cmd_rx->data;
            
            if (zerSetting[cmd_rx->sid].tar_speed != pData->profile_target_speed ||
                zerSetting[cmd_rx->sid].tar_acc != pData->profile_acc_cnt) {
                
                zerSetting[cmd_rx->sid].tar_speed = pData->profile_target_speed;
                zerSetting[cmd_rx->sid].tar_acc = pData->profile_acc_cnt;
                zerSetting[cmd_rx->sid].settings_changed = true;
            }

            motors.add_motor(
                cmd_rx->sid,
                zerSetting[cmd_rx->sid].rot_dir,
                zerSetting[cmd_rx->sid].angle,
                zerSetting[cmd_rx->sid].tar_speed,
                zerSetting[cmd_rx->sid].tar_acc,
                zerSetting[cmd_rx->sid].defult_posi);

            if (zerSetting[cmd_rx->sid].settings_changed && motors.init_motor(cmd_rx->sid) == 1) {
                zerSetting[cmd_rx->sid].settings_changed = false;  // 초기화 성공 후 플래그 리셋
                sendAckMessage(cmd_rx->sid, MRS_TX_DATA_OP_ACK, pData);
            }
            else if (!zerSetting[cmd_rx->sid].settings_changed) {
                sendAckMessage(cmd_rx->sid, MRS_TX_DATA_OP_ACK, pData);
            }
            else {
                sendAckMessage(cmd_rx->sid, MRS_TX_DATA_OP_FAIL, pData);
            }
            
            break;
        }

        case MRS_RX_MOVE_DEFAULT_POSI: {

			if(zerSetting[cmd_rx->sid].f_data1 == true) {
				motors.init_default_posi(cmd_rx->sid);
				zerSetting[cmd_rx->sid].f_data1 = false;
			}

            break;
        }

        case MRS_RX_MOVE_DEFAULT_POSI_CHECK: {

            if(motors.getOperatingMode(cmd_rx->sid) == Op_STATUS_OPERATING) {

                sendAckMessage(cmd_rx->sid, MRS_TX_MOVE_DEFAULT_POSI_CHECK, nullptr);
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
