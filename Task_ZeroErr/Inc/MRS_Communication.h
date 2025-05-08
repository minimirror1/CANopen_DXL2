#ifndef MRS_COMMUNICATION_H
#define MRS_COMMUNICATION_H

#include "main.h"
#include "cmsis_os.h"
#include "cpp_tick.h"
#include "CANopen_Motor.h"
#include "main_ZeroErr.h"
#include "main_MRS.h"

//// MRS Command Types
//#define MRS_RX_DATA1              0x01
//#define MRS_RX_DATA2              0x02  // 현재 사용하지 않음
//#define MRS_RX_DATA_OP            0x03
//#define MRS_RX_MOVE_DEFAULT_POSI  0x04
//#define MRS_RX_MOVE_DEFAULT_POSI_CHECK 0x05
//#define MRS_TX_MOTOR_STATUS_CHECK 0x06
//
//// MRS Response Types
//#define MRS_TX_DATA1_ACK         0x11
//#define MRS_TX_DATA2_ACK         0x12  // 현재 사용하지 않음
//#define MRS_TX_DATA_OP_ACK       0x13
//#define MRS_TX_DATA_OP_FAIL      0x14
//#define MRS_TX_ERROR_MSG         0x15
//#define MRS_TX_MOVE_DEFAULT_POSI_CHECK 0x16



#ifdef __cplusplus
class MRS_Communication {
public:
    MRS_Communication();
    ~MRS_Communication();

    void processCommandQueue();
    void processPositionQueue();
    void checkCommunicationStatus();
    void handleErrorStatus(uint8_t sid, ZER_StatusCheck_TypeDef status);

private:
    Tick t_ZerRxMotion;
    uint32_t zerRxMotion;
    uint8_t zergid;
    uint8_t zerErrorTxCnt;
    uint32_t os_rx_cnt;
    ZerSetting_TypeDef zerSetting[20];

    void sendErrorMessage(uint8_t sid, const char* errorCode);
    void processCommandMessage(BypassPacket_TypeDef* cmd_rx);
    void processPositionMessage(MotionPacket_TypeDef* motionMsg);
    void sendAckMessage(uint8_t sid, uint8_t cmd, void* data);
};
#endif

#endif // MRS_COMMUNICATION_H 
