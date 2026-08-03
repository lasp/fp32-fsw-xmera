#ifndef F32XMERA_THR_MOMENTUM_MANAGEMENT_H
#define F32XMERA_THR_MOMENTUM_MANAGEMENT_H

#include <stdint.h>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>

/*! @brief Assesses the net reaction wheel momentum and requests the angular momentum change needed to dump it. */
class ThrMomentumManagement : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    /* declare module private variables */
    int initRequest;  //!< [-] one-shot latch; 1 requests a momentum check on the next update, cleared once performed
    RWArrayConfigMsgPayload
        rwConfigParams;  //!< [-] struct to store message containing RW config parameters in body B frame

    /* declare module public variables */
    double hs_min;  //!< [Nms]  minimum RW cluster momentum for dumping

    /* declare module IO interfaces */
    Message<CmdTorqueBodyMsgPayload> deltaHOutMsg;           //!< [Nms] requested body-frame angular momentum change
    ReadFunctor<RWSpeedMsgPayload> rwSpeedsInMsg;            //!< [r/s] reaction wheel speeds input message
    ReadFunctor<RWArrayConfigMsgPayload> rwConfigDataInMsg;  //!< [-] RW array configuration input message
};

#endif
