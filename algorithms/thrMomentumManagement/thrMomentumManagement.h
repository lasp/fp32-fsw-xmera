#ifndef F32XMERA_THR_MOMENTUM_MANAGEMENT_H
#define F32XMERA_THR_MOMENTUM_MANAGEMENT_H

#include "thrMomentumManagementAlgorithm.h"
#include <stdint.h>

#include <memory>

#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/RWSpeedMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

/*! @brief Assesses the net reaction wheel momentum and requests the angular momentum change needed to dump it. */
class ThrMomentumManagement : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    //! Re-validate the module properties and push them onto the live algorithm.
    void reconfigure();

    /* declare module public variables */
    float hsMin{};  //!< [Nms]  minimum RW cluster momentum for dumping

    /* declare module IO interfaces */
    Message<CmdTorqueBodyMsgF32Payload> deltaHOutMsg;           //!< [Nms] requested body-frame momentum change
    ReadFunctor<RWSpeedMsgF32Payload> rwSpeedsInMsg;            //!< [r/s] reaction wheel speeds input message
    ReadFunctor<RWArrayConfigMsgF32Payload> rwConfigDataInMsg;  //!< [-] RW array configuration input message

   private:
    ThrMomentumManagementConfig toConfig();
    std::unique_ptr<ThrMomentumManagementAlgorithm> algorithm = nullptr;
};

#endif
