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

/*! @brief Assesses the net reaction wheel momentum and requests the torque needed to dump its excess. */
class ThrMomentumManagement : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    //! Re-validate the module properties and push them onto the live algorithm.
    void reconfigure();

    /* declare module public variables */
    float hsMin{};  //!< [Nms]  minimum RW cluster momentum for dumping
    float K{};      //!< [1/s]  proportional gain on the excess momentum (must be > 0)

    /* declare module IO interfaces */
    Message<CmdTorqueBodyMsgF32Payload> cmdTorqueOutMsg;        //!< [Nm] requested body-frame dumping torque
    ReadFunctor<RWSpeedMsgF32Payload> rwSpeedsInMsg;            //!< [r/s] reaction wheel speeds input message
    ReadFunctor<RWArrayConfigMsgF32Payload> rwConfigDataInMsg;  //!< [-] RW array configuration input message

   private:
    ThrMomentumManagementConfig toConfig();
    std::unique_ptr<ThrMomentumManagementAlgorithm> algorithm = nullptr;
};

#endif
