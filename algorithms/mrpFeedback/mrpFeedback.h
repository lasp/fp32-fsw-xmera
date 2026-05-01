#ifndef F32XMERA_MRP_FEEDBACK_H
#define F32XMERA_MRP_FEEDBACK_H

#include <stdint.h>
#include <memory>

#include "mrpFeedbackAlgorithm.h"
#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/RWAvailabilityMsgPayload.h"
#include "msgPayloadDef/RWSpeedMsgF32Payload.h"
#include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

#include <Eigen/Core>

/*! @brief MRP feedback attitude-control adapter. */
class MrpFeedback final : public SysModel {
   public:
    MrpFeedback() = default;
    ~MrpFeedback() override = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    // Phase 1: public config properties -- set before reset().
    float K = 0.0F;                                               //!< [N*m]    proportional gain on MRP error
    float P = 0.0F;                                               //!< [N*m*s]  rate-error feedback gain
    float Ki = 0.0F;                                              //!< [N*m]    integral feedback gain (0 disables)
    float integralLimit = 0.0F;                                   //!< [N*m*s]  anti-windup clamp on int_sigma
    ControlLawType controlLawType = ControlLawType::NORMAL;       //!< control-law variant
    Eigen::Vector3f knownTorquePntB_B = Eigen::Vector3f::Zero();  //!< [N*m]    feedforward known external torque

    ReadFunctor<RWSpeedMsgF32Payload> rwSpeedsInMsg;        //!< RW speed input message (Optional)
    ReadFunctor<RWAvailabilityMsgPayload> rwAvailInMsg;     //!< RW availability input message (Optional)
    ReadFunctor<RWArrayConfigMsgF32Payload> rwParamsInMsg;  //!< RW parameter input message.  (Optional)
    Message<CmdTorqueBodyMsgF32Payload>
        cmdTorqueOutMsg;  //!< commanded spacecraft external control torque output message
    Message<CmdTorqueBodyMsgF32Payload>
        intFeedbackTorqueOutMsg;                  //!< commanded integral feedback control torque output message
    ReadFunctor<AttGuidMsgF32Payload> guidInMsg;  //!< attitude guidance input message
    ReadFunctor<VehicleConfigMsgF32Payload> vehConfigInMsg;  //!< vehicle configuration input message

   private:
    std::unique_ptr<MrpFeedbackAlgorithm> algorithm = nullptr;
    uint32_t numRW{};
};

#endif
