#ifndef F32XMERA_MRP_FEEDBACK_H
#define F32XMERA_MRP_FEEDBACK_H

#include <stdint.h>

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

/*! @brief Data configuration structure for the MRP feedback attitude control routine. */
class MrpFeedback final : public SysModel {
   public:
    MrpFeedback() = default;
    ~MrpFeedback() override = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setK(float gain);
    float getK() const;
    void setP(float gain);
    float getP() const;
    void setKi(float gain);
    float getKi() const;
    void setIntegralLimit(float limit);
    float getIntegralLimit() const;
    void setControlLawType(int type);
    int getControlLawType() const;
    void setKnownTorquePntB_B(const Eigen::Vector3f& torque);
    Eigen::Vector3f getKnownTorquePntB_B() const;

    /* declare module IO interfaces */
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
    void rebuildAlgorithmConfig();

    float K = 0.0F;
    float P = 0.0F;
    float Ki = 0.0F;
    float integralLimit = 0.0F;
    ControlLawType controlLawType = ControlLawType::NORMAL;
    Eigen::Vector3f knownTorquePntB_B = Eigen::Vector3f::Zero();

    MrpFeedbackAlgorithm algorithm{
        MrpFeedbackConfig::create(0.0F, 0.0F, 0.0F, 0.0F, ControlLawType::NORMAL, Eigen::Vector3f::Zero())};
    uint32_t numRW{};  //!< number of reaction wheels
};

#endif
