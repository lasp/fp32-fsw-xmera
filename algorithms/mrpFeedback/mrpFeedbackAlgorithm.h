#ifndef F32XMERA_MRP_FEEDBACK_ALGORITHM_H
#define F32XMERA_MRP_FEEDBACK_ALGORITHM_H

#include <cstdint>

#include "mrpFeedbackTypes.h"
#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/RWAvailabilityMsgPayload.h"
#include "msgPayloadDef/RWSpeedMsgF32Payload.h"
#include "msgPayloadDef/VehicleConfigMsgF32Payload.h"

#include <Eigen/Core>

/*! @brief Data configuration structure for the MRP feedback attitude control routine. */
class MrpFeedbackAlgorithm final {
   public:
    explicit MrpFeedbackAlgorithm(const MrpFeedbackConfig& config);

    void setConfig(const MrpFeedbackConfig& config);

    void reset(VehicleConfigMsgF32Payload vehConfigMsg, const RWArrayConfigMsgF32Payload& rwConfigMsg, bool rwIsLinked);
    MrpFeedbackOutput update(uint64_t callTime,
                             AttGuidMsgF32Payload& guidCmd,
                             const RWSpeedMsgF32Payload& wheelSpeeds,
                             const RWAvailabilityMsgPayload& wheelsAvailability);

   private:
    MrpFeedbackConfig cfg;
    uint64_t priorTime{};                       //!< [ns]      Last time the attitude control is called
    Eigen::Vector3f int_sigma{};                //!< [s] integral of the MPR attitude error
    Eigen::Matrix3f ISCPntB_B{};                //!< [kg m^2] Spacecraft Inertia
    RWArrayConfigMsgF32Payload rwConfigParams{};  //!< RW config snapshot taken at reset() time
};

#endif
