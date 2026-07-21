#include "mrpFeedbackAlgorithm_c.h"
#include "mrpFeedbackAlgorithm.h"
#include "mrpFeedbackTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

namespace {
MrpFeedbackConfig configFromC(const MrpFeedbackConfig_c& c) {
    return MrpFeedbackConfig::create(c.K,
                                     c.P,
                                     c.Ki,
                                     c.integralLimit,
                                     static_cast<ControlLawType>(c.controlLawType),
                                     cArrayToEigenVector3<float>(c.knownTorquePntB_B.data));
}
}  // namespace

MrpFeedbackAlgorithmHandle* MrpFeedbackAlgorithm_create(const MrpFeedbackConfig_c* config) {
    return fsw::createHandle<::MrpFeedbackAlgorithm, MrpFeedbackAlgorithmHandle>(configFromC(*config));
}

void MrpFeedbackAlgorithm_destroy(MrpFeedbackAlgorithmHandle* self) { fsw::deleteHandle<::MrpFeedbackAlgorithm>(self); }

void MrpFeedbackAlgorithm_setConfig(MrpFeedbackAlgorithmHandle* self, const MrpFeedbackConfig_c* config) {
    fsw::fromHandle<::MrpFeedbackAlgorithm>(self)->setConfig(configFromC(*config));
}

void MrpFeedbackAlgorithm_reset(MrpFeedbackAlgorithmHandle* self,
                                const VehicleConfigMsgF32Payload* vehConfigMsg,
                                const RWArrayConfigMsgF32Payload* rwConfigMsg,
                                int rwIsLinked) {
    fsw::fromHandle<::MrpFeedbackAlgorithm>(self)->reset(*vehConfigMsg, *rwConfigMsg, rwIsLinked != 0);
}

MrpFeedbackOutput_c MrpFeedbackAlgorithm_update(MrpFeedbackAlgorithmHandle* self,
                                                uint64_t callTime,
                                                const AttGuidMsgF32Payload* guidCmd,
                                                const RWSpeedMsgF32Payload* wheelSpeeds,
                                                const RWAvailabilityMsgPayload* wheelsAvailability) {
    const MrpFeedbackOutput out =
        fsw::fromHandle<::MrpFeedbackAlgorithm>(self)->update(callTime, *guidCmd, *wheelSpeeds, *wheelsAvailability);

    MrpFeedbackOutput_c result{};
    result.controlOut = out.controlOut;
    result.intFeedbackOut = out.intFeedbackOut;
    return result;
}
