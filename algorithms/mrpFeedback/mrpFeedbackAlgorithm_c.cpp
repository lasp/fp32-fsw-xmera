#include "mrpFeedbackAlgorithm_c.h"
#include "architecture/utilities/eigenSupport.h"
#include "mrpFeedbackAlgorithm.h"
#include "mrpFeedbackTypes.h"

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

MrpFeedbackAlgorithm* MrpFeedbackAlgorithm_create(const MrpFeedbackConfig_c* config) {
    // clang-format off
    return reinterpret_cast<MrpFeedbackAlgorithm*>(new ::MrpFeedbackAlgorithm(configFromC(*config)));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    // clang-format on
}

void MrpFeedbackAlgorithm_destroy(MrpFeedbackAlgorithm* self) {
    // clang-format off
    delete reinterpret_cast<::MrpFeedbackAlgorithm*>(self);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    // clang-format on
}

void MrpFeedbackAlgorithm_setConfig(MrpFeedbackAlgorithm* self, const MrpFeedbackConfig_c* config) {
    // clang-format off
    reinterpret_cast<::MrpFeedbackAlgorithm*>(self)->setConfig(configFromC(*config));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

void MrpFeedbackAlgorithm_reset(MrpFeedbackAlgorithm* self,
                                const VehicleConfigMsgF32Payload* vehConfigMsg,
                                const RWArrayConfigMsgF32Payload* rwConfigMsg,
                                int rwIsLinked) {
    // clang-format off
    reinterpret_cast<::MrpFeedbackAlgorithm*>(self)->reset(*vehConfigMsg, *rwConfigMsg, rwIsLinked != 0);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

MrpFeedbackOutput_c MrpFeedbackAlgorithm_update(MrpFeedbackAlgorithm* self,
                                                uint64_t callTime,
                                                const AttGuidMsgF32Payload* guidCmd,
                                                const RWSpeedMsgF32Payload* wheelSpeeds,
                                                const RWAvailabilityMsgPayload* wheelsAvailability) {
    // clang-format off
    const MrpFeedbackOutput out = reinterpret_cast<::MrpFeedbackAlgorithm*>(self)->update(callTime, *guidCmd, *wheelSpeeds, *wheelsAvailability);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on

    MrpFeedbackOutput_c result{};
    result.controlOut = out.controlOut;
    result.intFeedbackOut = out.intFeedbackOut;
    return result;
}
