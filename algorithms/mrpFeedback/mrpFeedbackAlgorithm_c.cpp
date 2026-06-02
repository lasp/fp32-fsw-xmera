#include "mrpFeedbackAlgorithm_c.h"
#include "architecture/utilities/eigenSupport.h"
#include "mrpFeedbackAlgorithm.h"
#include "mrpFeedbackTypes.h"

#include <Eigen/Core>
#include <algorithm>
#include <array>

namespace {
MrpFeedbackConfig configFromC(const MrpFeedbackConfig_c& c) {
    std::array<float, RW_EFF_CNT> jsList{};
    std::copy(std::begin(c.JsList), std::end(c.JsList), jsList.begin());
    return MrpFeedbackConfig::create(c.K,
                                     c.P,
                                     c.Ki,
                                     c.integralLimit,
                                     static_cast<ControlLawType>(c.controlLawType),
                                     cArrayToEigenVector3<float>(c.knownTorquePntB_B.data),
                                     c2DArrayToEigenMatrix3<float>(c.ISCPntB_B.data),
                                     c.numRW,
                                     cArrayToEigenMatrix<float, 3, RW_EFF_CNT>(c.GsMatrix_B),
                                     jsList);
}
}  // namespace

MrpFeedbackAlgorithmHandle* MrpFeedbackAlgorithm_create(const MrpFeedbackConfig_c* config) {
    // clang-format off
    return reinterpret_cast<MrpFeedbackAlgorithmHandle*>(new ::MrpFeedbackAlgorithm(configFromC(*config)));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    // clang-format on
}

void MrpFeedbackAlgorithm_destroy(MrpFeedbackAlgorithmHandle* self) {
    // clang-format off
    delete reinterpret_cast<::MrpFeedbackAlgorithm*>(self);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    // clang-format on
}

void MrpFeedbackAlgorithm_setConfig(MrpFeedbackAlgorithmHandle* self, const MrpFeedbackConfig_c* config) {
    // clang-format off
    reinterpret_cast<::MrpFeedbackAlgorithm*>(self)->setConfig(configFromC(*config));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

void MrpFeedbackAlgorithm_reset(MrpFeedbackAlgorithmHandle* self) {
    // clang-format off
    reinterpret_cast<::MrpFeedbackAlgorithm*>(self)->reset();  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

MrpFeedbackOutput_c MrpFeedbackAlgorithm_update(MrpFeedbackAlgorithmHandle* self,
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
