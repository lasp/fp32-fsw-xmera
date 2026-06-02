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
                                                const MrpFeedbackGuidInput_c* guid,
                                                const float wheelSpeeds[RW_EFF_CNT],
                                                const bool wheelAvailability[RW_EFF_CNT]) {
    MrpFeedbackGuidInput guidCpp;
    guidCpp.sigma_BR = cArrayToEigenVector3<float>(guid->sigma_BR.data);
    guidCpp.omega_BR_B = cArrayToEigenVector3<float>(guid->omega_BR_B.data);
    guidCpp.omega_RN_B = cArrayToEigenVector3<float>(guid->omega_RN_B.data);
    guidCpp.domega_RN_B = cArrayToEigenVector3<float>(guid->domega_RN_B.data);

    const Eigen::Vector<float, RW_EFF_CNT> speeds = Eigen::Map<const Eigen::Vector<float, RW_EFF_CNT>>(wheelSpeeds);
    std::array<bool, RW_EFF_CNT> availability{};
    std::copy(wheelAvailability, wheelAvailability + RW_EFF_CNT, availability.begin());

    // clang-format off
    const MrpFeedbackOutput out = reinterpret_cast<::MrpFeedbackAlgorithm*>(self)->update(callTime, guidCpp, speeds, availability);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on

    MrpFeedbackOutput_c result{};
    result.controlOut = out.controlOut;
    result.intFeedbackOut = out.intFeedbackOut;
    return result;
}
