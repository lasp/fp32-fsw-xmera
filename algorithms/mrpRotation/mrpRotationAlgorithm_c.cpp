#include "mrpRotationAlgorithm_c.h"
#include "architecture/utilities/eigenSupport.h"
#include "mrpRotationAlgorithm.h"
#include "mrpRotationTypes.h"

#include <Eigen/Core>

namespace {
MrpRotationConfig configFromC(const MrpRotationConfig_c& c) {
    return MrpRotationConfig::create(cArrayToEigenVector3<float>(c.initialSigmaRR0.data),
                                     cArrayToEigenVector3<float>(c.omegaRR0R.data),
                                     c.controlPeriod);
}

MrpRotationAttRefInputs attRefFromC(const MrpRotationAttRefInputs_c& c) {
    return MrpRotationAttRefInputs{
        cArrayToEigenVector3<float>(c.sigma_R0N.data),
        cArrayToEigenVector3<float>(c.omega_R0N_N.data),
        cArrayToEigenVector3<float>(c.domega_R0N_N.data),
    };
}

MrpRotationOutput_c outputToC(const MrpRotationOutput& out) {
    MrpRotationOutput_c result{};
    eigenVectorToCArray(out.sigma_RN, result.sigma_RN.data);
    eigenVectorToCArray(out.omega_RN_N, result.omega_RN_N.data);
    eigenVectorToCArray(out.domega_RN_N, result.domega_RN_N.data);
    return result;
}
}  // namespace

MrpRotationAlgorithmHandle* MrpRotationAlgorithm_create(const MrpRotationConfig_c* config) {
    // clang-format off
    return reinterpret_cast<MrpRotationAlgorithmHandle*>(new ::MrpRotationAlgorithm(configFromC(*config)));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    // clang-format on
}

void MrpRotationAlgorithm_destroy(MrpRotationAlgorithmHandle* self) {
    // clang-format off
    delete reinterpret_cast<::MrpRotationAlgorithm*>(self);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    // clang-format on
}

void MrpRotationAlgorithm_setConfig(MrpRotationAlgorithmHandle* self, const MrpRotationConfig_c* config) {
    // clang-format off
    reinterpret_cast<::MrpRotationAlgorithm*>(self)->setConfig(configFromC(*config));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

void MrpRotationAlgorithm_reset(MrpRotationAlgorithmHandle* self) {
    // clang-format off
    reinterpret_cast<::MrpRotationAlgorithm*>(self)->reset();  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

MrpRotationOutput_c MrpRotationAlgorithm_update(MrpRotationAlgorithmHandle* self,
                                                const MrpRotationAttRefInputs_c* attRef) {
    // clang-format off
    const MrpRotationOutput out = reinterpret_cast<::MrpRotationAlgorithm*>(self)->update(attRefFromC(*attRef));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
    return outputToC(out);
}
