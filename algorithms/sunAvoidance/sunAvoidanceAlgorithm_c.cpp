#include "sunAvoidanceAlgorithm_c.h"
#include "sunAvoidanceAlgorithm.h"
#include "sunAvoidanceTypes.h"
#include "utilities/fsw/eigenSupport.h"

#include <Eigen/Core>

namespace {
SunAvoidanceConfig configFromC(const SunAvoidanceConfig_c& c) {
    return SunAvoidanceConfig::create(
        cArrayToEigenVector3<float>(c.sensitiveHat_B.data), c.angleRate, c.computeAngleStart);
}

SunAvoidanceAttRefInputs refFromC(const SunAvoidanceAttRefInputs_c& c) {
    return SunAvoidanceAttRefInputs{
        cArrayToEigenVector3<float>(c.sigma_RN.data),
        cArrayToEigenVector3<float>(c.omega_RN_N.data),
        cArrayToEigenVector3<float>(c.domega_RN_N.data),
    };
}

SunAvoidanceOutput_c outputToC(const SunAvoidanceOutput& out) {
    SunAvoidanceOutput_c result{};
    eigenVectorToCArray(out.sigma_RN, result.sigma_RN.data);
    eigenVectorToCArray(out.omega_RN_N, result.omega_RN_N.data);
    eigenVectorToCArray(out.domega_RN_N, result.domega_RN_N.data);
    return result;
}
}  // namespace

SunAvoidanceAlgorithmHandle* SunAvoidanceAlgorithm_create(const SunAvoidanceConfig_c* config) {
    // clang-format off
    return reinterpret_cast<SunAvoidanceAlgorithmHandle*>(new ::SunAvoidanceAlgorithm(configFromC(*config)));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    // clang-format on
}

void SunAvoidanceAlgorithm_destroy(SunAvoidanceAlgorithmHandle* self) {
    // clang-format off
    delete reinterpret_cast<::SunAvoidanceAlgorithm*>(self);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    // clang-format on
}

void SunAvoidanceAlgorithm_setConfig(SunAvoidanceAlgorithmHandle* self, const SunAvoidanceConfig_c* config) {
    // clang-format off
    reinterpret_cast<::SunAvoidanceAlgorithm*>(self)->setConfig(configFromC(*config));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

void SunAvoidanceAlgorithm_reInitialize(SunAvoidanceAlgorithmHandle* self) {
    // clang-format off
    reinterpret_cast<::SunAvoidanceAlgorithm*>(self)->reInitialize();  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

SunAvoidanceOutput_c SunAvoidanceAlgorithm_update(SunAvoidanceAlgorithmHandle* self,
                                                  const Vector3f_c* sigma_BN,
                                                  const SunAvoidanceAttRefInputs_c* ref,
                                                  const Vector3d_c* r_BN_N,
                                                  const Vector3d_c* r_SN_N,
                                                  uint64_t callTime) {
    // clang-format off
    const SunAvoidanceOutput out = reinterpret_cast<::SunAvoidanceAlgorithm*>(self)->update(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        cArrayToEigenVector3<float>(sigma_BN->data), refFromC(*ref), cArrayToEigenVector3<double>(r_BN_N->data),
        cArrayToEigenVector3<double>(r_SN_N->data), callTime);
    // clang-format on
    return outputToC(out);
}
