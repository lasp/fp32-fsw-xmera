#include "sunTrackErrorAlgorithm_c.h"
#include "sunTrackErrorAlgorithm.h"
#include "sunTrackErrorTypes.h"
#include "utilities/fsw/eigenSupport.h"

#include <Eigen/Core>

namespace {
SunTrackErrorConfig configFromC(const SunTrackErrorConfig_c& c) {
    return SunTrackErrorConfig::create(
        cArrayToEigenVector3<float>(c.sensitiveHat_B.data), c.angleRate, c.computeAngleStart);
}

SunTrackErrorAttRefInputs refFromC(const SunTrackErrorAttRefInputs_c& c) {
    return SunTrackErrorAttRefInputs{
        cArrayToEigenVector3<float>(c.sigma_RN.data),
        cArrayToEigenVector3<float>(c.omega_RN_N.data),
        cArrayToEigenVector3<float>(c.domega_RN_N.data),
    };
}

SunTrackErrorOutput_c outputToC(const SunTrackErrorOutput& out) {
    SunTrackErrorOutput_c result{};
    eigenVectorToCArray(out.sigma_RN, result.sigma_RN.data);
    eigenVectorToCArray(out.omega_RN_N, result.omega_RN_N.data);
    eigenVectorToCArray(out.domega_RN_N, result.domega_RN_N.data);
    return result;
}
}  // namespace

SunTrackErrorAlgorithmHandle* SunTrackErrorAlgorithm_create(const SunTrackErrorConfig_c* config) {
    // clang-format off
    return reinterpret_cast<SunTrackErrorAlgorithmHandle*>(new ::SunTrackErrorAlgorithm(configFromC(*config)));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    // clang-format on
}

void SunTrackErrorAlgorithm_destroy(SunTrackErrorAlgorithmHandle* self) {
    // clang-format off
    delete reinterpret_cast<::SunTrackErrorAlgorithm*>(self);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    // clang-format on
}

void SunTrackErrorAlgorithm_setConfig(SunTrackErrorAlgorithmHandle* self, const SunTrackErrorConfig_c* config) {
    // clang-format off
    reinterpret_cast<::SunTrackErrorAlgorithm*>(self)->setConfig(configFromC(*config));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

void SunTrackErrorAlgorithm_reInitialize(SunTrackErrorAlgorithmHandle* self) {
    // clang-format off
    reinterpret_cast<::SunTrackErrorAlgorithm*>(self)->reInitialize();  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

SunTrackErrorOutput_c SunTrackErrorAlgorithm_update(SunTrackErrorAlgorithmHandle* self,
                                                    const Vector3f_c* sigma_BN,
                                                    const SunTrackErrorAttRefInputs_c* ref,
                                                    const Vector3d_c* r_BN_N,
                                                    const Vector3d_c* r_SN_N,
                                                    uint64_t callTime) {
    // clang-format off
    const SunTrackErrorOutput out = reinterpret_cast<::SunTrackErrorAlgorithm*>(self)->update(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        cArrayToEigenVector3<float>(sigma_BN->data), refFromC(*ref), cArrayToEigenVector3<double>(r_BN_N->data),
        cArrayToEigenVector3<double>(r_SN_N->data), callTime);
    // clang-format on
    return outputToC(out);
}
