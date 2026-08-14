#include "sunAvoidanceAlgorithm_c.h"
#include "sunAvoidanceAlgorithm.h"
#include "sunAvoidanceTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

namespace {
SunAvoidanceConfig configFromC(const SunAvoidanceConfig_c& c) {
    return SunAvoidanceConfig::create(cArrayToEigenVector3<float>(c.sensitiveHat_B.data), c.slewRate);
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
    return fsw::createHandle<::SunAvoidanceAlgorithm, SunAvoidanceAlgorithmHandle>(configFromC(*config));
}

void SunAvoidanceAlgorithm_destroy(SunAvoidanceAlgorithmHandle* self) {
    fsw::deleteHandle<::SunAvoidanceAlgorithm>(self);
}

void SunAvoidanceAlgorithm_setConfig(SunAvoidanceAlgorithmHandle* self, const SunAvoidanceConfig_c* config) {
    fsw::fromHandle<::SunAvoidanceAlgorithm>(self)->setConfig(configFromC(*config));
}

void SunAvoidanceAlgorithm_reInitialize(SunAvoidanceAlgorithmHandle* self) {
    fsw::fromHandle<::SunAvoidanceAlgorithm>(self)->reInitialize();
}

SunAvoidanceOutput_c SunAvoidanceAlgorithm_update(SunAvoidanceAlgorithmHandle* self,
                                                  const Vector3f_c* sigma_BN,
                                                  const SunAvoidanceAttRefInputs_c* ref,
                                                  const Vector3d_c* r_BN_N,
                                                  const Vector3d_c* r_SN_N,
                                                  uint64_t callTime) {
    const SunAvoidanceOutput out =
        fsw::fromHandle<::SunAvoidanceAlgorithm>(self)->update(cArrayToEigenVector3<float>(sigma_BN->data),
                                                               refFromC(*ref),
                                                               cArrayToEigenVector3<double>(r_BN_N->data),
                                                               cArrayToEigenVector3<double>(r_SN_N->data),
                                                               callTime);
    return outputToC(out);
}
