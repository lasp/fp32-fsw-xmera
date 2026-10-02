#include "sunAvoidanceAlgorithm_c.h"
#include "sunAvoidanceAlgorithm.h"
#include "sunAvoidanceTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

namespace {
SunAvoidanceConfig makeConfig(const Vector3f_c& sensitiveHat_B, float slewRate) {
    return SunAvoidanceConfig::create(cArrayToEigenVector3<float>(sensitiveHat_B.data), slewRate);
}

SunAvoidanceAttRef attRefFromC(const SunAvoidanceAttRef_c& c) {
    return SunAvoidanceAttRef{
        cArrayToEigenVector3<float>(c.sigma_RN.data),
        cArrayToEigenVector3<float>(c.omega_RN_N.data),
        cArrayToEigenVector3<float>(c.domega_RN_N.data),
    };
}

SunAvoidanceAttRef_c attRefToC(const SunAvoidanceAttRef& attRef) {
    SunAvoidanceAttRef_c result{};
    eigenVectorToCArray(attRef.sigma_RN, result.sigma_RN.data);
    eigenVectorToCArray(attRef.omega_RN_N, result.omega_RN_N.data);
    eigenVectorToCArray(attRef.domega_RN_N, result.domega_RN_N.data);
    return result;
}
}  // namespace

bool SunAvoidanceAlgorithm_validateConfig(const Vector3f_c* sensitiveHat_B, float slewRate) {
    try {
        (void)makeConfig(*sensitiveHat_B, slewRate);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

SunAvoidanceAlgorithmHandle* SunAvoidanceAlgorithm_create(const Vector3f_c* sensitiveHat_B, float slewRate) {
    return fsw::createHandle<::SunAvoidanceAlgorithm, SunAvoidanceAlgorithmHandle>(
        makeConfig(*sensitiveHat_B, slewRate));
}

void SunAvoidanceAlgorithm_destroy(SunAvoidanceAlgorithmHandle* self) {
    fsw::deleteHandle<::SunAvoidanceAlgorithm>(self);
}

void SunAvoidanceAlgorithm_setConfig(SunAvoidanceAlgorithmHandle* self,
                                     const Vector3f_c* sensitiveHat_B,
                                     float slewRate) {
    fsw::fromHandle<::SunAvoidanceAlgorithm>(self)->setConfig(makeConfig(*sensitiveHat_B, slewRate));
}

void SunAvoidanceAlgorithm_reInitialize(SunAvoidanceAlgorithmHandle* self) {
    fsw::fromHandle<::SunAvoidanceAlgorithm>(self)->reInitialize();
}

SunAvoidanceAttRef_c SunAvoidanceAlgorithm_update(SunAvoidanceAlgorithmHandle* self,
                                                  const Vector3f_c* sigma_BN,
                                                  const SunAvoidanceAttRef_c* ref,
                                                  const Vector3f_c* sHat_B,
                                                  uint64_t callTime) {
    const SunAvoidanceAttRef out =
        fsw::fromHandle<::SunAvoidanceAlgorithm>(self)->update(cArrayToEigenVector3<float>(sigma_BN->data),
                                                               attRefFromC(*ref),
                                                               cArrayToEigenVector3<float>(sHat_B->data),
                                                               callTime);
    return attRefToC(out);
}
