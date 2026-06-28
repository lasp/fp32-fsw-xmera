#include "sunSearchPointAlgorithm_c.h"
#include "sunSearchPointAlgorithm.h"
#include "sunSearchPointTypes.h"
#include "utilities/fsw/eigenSupport.h"

#include <Eigen/Core>
#include <array>

namespace {
SunSearchPointConfig configFromC(const SunSearchPointConfig_c& c) {
    std::array<RotationProperties, kNumRotations> rotations{};
    for (uint32_t i = 0U; i < kNumRotations; ++i) {
        rotations[i].rotationDuration = c.rotations[i].rotationDuration;
        rotations[i].rotationRate = c.rotations[i].rotationRate;
        rotations[i].rotationAxis = static_cast<RotationAxis>(c.rotations[i].rotationAxis);
    }
    return SunSearchPointConfig::create(rotations,
                                        cArrayToEigenVector3<float>(c.sHatBdyCmd.data),
                                        c.sunAxisSpinRate,
                                        cArrayToEigenVector3<float>(c.omega_RN_B.data),
                                        c.observationThreshold);
}
}  // namespace

uint32_t SunSearchPointAlgorithm_getNumRotations(void) { return SUN_SEARCH_POINT_NUM_ROTATIONS; }

SunSearchPointAlgorithmHandle* SunSearchPointAlgorithm_create(const SunSearchPointConfig_c* config) {
    return reinterpret_cast<SunSearchPointAlgorithmHandle*>(new ::SunSearchPointAlgorithm(configFromC(*config)));
}

void SunSearchPointAlgorithm_destroy(SunSearchPointAlgorithmHandle* self) {
    delete reinterpret_cast<::SunSearchPointAlgorithm*>(self);
}

void SunSearchPointAlgorithm_setConfig(SunSearchPointAlgorithmHandle* self, const SunSearchPointConfig_c* config) {
    reinterpret_cast<::SunSearchPointAlgorithm*>(self)->setConfig(configFromC(*config));
}

void SunSearchPointAlgorithm_reInitialize(SunSearchPointAlgorithmHandle* self) {
    reinterpret_cast<::SunSearchPointAlgorithm*>(self)->reInitialize();
}

SunSearchPointOutput_c SunSearchPointAlgorithm_update(SunSearchPointAlgorithmHandle* self,
                                                      const uint64_t callTime,
                                                      const Vector3f_c rHat_SB_B,
                                                      const Vector3f_c omega_BN_B,
                                                      const int numCssViewingSun) {
    const SunSearchPointOutput out =
        reinterpret_cast<::SunSearchPointAlgorithm*>(self)->update(callTime,
                                                                   cArrayToEigenVector3<float>(rHat_SB_B.data),
                                                                   cArrayToEigenVector3<float>(omega_BN_B.data),
                                                                   numCssViewingSun);

    SunSearchPointOutput_c result{};
    eigenVectorToCArray(out.sigma_BR, result.sigma_BR.data);
    eigenVectorToCArray(out.omega_BR_B, result.omega_BR_B.data);
    eigenVectorToCArray(out.omega_RN_B, result.omega_RN_B.data);
    result.faultDetected = out.faultDetected;
    return result;
}
