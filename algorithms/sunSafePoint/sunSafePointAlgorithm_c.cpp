#include "sunSafePointAlgorithm_c.h"
#include "sunSafePointAlgorithm.h"
#include "sunSafePointTypes.h"
#include "utilities/fsw/eigenSupport.h"

#include <Eigen/Core>
#include <array>

namespace {
SunSafePointConfig configFromC(const SunSafePointConfig_c& c) {
    std::array<RotationProperties, kNumRotations> rotations{};
    for (uint32_t i = 0U; i < kNumRotations; ++i) {
        rotations[i].rotationDuration = c.rotations[i].rotationDuration;
        rotations[i].rotationRate = c.rotations[i].rotationRate;
        rotations[i].rotationAxis = static_cast<RotationAxis>(c.rotations[i].rotationAxis);
    }
    return SunSafePointConfig::create(rotations,
                                      cArrayToEigenVector3<float>(c.sHatBdyCmd.data),
                                      c.sunAxisSpinRate,
                                      cArrayToEigenVector3<float>(c.omega_RN_B.data),
                                      c.observationThreshold);
}
}  // namespace

uint32_t SunSafePointAlgorithm_getNumRotations(void) { return SUN_SAFE_POINT_NUM_ROTATIONS; }

SunSafePointAlgorithmHandle* SunSafePointAlgorithm_create(const SunSafePointConfig_c* config) {
    return reinterpret_cast<SunSafePointAlgorithmHandle*>(new ::SunSafePointAlgorithm(configFromC(*config)));
}

void SunSafePointAlgorithm_destroy(SunSafePointAlgorithmHandle* self) {
    delete reinterpret_cast<::SunSafePointAlgorithm*>(self);
}

void SunSafePointAlgorithm_setConfig(SunSafePointAlgorithmHandle* self, const SunSafePointConfig_c* config) {
    reinterpret_cast<::SunSafePointAlgorithm*>(self)->setConfig(configFromC(*config));
}

void SunSafePointAlgorithm_reInitialize(SunSafePointAlgorithmHandle* self) {
    reinterpret_cast<::SunSafePointAlgorithm*>(self)->reInitialize();
}

SunSafePointOutput_c SunSafePointAlgorithm_update(SunSafePointAlgorithmHandle* self,
                                                  const uint64_t callTime,
                                                  const Vector3f_c rHat_SB_B,
                                                  const Vector3f_c omega_BN_B,
                                                  const int numCssViewingSun) {
    const SunSafePointOutput out =
        reinterpret_cast<::SunSafePointAlgorithm*>(self)->update(callTime,
                                                                 cArrayToEigenVector3<float>(rHat_SB_B.data),
                                                                 cArrayToEigenVector3<float>(omega_BN_B.data),
                                                                 numCssViewingSun);

    SunSafePointOutput_c result{};
    eigenVectorToCArray(out.sigma_BR, result.sigma_BR.data);
    eigenVectorToCArray(out.omega_BR_B, result.omega_BR_B.data);
    eigenVectorToCArray(out.omega_RN_B, result.omega_RN_B.data);
    result.faultDetected = out.faultDetected;
    return result;
}
