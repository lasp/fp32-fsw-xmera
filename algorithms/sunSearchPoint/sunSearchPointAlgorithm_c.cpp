#include "sunSearchPointAlgorithm_c.h"
#include "sunSearchPointAlgorithm.h"
#include "sunSearchPointTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>
#include <array>

static_assert(SUN_SEARCH_POINT_NUM_ROTATIONS == kNumRotations,
              "C-shim rotation count must match the algorithm's kNumRotations");

namespace {
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
SunSearchPointConfig configFromC(const RotationPropertiesArray4_c& rotationsIn,
                                 const Vector3f_c sHatBdyCmd,
                                 const float sunAxisSpinRate,
                                 const Vector3f_c omega_RN_B,
                                 const uint32_t observationThreshold,
                                 const float controlPeriod) {
    std::array<RotationProperties, kNumRotations> rotations{};
    for (uint32_t i = 0U; i < kNumRotations; ++i) {
        rotations[i].rotationDuration = rotationsIn.rotations[i].rotationDuration;
        rotations[i].rotationRate = rotationsIn.rotations[i].rotationRate;
        rotations[i].rotationAxis = static_cast<RotationAxis>(rotationsIn.rotations[i].rotationAxis);
    }
    return SunSearchPointConfig::create(rotations,
                                        cArrayToEigenVector3<float>(sHatBdyCmd.data),
                                        sunAxisSpinRate,
                                        cArrayToEigenVector3<float>(omega_RN_B.data),
                                        observationThreshold,
                                        controlPeriod);
}
}  // namespace

uint32_t SunSearchPointAlgorithm_getNumRotations(void) { return SUN_SEARCH_POINT_NUM_ROTATIONS; }

SunSearchPointAlgorithmHandle* SunSearchPointAlgorithm_create(const RotationPropertiesArray4_c* rotations,
                                                              const Vector3f_c sHatBdyCmd,
                                                              const float sunAxisSpinRate,
                                                              const Vector3f_c omega_RN_B,
                                                              const uint32_t observationThreshold,
                                                              const float controlPeriod) {
    return fsw::createHandle<::SunSearchPointAlgorithm, SunSearchPointAlgorithmHandle>(configFromC(
        *rotations, sHatBdyCmd, sunAxisSpinRate, omega_RN_B, observationThreshold, controlPeriod));
}

void SunSearchPointAlgorithm_destroy(SunSearchPointAlgorithmHandle* self) {
    fsw::deleteHandle<::SunSearchPointAlgorithm>(self);
}

void SunSearchPointAlgorithm_setConfig(SunSearchPointAlgorithmHandle* self,
                                       const RotationPropertiesArray4_c* rotations,
                                       const Vector3f_c sHatBdyCmd,
                                       const float sunAxisSpinRate,
                                       const Vector3f_c omega_RN_B,
                                       const uint32_t observationThreshold,
                                       const float controlPeriod) {
    fsw::fromHandle<::SunSearchPointAlgorithm>(self)->setConfig(configFromC(
        *rotations, sHatBdyCmd, sunAxisSpinRate, omega_RN_B, observationThreshold, controlPeriod));
}

void SunSearchPointAlgorithm_reInitialize(SunSearchPointAlgorithmHandle* self) {
    fsw::fromHandle<::SunSearchPointAlgorithm>(self)->reInitialize();
}

SunSearchPointOutput_c SunSearchPointAlgorithm_update(SunSearchPointAlgorithmHandle* self,
                                                      const Vector3f_c rHat_SB_B,
                                                      const Vector3f_c omega_BN_B,
                                                      const uint32_t numCssViewingSun) {
    const SunSearchPointOutput out = fsw::fromHandle<::SunSearchPointAlgorithm>(self)->update(
        cArrayToEigenVector3<float>(rHat_SB_B.data), cArrayToEigenVector3<float>(omega_BN_B.data), numCssViewingSun);

    SunSearchPointOutput_c result{};
    eigenVectorToCArray(out.sigma_BR, result.sigma_BR.data);
    eigenVectorToCArray(out.omega_BR_B, result.omega_BR_B.data);
    eigenVectorToCArray(out.omega_RN_B, result.omega_RN_B.data);
    result.faultDetected = out.faultDetected;
    return result;
}
