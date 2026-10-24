#include "sunSearchPointAlgorithm_c.h"
#include "sunSearchPointAlgorithm.h"
#include "sunSearchPointTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>
#include <array>

static_assert(SUN_SEARCH_POINT_NUM_ROTATIONS == kNumSunSearchRotations,
              "C-shim rotation count must match the algorithm's kNumSunSearchRotations");

namespace {

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
SunSearchPointConfig configFromC(const RotationPropertiesArray4_c& rotationsIn,
                                 const Vector3f_c sHatBdyCmd,
                                 const float sunAxisSpinRate,
                                 const Vector3f_c omega_RN_B,
                                 const uint32_t observationThreshold,
                                 const float controlPeriod,
                                 const float deadbandAngle,
                                 const float deadbandRate) {
    std::array<RotationProperties, kNumSunSearchRotations> rotations{};
    for (uint32_t i = 0U; i < kNumSunSearchRotations; ++i) {
        rotations[i].rotationDuration = rotationsIn.rotations[i].rotationDuration;
        rotations[i].rotationRate = rotationsIn.rotations[i].rotationRate;
        rotations[i].rotationAxis = static_cast<RotationAxis>(rotationsIn.rotations[i].rotationAxis);
    }
    return SunSearchPointConfig::create(rotations,
                                        cArrayToEigenVector3<float>(sHatBdyCmd.data),
                                        sunAxisSpinRate,
                                        cArrayToEigenVector3<float>(omega_RN_B.data),
                                        observationThreshold,
                                        controlPeriod,
                                        deadbandAngle,
                                        deadbandRate);
}
}  // namespace

uint32_t SunSearchPointAlgorithm_getNumRotations(void) { return kNumSunSearchRotations; }

uint32_t SunSearchPointAlgorithm_getMaxObservationThreshold(void) { return kMaxNumCssSensors; }

bool SunSearchPointAlgorithm_validateConfig(const RotationPropertiesArray4_c* rotations,
                                            const Vector3f_c sHatBdyCmd,
                                            const float sunAxisSpinRate,
                                            const Vector3f_c omega_RN_B,
                                            const uint32_t observationThreshold,
                                            const float controlPeriod,
                                            const float deadbandAngle,
                                            const float deadbandRate) {
    // Build the config through the real create path; success means valid, a throw means invalid.
    // Reusing configFromC keeps the validity rules from drifting away from create/setConfig.
    try {
        (void)configFromC(*rotations,
                          sHatBdyCmd,
                          sunAxisSpinRate,
                          omega_RN_B,
                          observationThreshold,
                          controlPeriod,
                          deadbandAngle,
                          deadbandRate);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

SunSearchPointAlgorithmHandle* SunSearchPointAlgorithm_create(const RotationPropertiesArray4_c* rotations,
                                                              const Vector3f_c sHatBdyCmd,
                                                              const float sunAxisSpinRate,
                                                              const Vector3f_c omega_RN_B,
                                                              const uint32_t observationThreshold,
                                                              const float controlPeriod,
                                                              const float deadbandAngle,
                                                              const float deadbandRate) {
    const SunSearchPointConfig cfg = configFromC(*rotations,
                                                 sHatBdyCmd,
                                                 sunAxisSpinRate,
                                                 omega_RN_B,
                                                 observationThreshold,
                                                 controlPeriod,
                                                 deadbandAngle,
                                                 deadbandRate);
    return fsw::createHandle<::SunSearchPointAlgorithm, SunSearchPointAlgorithmHandle>(cfg);
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
                                       const float controlPeriod,
                                       const float deadbandAngle,
                                       const float deadbandRate) {
    const SunSearchPointConfig cfg = configFromC(*rotations,
                                                 sHatBdyCmd,
                                                 sunAxisSpinRate,
                                                 omega_RN_B,
                                                 observationThreshold,
                                                 controlPeriod,
                                                 deadbandAngle,
                                                 deadbandRate);
    fsw::fromHandle<::SunSearchPointAlgorithm>(self)->setConfig(cfg);
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
    result.sunNotFound = out.sunNotFound;
    return result;
}
