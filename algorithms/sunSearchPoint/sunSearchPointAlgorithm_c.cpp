#include "sunSearchPointAlgorithm_c.h"
#include "sunSearchPointAlgorithm.h"
#include "sunSearchPointTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

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
                                        c.observationThreshold,
                                        c.controlPeriod);
}
}  // namespace

uint32_t SunSearchPointAlgorithm_getNumRotations(void) { return SUN_SEARCH_POINT_NUM_ROTATIONS; }

SunSearchPointAlgorithmHandle* SunSearchPointAlgorithm_create(const SunSearchPointConfig_c* config) {
    return fsw::createHandle<::SunSearchPointAlgorithm, SunSearchPointAlgorithmHandle>(configFromC(*config));
}

void SunSearchPointAlgorithm_destroy(SunSearchPointAlgorithmHandle* self) {
    fsw::deleteHandle<::SunSearchPointAlgorithm>(self);
}

void SunSearchPointAlgorithm_setConfig(SunSearchPointAlgorithmHandle* self, const SunSearchPointConfig_c* config) {
    fsw::fromHandle<::SunSearchPointAlgorithm>(self)->setConfig(configFromC(*config));
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
