#include "sunSearchAlgorithm_c.h"
#include "architecture/utilities/eigenSupport.h"
#include "sunSearchAlgorithm.h"
#include "sunSearchTypes.h"

#include <Eigen/Core>
#include <array>

namespace {
SunSearchConfig configFromC(const SunSearchConfig_c& c) {
    std::array<RotationProperties, kNumRotations> rotations{};
    for (uint32_t i = 0U; i < kNumRotations; ++i) {
        rotations[i].rotationDuration = c.rotations[i].rotationDuration;
        rotations[i].rotationRate = c.rotations[i].rotationRate;
        rotations[i].rotationAxis = static_cast<RotationAxis>(c.rotations[i].rotationAxis);
    }
    return SunSearchConfig::create(rotations);
}
}  // namespace

uint32_t SunSearchAlgorithm_getNumRotations(void) { return SUN_SEARCH_NUM_ROTATIONS; }

SunSearchAlgorithmHandle* SunSearchAlgorithm_create(const SunSearchConfig_c* config) {
    return reinterpret_cast<SunSearchAlgorithmHandle*>(new ::SunSearchAlgorithm(configFromC(*config)));
}

void SunSearchAlgorithm_destroy(SunSearchAlgorithmHandle* self) {
    delete reinterpret_cast<::SunSearchAlgorithm*>(self);
}

void SunSearchAlgorithm_setConfig(SunSearchAlgorithmHandle* self, const SunSearchConfig_c* config) {
    reinterpret_cast<::SunSearchAlgorithm*>(self)->setConfig(configFromC(*config));
}

SunSearchOutput_c SunSearchAlgorithm_update(SunSearchAlgorithmHandle* self,
                                            const uint64_t callTime,
                                            const Vector3f_c omega_BN_B) {
    const SunSearchOutput out =
        reinterpret_cast<::SunSearchAlgorithm*>(self)->update(callTime, cArrayToEigenVector3<float>(omega_BN_B.data));

    SunSearchOutput_c result{};
    eigenVectorToCArray(out.omega_RN_B, result.omega_RN_B.data);
    eigenVectorToCArray(out.omega_BR_B, result.omega_BR_B.data);
    return result;
}
