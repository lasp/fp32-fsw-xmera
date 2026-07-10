#include "mimuMajorityVoteAlgorithm_c.h"
#include "mimuMajorityVoteAlgorithm.h"
#include "mimuMajorityVoteTypes.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

uint32_t MimuMajorityVoteAlgorithm_getMimuCount(void) { return MIMU_COUNT_C; }


MimuMajorityVoteAlgorithmHandle* MimuMajorityVoteAlgorithm_create(void) {
    return fsw::createHandle<::MimuMajorityVoteAlgorithm, MimuMajorityVoteAlgorithmHandle>(
        MimuMajorityVoteConfig::create(omegaThreshold, faultPersistenceLimit));
}

bool MimuMajorityVoteAlgorithm_validateConfig(float omegaThreshold, uint32_t faultPersistenceLimit) {
    try {
        (void) MimuMajorityVoteConfig::create(omegaThreshold, faultPersistenceLimit);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

void MimuMajorityVoteAlgorithm_destroy(MimuMajorityVoteAlgorithmHandle* self) {
    fsw::deleteHandle<::MimuMajorityVoteAlgorithm>(self);
}

void MimuMajorityVoteAlgorithm_setConfig(MimuMajorityVoteAlgorithmHandle* self,
                                         float omegaThreshold,
                                         uint32_t faultPersistenceLimit) {
    fsw::fromHandle<::MimuMajorityVoteAlgorithm>(self)->setConfig(omegaThreshold, faultPersistenceLimit);
}

void MimuMajorityVoteAlgorithm_reInitialize(MimuMajorityVoteAlgorithmHandle* self) {
    fsw::fromHandle<::MimuMajorityVoteAlgorithm>(self)->reInitialize();
}

MimuMajorityVoteOutput_c MimuMajorityVoteAlgorithm_update(MimuMajorityVoteAlgorithmHandle* self,
                                                          const Vector3fArray3_c* imuOmegas_BN_B) {
    auto* alg = fsw::fromHandle<::MimuMajorityVoteAlgorithm>(self);

    // Convert POD Vector3f_c inputs to C++ Eigen array:
    std::array<Eigen::Vector3f, MIMU_COUNT_C> inputs{};
    for (uint32_t i = 0; i < MIMU_COUNT_C; ++i) {
        inputs[i] << imuOmegas_BN_B->vec[i].data[0], imuOmegas_BN_B->vec[i].data[1], imuOmegas_BN_B->vec[i].data[2];
    }

    // Call the C++ algorithm:
    MimuMajorityVoteOutput cppOutput = alg->update(inputs);

    // Convert C++ output to POD:
    MimuMajorityVoteOutput_c out{};
    out.avgOmega_BN_B.data[0] = cppOutput.avgOmega_BN_B[0];
    out.avgOmega_BN_B.data[1] = cppOutput.avgOmega_BN_B[1];
    out.avgOmega_BN_B.data[2] = cppOutput.avgOmega_BN_B[2];
    out.faultDetected = cppOutput.faultDetected ? 1U : 0U;

    // Reduce per-IMU validity to a single faulted index (-1 if no fault):
    out.mimuIndexFaulted = -1;
    if (cppOutput.faultDetected) {
        for (uint32_t i = 0; i < MIMU_COUNT_C; ++i) {
            if (!cppOutput.validImus[i]) {
                out.mimuIndexFaulted = static_cast<int32_t>(i);
                break;
            }
        }
    }

    return out;
}
