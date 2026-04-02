#include "mimuMajorityVoteAlgorithm_c.h"
#include "mimuMajorityVoteAlgorithm.h"

#include <Eigen/Core>

uint32_t MimuMajorityVoteAlgorithm_getMimuCount(void) { return MIMU_COUNT_C; }

MimuMajorityVoteAlgorithm* MimuMajorityVoteAlgorithm_create(void) {
    return reinterpret_cast<MimuMajorityVoteAlgorithm*>(new ::MimuMajorityVoteAlgorithm());
}

void MimuMajorityVoteAlgorithm_destroy(MimuMajorityVoteAlgorithm* self) {
    delete reinterpret_cast<::MimuMajorityVoteAlgorithm*>(self);
}

void MimuMajorityVoteAlgorithm_reset(MimuMajorityVoteAlgorithm* self) {
    reinterpret_cast<::MimuMajorityVoteAlgorithm*>(self)->reset();
}

MimuMajorityVoteOutput_c MimuMajorityVoteAlgorithm_update(MimuMajorityVoteAlgorithm* self,
                                                          const Vector3f_c* imuInputs) {
    auto* alg = reinterpret_cast<::MimuMajorityVoteAlgorithm*>(self);

    // Convert POD Vector3f_c inputs to C++ MimuInput array:
    std::array<MimuInput, MIMU_COUNT_C> inputs{};
    for (uint32_t i = 0; i < MIMU_COUNT_C; ++i) {
        inputs[i].omega_BN_B << imuInputs[i].data[0], imuInputs[i].data[1], imuInputs[i].data[2];
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

void MimuMajorityVoteAlgorithm_setOmegaThreshold(MimuMajorityVoteAlgorithm* self, float value) {
    reinterpret_cast<::MimuMajorityVoteAlgorithm*>(self)->setOmegaThreshold(value);
}

float MimuMajorityVoteAlgorithm_getOmegaThreshold(const MimuMajorityVoteAlgorithm* self) {
    return reinterpret_cast<const ::MimuMajorityVoteAlgorithm*>(self)->getOmegaThreshold();
}

void MimuMajorityVoteAlgorithm_setFaultPersistenceLimit(MimuMajorityVoteAlgorithm* self, uint32_t value) {
    reinterpret_cast<::MimuMajorityVoteAlgorithm*>(self)->setFaultPersistenceLimit(value);
}

uint32_t MimuMajorityVoteAlgorithm_getFaultPersistenceLimit(const MimuMajorityVoteAlgorithm* self) {
    return reinterpret_cast<const ::MimuMajorityVoteAlgorithm*>(self)->getFaultPersistenceLimit();
}
