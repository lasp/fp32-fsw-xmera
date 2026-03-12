/* MIT License
 *
 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "mimuMajorityVoteAlgorithm_c.h"
#include "mimuMajorityVoteAlgorithm.h"

#include <Eigen/Core>

uint32_t MimuMajorityVoteAlgorithm_getMaxImuVehCount(void) { return MAX_IMU_VEH_COUNT; }

MimuMajorityVoteAlgorithm* MimuMajorityVoteAlgorithm_create(void) {
    return reinterpret_cast<MimuMajorityVoteAlgorithm*>(new ::MimuMajorityVoteAlgorithm());
}

void MimuMajorityVoteAlgorithm_destroy(MimuMajorityVoteAlgorithm* self) {
    delete reinterpret_cast<::MimuMajorityVoteAlgorithm*>(self);
}

MimuMajorityVoteOutput_c MimuMajorityVoteAlgorithm_update(MimuMajorityVoteAlgorithm* self,
                                                          const Vector3f_c* imuInputs,
                                                          uint32_t numberOfImus) {
    auto* alg = reinterpret_cast<::MimuMajorityVoteAlgorithm*>(self);

    // Set the number of IMUs before calling update:
    alg->setNumberOfImus(static_cast<size_t>(numberOfImus));

    // Convert POD Vector3f_c inputs to C++ MimuInput array:
    std::array<MimuInput, MAX_IMU_VEH_COUNT> inputs{};
    for (uint32_t i = 0; i < numberOfImus && i < MAX_IMU_VEH_COUNT; ++i) {
        inputs[i].angVelBody << imuInputs[i].data[0], imuInputs[i].data[1], imuInputs[i].data[2];
    }

    // Call the C++ algorithm:
    MimuMajorityVoteOutput cppOutput = alg->update(inputs);

    // Convert C++ output to POD:
    MimuMajorityVoteOutput_c out{};
    out.avgAngVelBody.data[0] = cppOutput.avgAngVelBody[0];
    out.avgAngVelBody.data[1] = cppOutput.avgAngVelBody[1];
    out.avgAngVelBody.data[2] = cppOutput.avgAngVelBody[2];
    out.faultDetected = cppOutput.faultDetected ? 1U : 0U;

    // Reduce per-IMU validity to a single faulted index (-1 if no fault):
    out.mimuIndexFaulted = -1;
    if (cppOutput.faultDetected) {
        for (uint32_t i = 0; i < numberOfImus && i < MAX_IMU_VEH_COUNT; ++i) {
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
