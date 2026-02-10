/* MIT License
 *
 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "mimuMajorityVoteAlgorithm_c.h"
#include "mimuMajorityVoteAlgorithm.h"

#include <Eigen/Core>
#include <array>

uint32_t MimuMajorityVoteAlgorithm_getMaxImuVehCount(void) { return MAX_IMU_VEH_COUNT; }

MimuMajorityVoteAlgorithm* MimuMajorityVoteAlgorithm_create(void) {
    return reinterpret_cast<MimuMajorityVoteAlgorithm*>(new ::MimuMajorityVoteAlgorithm());
}

void MimuMajorityVoteAlgorithm_destroy(MimuMajorityVoteAlgorithm* self) {
    delete reinterpret_cast<::MimuMajorityVoteAlgorithm*>(self);
}

MimuMajorityVoteOutput_c MimuMajorityVoteAlgorithm_update(MimuMajorityVoteAlgorithm* self,
                                                            const MimuInput_c* imuInputs,
                                                            uint32_t numberOfImus) {
    std::array<MimuInput, MAX_IMU_VEH_COUNT> imuArray{};
    for (uint32_t i = 0; i < numberOfImus; ++i) {
        imuArray[i].angVelBody << imuInputs[i].angVelBody.data[0], imuInputs[i].angVelBody.data[1],
            imuInputs[i].angVelBody.data[2];
    }

    MimuMajorityVoteOutput result =
        reinterpret_cast<::MimuMajorityVoteAlgorithm*>(self)->update(imuArray, numberOfImus);

    MimuMajorityVoteOutput_c out;
    out.avgAngVelBody.data[0] = result.avgAngVelBody[0];
    out.avgAngVelBody.data[1] = result.avgAngVelBody[1];
    out.avgAngVelBody.data[2] = result.avgAngVelBody[2];
    out.faultDetected = result.faultDetected;
    out.mimuIndexFaulted = result.mimuIndexFaulted;
    return out;
}

void MimuMajorityVoteAlgorithm_setOmegaThreshold(MimuMajorityVoteAlgorithm* self, float value) {
    reinterpret_cast<::MimuMajorityVoteAlgorithm*>(self)->setOmegaThreshold(value);
}

float MimuMajorityVoteAlgorithm_getOmegaThreshold(const MimuMajorityVoteAlgorithm* self) {
    return reinterpret_cast<const ::MimuMajorityVoteAlgorithm*>(self)->getOmegaThreshold();
}
