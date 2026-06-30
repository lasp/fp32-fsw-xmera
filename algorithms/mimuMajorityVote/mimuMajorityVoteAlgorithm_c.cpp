#include "mimuMajorityVoteAlgorithm_c.h"
#include "mimuMajorityVoteAlgorithm.h"
#include "mimuMajorityVoteTypes.h"

#include <Eigen/Core>

namespace {
MimuMajorityVoteConfig configFromC(const MimuMajorityVoteConfig_c& c) {
    return MimuMajorityVoteConfig::create(
        c.omegaThreshold, c.gyroFaultPersistenceLimit, c.accelThreshold, c.accelFaultPersistenceLimit);
}

std::array<Eigen::Vector3f, MIMU_COUNT_C> toEigenArray(const Vector3fArray3_c& in) {
    std::array<Eigen::Vector3f, MIMU_COUNT_C> out{};
    for (uint32_t i = 0; i < MIMU_COUNT_C; ++i) {
        out[i] << in.vec[i].data[0], in.vec[i].data[1], in.vec[i].data[2];
    }
    return out;
}

// Reduce a per-IMU validity array to a single faulted index (-1 if no fault).
int32_t faultedIndex(bool faultDetected, const std::array<bool, kMimuCount>& imuValid) {
    if (faultDetected) {
        for (uint32_t i = 0; i < MIMU_COUNT_C; ++i) {
            if (!imuValid[i]) {
                return static_cast<int32_t>(i);
            }
        }
    }
    return -1;
}
}  // namespace

uint32_t MimuMajorityVoteAlgorithm_getMimuCount(void) { return MIMU_COUNT_C; }

MimuMajorityVoteAlgorithmHandle* MimuMajorityVoteAlgorithm_create(const MimuMajorityVoteConfig_c* config) {
    return reinterpret_cast<MimuMajorityVoteAlgorithmHandle*>(new ::MimuMajorityVoteAlgorithm(configFromC(*config)));
}

void MimuMajorityVoteAlgorithm_destroy(MimuMajorityVoteAlgorithmHandle* self) {
    delete reinterpret_cast<::MimuMajorityVoteAlgorithm*>(self);
}

void MimuMajorityVoteAlgorithm_setConfig(MimuMajorityVoteAlgorithmHandle* self,
                                         const MimuMajorityVoteConfig_c* config) {
    reinterpret_cast<::MimuMajorityVoteAlgorithm*>(self)->setConfig(configFromC(*config));
}

void MimuMajorityVoteAlgorithm_reInitialize(MimuMajorityVoteAlgorithmHandle* self) {
    reinterpret_cast<::MimuMajorityVoteAlgorithm*>(self)->reInitialize();
}

MimuMajorityVoteOutput_c MimuMajorityVoteAlgorithm_update(MimuMajorityVoteAlgorithmHandle* self,
                                                          const Vector3fArray3_c* imuOmegas_BN_B,
                                                          const Vector3fArray3_c* imuAccels_B) {
    auto* alg = reinterpret_cast<::MimuMajorityVoteAlgorithm*>(self);

    // Call the C++ algorithm with both quantities:
    MimuMajorityVoteOutput cppOutput = alg->update(toEigenArray(*imuOmegas_BN_B), toEigenArray(*imuAccels_B));

    // Convert C++ output to POD:
    MimuMajorityVoteOutput_c out{};
    out.avgOmega_BN_B.data[0] = cppOutput.gyro.average[0];
    out.avgOmega_BN_B.data[1] = cppOutput.gyro.average[1];
    out.avgOmega_BN_B.data[2] = cppOutput.gyro.average[2];
    out.gyroFaultDetected = cppOutput.gyro.faultDetected ? 1U : 0U;
    out.gyroMimuIndexFaulted = faultedIndex(cppOutput.gyro.faultDetected, cppOutput.gyro.imuValid);

    out.avgAccel_B.data[0] = cppOutput.accel.average[0];
    out.avgAccel_B.data[1] = cppOutput.accel.average[1];
    out.avgAccel_B.data[2] = cppOutput.accel.average[2];
    out.accelFaultDetected = cppOutput.accel.faultDetected ? 1U : 0U;
    out.accelMimuIndexFaulted = faultedIndex(cppOutput.accel.faultDetected, cppOutput.accel.imuValid);

    return out;
}
