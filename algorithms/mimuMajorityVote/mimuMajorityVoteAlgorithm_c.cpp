#include "mimuMajorityVoteAlgorithm_c.h"
#include "mimuMajorityVoteAlgorithm.h"
#include "mimuMajorityVoteTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

namespace {
std::array<Eigen::Vector3f, MIMU_COUNT_C> toEigenArray(const Vector3fArray3_c& in) {
    std::array<Eigen::Vector3f, MIMU_COUNT_C> out{};
    for (uint32_t i = 0; i < MIMU_COUNT_C; ++i) {
        out[i] = cArrayToEigenVector3<float>(in.vec[i].data);
    }
    return out;
}

// Marshal one C++ vote result into its POD mirror (full per-IMU fidelity).
MimuVoteResult_c toResultC(const MimuVoteResult& result) {
    MimuVoteResult_c out{};
    eigenVectorToCArray(result.average, out.average.data);
    out.faultDetected = result.faultDetected ? 1U : 0U;
    for (uint32_t i = 0; i < MIMU_COUNT_C; ++i) {
        out.imuDifferenceMag[i] = result.imuDifferenceMag.at(i);
        out.imuValid[i] = result.imuValid.at(i) ? 1U : 0U;
    }
    return out;
}
}  // namespace

uint32_t MimuMajorityVoteAlgorithm_getMimuCount(void) { return MIMU_COUNT_C; }

bool MimuMajorityVoteAlgorithm_validateConfig(float omegaThreshold,
                                              uint32_t gyroFaultPersistenceLimit,
                                              float accelThreshold,
                                              uint32_t accelFaultPersistenceLimit) {
    try {
        (void)MimuMajorityVoteConfig::create(
            omegaThreshold, gyroFaultPersistenceLimit, accelThreshold, accelFaultPersistenceLimit);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

MimuMajorityVoteAlgorithmHandle* MimuMajorityVoteAlgorithm_create(float omegaThreshold,
                                                                  uint32_t gyroFaultPersistenceLimit,
                                                                  float accelThreshold,
                                                                  uint32_t accelFaultPersistenceLimit) {
    return reinterpret_cast<MimuMajorityVoteAlgorithmHandle*>(
        new ::MimuMajorityVoteAlgorithm(MimuMajorityVoteConfig::create(
            omegaThreshold, gyroFaultPersistenceLimit, accelThreshold, accelFaultPersistenceLimit)));
}

void MimuMajorityVoteAlgorithm_destroy(MimuMajorityVoteAlgorithmHandle* self) {
    fsw::deleteHandle<::MimuMajorityVoteAlgorithm>(self);
}

void MimuMajorityVoteAlgorithm_setConfig(MimuMajorityVoteAlgorithmHandle* self,
                                         float omegaThreshold,
                                         uint32_t gyroFaultPersistenceLimit,
                                         float accelThreshold,
                                         uint32_t accelFaultPersistenceLimit) {
    fsw::fromHandle<::MimuMajorityVoteAlgorithm>(self)->setConfig(MimuMajorityVoteConfig::create(
        omegaThreshold, gyroFaultPersistenceLimit, accelThreshold, accelFaultPersistenceLimit));
}

void MimuMajorityVoteAlgorithm_reInitialize(MimuMajorityVoteAlgorithmHandle* self) {
    fsw::fromHandle<::MimuMajorityVoteAlgorithm>(self)->reInitialize();
}

MimuMajorityVoteOutput_c MimuMajorityVoteAlgorithm_update(MimuMajorityVoteAlgorithmHandle* self,
                                                          const Vector3fArray3_c* imuOmegas_BN_B,
                                                          const Vector3fArray3_c* imuAccels_B) {
    auto* alg = fsw::fromHandle<::MimuMajorityVoteAlgorithm>(self);

    // Call the C++ algorithm with both quantities:
    MimuMajorityVoteOutput cppOutput = alg->update(toEigenArray(*imuOmegas_BN_B), toEigenArray(*imuAccels_B));

    // Convert C++ output to POD, preserving each vote's full per-IMU fidelity:
    MimuMajorityVoteOutput_c out{};
    out.gyro = toResultC(cppOutput.gyro);
    out.accel = toResultC(cppOutput.accel);
    return out;
}
