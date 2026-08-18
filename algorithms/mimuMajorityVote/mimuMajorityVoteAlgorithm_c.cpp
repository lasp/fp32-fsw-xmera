#include "mimuMajorityVoteAlgorithm_c.h"
#include "mimuMajorityVoteAlgorithm.h"
#include "mimuMajorityVoteTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

namespace {
MimuMajorityVoteConfig configFromC(const float omegaThreshold,
                                   const uint32_t gyroFaultPersistenceLimit,
                                   const float accelThreshold,
                                   const uint32_t accelFaultPersistenceLimit) {
    return MimuMajorityVoteConfig::create(
        omegaThreshold, gyroFaultPersistenceLimit, accelThreshold, accelFaultPersistenceLimit);
}

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
    out.faultDetected = result.faultDetected;
    for (uint32_t i = 0; i < MIMU_COUNT_C; ++i) {
        out.imuDifferenceMag[i] = result.imuDifferenceMag.at(i);
        out.imuValid[i] = result.imuValid.at(i);
    }
    return out;
}
}  // namespace

uint32_t MimuMajorityVoteAlgorithm_getMimuCount(void) { return MIMU_COUNT_C; }

bool MimuMajorityVoteAlgorithm_validateConfig(const float omegaThreshold,
                                              const uint32_t gyroFaultPersistenceLimit,
                                              const float accelThreshold,
                                              const uint32_t accelFaultPersistenceLimit) {
    // Attempt to build the config through the real create path; success means valid,
    // a throw means invalid. Reusing configFromC keeps validation from drifting.
    try {
        (void)configFromC(omegaThreshold, gyroFaultPersistenceLimit, accelThreshold, accelFaultPersistenceLimit);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

MimuMajorityVoteAlgorithmHandle* MimuMajorityVoteAlgorithm_create(const float omegaThreshold,
                                                                  const uint32_t gyroFaultPersistenceLimit,
                                                                  const float accelThreshold,
                                                                  const uint32_t accelFaultPersistenceLimit) {
    return fsw::createHandle<::MimuMajorityVoteAlgorithm, MimuMajorityVoteAlgorithmHandle>(
        configFromC(omegaThreshold, gyroFaultPersistenceLimit, accelThreshold, accelFaultPersistenceLimit));
}

void MimuMajorityVoteAlgorithm_destroy(MimuMajorityVoteAlgorithmHandle* self) {
    fsw::deleteHandle<::MimuMajorityVoteAlgorithm>(self);
}

void MimuMajorityVoteAlgorithm_setConfig(MimuMajorityVoteAlgorithmHandle* self,
                                         const float omegaThreshold,
                                         const uint32_t gyroFaultPersistenceLimit,
                                         const float accelThreshold,
                                         const uint32_t accelFaultPersistenceLimit) {
    fsw::fromHandle<::MimuMajorityVoteAlgorithm>(self)->setConfig(
        configFromC(omegaThreshold, gyroFaultPersistenceLimit, accelThreshold, accelFaultPersistenceLimit));
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
