#ifndef F32XMERA_MIMU_MAJORITY_VOTE_ALGORITHM_C_H
#define F32XMERA_MIMU_MAJORITY_VOTE_ALGORITHM_C_H

#include "mimuMajorityVoteTypes.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ MimuMajorityVoteAlgorithm instance.
 */
typedef struct MimuMajorityVoteAlgorithmHandle MimuMajorityVoteAlgorithmHandle;

/**
 * @brief Get the kMimuCount constant for Ada validation.
 * @return The IMU count (MIMU_COUNT_C).
 */
uint32_t MimuMajorityVoteAlgorithm_getMimuCount(void);

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param omegaThreshold             [rad/s] gyro threshold; must be finite and > 0.
 * @param gyroFaultPersistenceLimit  [-] consecutive gyro faults to trigger; must be > 0.
 * @param accelThreshold             [m/s^2] accel threshold; must be finite and > 0.
 * @param accelFaultPersistenceLimit [-] consecutive accel faults to trigger; must be > 0.
 * @return true when the configuration is valid. Never throws, so it can guard the
 *         throwing create/setConfig from an invalid configuration.
 */
bool MimuMajorityVoteAlgorithm_validateConfig(float omegaThreshold,
                                              uint32_t gyroFaultPersistenceLimit,
                                              float accelThreshold,
                                              uint32_t accelFaultPersistenceLimit);

/**
 * @brief Construct a new MimuMajorityVoteAlgorithm instance from the supplied configuration.
 * @param omegaThreshold             [rad/s] gyro threshold; must be finite and > 0.
 * @param gyroFaultPersistenceLimit  [-] consecutive gyro faults to trigger; must be > 0.
 * @param accelThreshold             [m/s^2] accel threshold; must be finite and > 0.
 * @param accelFaultPersistenceLimit [-] consecutive accel faults to trigger; must be > 0.
 * @return Pointer to a new MimuMajorityVoteAlgorithm (must be destroyed).
 * Validate the values with validateConfig first; invalid input throws.
 */
MimuMajorityVoteAlgorithmHandle* MimuMajorityVoteAlgorithm_create(float omegaThreshold,
                                                                  uint32_t gyroFaultPersistenceLimit,
                                                                  float accelThreshold,
                                                                  uint32_t accelFaultPersistenceLimit);

/**
 * @brief Destroy a previously created MimuMajorityVoteAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void MimuMajorityVoteAlgorithm_destroy(MimuMajorityVoteAlgorithmHandle* self);

/**
 * @brief Install the configuration on an existing instance (parameters only; call _reInitialize to
 *        reset the persistence counters).
 * @param self                       Pointer to the instance.
 * @param omegaThreshold             [rad/s] gyro threshold; must be finite and > 0.
 * @param gyroFaultPersistenceLimit  [-] consecutive gyro faults to trigger; must be > 0.
 * @param accelThreshold             [m/s^2] accel threshold; must be finite and > 0.
 * @param accelFaultPersistenceLimit [-] consecutive accel faults to trigger; must be > 0.
 * Validate the values with validateConfig first; invalid input throws.
 */
void MimuMajorityVoteAlgorithm_setConfig(MimuMajorityVoteAlgorithmHandle* self,
                                         float omegaThreshold,
                                         uint32_t gyroFaultPersistenceLimit,
                                         float accelThreshold,
                                         uint32_t accelFaultPersistenceLimit);

/**
 * @brief Reset fault persistence counters to zero.
 * @param self Pointer to the instance.
 */
void MimuMajorityVoteAlgorithm_reInitialize(MimuMajorityVoteAlgorithmHandle* self);

/**
 * @brief Run the majority vote update step.
 * @param self           Pointer to the instance.
 * @param imuOmegas_BN_B IMU angular velocity 3-vectors.
 * @param imuAccels_B    IMU apparent acceleration 3-vectors.
 * @return MimuMajorityVoteOutput_c  The computed majority vote output (gyro + accel).
 */
MimuMajorityVoteOutput_c MimuMajorityVoteAlgorithm_update(MimuMajorityVoteAlgorithmHandle* self,
                                                          const Vector3fArray3_c* imuOmegas_BN_B,
                                                          const Vector3fArray3_c* imuAccels_B);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_MIMU_MAJORITY_VOTE_ALGORITHM_C_H
