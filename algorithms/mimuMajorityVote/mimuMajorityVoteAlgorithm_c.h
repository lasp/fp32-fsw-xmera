#ifndef F32XMERA_MIMUMAJORITYVOTEALGORITHM_C_H
#define F32XMERA_MIMUMAJORITYVOTEALGORITHM_C_H

#include "mimuMajorityVoteTypes.h"
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
 * @brief Construct a new MimuMajorityVoteAlgorithm instance from the supplied configuration.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 * @return Pointer to a new MimuMajorityVoteAlgorithm (must be destroyed).
 */
MimuMajorityVoteAlgorithmHandle* MimuMajorityVoteAlgorithm_create(const MimuMajorityVoteConfig_c* config);

/**
 * @brief Destroy a previously created MimuMajorityVoteAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void MimuMajorityVoteAlgorithm_destroy(MimuMajorityVoteAlgorithmHandle* self);

/**
 * @brief Install the configuration on an existing instance (parameters only; call _reInitialize to
 *        reset the persistence counters).
 * @param self   Pointer to the instance.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 */
void MimuMajorityVoteAlgorithm_setConfig(MimuMajorityVoteAlgorithmHandle* self, const MimuMajorityVoteConfig_c* config);

/**
 * @brief Reset fault persistence counters to zero.
 * @param self Pointer to the instance.
 */
void MimuMajorityVoteAlgorithm_reInitialize(MimuMajorityVoteAlgorithmHandle* self);

/**
 * @brief Run the majority vote update step.
 * @param self      Pointer to the instance.
 * @param imuOmegas_BN_B IMU angular velocity 3-vectors.
 * @return MimuMajorityVoteOutput_c  The computed majority vote output.
 */
MimuMajorityVoteOutput_c MimuMajorityVoteAlgorithm_update(MimuMajorityVoteAlgorithmHandle* self,
                                                          const Vector3fArray3_c* imuOmegas_BN_B);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_MIMUMAJORITYVOTEALGORITHM_C_H
