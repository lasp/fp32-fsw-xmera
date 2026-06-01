#ifndef F32XMERA_SUN_SEARCH_ALGORITHM_C_H
#define F32XMERA_SUN_SEARCH_ALGORITHM_C_H

#include "sunSearchTypes.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ SunSearchAlgorithm instance.
 */
typedef struct SunSearchAlgorithmHandle SunSearchAlgorithmHandle;

/**
 * @brief Get the SUN_SEARCH_NUM_ROTATIONS constant for Ada validation.
 * @return The number of rotation slots in the sun-search sequence.
 */
uint32_t SunSearchAlgorithm_getNumRotations(void);

/**
 * @brief Construct a new SunSearchAlgorithm instance from the supplied configuration.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 * @return Pointer to a new SunSearchAlgorithm (must be destroyed).
 */
SunSearchAlgorithmHandle* SunSearchAlgorithm_create(const SunSearchConfig_c* config);

/**
 * @brief Destroy a previously created SunSearchAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void SunSearchAlgorithm_destroy(SunSearchAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime.
 *
 * Re-arms the algorithm so the next update() call latches a new sequence start time.
 *
 * @param self   Pointer to the instance.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 */
void SunSearchAlgorithm_setConfig(SunSearchAlgorithmHandle* self, const SunSearchConfig_c* config);

/**
 * @brief Compute the reference and body-rate-error angular velocities at the current time.
 *
 * On the first call after construction or setConfig, callTime is latched as the sequence start.
 *
 * @param self       Pointer to the instance.
 * @param callTime   Current simulation time [ns].
 * @param omega_BN_B Body angular velocity in the body frame [rad/s].
 * @return SunSearchOutput_c  Reference angular velocity and body-rate error.
 */
SunSearchOutput_c SunSearchAlgorithm_update(SunSearchAlgorithmHandle* self, uint64_t callTime, Vector3f_c omega_BN_B);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif /* F32XMERA_SUN_SEARCH_ALGORITHM_C_H */
