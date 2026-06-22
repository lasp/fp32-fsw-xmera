#ifndef F32XMERA_SUNSEARCHPOINTALGORITHM_C_H
#define F32XMERA_SUNSEARCHPOINTALGORITHM_C_H

#include "sunSearchPointTypes.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ SunSearchPointAlgorithm instance.
 */
typedef struct SunSearchPointAlgorithmHandle SunSearchPointAlgorithmHandle;

/**
 * @brief C-compatible sun search point attitude guidance output.
 */
typedef struct {
    Vector3f_c sigma_BR;   /*!< attitude error (MRPs) of B relative to R */
    Vector3f_c omega_BR_B; /*!< [rad/s] body rate error of B relative to R in B frame */
    Vector3f_c omega_RN_B; /*!< [rad/s] reference frame rate of R relative to N in B frame */
    bool faultDetected;    /*!< [-] true once the search fails to acquire the sun (forced to pointing) */
} SunSearchPointOutput_c;

/**
 * @brief Get the SUN_SEARCH_POINT_NUM_ROTATIONS constant for Ada validation.
 * @return The number of rotation slots in the sun-search sequence.
 */
uint32_t SunSearchPointAlgorithm_getNumRotations(void);

/**
 * @brief Construct a new SunSearchPointAlgorithm instance from the supplied configuration.
 * @param config Pointer to the rotation-sequence configuration (validated; throws on invalid input).
 * @return Pointer to a new SunSearchPointAlgorithm (must be destroyed).
 */
SunSearchPointAlgorithmHandle* SunSearchPointAlgorithm_create(const SunSearchPointConfig_c* config);

/**
 * @brief Destroy a previously created SunSearchPointAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void SunSearchPointAlgorithm_destroy(SunSearchPointAlgorithmHandle* self);

/**
 * @brief Install the sun-search rotation sequence configuration (parameters only; call _reInitialize
 *        to re-arm the search phase).
 * @param self   Pointer to the instance.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 */
void SunSearchPointAlgorithm_setConfig(SunSearchPointAlgorithmHandle* self, const SunSearchPointConfig_c* config);

/**
 * @brief Re-arm the runtime state machine so the next update begins a fresh search sequence.
 * @param self Pointer to the instance.
 */
void SunSearchPointAlgorithm_reInitialize(SunSearchPointAlgorithmHandle* self);

/**
 * @brief Run the update step.
 * @param self              Pointer to the instance.
 * @param callTime          Current simulation time [ns].
 * @param rHat_SB_B      Sun direction vector in body frame.
 * @param omega_BN_B        Inertial body angular velocity in body frame.
 * @param numCssViewingSun Number of valid coarse-sun-sensor observations this cycle.
 * @return SunSearchPointOutput_c  The computed guidance output.
 */
SunSearchPointOutput_c SunSearchPointAlgorithm_update(SunSearchPointAlgorithmHandle* self,
                                                      uint64_t callTime,
                                                      Vector3f_c rHat_SB_B,
                                                      Vector3f_c omega_BN_B,
                                                      int numCssViewingSun);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_SUNSEARCHPOINTALGORITHM_C_H
