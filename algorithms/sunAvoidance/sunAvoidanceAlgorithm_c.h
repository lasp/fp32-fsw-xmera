#ifndef F32XMERA_SUNAVOIDANCEALGORITHM_C_H
#define F32XMERA_SUNAVOIDANCEALGORITHM_C_H

#include "sunAvoidanceTypes.h"
#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ SunAvoidanceAlgorithm instance.
 */
typedef struct SunAvoidanceAlgorithmHandle SunAvoidanceAlgorithmHandle;

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param sensitiveHat_B Body vector to keep off the Sun; must be finite and within 1e-3 of unit length.
 * @param slewRate       [r/s] rate at which the maneuver slews toward the input reference; must be finite
 *                       and greater than zero.
 * @return true when the configuration is valid. Never throws, so it can guard the
 *         throwing create/setConfig from an invalid configuration.
 */
bool SunAvoidanceAlgorithm_validateConfig(const Vector3f_c* sensitiveHat_B, float slewRate);

/**
 * @brief Construct a new SunAvoidanceAlgorithm instance from the supplied configuration.
 * @param sensitiveHat_B Body vector to keep off the Sun (stored normalized).
 * @param slewRate       [r/s] rate at which the maneuver slews toward the input reference.
 * @return Pointer to a new SunAvoidanceAlgorithm (must be destroyed).
 * Validate the configuration with validateConfig first; invalid input throws.
 */
SunAvoidanceAlgorithmHandle* SunAvoidanceAlgorithm_create(const Vector3f_c* sensitiveHat_B, float slewRate);

/**
 * @brief Destroy a previously created SunAvoidanceAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void SunAvoidanceAlgorithm_destroy(SunAvoidanceAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime. The runtime maneuver state is preserved.
 * @param self           Pointer to the instance.
 * @param sensitiveHat_B Body vector to keep off the Sun (stored normalized).
 * @param slewRate       [r/s] rate at which the maneuver slews toward the input reference.
 * Validate the configuration with validateConfig first; invalid input throws.
 */
void SunAvoidanceAlgorithm_setConfig(SunAvoidanceAlgorithmHandle* self,
                                     const Vector3f_c* sensitiveHat_B,
                                     float slewRate);

/**
 * @brief Re-seed the runtime maneuver state so the maneuver reinitializes on the next update.
 * @param self Pointer to the instance.
 */
void SunAvoidanceAlgorithm_reInitialize(SunAvoidanceAlgorithmHandle* self);

/**
 * @brief Compute the Sun-avoidance maneuver-adjusted reference frame.
 * @param self     Pointer to the instance.
 * @param sigma_BN Measured MRP attitude of the body wrt inertial N.
 * @param ref      Attitude reference inputs (algorithm-native POD, mirrors AttRefMsgF32Payload).
 * @param r_BN_N   Spacecraft inertial position [m].
 * @param r_SN_N   Sun inertial position [m].
 * @param callTime The clock time at which the function was called (nanoseconds).
 * @return SunAvoidanceOutput_c  The maneuver-adjusted reference frame.
 */
SunAvoidanceOutput_c SunAvoidanceAlgorithm_update(SunAvoidanceAlgorithmHandle* self,
                                                  const Vector3f_c* sigma_BN,
                                                  const SunAvoidanceAttRefInputs_c* ref,
                                                  const Vector3d_c* r_BN_N,
                                                  const Vector3d_c* r_SN_N,
                                                  uint64_t callTime);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_SUNAVOIDANCEALGORITHM_C_H
