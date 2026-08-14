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
 * @brief Plain-old-data mirror of the C++ SunAvoidanceConfig fields.
 *
 * Caller fills this struct and passes it to SunAvoidanceAlgorithm_create or _setConfig. The C++
 * side validates each field via SunAvoidanceConfig::create and throws on invalid input.
 *  - sensitiveHat_B must be finite and within 1e-3 of unit length (stored normalized)
 *  - slewRate [r/s] must be finite and greater than zero
 */
typedef struct {
    Vector3f_c sensitiveHat_B;
    float slewRate;
} SunAvoidanceConfig_c;

/**
 * @brief Construct a new SunAvoidanceAlgorithm instance from the supplied configuration.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 * @return Pointer to a new SunAvoidanceAlgorithm (must be destroyed).
 */
SunAvoidanceAlgorithmHandle* SunAvoidanceAlgorithm_create(const SunAvoidanceConfig_c* config);

/**
 * @brief Destroy a previously created SunAvoidanceAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void SunAvoidanceAlgorithm_destroy(SunAvoidanceAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime. The runtime maneuver state is preserved.
 * @param self   Pointer to the instance.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 */
void SunAvoidanceAlgorithm_setConfig(SunAvoidanceAlgorithmHandle* self, const SunAvoidanceConfig_c* config);

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
