#ifndef F32XMERA_SUNTRACKERRORALGORITHM_C_H
#define F32XMERA_SUNTRACKERRORALGORITHM_C_H

#include "sunTrackErrorTypes.h"
#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ SunTrackErrorAlgorithm instance.
 */
typedef struct SunTrackErrorAlgorithmHandle SunTrackErrorAlgorithmHandle;

/**
 * @brief Plain-old-data mirror of the C++ SunTrackErrorConfig fields.
 *
 * Caller fills this struct and passes it to SunTrackErrorAlgorithm_create or _setConfig. The C++
 * side validates each field via SunTrackErrorConfig::create and throws on invalid input.
 *  - sigma_R0R must be finite
 *  - sensitiveHat_B must be finite (stored normalized)
 *  - angleRate [r/s] must be finite
 *  - computeAngleStart selects whether the initial maneuver angle is computed from the sun geometry
 *    (true when the trans/ephemeris messages are connected) or assumed to be 0
 */
typedef struct {
    Vector3f_c sigma_R0R;
    Vector3f_c sensitiveHat_B;
    float angleRate;
    bool computeAngleStart;
} SunTrackErrorConfig_c;

/**
 * @brief Construct a new SunTrackErrorAlgorithm instance from the supplied configuration.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 * @return Pointer to a new SunTrackErrorAlgorithm (must be destroyed).
 */
SunTrackErrorAlgorithmHandle* SunTrackErrorAlgorithm_create(const SunTrackErrorConfig_c* config);

/**
 * @brief Destroy a previously created SunTrackErrorAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void SunTrackErrorAlgorithm_destroy(SunTrackErrorAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime. The runtime maneuver state is preserved.
 * @param self   Pointer to the instance.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 */
void SunTrackErrorAlgorithm_setConfig(SunTrackErrorAlgorithmHandle* self, const SunTrackErrorConfig_c* config);

/**
 * @brief Re-seed the runtime maneuver state so the maneuver reinitializes on the next update.
 * @param self Pointer to the instance.
 */
void SunTrackErrorAlgorithm_reInitialize(SunTrackErrorAlgorithmHandle* self);

/**
 * @brief Compute the attitude tracking error for sun avoidance.
 * @param self     Pointer to the instance.
 * @param nav      Attitude navigation inputs (algorithm-native POD, mirrors NavAttMsgF32Payload).
 * @param ref      Attitude reference inputs (algorithm-native POD, mirrors AttRefMsgF32Payload).
 * @param r_BN_N   Spacecraft inertial position [m].
 * @param r_SN_N   Sun inertial position [m].
 * @param callTime The clock time at which the function was called (nanoseconds).
 * @return SunTrackErrorOutput_c  Output attitude guidance error.
 */
SunTrackErrorOutput_c SunTrackErrorAlgorithm_update(SunTrackErrorAlgorithmHandle* self,
                                                    const SunTrackErrorNavAttInputs_c* nav,
                                                    const SunTrackErrorAttRefInputs_c* ref,
                                                    const Vector3f_c* r_BN_N,
                                                    const Vector3f_c* r_SN_N,
                                                    uint64_t callTime);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_SUNTRACKERRORALGORITHM_C_H
