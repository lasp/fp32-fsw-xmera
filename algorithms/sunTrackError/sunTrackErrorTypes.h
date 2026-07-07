#ifndef F32XMERA_SUN_TRACK_ERROR_TYPES_H
#define F32XMERA_SUN_TRACK_ERROR_TYPES_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Plain-old-data mirror of the C++ SunTrackErrorAttRefInputs fields.
 *
 *  - sigma_RN    [-]      reference MRP attitude of R wrt inertial N
 *  - omega_RN_N  [r/s]    reference angular velocity in N-frame components
 *  - domega_RN_N [r/s^2]  reference angular acceleration in N-frame components
 */
typedef struct {
    Vector3f_c sigma_RN;
    Vector3f_c omega_RN_N;
    Vector3f_c domega_RN_N;
} SunTrackErrorAttRefInputs_c;

/**
 * @brief Plain-old-data mirror of the C++ SunTrackErrorOutput fields: the maneuver-adjusted
 * reference frame, shaped like AttRefMsgF32Payload.
 *  - sigma_RN    [-]      adjusted reference MRP attitude wrt inertial N
 *  - omega_RN_N  [r/s]    adjusted reference angular velocity in N-frame components
 *  - domega_RN_N [r/s^2]  adjusted reference angular acceleration in N-frame components
 */
typedef struct {
    Vector3f_c sigma_RN;
    Vector3f_c omega_RN_N;
    Vector3f_c domega_RN_N;
} SunTrackErrorOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_SUN_TRACK_ERROR_TYPES_H
