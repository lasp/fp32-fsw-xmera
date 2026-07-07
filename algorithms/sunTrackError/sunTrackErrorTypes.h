#ifndef F32XMERA_SUN_TRACK_ERROR_TYPES_H
#define F32XMERA_SUN_TRACK_ERROR_TYPES_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Plain-old-data mirror of the C++ SunTrackErrorNavAttInputs fields.
 *
 * Same content as NavAttMsgF32Payload's sigma_BN / omega_BN_B, but the C shim takes the
 * algorithm-native bundle so the algorithm boundary is decoupled from the messaging layer.
 *  - sigma_BN    [-]    measured MRP attitude of B wrt inertial N
 *  - omega_BN_B  [r/s]  measured body rate of B wrt N in B-frame components
 */
typedef struct {
    Vector3f_c sigma_BN;
    Vector3f_c omega_BN_B;
} SunTrackErrorNavAttInputs_c;

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
 * @brief Plain-old-data mirror of the C++ SunTrackErrorOutput fields.
 *
 * Same content as the output AttGuidMsgF32Payload's sigma_BR / omega_BR_B / omega_RN_B / domega_RN_B.
 *  - sigma_BR    [-]      attitude error MRP of B wrt R
 *  - omega_BR_B  [r/s]    body rate error of B wrt R in B-frame components
 *  - omega_RN_B  [r/s]    reference rate of R wrt N in B-frame components
 *  - domega_RN_B [r/s^2]  reference angular acceleration in B-frame components
 */
typedef struct {
    Vector3f_c sigma_BR;
    Vector3f_c omega_BR_B;
    Vector3f_c omega_RN_B;
    Vector3f_c domega_RN_B;
} SunTrackErrorOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_SUN_TRACK_ERROR_TYPES_H
