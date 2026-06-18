#ifndef F32XMERA_MRP_ROTATION_TYPES_H
#define F32XMERA_MRP_ROTATION_TYPES_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Plain-old-data mirror of the C++ MrpRotationAttRefInputs fields.
 *
 * Same content as AttRefMsgF32Payload's sigma_RN / omega_RN_N / domega_RN_N, but the C shim takes
 * the algorithm-native bundle so the algorithm boundary is decoupled from the messaging layer.
 *  - sigma_R0N    [-]      input reference MRP attitude wrt inertial N
 *  - omega_R0N_N  [rad/s]  input reference angular velocity in N-frame components
 *  - domega_R0N_N [rad/s^2] input reference angular acceleration in N-frame components
 */
typedef struct {
    Vector3f_c sigma_R0N;
    Vector3f_c omega_R0N_N;
    Vector3f_c domega_R0N_N;
} MrpRotationAttRefInputs_c;

/**
 * @brief Plain-old-data mirror of the C++ MrpRotationOutput fields.
 *
 * Same content as the output AttRefMsgF32Payload's sigma_RN / omega_RN_N / domega_RN_N.
 *  - sigma_RN    [-]      output reference MRP attitude wrt inertial N
 *  - omega_RN_N  [rad/s]  output reference angular velocity in N-frame components
 *  - domega_RN_N [rad/s^2] output reference angular acceleration in N-frame components
 */
typedef struct {
    Vector3f_c sigma_RN;
    Vector3f_c omega_RN_N;
    Vector3f_c domega_RN_N;
} MrpRotationOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_MRP_ROTATION_TYPES_H
