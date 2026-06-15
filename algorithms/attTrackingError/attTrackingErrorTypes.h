#ifndef F32XMERA_ATT_TRACKING_ERROR_TYPES_H
#define F32XMERA_ATT_TRACKING_ERROR_TYPES_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief C-compatible navigation attitude input.
 */
typedef struct {
    Vector3f_c sigma_BN;
    Vector3f_c omega_BN_B;
} AttNavInput_c;

/**
 * @brief C-compatible reference attitude input.
 */
typedef struct {
    Vector3f_c sigma_RN;
    Vector3f_c omega_RN_N;
    Vector3f_c domega_RN_N;
} AttRefInput_c;

/**
 * @brief C-compatible attitude guidance output.
 */
typedef struct {
    Vector3f_c sigma_BR;
    Vector3f_c omega_BR_B;
    Vector3f_c omega_RN_B;
    Vector3f_c domega_RN_B;
} AttGuidOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_ATT_TRACKING_ERROR_TYPES_H
