#ifndef F32XMERA_MRP_STEERING_TYPES_H
#define F32XMERA_MRP_STEERING_TYPES_H

#include "msgPayloadDef/definitions.h"
#include "utilities/fsw/deviceAvailability.h"
#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Plain-old-data mirror of the C++ algorithm guidance input.
 */
typedef struct {
    Vector3f_c sigma_BR;    /*!< [-] MRP attitude tracking error */
    Vector3f_c omega_BR_B;  /*!< [rad/s] angular rate tracking error in body-frame components */
    Vector3f_c omega_RN_B;  /*!< [rad/s] reference angular rate in body-frame components */
    Vector3f_c domega_RN_B; /*!< [rad/s^2] reference angular acceleration in body-frame components */
} MrpSteeringInputGuidance_c;

/**
 * @brief Plain-old-data carrier for the per-wheel RW speed vector.
 */
typedef struct {
    float wheelSpeeds[RW_EFF_CNT]; /*!< [r/s] reaction-wheel speeds */
} MrpSteeringRwSpeeds_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_MRP_STEERING_TYPES_H
