#ifndef F32XMERA_SUN_SEARCH_TYPES_H
#define F32XMERA_SUN_SEARCH_TYPES_H

#include "utilities/plainCAlgorithmDataTypes.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SUN_SEARCH_NUM_ROTATIONS 4

/**
 * @brief C-compatible enumeration of body axes for sun-search rotations.
 *
 * Numeric values must stay in lockstep with the C++ enum class in sunSearchAlgorithm.h.
 */
typedef enum { ROTATION_AXIS_B1HAT_B_C = 0, ROTATION_AXIS_B2HAT_B_C = 1, ROTATION_AXIS_B3HAT_B_C = 2 } RotationAxis_c;

/**
 * @brief Plain-old-data mirror of the C++ RotationProperties fields.
 *
 *  - rotationDuration must be finite and > 0
 *  - rotationRate must be finite (sign selects rotation direction)
 *  - rotationAxis must be one of the RotationAxis_c values
 */
typedef struct {
    float rotationDuration;      /*!< [s]    duration of this rotation */
    float rotationRate;          /*!< [rad/s] signed scalar body rate during this rotation */
    RotationAxis_c rotationAxis; /*!< [-]    axis about which to rotate */
} RotationProperties_c;

/**
 * @brief Plain-old-data mirror of the C++ SunSearchConfig fields.
 *
 * Caller fills this struct and passes it to SunSearchAlgorithm_create or _setConfig.
 * The C++ side validates each rotation via SunSearchConfig::create and throws on
 * invalid input.
 */
typedef struct {
    RotationProperties_c rotations[SUN_SEARCH_NUM_ROTATIONS];
} SunSearchConfig_c;

/**
 * @brief C-compatible mirror of the C++ SunSearchOutput.
 */
typedef struct {
    Vector3f_c omega_RN_B; /*!< [rad/s] reference angular velocity in body frame */
    Vector3f_c omega_BR_B; /*!< [rad/s] body-rate error in body frame */
} SunSearchOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif /* F32XMERA_SUN_SEARCH_TYPES_H */
