#ifndef F32XMERA_SUN_SAFE_POINT_TYPES_H
#define F32XMERA_SUN_SAFE_POINT_TYPES_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SUN_SAFE_POINT_NUM_ROTATIONS 4

/**
 * @brief C-compatible enumeration of body axes for the sun-search rotations.
 *
 * Numeric values must stay in lockstep with the C++ enum class in sunSafePointAlgorithm.h.
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
 * @brief Plain-old-data mirror of the full C++ SunSafePointConfig.
 *
 * Caller fills this struct and passes it to SunSafePointAlgorithm_create / _setConfig. The C++ side
 * validates it via SunSafePointConfig::create (rotations and sHatBdyCmd norm) and throws on invalid
 * input.
 *
 *  - sHatBdyCmd norm must be within 1e-3 of 1.0 (renormalized on storage)
 *  - sunAxisSpinRate and omega_RN_B are unconstrained
 *  - observationThreshold is the CSS count at or above which to transition to pointing
 */
typedef struct {
    RotationProperties_c rotations[SUN_SAFE_POINT_NUM_ROTATIONS]; /*!< [-] sun-search rotation sequence */
    Vector3f_c sHatBdyCmd;                                        /*!< [-] commanded body vector to point at the sun */
    float sunAxisSpinRate;    /*!< [rad/s] constant spin rate about the sun heading vector */
    Vector3f_c omega_RN_B;    /*!< [rad/s] fallback body rate when no sun direction is available */
    int observationThreshold; /*!< [-] CSS count at or above which to transition to pointing */
} SunSafePointConfig_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif /* F32XMERA_SUN_SAFE_POINT_TYPES_H */
