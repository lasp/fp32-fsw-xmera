#ifndef F32XMERA_SUN_SEARCH_POINT_TYPES_H
#define F32XMERA_SUN_SEARCH_POINT_TYPES_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SUN_SEARCH_POINT_NUM_ROTATIONS 4

/**
 * @brief C-compatible enumeration of body axes for the sun-search rotations.
 *
 * Numeric values must stay in lockstep with the C++ enum class in sunSearchPointAlgorithm.h.
 *
 * The underlying type is fixed at uint8_t to match the Ada side's 8-bit Rotation_Axis. A plain
 * C enum is int-width, which would read four bytes where Ada wrote one.
 */
typedef enum RotationAxis_c : uint8_t {
    ROTATION_AXIS_B1HAT_B_C = 0,
    ROTATION_AXIS_B2HAT_B_C = 1,
    ROTATION_AXIS_B3HAT_B_C = 2
} RotationAxis_c;

/**
 * @brief Plain-old-data mirror of the C++ RotationProperties fields.
 *
 *  - rotationDuration must be finite and > 0
 *  - rotationRate must be finite (sign selects rotation direction)
 *  - rotationAxis must be one of the RotationAxis_c values
 *
 * The rotations array crosses by reference, so the C++ side reads it at fixed offsets:
 * changing rotationAxis's width here silently misreads data rather than failing to compile.
 * The binding's count asserts do not catch it, because the padding after rotationAxis absorbs
 * any width up to 32 bits. The behavioural component tests are what guard the flag width.
 */
typedef struct {
    float rotationDuration;      /*!< [s]    duration of this rotation */
    float rotationRate;          /*!< [rad/s] signed scalar body rate during this rotation */
    RotationAxis_c rotationAxis; /*!< [-]    axis about which to rotate */
} RotationProperties_c;

/**
 * @brief Plain-old-data mirror of the full C++ SunSearchPointConfig.
 *
 * Caller fills this struct and passes it to SunSearchPointAlgorithm_create / _setConfig. The C++ side
 * validates it via SunSearchPointConfig::create (rotations and sHatBdyCmd norm) and throws on invalid
 * input.
 *
 *  - sHatBdyCmd norm must be within 1e-3 of 1.0 (renormalized on storage)
 *  - sunAxisSpinRate and omega_RN_B are unconstrained
 *  - observationThreshold is the CSS count at or above which to transition to pointing
 *  - controlPeriod is the per-update time step [s] (must be finite and > 0)
 */
typedef struct {
    RotationProperties_c rotations[SUN_SEARCH_POINT_NUM_ROTATIONS]; /*!< [-] sun-search rotation sequence */
    Vector3f_c sHatBdyCmd;    /*!< [-] commanded body vector to point at the sun */
    float sunAxisSpinRate;    /*!< [rad/s] constant spin rate about the sun heading vector */
    Vector3f_c omega_RN_B;    /*!< [rad/s] fallback body rate when no sun direction is available */
    uint32_t observationThreshold; /*!< [-] CSS count at or above which to transition to pointing */
    float controlPeriod;      /*!< [s] per-update time step; advances the search timeline (> 0) */
} SunSearchPointConfig_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif /* F32XMERA_SUN_SEARCH_POINT_TYPES_H */
