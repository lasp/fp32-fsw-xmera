#ifndef F32XMERA_SUN_SEARCH_POINT_TYPES_H
#define F32XMERA_SUN_SEARCH_POINT_TYPES_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Number of rotations in the sun-search sequence. Intrinsic to the algorithm,
    not a mission parameter. C++ callers use kNumSunSearchRotations. */
#define SUN_SEARCH_POINT_NUM_ROTATIONS 4

/**
 * @brief C-compatible enumeration of body axes for the sun-search rotations.
 *
 * Numeric values must stay in lockstep with the C++ enum class in sunSearchPointAlgorithm.h.
 *
 * A plain C enum, so it has the width of an int. The Ada side declares the matching field
 * with the C version of its enumeration, which also has the width of an int.
 */
typedef enum RotationAxis_c {
    ROTATION_AXIS_B1HAT_B_C = 0,
    ROTATION_AXIS_B2HAT_B_C = 1,
    ROTATION_AXIS_B3HAT_B_C = 2
} RotationAxis_c;

/**
 * @brief Plain-old-data mirror of the C++ RotationProperties fields.
 *
 *  - rotationDuration must be finite and convert to at least 1 ns
 *  - rotationRate must be finite (sign selects rotation direction)
 *  - rotationAxis must be one of the RotationAxis_c values
 *
 * The rotations array crosses by reference, so the C++ side reads it at fixed offsets:
 * narrowing rotationAxis here silently misreads data rather than failing to compile. The
 * binding's count asserts do not catch it, because padding would absorb a narrower width.
 * The behavioural component tests are what guard the flag width.
 */
typedef struct {
    float rotationDuration;      /*!< [s]    duration of this rotation */
    float rotationRate;          /*!< [rad/s] signed scalar body rate during this rotation */
    RotationAxis_c rotationAxis; /*!< [-]    axis about which to rotate */
} RotationProperties_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif /* F32XMERA_SUN_SEARCH_POINT_TYPES_H */
