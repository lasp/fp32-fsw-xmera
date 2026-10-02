#ifndef F32XMERA_DV_MANEUVER_TYPES_H
#define F32XMERA_DV_MANEUVER_TYPES_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Plain-old-data mirror of the C++ DvManeuverOutput fields.
 *  - burnExecuting [-] flag indicating whether the burn is in progress
 *  - burnComplete  [-] flag indicating whether the burn has completed
 *  - cmdForce_B    [N] configured body force while the burn executes, else zero
 */
typedef struct {
    uint32_t burnExecuting;
    uint32_t burnComplete;
    Vector3f_c cmdForce_B;
} DvManeuverOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_DV_MANEUVER_TYPES_H
