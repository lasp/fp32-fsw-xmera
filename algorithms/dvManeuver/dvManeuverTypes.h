#ifndef F32XMERA_DV_MANEUVER_TYPES_H
#define F32XMERA_DV_MANEUVER_TYPES_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief C-compatible enumeration mirroring DvManeuverBurnState.
 *
 * Numeric values must stay in lockstep with the C++ enum class in dvManeuverAlgorithm.h.
 */
typedef enum {
    DV_MANEUVER_BURN_STATE_PENDING_C = 0,
    DV_MANEUVER_BURN_STATE_EXECUTING_C = 1,
    DV_MANEUVER_BURN_STATE_COMPLETE_C = 2
} DvManeuverBurnState_c;

/**
 * @brief Plain-old-data mirror of the C++ DvManeuverOutput fields.
 *  - state      [-] burn state after this update
 *  - cmdForce_B [N] configured body force while the burn executes, else zero
 */
typedef struct {
    DvManeuverBurnState_c state;
    Vector3f_c cmdForce_B;
} DvManeuverOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_DV_MANEUVER_TYPES_H
