#ifndef F32XMERA_DV_MANEUVER_ALGORITHM_C_H
#define F32XMERA_DV_MANEUVER_ALGORITHM_C_H

#include "dvManeuverTypes.h"
#include "utilities/fsw/plainCAlgorithmDataTypes.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ DvManeuverAlgorithm instance.
 */
typedef struct DvManeuverAlgorithmHandle DvManeuverAlgorithmHandle;

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param minTime       [s] minimum burn time before completion; must be >= 0 and finite.
 * @param maxTime       [s] maximum burn time; must be positive and finite.
 * @param controlPeriod [s] FSW time step used as the burn-time delta-t; must be > 0 and finite.
 * @return true when the configuration is valid. Never throws, so it can guard the throwing
 *         create/setConfig from an invalid configuration.
 */
bool DvManeuverAlgorithm_validateConfig(float minTime, float maxTime, float controlPeriod);

/**
 * @brief Construct a new DvManeuverAlgorithm instance from the supplied configuration.
 * @param minTime       [s] minimum burn time before completion; must be >= 0 and finite.
 * @param maxTime       [s] maximum burn time; must be positive and finite.
 * @param controlPeriod [s] FSW time step used as the burn-time delta-t; must be > 0 and finite.
 * @return Pointer to a new DvManeuverAlgorithm (must be destroyed).
 * Validate the values with validateConfig first; invalid input throws.
 */
DvManeuverAlgorithmHandle* DvManeuverAlgorithm_create(float minTime, float maxTime, float controlPeriod);

/**
 * @brief Destroy a previously created DvManeuverAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void DvManeuverAlgorithm_destroy(DvManeuverAlgorithmHandle* self);

/**
 * @brief Install the configuration on an existing instance (parameters only; call _reInitialize to
 *        reset the burn state machine).
 * @param self          Pointer to the instance.
 * @param minTime       [s] minimum burn time before completion; must be >= 0 and finite.
 * @param maxTime       [s] maximum burn time; must be positive and finite.
 * @param controlPeriod [s] FSW time step used as the burn-time delta-t; must be > 0 and finite.
 * Validate the values with validateConfig first; invalid input throws.
 */
void DvManeuverAlgorithm_setConfig(DvManeuverAlgorithmHandle* self, float minTime, float maxTime, float controlPeriod);

/**
 * @brief Reset the burn state machine to its initial (pre-burn) condition.
 * @param self Pointer to the instance.
 */
void DvManeuverAlgorithm_reInitialize(DvManeuverAlgorithmHandle* self);

/**
 * @brief Advance the burn state machine one step.
 * @param self          Pointer to the instance.
 * @param callTime      Evaluation time [ns].
 * @param vehAccumDV    Total accumulated delta-V from navigation [m/s].
 * @param dvInrtlCmd    Commanded delta-V in inertial coordinates [m/s].
 * @param burnStartTime Commanded burn start time [ns].
 * @return DvManeuverOutput_c  Burn execution status and thruster-off command flag.
 */
DvManeuverOutput_c DvManeuverAlgorithm_update(DvManeuverAlgorithmHandle* self,
                                              uint64_t callTime,
                                              const Vector3f_c* vehAccumDV,
                                              const Vector3f_c* dvInrtlCmd,
                                              uint64_t burnStartTime);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_DV_MANEUVER_ALGORITHM_C_H
