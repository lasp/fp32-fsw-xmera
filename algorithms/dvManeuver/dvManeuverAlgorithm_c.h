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
 * @param minTime       [ns] minimum burn time before completion.
 * @param maxTime       [ns] maximum burn time; must be positive and greater than minTime.
 * @param controlPeriod [s] FSW time step used as the burn-time delta-t; must be > 0 and finite.
 * @param cmdForce_B    [N] body force commanded while the burn executes; must be finite.
 * @param cmdDv_N       [m/s] commanded delta-V in inertial frame components; must be finite.
 * @param burnStartTime [ns] time at which the burn starts.
 * @return true when the configuration is valid. Never throws, so it can guard the throwing
 *         create/setConfig from an invalid configuration.
 */
bool DvManeuverAlgorithm_validateConfig(uint64_t minTime,
                                        uint64_t maxTime,
                                        float controlPeriod,
                                        const Vector3f_c* cmdForce_B,
                                        const Vector3f_c* cmdDv_N,
                                        uint64_t burnStartTime);

/**
 * @brief Construct a new DvManeuverAlgorithm instance from the supplied configuration.
 * @param minTime       [ns] minimum burn time before completion.
 * @param maxTime       [ns] maximum burn time; must be positive and greater than minTime.
 * @param controlPeriod [s] FSW time step used as the burn-time delta-t; must be > 0 and finite.
 * @param cmdForce_B    [N] body force commanded while the burn executes; must be finite.
 * @param cmdDv_N       [m/s] commanded delta-V in inertial frame components; must be finite.
 * @param burnStartTime [ns] time at which the burn starts.
 * @return Pointer to a new DvManeuverAlgorithm (must be destroyed).
 * Validate the values with validateConfig first; invalid input throws.
 */
DvManeuverAlgorithmHandle* DvManeuverAlgorithm_create(uint64_t minTime,
                                                      uint64_t maxTime,
                                                      float controlPeriod,
                                                      const Vector3f_c* cmdForce_B,
                                                      const Vector3f_c* cmdDv_N,
                                                      uint64_t burnStartTime);

/**
 * @brief Destroy a previously created DvManeuverAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void DvManeuverAlgorithm_destroy(DvManeuverAlgorithmHandle* self);

/**
 * @brief Install the configuration on an existing instance (parameters only; call _reInitialize to
 *        reset the burn state machine).
 * @param self          Pointer to the instance.
 * @param minTime       [ns] minimum burn time before completion.
 * @param maxTime       [ns] maximum burn time; must be positive and greater than minTime.
 * @param controlPeriod [s] FSW time step used as the burn-time delta-t; must be > 0 and finite.
 * @param cmdForce_B    [N] body force commanded while the burn executes; must be finite.
 * @param cmdDv_N       [m/s] commanded delta-V in inertial frame components; must be finite.
 * @param burnStartTime [ns] time at which the burn starts.
 * Validate the values with validateConfig first; invalid input throws.
 */
void DvManeuverAlgorithm_setConfig(DvManeuverAlgorithmHandle* self,
                                   uint64_t minTime,
                                   uint64_t maxTime,
                                   float controlPeriod,
                                   const Vector3f_c* cmdForce_B,
                                   const Vector3f_c* cmdDv_N,
                                   uint64_t burnStartTime);

/**
 * @brief Reset the burn state machine to its initial (pre-burn) condition.
 * @param self Pointer to the instance.
 */
void DvManeuverAlgorithm_reInitialize(DvManeuverAlgorithmHandle* self);

/**
 * @brief Advance the burn state machine one step.
 * @param self          Pointer to the instance.
 * @param callTime      Evaluation time [ns].
 * @param dvAccumulated Total accumulated delta-V from navigation [m/s].
 * @return DvManeuverOutput_c  Burn state and body force command.
 */
DvManeuverOutput_c DvManeuverAlgorithm_update(DvManeuverAlgorithmHandle* self,
                                              uint64_t callTime,
                                              const Vector3f_c* dvAccumulated);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_DV_MANEUVER_ALGORITHM_C_H
