#ifndef F32XMERA_FORCE_TORQUE_THR_FORCE_MAPPING_ALGORITHM_C_H
#define F32XMERA_FORCE_TORQUE_THR_FORCE_MAPPING_ALGORITHM_C_H

#include "forceTorqueThrForceMappingTypes.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ ForceTorqueThrForceMappingAlgorithm instance.
 */
typedef struct ForceTorqueThrForceMappingAlgorithm ForceTorqueThrForceMappingAlgorithm;

/**
 * @brief Get the MAX_EFF_CNT constant for Ada validation.
 * @return Maximum number of thruster slots in the configuration array.
 */
uint32_t ForceTorqueThrForceMappingAlgorithm_getMaxEffCnt(void);

/**
 * @brief Construct a new ForceTorqueThrForceMappingAlgorithm instance from the supplied
 *        configuration.
 *
 * Validates the configuration and immediately computes the thruster mapping matrix. Throws if any
 * axis flagged in desiredControlAxes is not controllable by the configured thruster array.
 *
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 * @return Pointer to a new ForceTorqueThrForceMappingAlgorithm (must be destroyed).
 */
ForceTorqueThrForceMappingAlgorithm* ForceTorqueThrForceMappingAlgorithm_create(
    const ForceTorqueThrForceMappingConfig_c* config);

/**
 * @brief Destroy a previously created ForceTorqueThrForceMappingAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void ForceTorqueThrForceMappingAlgorithm_destroy(ForceTorqueThrForceMappingAlgorithm* self);

/**
 * @brief Replace the configuration at runtime and recompute the thruster mapping matrix.
 * @param self   Pointer to the instance.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 */
void ForceTorqueThrForceMappingAlgorithm_setConfig(ForceTorqueThrForceMappingAlgorithm* self,
                                                   const ForceTorqueThrForceMappingConfig_c* config);

/**
 * @brief Compute thruster force commands from the requested torque and force vectors.
 *
 * Entries 0..numThrusters-1 carry the non-negative, min-shifted per-thruster commands; trailing
 * slots are exactly zero. update() does not throw.
 *
 * @param self        Pointer to the instance.
 * @param cmdTorque_B [Nm] requested control torque in body frame
 * @param cmdForce_B  [N]  requested control force in body frame
 * @return ThrForceArray_c per-thruster force commands.
 */
ThrForceArray_c ForceTorqueThrForceMappingAlgorithm_update(const ForceTorqueThrForceMappingAlgorithm* self,
                                                           Vector3f_c cmdTorque_B,
                                                           Vector3f_c cmdForce_B);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_FORCE_TORQUE_THR_FORCE_MAPPING_ALGORITHM_C_H
