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
typedef struct ForceTorqueThrForceMappingAlgorithmHandle ForceTorqueThrForceMappingAlgorithmHandle;

/**
 * @brief Get the maximum thruster count constant for validation.
 * @return The maximum thruster count (kMaxThrusterCount).
 */
uint32_t ForceTorqueThrForceMappingAlgorithm_getMaxThrusterCount(void);

/**
 * @brief Construct a new ForceTorqueThrForceMappingAlgorithm from the supplied configuration.
 *
 * Validates the configuration and immediately computes the thruster mapping matrix. Throws on
 * invalid input.
 * @param rThruster_B          [m] Thruster locations in the body frame, three components per thruster
 *                             in row major order.
 * @param tHatThruster_B       [-] Thrust directions in the body frame, three components per thruster in
 *                             row major order; each must be a unit vector to within 1e-3.
 * @param centerOfMass_B       [m] Center of mass in the body frame; must be finite.
 * @param desiredControlAxes_B [-] Per-axis controllability assertions.
 * @return Pointer to a new ForceTorqueThrForceMappingAlgorithm (must be destroyed).
 */
ForceTorqueThrForceMappingAlgorithmHandle* ForceTorqueThrForceMappingAlgorithm_create(
    float rThruster_B[MAX_EFF_CNT * 3],
    float tHatThruster_B[MAX_EFF_CNT * 3],
    float centerOfMass_B[3],
    const ForceTorqueControlAxes_c* desiredControlAxes_B);

/**
 * @brief Destroy a previously created ForceTorqueThrForceMappingAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void ForceTorqueThrForceMappingAlgorithm_destroy(ForceTorqueThrForceMappingAlgorithmHandle* self);

/**
 * @brief Replace the configuration at runtime and recompute the thruster mapping matrix.
 *
 * Throws on invalid input.
 * @param self                 Pointer to the instance.
 * @param rThruster_B          [m] Thruster locations in the body frame, three components per thruster
 *                             in row major order.
 * @param tHatThruster_B       [-] Thrust directions in the body frame, three components per thruster in
 *                             row major order; each must be a unit vector to within 1e-3.
 * @param centerOfMass_B       [m] Center of mass in the body frame; must be finite.
 * @param desiredControlAxes_B [-] Per-axis controllability assertions.
 */
void ForceTorqueThrForceMappingAlgorithm_setConfig(ForceTorqueThrForceMappingAlgorithmHandle* self,
                                                   float rThruster_B[MAX_EFF_CNT * 3],
                                                   float tHatThruster_B[MAX_EFF_CNT * 3],
                                                   float centerOfMass_B[3],
                                                   const ForceTorqueControlAxes_c* desiredControlAxes_B);

/**
 * @brief Compute thruster force commands from the requested torque and force vectors.
 *
 * Every entry carries a non-negative, min-shifted per-thruster command. update() does not throw.
 *
 * @param self        Pointer to the instance.
 * @param cmdTorque_B [Nm] requested control torque in body frame
 * @param cmdForce_B  [N]  requested control force in body frame
 * @return ThrForceArray_c per-thruster force commands.
 */
ThrForceArray_c ForceTorqueThrForceMappingAlgorithm_update(const ForceTorqueThrForceMappingAlgorithmHandle* self,
                                                           float cmdTorque_B[3],
                                                           float cmdForce_B[3]);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_FORCE_TORQUE_THR_FORCE_MAPPING_ALGORITHM_C_H
