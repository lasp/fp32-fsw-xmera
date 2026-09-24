#ifndef F32XMERA_FORCE_TORQUE_THR_FORCE_MAPPING_ALGORITHM_C_H
#define F32XMERA_FORCE_TORQUE_THR_FORCE_MAPPING_ALGORITHM_C_H

#include "forceTorqueThrForceMappingTypes.h"
#include "utilities/fsw/deviceAvailability.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ ForceTorqueThrForceMappingAlgorithm instance.
 */
typedef struct ForceTorqueThrForceMappingAlgorithmHandle ForceTorqueThrForceMappingAlgorithmHandle;

/**
 * @brief Thruster geometry table, three components per thruster in row major order.
 */
typedef struct {
    float data[MAX_EFF_CNT * 3];
} ThrusterGeometryArray_c;

/**
 * @brief Availability of each thruster slot, one entry per slot: 0 available, 1 unavailable.
 */
typedef struct {
    DeviceAvailability_c availability[MAX_EFF_CNT];
} ThrusterAvailabilityArray_c;

/**
 * @brief Get the maximum thruster count constant for validation.
 * @return The maximum thruster count (kMaxThrusterCount).
 */
uint32_t ForceTorqueThrForceMappingAlgorithm_getMaxThrusterCount(void);

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param numThrusters         [-] Number of configured thrusters, in [1, MAX_EFF_CNT]. Only the
 *                             first numThrusters entries of the geometry take part in the mapping.
 * @param rThruster_B          [m] Thruster locations in the body frame, three components per thruster
 *                             in row major order.
 * @param tHatThruster_B       [-] Thrust directions in the body frame, three components per thruster in
 *                             row major order; each must be a unit vector to within 1e-3.
 * @param centerOfMass_B       [m] Center of mass in the body frame; must be finite.
 * @param desiredControlAxes_B [-] Per-axis controllability assertions.
 * @param thrusterAvailability [-] Availability of each thruster, one entry per slot: 0 available,
 *                             1 unavailable. An unavailable thruster is left out of the mapping
 *                             and always receives a zero command; a minimum of one must be available.
 * @return true when the configuration is valid. Never throws, so it can guard the throwing
 *         create/setConfig from an invalid configuration.
 */
bool ForceTorqueThrForceMappingAlgorithm_validateConfig(uint32_t numThrusters,
                                                        const ThrusterGeometryArray_c* rThruster_B,
                                                        const ThrusterGeometryArray_c* tHatThruster_B,
                                                        const Vector3f_c* centerOfMass_B,
                                                        const ForceTorqueControlAxes_c* desiredControlAxes_B,
                                                        const ThrusterAvailabilityArray_c* thrusterAvailability);

/**
 * @brief Construct a new ForceTorqueThrForceMappingAlgorithm from the supplied configuration.
 *
 * Validates the configuration and immediately computes the thruster mapping matrix. Validate the
 * values with validateConfig first; invalid input throws -- if desiredControlAxes_B selects no axis,
 * or if a selected axis is not controllable by the configured thruster array.
 * @param numThrusters         [-] Number of configured thrusters, in [1, MAX_EFF_CNT]. Only the
 *                             first numThrusters entries of the geometry take part in the mapping.
 * @param rThruster_B          [m] Thruster locations in the body frame, three components per thruster
 *                             in row major order.
 * @param tHatThruster_B       [-] Thrust directions in the body frame, three components per thruster in
 *                             row major order; each must be a unit vector to within 1e-3.
 * @param centerOfMass_B       [m] Center of mass in the body frame; must be finite.
 * @param desiredControlAxes_B [-] The axes the mapping controls; a minimum of one must be selected.
 * @param thrusterAvailability [-] Availability of each thruster, one entry per slot: 0 available,
 *                             1 unavailable. An unavailable thruster is left out of the mapping
 *                             and always receives a zero command; a minimum of one must be available.
 * @return Pointer to a new ForceTorqueThrForceMappingAlgorithm (must be destroyed).
 */
ForceTorqueThrForceMappingAlgorithmHandle* ForceTorqueThrForceMappingAlgorithm_create(
    uint32_t numThrusters,
    const ThrusterGeometryArray_c* rThruster_B,
    const ThrusterGeometryArray_c* tHatThruster_B,
    const Vector3f_c* centerOfMass_B,
    const ForceTorqueControlAxes_c* desiredControlAxes_B,
    const ThrusterAvailabilityArray_c* thrusterAvailability);

/**
 * @brief Destroy a previously created ForceTorqueThrForceMappingAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void ForceTorqueThrForceMappingAlgorithm_destroy(ForceTorqueThrForceMappingAlgorithmHandle* self);

/**
 * @brief Replace the configuration at runtime and recompute the thruster mapping matrix.
 *
 * Validate the values with validateConfig first; invalid input throws.
 * @param self                 Pointer to the instance.
 * @param numThrusters         [-] Number of configured thrusters, in [1, MAX_EFF_CNT]. Only the
 *                             first numThrusters entries of the geometry take part in the mapping.
 * @param rThruster_B          [m] Thruster locations in the body frame, three components per thruster
 *                             in row major order.
 * @param tHatThruster_B       [-] Thrust directions in the body frame, three components per thruster in
 *                             row major order; each must be a unit vector to within 1e-3.
 * @param centerOfMass_B       [m] Center of mass in the body frame; must be finite.
 * @param desiredControlAxes_B [-] Per-axis controllability assertions.
 * @param thrusterAvailability [-] Availability of each thruster, one entry per slot: 0 available,
 *                             1 unavailable. An unavailable thruster is left out of the mapping
 *                             and always receives a zero command; a minimum of one must be available.
 */
void ForceTorqueThrForceMappingAlgorithm_setConfig(ForceTorqueThrForceMappingAlgorithmHandle* self,
                                                   uint32_t numThrusters,
                                                   const ThrusterGeometryArray_c* rThruster_B,
                                                   const ThrusterGeometryArray_c* tHatThruster_B,
                                                   const Vector3f_c* centerOfMass_B,
                                                   const ForceTorqueControlAxes_c* desiredControlAxes_B,
                                                   const ThrusterAvailabilityArray_c* thrusterAvailability);

/**
 * @brief Compute thruster force commands from the requested torque and force vectors.
 *
 * Every entry carries a non-negative per-thruster command; an unavailable thruster receives exactly
 * zero. update() does not throw.
 *
 * @param self        Pointer to the instance.
 * @param cmdTorque_B [Nm] requested control torque in body frame
 * @param cmdForce_B  [N]  requested control force in body frame
 * @return ThrForceArray_c per-thruster force commands.
 */
ThrForceArray_c ForceTorqueThrForceMappingAlgorithm_update(const ForceTorqueThrForceMappingAlgorithmHandle* self,
                                                           const Vector3f_c* cmdTorque_B,
                                                           const Vector3f_c* cmdForce_B);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_FORCE_TORQUE_THR_FORCE_MAPPING_ALGORITHM_C_H
