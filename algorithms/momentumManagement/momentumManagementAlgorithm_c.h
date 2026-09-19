#ifndef F32XMERA_MOMENTUM_MANAGEMENT_ALGORITHM_C_H
#define F32XMERA_MOMENTUM_MANAGEMENT_ALGORITHM_C_H

#include "momentumManagementTypes.h"
#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ MomentumManagementAlgorithm instance.
 */
typedef struct MomentumManagementAlgorithmHandle MomentumManagementAlgorithmHandle;

/**
 * @brief RW spin axes in body frame, three components per wheel in row major order.
 */
typedef struct {
    float data[3 * RW_EFF_CNT]; /*!< [-] three components per wheel */
} MomentumManagementRwSpinAxes_c;

/**
 * @brief Per-wheel spin-axis inertia, one entry per wheel slot.
 */
typedef struct {
    float data[RW_EFF_CNT]; /*!< [kgm2] one entry per wheel */
} MomentumManagementRwInertias_c;

/**
 * @brief Availability of each wheel slot, one byte per slot: 0 available, 1 unavailable.
 */
typedef struct {
    uint8_t availability[RW_EFF_CNT]; /*!< [-] one entry per wheel */
} MomentumManagementRwAvailability_c;

/**
 * @brief Get the RW_EFF_CNT constant for Ada validation.
 * @return The maximum number of reaction wheels handled at the C boundary.
 */
uint32_t MomentumManagementAlgorithm_getMaxNumRw(void);

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param hsMin         [Nms]  minimum RW cluster momentum for dumping; must be finite and non-negative.
 * @param K             [1/s]  proportional gain on the stored momentum; must be finite and non-negative.
 * @param Ki            [1/s2] integral gain on the accumulated stored momentum; must be finite and non-negative.
 * @param integralLimit [Nms2] anti-windup clamp on each integral component; must be finite and non-negative,
 *                             and positive when Ki > 0.
 * @param controlPeriod [s]    integration step between update() calls; must be finite and non-negative,
 *                             and positive when Ki > 0 (only the integral term uses it).
 * @param dumpableProjection_B [-] projector onto the directions the effectors can dump about; must be a
 *                             finite, symmetric and idempotent orthogonal projector. Pass the identity when
 *                             every direction can be dumped.
 * @param GsMatrix_B        [-]     RW spin axes, three per wheel in row major order; every axis must be a
 *                                  unit vector.
 * @param JsList            [kgm2]  per-wheel spin-axis inertia.
 * @param wheelAvailability [-]     availability of each wheel, one byte per slot: 0 available, 1 unavailable.
 * @return true when the configuration is valid. Never throws, so it can guard the
 *         throwing create/setConfig from an invalid configuration.
 */
bool MomentumManagementAlgorithm_validateConfig(float hsMin,
                                                float K,
                                                float Ki,
                                                float integralLimit,
                                                float controlPeriod,
                                                const Matrix3f_c* dumpableProjection_B,
                                                const MomentumManagementRwSpinAxes_c* GsMatrix_B,
                                                const MomentumManagementRwInertias_c* JsList,
                                                const MomentumManagementRwAvailability_c* wheelAvailability);

/**
 * @brief Construct a new MomentumManagementAlgorithm instance from the supplied configuration.
 * @param hsMin         [Nms]  minimum RW cluster momentum for dumping; must be finite and non-negative.
 * @param K             [1/s]  proportional gain on the stored momentum; must be finite and non-negative.
 * @param Ki            [1/s2] integral gain on the accumulated stored momentum; must be finite and non-negative.
 * @param integralLimit [Nms2] anti-windup clamp on each integral component; must be finite and non-negative,
 *                             and positive when Ki > 0.
 * @param controlPeriod [s]    integration step between update() calls; must be finite and non-negative,
 *                             and positive when Ki > 0 (only the integral term uses it).
 * @param dumpableProjection_B [-] projector onto the directions the effectors can dump about; must be a
 *                             finite, symmetric and idempotent orthogonal projector. Pass the identity when
 *                             every direction can be dumped.
 * @param GsMatrix_B        [-]     RW spin axes, three per wheel in row major order; every axis must be a
 *                                  unit vector.
 * @param JsList            [kgm2]  per-wheel spin-axis inertia.
 * @param wheelAvailability [-]     availability of each wheel, one byte per slot: 0 available, 1 unavailable.
 * @return Pointer to a new MomentumManagementAlgorithm (must be destroyed).
 * Validate the configuration with validateConfig first; invalid input throws.
 */
MomentumManagementAlgorithmHandle* MomentumManagementAlgorithm_create(
    float hsMin,
    float K,
    float Ki,
    float integralLimit,
    float controlPeriod,
    const Matrix3f_c* dumpableProjection_B,
    const MomentumManagementRwSpinAxes_c* GsMatrix_B,
    const MomentumManagementRwInertias_c* JsList,
    const MomentumManagementRwAvailability_c* wheelAvailability);

/**
 * @brief Destroy a previously created MomentumManagementAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void MomentumManagementAlgorithm_destroy(MomentumManagementAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime without disturbing its runtime state.
 * @param self          Pointer to the instance.
 * @param hsMin         [Nms]  minimum RW cluster momentum for dumping; must be finite and non-negative.
 * @param K             [1/s]  proportional gain on the stored momentum; must be finite and non-negative.
 * @param Ki            [1/s2] integral gain on the accumulated stored momentum; must be finite and non-negative.
 * @param integralLimit [Nms2] anti-windup clamp on each integral component; must be finite and non-negative,
 *                             and positive when Ki > 0.
 * @param controlPeriod [s]    integration step between update() calls; must be finite and non-negative,
 *                             and positive when Ki > 0 (only the integral term uses it).
 * @param dumpableProjection_B [-] projector onto the directions the effectors can dump about; must be a
 *                             finite, symmetric and idempotent orthogonal projector. Pass the identity when
 *                             every direction can be dumped.
 * @param GsMatrix_B        [-]     RW spin axes, three per wheel in row major order; every axis must be a
 *                                  unit vector.
 * @param JsList            [kgm2]  per-wheel spin-axis inertia.
 * @param wheelAvailability [-]     availability of each wheel, one byte per slot: 0 available, 1 unavailable.
 * Validate the configuration with validateConfig first; invalid input throws.
 */
void MomentumManagementAlgorithm_setConfig(MomentumManagementAlgorithmHandle* self,
                                           float hsMin,
                                           float K,
                                           float Ki,
                                           float integralLimit,
                                           float controlPeriod,
                                           const Matrix3f_c* dumpableProjection_B,
                                           const MomentumManagementRwSpinAxes_c* GsMatrix_B,
                                           const MomentumManagementRwInertias_c* JsList,
                                           const MomentumManagementRwAvailability_c* wheelAvailability);

/**
 * @brief Re-seed the runtime integrator state to its initial values.
 * @param self Pointer to the instance.
 */
void MomentumManagementAlgorithm_reInitialize(MomentumManagementAlgorithmHandle* self);

/**
 * @brief Assess the RW cluster momentum and compute the torque that dumps it.
 * Advances the integrator state, so the handle is non-const.
 * @param self        Pointer to the instance.
 * @param wheelSpeeds Pointer to the current reaction-wheel speeds.
 * @return Vector3f_c [Nm] the requested body-frame torque.
 */
Vector3f_c MomentumManagementAlgorithm_update(MomentumManagementAlgorithmHandle* self,
                                              const MomentumManagementWheelSpeeds_c* wheelSpeeds);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif /* F32XMERA_MOMENTUM_MANAGEMENT_ALGORITHM_C_H */
