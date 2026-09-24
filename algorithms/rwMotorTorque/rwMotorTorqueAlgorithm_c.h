#ifndef F32XMERA_RW_MOTOR_TORQUE_ALGORITHM_C_H
#define F32XMERA_RW_MOTOR_TORQUE_ALGORITHM_C_H

#include "rwMotorTorqueTypes.h"
#include "utilities/fsw/deviceAvailability.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ RwMotorTorqueAlgorithm instance.
 */
typedef struct RwMotorTorqueAlgorithmHandle RwMotorTorqueAlgorithmHandle;

/**
 * @brief Control body axis selection, so the bound is part of the type at the C boundary.
 */
typedef struct {
    uint8_t axis[3]; /*!< [-] x, y, z; nonzero selects the axis */
} RwMotorTorqueControlAxes_c;

/**
 * @brief RW spin axes in body frame, three components per wheel in row major order.
 */
typedef struct {
    float data[3 * RW_EFF_CNT]; /*!< [-] three components per wheel */
} RwMotorTorqueRwSpinAxes_c;

/**
 * @brief Availability of each wheel slot, one entry per slot: 0 available, 1 unavailable.
 */
typedef struct {
    DeviceAvailability_c availability[RW_EFF_CNT]; /*!< [-] one entry per wheel */
} RwMotorTorqueRwAvailability_c;

/**
 * @brief Get the kMaxNumRw constant for Ada validation.
 * @return The maximum number of reaction wheels handled at the C boundary.
 */
uint32_t RwMotorTorqueAlgorithm_getMaxNumRw(void);

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param desiredControlAxes_B [-] control body axis selection (x, y, z); nonzero selects the axis, and a
 * @param rwConfiguration      [-] reaction-wheel spin axes and per-wheel availability. Every slot is
 * @param omegaGain            [-] RW null-space feedback gain; must be finite and non-negative.
 * @return true when the configuration is valid. Never throws, so it can guard the throwing
 *         create/setConfig from an invalid configuration, including a control mapping matrix that is
 *         not full rank.
 */
bool RwMotorTorqueAlgorithm_validateConfig(const RwMotorTorqueControlAxes_c* desiredControlAxes_B,
                                           const RwMotorTorqueRwSpinAxes_c* GsMatrix_B,
                                           const RwMotorTorqueRwAvailability_c* wheelAvailability,
                                           float omegaGain);

/**
 * @brief Construct a new RwMotorTorqueAlgorithm instance from the supplied configuration.
 *
 * The RW motor torque mapping and the null-space projection are computed during construction.
 * Validate the values with validateConfig first; invalid input throws, as does a control mapping
 * matrix that is not full rank.
 * @param desiredControlAxes_B [-] control body axis selection (x, y, z); nonzero selects the axis, and a
 *                             minimum of one must be selected.
 * @param GsMatrix_B           [-] RW spin axes, three per wheel in row major order. Every slot is
 *                             configured, and each spin axis must be a unit vector.
 * @param wheelAvailability    [-] availability of each wheel, one entry per slot: 0 available, 1 unavailable.
 *                             A slot carrying no wheel is marked unavailable.
 * @param omegaGain            [-] RW null-space feedback gain; must be finite and non-negative.
 * @return Pointer to a new RwMotorTorqueAlgorithm (must be destroyed).
 */
RwMotorTorqueAlgorithmHandle* RwMotorTorqueAlgorithm_create(const RwMotorTorqueControlAxes_c* desiredControlAxes_B,
                                                            const RwMotorTorqueRwSpinAxes_c* GsMatrix_B,
                                                            const RwMotorTorqueRwAvailability_c* wheelAvailability,
                                                            float omegaGain);

/**
 * @brief Destroy a previously created RwMotorTorqueAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void RwMotorTorqueAlgorithm_destroy(RwMotorTorqueAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime and recompute the mapping.
 *
 * Validate the values with validateConfig first; invalid input throws.
 * @param self                 Pointer to the instance.
 * @param desiredControlAxes_B [-] control body axis selection (x, y, z); nonzero selects the axis, and a
 *                             minimum of one must be selected.
 * @param GsMatrix_B           [-] RW spin axes, three per wheel in row major order. Every slot is
 *                             configured, and each spin axis must be a unit vector.
 * @param wheelAvailability    [-] availability of each wheel, one entry per slot: 0 available, 1 unavailable.
 *                             A slot carrying no wheel is marked unavailable.
 * @param omegaGain            [-] RW null-space feedback gain; must be finite and non-negative.
 */
void RwMotorTorqueAlgorithm_setConfig(RwMotorTorqueAlgorithmHandle* self,
                                      const RwMotorTorqueControlAxes_c* desiredControlAxes_B,
                                      const RwMotorTorqueRwSpinAxes_c* GsMatrix_B,
                                      const RwMotorTorqueRwAvailability_c* wheelAvailability,
                                      float omegaGain);

/**
 * @brief Compute the per-wheel motor torques (control mapping + null-space) for a body torque.
 * @param self            Pointer to the instance.
 * @param Lr_B            Commanded control torque on the spacecraft, body frame.
 * @param rwSpeeds        Current RW speeds (pass zero-filled to disable null-space).
 * @param rwDesiredSpeeds Desired RW speeds.
 * @return The per-wheel commanded motor torques.
 */
RwMotorTorqueOutput_c RwMotorTorqueAlgorithm_update(const RwMotorTorqueAlgorithmHandle* self,
                                                    Vector3f_c Lr_B,
                                                    const RwSpeeds_c* rwSpeeds,
                                                    const RwSpeeds_c* rwDesiredSpeeds);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif /* F32XMERA_RW_MOTOR_TORQUE_ALGORITHM_C_H */
