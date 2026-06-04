#ifndef F32XMERA_RW_MOTOR_TORQUE_ALGORITHM_C_H
#define F32XMERA_RW_MOTOR_TORQUE_ALGORITHM_C_H

#include "rwMotorTorqueTypes.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ RwMotorTorqueAlgorithm instance.
 */
typedef struct RwMotorTorqueAlgorithmHandle RwMotorTorqueAlgorithmHandle;

/**
 * @brief Get the RW_MOTOR_TORQUE_MAX_NUM_RW constant for Ada validation.
 * @return The maximum number of reaction wheels handled at the C boundary.
 */
uint32_t RwMotorTorqueAlgorithm_getMaxNumRw(void);

/**
 * @brief Construct a new RwMotorTorqueAlgorithm instance from the supplied configuration.
 *
 * The configuration carries the control axes, the reaction-wheel spin-axis configuration, the
 * per-wheel availability, and the null-space despin gain; the RW motor torque mapping and the
 * null-space projection are computed during construction.
 *
 * @param config Pointer to the configuration to apply (validated; throws on invalid input or if the
 *               resulting control mapping matrix is not full rank).
 * @return Pointer to a new RwMotorTorqueAlgorithm (must be destroyed).
 */
RwMotorTorqueAlgorithmHandle* RwMotorTorqueAlgorithm_create(const RwMotorTorqueConfig_c* config);

/**
 * @brief Destroy a previously created RwMotorTorqueAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void RwMotorTorqueAlgorithm_destroy(RwMotorTorqueAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime.
 * @param self   Pointer to the instance.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 */
void RwMotorTorqueAlgorithm_setConfig(RwMotorTorqueAlgorithmHandle* self, const RwMotorTorqueConfig_c* config);

/**
 * @brief Compute the per-wheel motor torques (control mapping + null-space despin) for a body torque.
 * @param self            Pointer to the instance.
 * @param Lr_B            Commanded control torque on the spacecraft, body frame.
 * @param rwSpeeds        Current RW speeds (pass zero-filled to disable despin).
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
