#ifndef F32XMERA_RW_MOTOR_TORQUE_ALGORITHM_C_H
#define F32XMERA_RW_MOTOR_TORQUE_ALGORITHM_C_H

#include "rwMotorTorqueTypes.h"

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
 * @brief Get the RW_MOTOR_TORQUE_MAX_NUM_RW constant for Ada validation.
 * @return The maximum number of reaction wheels handled at the C boundary.
 */
uint32_t RwMotorTorqueAlgorithm_getMaxNumRw(void);

/**
 * @brief Construct a new RwMotorTorqueAlgorithm instance from the supplied configuration.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
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
 * @brief Configure the algorithm: snapshot the RW spin-axis configuration and availability, and
 *        build the control mapping matrix.
 * @param self         Pointer to the instance.
 * @param rwConfig     Reaction-wheel spin-axis configuration in body-frame components.
 * @param availability Per-wheel availability (set every wheel AVAILABLE to use all of them).
 */
void RwMotorTorqueAlgorithm_configure(RwMotorTorqueAlgorithmHandle* self,
                                      const RwMotorTorqueArrayConfig_c* rwConfig,
                                      const RwMotorTorqueAvailability_c* availability);

/**
 * @brief Compute the reaction wheel motor torques for a commanded body torque.
 * @param self Pointer to the instance.
 * @param Lr_B Total commanded control torque on the spacecraft in body-frame components.
 * @return RwMotorTorqueOutput_c  The per-wheel commanded motor torques.
 */
RwMotorTorqueOutput_c RwMotorTorqueAlgorithm_update(const RwMotorTorqueAlgorithmHandle* self, Vector3f_c Lr_B);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif /* F32XMERA_RW_MOTOR_TORQUE_ALGORITHM_C_H */
