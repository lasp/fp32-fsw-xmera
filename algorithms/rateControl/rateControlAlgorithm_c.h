#ifndef F32XIMERA_RATE_CONTROL_ALGORITHM_C_H
#define F32XIMERA_RATE_CONTROL_ALGORITHM_C_H

#include "utilities/plainCAlgorithmDataTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ RateControlAlgorithm instance.
 */
typedef struct RateControlAlgorithm RateControlAlgorithm;

/**
 * @brief Construct a new RateControlAlgorithm instance.
 * @return Pointer to a new RateControlAlgorithm (must be destroyed).
 */
RateControlAlgorithm* RateControlAlgorithm_create(void);

/**
 * @brief Destroy a previously created RateControlAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void RateControlAlgorithm_destroy(RateControlAlgorithm* self);

/**
 * @brief Run the rate control update step.
 * @param self         Pointer to the instance.
 * @param omega_BR_B   Angular velocity of body relative to reference frame in body frame components [rad/s].
 * @param domega_RN_B  Time derivative of reference frame angular velocity in body frame components [rad/s^2].
 * @return Eigen::Vector3f Required control torque about point B [Nm].
 */
Vector3f_c RateControlAlgorithm_update(const RateControlAlgorithm* self,
                                       const Vector3f_c& omega_BR_B,
                                       const Vector3f_c& domega_RN_B);

/**
 * @brief Set the spacecraft inertia configuration.
 * @param self Pointer to the instance.
 * @param vehicleConfigIn Pointer to the vehicle configuration payload.
 */
void RateControlAlgorithm_setSpacecraftInertia(RateControlAlgorithm* self, const Matrix3f_c& spacecraftInertia);

/**
 * @brief Set the derivative gain P.
 * @param self Pointer to the instance.
 * @param P Derivative gain value.
 */
void RateControlAlgorithm_setDerivativeGainP(RateControlAlgorithm* self, float P);

/**
 * @brief Get the derivative gain P.
 * @param self Pointer to the instance.
 * @return float The current derivative gain value.
 */
float RateControlAlgorithm_getDerivativeGainP(const RateControlAlgorithm* self);

/**
 * @brief Set the known external torque in body frame components.
 * @param self Pointer to the instance.
 * @param knownTorquePntB_B POD representation of the known torque vector.
 */
void RateControlAlgorithm_setKnownTorquePntB_B(RateControlAlgorithm* self, Vector3f_c knownTorquePntB_B);

/**
 * @brief Get the known external torque in body frame components.
 * @param self Pointer to the instance.
 * @return Vector3f_c POD representation of the known torque vector.
 */
Vector3f_c RateControlAlgorithm_getKnownTorquePntB_B(const RateControlAlgorithm* self);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XIMERA_RATE_CONTROL_ALGORITHM_C_H
