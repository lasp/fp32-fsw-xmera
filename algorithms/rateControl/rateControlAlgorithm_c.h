/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

#ifndef F32XIMERA_RATE_CONTROL_ALGORITHM_C_H
#define F32XIMERA_RATE_CONTROL_ALGORITHM_C_H

#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include "msgPayloadDef/VehicleConfigMsgF32Payload.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ RateControlAlgorithm instance.
 */
typedef struct RateControlAlgorithm RateControlAlgorithm;

/**
 * @brief POD representation of a 3-vector (Eigen::Vector3f).
 */
typedef struct {
    float data[3];
} Vector3f_c;

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
 * @param self     Pointer to the instance.
 * @param attGuidIn Pointer to the attitude guidance input.
 * @return CmdTorqueBodyMsgF32Payload The computed torque command.
 */
CmdTorqueBodyMsgF32Payload RateControlAlgorithm_update(const RateControlAlgorithm* self,
                                                       const AttGuidMsgF32Payload* attGuidIn);

/**
 * @brief Set the spacecraft inertia configuration.
 * @param self Pointer to the instance.
 * @param vehicleConfigIn Pointer to the vehicle configuration payload.
 */
void RateControlAlgorithm_setSpacecraftInertia(RateControlAlgorithm* self,
                                               const VehicleConfigMsgF32Payload* vehicleConfigIn);

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
