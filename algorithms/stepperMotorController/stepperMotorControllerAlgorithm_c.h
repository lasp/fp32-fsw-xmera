#ifndef F32XMERA_STEPPERMOTORCONTROLLERALGORITHM_C_H
#define F32XMERA_STEPPERMOTORCONTROLLERALGORITHM_C_H

#include "stepperMotorControllerTypes.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ StepperMotorControllerAlgorithm instance.
 */
typedef struct StepperMotorControllerAlgorithmHandle StepperMotorControllerAlgorithmHandle;

/**
 * @brief Construct a new StepperMotorControllerAlgorithm instance from the supplied configuration.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 * @return Pointer to a new StepperMotorControllerAlgorithm (must be destroyed).
 */
StepperMotorControllerAlgorithmHandle* StepperMotorControllerAlgorithm_create(
    const StepperMotorControllerConfig_c* config);

/**
 * @brief Destroy a previously created StepperMotorControllerAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void StepperMotorControllerAlgorithm_destroy(StepperMotorControllerAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime.
 * @param self   Pointer to the instance.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 */
void StepperMotorControllerAlgorithm_setConfig(StepperMotorControllerAlgorithmHandle* self,
                                               const StepperMotorControllerConfig_c* config);

/**
 * @brief Reset the algorithm state machine to IDLE and clear cached positions.
 * @param self Pointer to the instance.
 */
void StepperMotorControllerAlgorithm_reInitialize(StepperMotorControllerAlgorithmHandle* self);

/**
 * @brief Run one tick of the controller state machine.
 * @param self            Pointer to the instance.
 * @param currentPosition [steps] Current motor step position (tracked by the caller).
 * @param referenceAngle  [rad] Reference motor angle.
 * @param isMotorMoving   True if the motor is currently moving.
 * @return StepperMotorControllerOutput  Command type (NONE, MOVE, STOP) and step delta.
 */
StepperMotorControllerOutput StepperMotorControllerAlgorithm_update(StepperMotorControllerAlgorithmHandle* self,
                                                                    int32_t currentPosition,
                                                                    float referenceAngle,
                                                                    bool isMotorMoving);

/**
 * @brief Convert a reference angle to an integer step position using the configured stepAngle.
 * @param self  Pointer to the instance.
 * @param angle [rad] Reference angle.
 * @return int32_t  Step position rounded to the nearest integer.
 */
int32_t StepperMotorControllerAlgorithm_angleToSteps(const StepperMotorControllerAlgorithmHandle* self, float angle);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_STEPPERMOTORCONTROLLERALGORITHM_C_H
