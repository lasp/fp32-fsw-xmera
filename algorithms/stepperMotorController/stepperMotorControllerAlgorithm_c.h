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
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param stepAngle      [rad/step] Angle per motor step.
 * @param minAngle       [rad] Lower bound of the motor travel range.
 * @param maxAngle       [rad] Upper bound of the motor travel range.
 * @param settleCountMax [ticks] Settling duration after stop.
 * @param minStepCommand [steps] Minimum step delta magnitude that triggers a command.
 * @return True if the configuration is valid. Never throws, so it can guard the throwing
 *         create/setConfig from an invalid configuration.
 * @note The accepted value ranges are defined by StepperMotorControllerConfig::create; this
 *       predicate reports whether a candidate set would be accepted, without throwing.
 */
bool StepperMotorControllerAlgorithm_validateConfig(float stepAngle,
                                                    float minAngle,
                                                    float maxAngle,
                                                    uint32_t settleCountMax,
                                                    uint32_t minStepCommand);

/**
 * @brief Construct a new StepperMotorControllerAlgorithm instance from the supplied configuration.
 * Validate the values with validateConfig before calling; throws on invalid input.
 * @param stepAngle      [rad/step] Angle per motor step.
 * @param minAngle       [rad] Lower bound of the motor travel range.
 * @param maxAngle       [rad] Upper bound of the motor travel range.
 * @param settleCountMax [ticks] Settling duration after stop.
 * @param minStepCommand [steps] Minimum step delta magnitude that triggers a command.
 * @return Pointer to a new StepperMotorControllerAlgorithm (must be destroyed).
 */
StepperMotorControllerAlgorithmHandle* StepperMotorControllerAlgorithm_create(float stepAngle,
                                                                              float minAngle,
                                                                              float maxAngle,
                                                                              uint32_t settleCountMax,
                                                                              uint32_t minStepCommand);

/**
 * @brief Destroy a previously created StepperMotorControllerAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void StepperMotorControllerAlgorithm_destroy(StepperMotorControllerAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime.
 * Validate the values with validateConfig before calling; throws on invalid input.
 * @param self           Pointer to the instance.
 * @param stepAngle      [rad/step] Angle per motor step.
 * @param minAngle       [rad] Lower bound of the motor travel range.
 * @param maxAngle       [rad] Upper bound of the motor travel range.
 * @param settleCountMax [ticks] Settling duration after stop.
 * @param minStepCommand [steps] Minimum step delta magnitude that triggers a command.
 */
void StepperMotorControllerAlgorithm_setConfig(StepperMotorControllerAlgorithmHandle* self,
                                               float stepAngle,
                                               float minAngle,
                                               float maxAngle,
                                               uint32_t settleCountMax,
                                               uint32_t minStepCommand);

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
 * @return StepperMotorControllerOutput  Command type (NONE, STOP, MOVE) and step delta.
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
