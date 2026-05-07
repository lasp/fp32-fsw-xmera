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
typedef struct StepperMotorControllerAlgorithm StepperMotorControllerAlgorithm;

/**
 * @brief POD representation of the motor's angular travel range [rad].
 */
typedef struct {
    float minAngle; /*!< [rad] Lower bound of the motor travel range */
    float maxAngle; /*!< [rad] Upper bound of the motor travel range */
} MotorAngleRange_c;

/**
 * @brief Construct a new StepperMotorControllerAlgorithm instance.
 * @return Pointer to a new StepperMotorControllerAlgorithm (must be destroyed).
 */
StepperMotorControllerAlgorithm* StepperMotorControllerAlgorithm_create(void);

/**
 * @brief Destroy a previously created StepperMotorControllerAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void StepperMotorControllerAlgorithm_destroy(StepperMotorControllerAlgorithm* self);

/**
 * @brief Reset the algorithm state machine to IDLE and clear cached positions.
 * @param self Pointer to the instance.
 */
void StepperMotorControllerAlgorithm_reset(StepperMotorControllerAlgorithm* self);

/**
 * @brief Run one tick of the controller state machine.
 * @param self            Pointer to the instance.
 * @param currentPosition [steps] Current motor step position (tracked by the caller).
 * @param referenceAngle  [rad] Reference motor angle.
 * @param isMotorMoving   True if the motor is currently moving.
 * @return StepperMotorControllerOutput  Command type (NONE, MOVE, STOP) and step delta.
 */
StepperMotorControllerOutput StepperMotorControllerAlgorithm_update(StepperMotorControllerAlgorithm* self,
                                                                    int32_t currentPosition,
                                                                    float referenceAngle,
                                                                    bool isMotorMoving);

/**
 * @brief Convert a reference angle to an integer step position using the configured stepAngle.
 * @param self  Pointer to the instance.
 * @param angle [rad] Reference angle.
 * @return int32_t  Step position rounded to the nearest integer.
 */
int32_t StepperMotorControllerAlgorithm_angleToSteps(const StepperMotorControllerAlgorithm* self, float angle);

/**
 * @brief Set the angle traversed per motor step.
 * @param self      Pointer to the instance.
 * @param stepAngle [rad/step] Motor step angle, must be in [2*pi/100000, 2*pi].
 */
void StepperMotorControllerAlgorithm_setStepAngle(StepperMotorControllerAlgorithm* self, float stepAngle);

/**
 * @brief Get the angle traversed per motor step.
 * @param self Pointer to the instance.
 * @return float  Motor step angle [rad/step].
 */
float StepperMotorControllerAlgorithm_getStepAngle(const StepperMotorControllerAlgorithm* self);

/**
 * @brief Set the motor's angular travel range. Reference angles outside this range are rejected
 *        for partial ranges; full-circle ranges accept any angle and wrap via shortest path.
 * @param self     Pointer to the instance.
 * @param minAngle [rad] Lower bound, must be in [-2*pi, 2*pi].
 * @param maxAngle [rad] Upper bound, must be in [-2*pi, 2*pi] and strictly greater than minAngle.
 */
void StepperMotorControllerAlgorithm_setMotorAngleRange(StepperMotorControllerAlgorithm* self,
                                                        float minAngle,
                                                        float maxAngle);

/**
 * @brief Get the motor's angular travel range.
 * @param self Pointer to the instance.
 * @return MotorAngleRange_c  {minAngle, maxAngle} in radians.
 */
MotorAngleRange_c StepperMotorControllerAlgorithm_getMotorAngleRange(const StepperMotorControllerAlgorithm* self);

/**
 * @brief Set the maximum settling tick count.
 * @param self           Pointer to the instance.
 * @param settleCountMax [ticks] Number of ticks to wait during settling.
 */
void StepperMotorControllerAlgorithm_setSettleCountMax(StepperMotorControllerAlgorithm* self, uint32_t settleCountMax);

/**
 * @brief Get the maximum settling tick count.
 * @param self Pointer to the instance.
 * @return uint32_t  Settling duration [ticks].
 */
uint32_t StepperMotorControllerAlgorithm_getSettleCountMax(const StepperMotorControllerAlgorithm* self);

/**
 * @brief Set the minimum step delta magnitude that triggers a MOVE (from IDLE) or a STOP-and-replan
 *        (from MOVING).
 * @param self           Pointer to the instance.
 * @param minStepCommand [steps] Minimum step delta magnitude that warrants a command (must be > 0).
 */
void StepperMotorControllerAlgorithm_setMinStepCommand(StepperMotorControllerAlgorithm* self, uint32_t minStepCommand);

/**
 * @brief Get the minimum commandable step delta.
 * @param self Pointer to the instance.
 * @return uint32_t  Minimum step delta magnitude [steps].
 */
uint32_t StepperMotorControllerAlgorithm_getMinStepCommand(const StepperMotorControllerAlgorithm* self);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_STEPPERMOTORCONTROLLERALGORITHM_C_H
