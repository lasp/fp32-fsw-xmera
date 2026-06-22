#ifndef F32XMERA_STEPPER_MOTOR_CONTROLLER_TYPES_H
#define F32XMERA_STEPPER_MOTOR_CONTROLLER_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*! @brief Stepper motor controller state machine states */
typedef enum StepperMotorState { OFF = 0, IDLE, MOVING, STOPPING, SETTLING } StepperMotorState;

/*! @brief Type of command produced by the stepper motor controller */
typedef enum StepperMotorCommandType { NONE = 0, STOP, MOVE } StepperMotorCommandType;

/*! @brief Output from the stepper motor controller algorithm */
typedef struct StepperMotorControllerOutput {
    StepperMotorCommandType commandType; /*!< Type of command to issue */
    int32_t stepsToMove;                 /*!< [steps] Number of steps to move (only valid for MOVE command) */
} StepperMotorControllerOutput;

/*! @brief POD mirror of the C++ StepperMotorAngleRange (motor angular travel range [rad]). */
typedef struct {
    float minAngle; /*!< [rad] Lower bound of the motor travel range */
    float maxAngle; /*!< [rad] Upper bound of the motor travel range */
} MotorAngleRange_c;

/*!
 * @brief Plain-old-data mirror of the C++ StepperMotorControllerConfig.
 *
 * The C++ side validates each field via StepperMotorControllerConfig::create and throws on invalid input:
 * stepAngle must be in [2*pi/100000, 2*pi]; angleRange bounds must be in [-2*pi, 2*pi] with minAngle strictly less
 * than maxAngle; minStepCommand must be greater than zero (settleCountMax has no constraint).
 */
typedef struct {
    float stepAngle;              /*!< [rad/step] angle per motor step */
    MotorAngleRange_c angleRange; /*!< [rad] motor travel range */
    uint32_t settleCountMax;      /*!< [ticks] settling duration after stop */
    uint32_t minStepCommand;      /*!< [steps] minimum step delta magnitude that triggers a command (> 0) */
} StepperMotorControllerConfig_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_STEPPER_MOTOR_CONTROLLER_TYPES_H
