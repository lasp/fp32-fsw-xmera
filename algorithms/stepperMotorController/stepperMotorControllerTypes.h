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

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_STEPPER_MOTOR_CONTROLLER_TYPES_H
