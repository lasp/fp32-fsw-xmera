#ifndef F32XIMERA_STEPPER_MOTOR_CONTROLLER_ALGORITHM_H
#define F32XIMERA_STEPPER_MOTOR_CONTROLLER_ALGORITHM_H

#include "stepperMotorControllerTypes.h"
#include <cstdint>

/*! @brief Stepper Motor Controller Algorithm
 *
 * State-machine-based controller that commands a stepper motor to a desired position.
 * Operates in integer step counts internally; reference angles are converted using stepAngle.
 * The current motor position is an input to update(); position tracking lives in the caller.
 */
class StepperMotorControllerAlgorithm {
   public:
    void reset();
    StepperMotorControllerOutput update(int currentPosition, float referenceAngle, bool isMotorMoving);

    void setStepsPerRevolution(int stepsPerRevolutionIn);
    int getStepsPerRevolution() const;
    void setSettleCountMax(int settleCountMaxIn);
    int getSettleCountMax() const;
    void setCurrentPositionTolerance(int currentPositionToleranceIn);
    int getCurrentPositionTolerance() const;
    void setDesiredPositionTolerance(int desiredPositionToleranceIn);
    int getDesiredPositionTolerance() const;

    int angleToSteps(float angle) const;

   private:
    int wrapDelta(int delta) const;

    // Parameters
    int stepsPerRevolution{360};      //!< [steps] Number of motor steps per full revolution
    int settleCountMax{10};           //!< [ticks] Settling duration after stop
    int currentPositionTolerance{1};  //!< [steps] Tolerance between current and target position (IDLE/STOPPING checks)
    int desiredPositionTolerance{
        0};  //!< [steps] Tolerance between commanded and desired position (MOVING interrupt check)

    // State
    StepperMotorState state{StepperMotorState::IDLE};
    int commandedPosition{};  //!< [steps] Target of the current move command
    int desiredPosition{};    //!< [steps] Desired position from reference angle
    int settleCount{};        //!< [ticks] Counter for settling phase
};

#endif
