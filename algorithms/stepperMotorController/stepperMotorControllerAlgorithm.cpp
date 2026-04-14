#include "stepperMotorControllerAlgorithm.h"
#include "utilities/freestandingInvalidArgument.h"
#include <math.h>

#include <numbers>

/*! Reset the controller to its initial state.
 @return void
*/
void StepperMotorControllerAlgorithm::reset() {
    this->state = StepperMotorState::IDLE;
    this->commandedPosition = 0;
    this->desiredPosition = 0;
    this->settleCount = 0;
}

/*! State-machine update.
 @return StepperMotorControllerOutput stepper motor command (NONE, MOVE, STOP) and commanded steps
 @param currentPosition [steps] current motor position, tracked by the caller
 @param referenceAngle [rad] reference angle to go to (desired angle)
 @param isMotorMoving indicator whether motor is moving
*/
StepperMotorControllerOutput StepperMotorControllerAlgorithm::update(
    const int currentPosition,  // NOLINT(bugprone-easily-swappable-parameters)
    const float referenceAngle,
    const bool isMotorMoving) {
    StepperMotorControllerOutput output{};

    this->desiredPosition = this->angleToSteps(referenceAngle);

    switch (this->state) {
        case StepperMotorState::OFF:
            break;

        case StepperMotorState::MOVING: {
            // Move completed (within current-position tolerance of commanded target)
            if (abs(this->wrapDelta(this->commandedPosition - currentPosition)) <= this->currentPositionTolerance) {
                this->state = StepperMotorState::STOPPING;
            }
            // Desired position changed beyond desired-position tolerance
            if (abs(this->wrapDelta(this->commandedPosition - this->desiredPosition)) >
                this->desiredPositionTolerance) {
                output.commandType = StepperMotorCommandType::STOP;
                this->state = StepperMotorState::STOPPING;
            }

            break;
        }

        case StepperMotorState::STOPPING:
            if (!isMotorMoving) {
                this->state = StepperMotorState::SETTLING;
                this->settleCount = 0;
            }
            break;

        case StepperMotorState::SETTLING:
            if (this->settleCount >= this->settleCountMax) {
                this->state = StepperMotorState::IDLE;
            } else {
                this->settleCount++;
            }
            break;

        case StepperMotorState::IDLE: {
            const int steps = this->wrapDelta(this->desiredPosition - currentPosition);
            if (abs(steps) > this->currentPositionTolerance) {
                output.commandType = StepperMotorCommandType::MOVE;
                output.stepsToMove = steps;
                this->commandedPosition = this->desiredPosition;
                this->state = StepperMotorState::MOVING;
            }
            break;
        }
    }

    return output;
}

/*! Convert a reference angle to an integer step position.
 @return int step position
 @param angle [rad] reference angle
*/
int StepperMotorControllerAlgorithm::angleToSteps(const float angle) const {
    constexpr float twoPi = 2.0F * static_cast<float>(std::numbers::pi);
    return static_cast<int>(round(angle * static_cast<float>(this->stepsPerRevolution) / twoPi));
}

/*! Wrap a step delta to the shortest path within [-stepsPerRevolution/2, +stepsPerRevolution/2].
 @return int wrapped delta
 @param delta [steps] unwrapped step difference
*/
int StepperMotorControllerAlgorithm::wrapDelta(int delta) const {
    const int half = this->stepsPerRevolution / 2;
    delta = delta % this->stepsPerRevolution;
    if (delta > half) {
        delta -= this->stepsPerRevolution;
    }
    if (delta < -half) {
        delta += this->stepsPerRevolution;
    }
    return delta;
}

/*! Setter for the number of motor steps per revolution.
 @return void
 @param stepsPerRevolutionIn [steps] steps per full revolution
*/
void StepperMotorControllerAlgorithm::setStepsPerRevolution(const int stepsPerRevolutionIn) {
    if (stepsPerRevolutionIn <= 0) {
        FSW_THROW_INVALID_ARGUMENT("stepsPerRevolution must be positive");
    }
    this->stepsPerRevolution = stepsPerRevolutionIn;
}

/*! Getter for the number of motor steps per revolution.
 @return int
*/
int StepperMotorControllerAlgorithm::getStepsPerRevolution() const { return this->stepsPerRevolution; }

/*! Setter for the maximum settling tick count.
 @return void
 @param settleCountMaxIn [ticks] number of ticks to wait during settling
*/
void StepperMotorControllerAlgorithm::setSettleCountMax(const int settleCountMaxIn) {
    if (settleCountMaxIn < 0) {
        FSW_THROW_INVALID_ARGUMENT("settleCountMax must be non-negative");
    }
    this->settleCountMax = settleCountMaxIn;
}

/*! Getter for the maximum settling tick count.
 @return int
*/
int StepperMotorControllerAlgorithm::getSettleCountMax() const { return this->settleCountMax; }

/*! Setter for the current-position tolerance (used for IDLE move trigger and MOVING stop condition).
 @return void
 @param currentPositionToleranceIn [steps] tolerance between current position and target
*/
void StepperMotorControllerAlgorithm::setCurrentPositionTolerance(const int currentPositionToleranceIn) {
    if (currentPositionToleranceIn < 0) {
        FSW_THROW_INVALID_ARGUMENT("currentPositionTolerance must be non-negative");
    }
    this->currentPositionTolerance = currentPositionToleranceIn;
}

/*! Getter for the current-position tolerance.
 @return int
*/
int StepperMotorControllerAlgorithm::getCurrentPositionTolerance() const { return this->currentPositionTolerance; }

/*! Setter for the desired-position tolerance (used in MOVING state to detect a changed reference).
 @return void
 @param desiredPositionToleranceIn [steps] tolerance between commanded and desired position
*/
void StepperMotorControllerAlgorithm::setDesiredPositionTolerance(const int desiredPositionToleranceIn) {
    if (desiredPositionToleranceIn < 0) {
        FSW_THROW_INVALID_ARGUMENT("desiredPositionTolerance must be non-negative");
    }
    this->desiredPositionTolerance = desiredPositionToleranceIn;
}

/*! Getter for the desired-position tolerance.
 @return int
*/
int StepperMotorControllerAlgorithm::getDesiredPositionTolerance() const { return this->desiredPositionTolerance; }
