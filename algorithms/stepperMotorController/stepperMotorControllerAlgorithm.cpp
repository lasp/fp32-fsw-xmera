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

    // For a full-circle range, any reference angle is acceptable — wrap-around math handles the
    // shortest-path conversion. For a partial range, reject references outside [minAngle, maxAngle]:
    // leave desiredPosition unchanged so the state machine ignores them, keeping the motor
    // quiescent rather than driving it across the forbidden seam.
    const bool isReferenceInRange =
        this->isFullCircle || (referenceAngle >= this->minAngle && referenceAngle <= this->maxAngle);
    if (isReferenceInRange) {
        this->desiredPosition = this->angleToSteps(referenceAngle);
    }

    switch (this->state) {
        case StepperMotorState::OFF:
            break;

        case StepperMotorState::IDLE: {
            if (!isReferenceInRange) {
                break;
            }
            const int steps = this->stepDelta(this->desiredPosition - currentPosition);
            if (abs(steps) > this->currentPositionTolerance) {
                output.commandType = StepperMotorCommandType::MOVE;
                output.stepsToMove = steps;
                this->commandedPosition = this->desiredPosition;
                this->state = StepperMotorState::MOVING;
            }
            break;
        }

        case StepperMotorState::MOVING: {
            // Desired position changed beyond desired-position tolerance
            if (abs(this->stepDelta(this->commandedPosition - this->desiredPosition)) >
                this->desiredPositionTolerance) {
                output.commandType = StepperMotorCommandType::STOP;
                this->state = StepperMotorState::STOPPING;
            }
            // Move completed (within current-position tolerance of commanded target)
            else if (abs(this->stepDelta(this->commandedPosition - currentPosition)) <=
                     this->currentPositionTolerance) {
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
    }

    return output;
}

/*! Convert a reference angle to an integer step position.
 @return int step position
 @param angle [rad] reference angle
*/
int StepperMotorControllerAlgorithm::angleToSteps(const float angle) const {
    return static_cast<int>(round(angle / this->stepAngle));
}

/*! Wrap a step delta to the shortest path within [-stepsPerRev/2, +stepsPerRev/2].
 @return int wrapped delta
 @param delta [steps] unwrapped step difference
*/
int StepperMotorControllerAlgorithm::wrapDelta(int delta) const {
    const int half = this->stepsPerRev / 2;
    delta = delta % this->stepsPerRev;
    if (delta > half) {
        delta -= this->stepsPerRev;
    }
    if (delta < -half) {
        delta += this->stepsPerRev;
    }
    return delta;
}

/*! Pick the path delta based on the configured motor range. For a full-circle range, return the
 *  shortest-path wrapped delta. For a partial range, return the linear delta unchanged so the
 *  commanded path stays within the bounded travel.
 @return int path delta [steps]
 @param delta [steps] linear (unwrapped) step difference
*/
int StepperMotorControllerAlgorithm::stepDelta(const int delta) const {
    return this->isFullCircle ? this->wrapDelta(delta) : delta;
}

/*! Setter for the angle per motor step.
 @return void
 @param stepAngleIn [rad/step] motor step angle, must be in [kMinStepAngle, 2*pi]
*/
void StepperMotorControllerAlgorithm::setStepAngle(const float stepAngleIn) {
    constexpr float twoPi = 2.0F * std::numbers::pi_v<float>;
    if (stepAngleIn < StepperMotorControllerAlgorithm::kMinStepAngle || stepAngleIn > twoPi) {
        FSW_THROW_INVALID_ARGUMENT("stepAngle must be in [2*pi/kMaxStepsPerRev, 2*pi]");
    }
    this->stepAngle = stepAngleIn;
    this->stepsPerRev = static_cast<int>(round(twoPi / stepAngleIn));
}

/*! Getter for the angle per motor step.
 @return float [rad/step]
*/
float StepperMotorControllerAlgorithm::getStepAngle() const { return this->stepAngle; }

/*! Setter for the motor travel range
 @return void
 @param minAngleIn [rad] lower bound, must be in [-2*pi, 2*pi]
 @param maxAngleIn [rad] upper bound, must be in [-2*pi, 2*pi] and strictly greater than minAngleIn
*/
void StepperMotorControllerAlgorithm::setMotorAngleRange(const float minAngleIn, const float maxAngleIn) {
    constexpr float twoPi = 2.0F * std::numbers::pi_v<float>;
    if (minAngleIn < -twoPi || minAngleIn > twoPi) {
        FSW_THROW_INVALID_ARGUMENT("minAngle must be in [-2*pi, 2*pi]");
    }
    if (maxAngleIn < -twoPi || maxAngleIn > twoPi) {
        FSW_THROW_INVALID_ARGUMENT("maxAngle must be in [-2*pi, 2*pi]");
    }
    if (minAngleIn >= maxAngleIn) {
        FSW_THROW_INVALID_ARGUMENT("minAngle must be strictly less than maxAngle");
    }
    this->minAngle = minAngleIn;
    this->maxAngle = maxAngleIn;
    this->isFullCircle = ((maxAngleIn - minAngleIn) >= (twoPi - StepperMotorControllerAlgorithm::kMinStepAngle));
}

/*! Getter for the motor travel range.
 @return std::array<float, 2> {minAngle, maxAngle} in radians
*/
std::array<float, 2> StepperMotorControllerAlgorithm::getMotorAngleRange() const {
    return {this->minAngle, this->maxAngle};
}

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
