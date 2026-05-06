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
            if (static_cast<uint32_t>(abs(steps)) >= this->minStepCommand) {
                output.commandType = StepperMotorCommandType::MOVE;
                output.stepsToMove = steps;
                this->commandedPosition = this->desiredPosition;
                this->state = StepperMotorState::MOVING;
            }
            break;
        }

        case StepperMotorState::MOVING: {
            // Desired position changed by at least the minimum commandable step delta
            if (static_cast<uint32_t>(abs(this->stepDelta(this->commandedPosition - this->desiredPosition))) >=
                this->minStepCommand) {
                output.commandType = StepperMotorCommandType::STOP;
                this->state = StepperMotorState::STOPPING;
            }
            // Move completed
            else if (!isMotorMoving) {
                this->state = StepperMotorState::SETTLING;
                this->settleCount = 0U;
            }
            break;
        }

        case StepperMotorState::STOPPING:
            if (!isMotorMoving) {
                this->state = StepperMotorState::SETTLING;
                this->settleCount = 0U;
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
void StepperMotorControllerAlgorithm::setSettleCountMax(const uint32_t settleCountMaxIn) {
    // NOLINTNEXTLINE(clang-diagnostic-tautological-unsigned-zero-compare)
    if (settleCountMaxIn < 0) {
        FSW_THROW_INVALID_ARGUMENT("settleCountMax must be non-negative");
    }
    this->settleCountMax = settleCountMaxIn;
}

/*! Getter for the maximum settling tick count.
 @return uint32_t
*/
uint32_t StepperMotorControllerAlgorithm::getSettleCountMax() const { return this->settleCountMax; }

/*! Setter for the minimum commandable step delta. A delta with magnitude greater than or equal to
 *  this value triggers a MOVE from IDLE or a STOP-and-replan from MOVING. Must be strictly positive
 *  so that zero-magnitude step deltas never trigger a command.
 @return void
 @param minStepCommandIn [steps] minimum step delta magnitude that warrants a command (must be > 0)
*/
void StepperMotorControllerAlgorithm::setMinStepCommand(const uint32_t minStepCommandIn) {
    if (minStepCommandIn <= 0U) {
        FSW_THROW_INVALID_ARGUMENT("minStepCommand must be greater than zero");
    }
    this->minStepCommand = minStepCommandIn;
}

/*! Getter for the minimum commandable step delta.
 @return uint32_t
*/
uint32_t StepperMotorControllerAlgorithm::getMinStepCommand() const { return this->minStepCommand; }
