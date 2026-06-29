#include "stepperMotorControllerAlgorithm.h"

#include <math.h>
#include <stdlib.h>
#include <numbers>
#include <utility>

/*! @brief Construct the controller with a validated configuration. The state machine starts in IDLE with cleared
 cached positions. */
StepperMotorControllerAlgorithm::StepperMotorControllerAlgorithm(const StepperMotorControllerConfig& config)
    : cfg(config) {
    setConfig(config);
    reInitialize();
}

/*! @brief Replace the stored configuration at runtime. The state machine and cached positions are preserved. */
void StepperMotorControllerAlgorithm::setConfig(const StepperMotorControllerConfig& config) {
    this->cfg = config;
    this->cacheDerivedValues();
}

/*! Recompute the cached values derived from the configuration: the integer steps per revolution and the
 full-circle flag.
 @return void
*/
void StepperMotorControllerAlgorithm::cacheDerivedValues() {
    constexpr float twoPi = 2.0F * std::numbers::pi_v<float>;
    this->stepsPerRev = static_cast<int>(round(twoPi / this->cfg.getStepAngle()));
    const StepperMotorAngleRange& angleRange = this->cfg.getAngleRange();
    this->isFullCircle =
        (angleRange.maxAngle - angleRange.minAngle) >= (twoPi - StepperMotorControllerConfig::kMinStepAngle);
}

/*! Reset the controller to its initial state.
 @return void
*/
void StepperMotorControllerAlgorithm::reInitialize() {
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
    const StepperMotorAngleRange& angleRange = this->cfg.getAngleRange();

    StepperMotorControllerOutput output{};

    // For a full-circle range, any reference angle is acceptable — wrap-around math handles the
    // shortest-path conversion. For a partial range, reject references outside [minAngle, maxAngle]:
    // leave desiredPosition unchanged so the state machine ignores them, keeping the motor
    // quiescent rather than driving it across the forbidden seam.
    const bool isReferenceInRange =
        this->isFullCircle || (referenceAngle >= angleRange.minAngle && referenceAngle <= angleRange.maxAngle);
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
            if (std::cmp_greater_equal(abs(steps), this->cfg.getMinStepCommand())) {
                output.commandType = StepperMotorCommandType::MOVE;
                output.stepsToMove = steps;
                this->commandedPosition = this->desiredPosition;
                this->state = StepperMotorState::MOVING;
            }
            break;
        }

        case StepperMotorState::MOVING: {
            // Desired position changed by at least the minimum commandable step delta
            if (std::cmp_greater_equal(abs(this->stepDelta(this->commandedPosition - this->desiredPosition)),
                                       this->cfg.getMinStepCommand())) {
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
            if (this->settleCount >= this->cfg.getSettleCountMax()) {
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
    return static_cast<int>(round(angle / this->cfg.getStepAngle()));
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
