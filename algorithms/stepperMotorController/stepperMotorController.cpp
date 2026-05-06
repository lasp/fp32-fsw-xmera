#include "stepperMotorController.h"
#include <math.h>
#include <stdexcept>

/*! Reset the module. Checks that the input message is linked.
 @return void
 @param callTime [ns] Time the method is called
*/
void StepperMotorController::reset(const uint64_t callTime) {
    if (!this->motorRefAngleInMsg.isLinked()) {
        throw std::invalid_argument("StepperMotorController.motorRefAngleInMsg wasn't connected.");
    }

    const int initialStep = this->algorithm.angleToSteps(this->initialAngle);
    this->currentPosition = initialStep;
    this->commandedPosition = initialStep;
    this->stepAccumulator = 0.0F;
    this->algorithm.reset();
}

/*! Read input messages, run the state machine, and write output commands.
 @return void
 @param callTime [ns] Time the method is called
*/
void StepperMotorController::updateState(const uint64_t callTime) {
    float referenceAngle{};
    if (this->motorRefAngleInMsg.isWritten()) {
        const HingedRigidBodyMsgF32Payload motorRefAngleIn = this->motorRefAngleInMsg();
        referenceAngle = motorRefAngleIn.theta;
    }

    const StepperMotorControllerOutput output =
        this->algorithm.update(this->currentPosition, referenceAngle, this->isMotorMoving);

    if (output.commandType == StepperMotorCommandType::MOVE) {
        this->commandedPosition = this->currentPosition + output.stepsToMove;
        this->stepAccumulator = 0.0F;
        MotorStepCommandMsgPayload motorStepCommandOut{};
        motorStepCommandOut.stepsCommanded = output.stepsToMove;
        this->motorStepCommandOutMsg.write(&motorStepCommandOut, moduleID, callTime);
    } else if (output.commandType == StepperMotorCommandType::STOP) {
        // Freeze the motor at its current position; ignore any remaining commanded steps
        this->commandedPosition = this->currentPosition;
    }

    this->advanceMotor();
}

/*! Advance the tracked motor position toward the commanded target, respecting the motor/control
 * frequency ratio via a fractional-step accumulator.
 @return void
*/
void StepperMotorController::advanceMotor() {
    if (this->currentPosition == this->commandedPosition) {
        return;
    }

    // Accumulate fractional steps based on motor/control frequency ratio.
    // E.g. if ratio is 1.5, steps per tick alternate 1, 2, 1, 2, ...
    this->stepAccumulator += this->motorFrequency / this->controlFrequency;
    const auto wholeSteps = static_cast<int>(this->stepAccumulator);
    this->stepAccumulator -= static_cast<float>(wholeSteps);

    const int remaining = abs(this->commandedPosition - this->currentPosition);
    const int stepsToAdvance = (wholeSteps < remaining) ? wholeSteps : remaining;

    if (this->currentPosition < this->commandedPosition) {
        this->currentPosition += stepsToAdvance;
    } else {
        this->currentPosition -= stepsToAdvance;
    }
}

void StepperMotorController::setStepAngle(const float stepAngleIn) { this->algorithm.setStepAngle(stepAngleIn); }

float StepperMotorController::getStepAngle() const { return this->algorithm.getStepAngle(); }

void StepperMotorController::setMotorAngleRange(const float minAngleIn, const float maxAngleIn) {
    this->algorithm.setMotorAngleRange(minAngleIn, maxAngleIn);
}

std::array<float, 2> StepperMotorController::getMotorAngleRange() const { return this->algorithm.getMotorAngleRange(); }

void StepperMotorController::setInitialAngle(const float initialAngleIn) { this->initialAngle = initialAngleIn; }

float StepperMotorController::getInitialAngle() const { return this->initialAngle; }

void StepperMotorController::setControlFrequency(const float controlFrequencyIn) {
    if (controlFrequencyIn <= 0.0F) {
        throw std::invalid_argument("controlFrequency must be positive");
    }
    this->controlFrequency = controlFrequencyIn;
}

float StepperMotorController::getControlFrequency() const { return this->controlFrequency; }

void StepperMotorController::setMotorFrequency(const float motorFrequencyIn) {
    if (motorFrequencyIn <= 0.0F) {
        throw std::invalid_argument("motorFrequency must be positive");
    }
    this->motorFrequency = motorFrequencyIn;
}

float StepperMotorController::getMotorFrequency() const { return this->motorFrequency; }

void StepperMotorController::setSettleCountMax(const int settleCountMaxIn) {
    this->algorithm.setSettleCountMax(settleCountMaxIn);
}

int StepperMotorController::getSettleCountMax() const { return this->algorithm.getSettleCountMax(); }

void StepperMotorController::setCurrentPositionTolerance(const int currentPositionToleranceIn) {
    this->algorithm.setCurrentPositionTolerance(currentPositionToleranceIn);
}

int StepperMotorController::getCurrentPositionTolerance() const {
    return this->algorithm.getCurrentPositionTolerance();
}

void StepperMotorController::setDesiredPositionTolerance(const int desiredPositionToleranceIn) {
    this->algorithm.setDesiredPositionTolerance(desiredPositionToleranceIn);
}

int StepperMotorController::getDesiredPositionTolerance() const {
    return this->algorithm.getDesiredPositionTolerance();
}
