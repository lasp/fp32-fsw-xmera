#include "stepperMotorController.h"
#include <stdexcept>

/*! Reset the module. Checks that the input messages are linked.
 @return void
 @param callTime [ns] Time the method is called
*/
void StepperMotorController::reset(const uint64_t callTime) {
    if (!this->motorRefAngleInMsg.isLinked()) {
        throw std::invalid_argument("StepperMotorController.motorRefAngleInMsg wasn't connected.");
    }
    if (!this->stepperMotorInMsg.isLinked()) {
        throw std::invalid_argument("StepperMotorController.stepperMotorInMsg wasn't connected.");
    }
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

    int currentPosition = 0;
    bool isMotorMoving = false;
    if (this->stepperMotorInMsg.isWritten()) {
        const StepperMotorMsgPayload stepperMotorIn = this->stepperMotorInMsg();
        currentPosition = stepperMotorIn.motorPosition;
        isMotorMoving = stepperMotorIn.isMotorMoving;
    }

    const StepperMotorControllerOutput output = this->algorithm.update(currentPosition, referenceAngle, isMotorMoving);

    if (output.commandType == StepperMotorCommandType::MOVE) {
        MotorStepCommandMsgPayload motorStepCommandOut{};
        motorStepCommandOut.stepsCommanded = output.stepsToMove;
        motorStepCommandOut.stopMotorCommand = false;
        this->motorStepCommandOutMsg.write(&motorStepCommandOut, moduleID, callTime);
    } else if (output.commandType == StepperMotorCommandType::STOP) {
        MotorStepCommandMsgPayload motorStepCommandOut{};
        motorStepCommandOut.stepsCommanded = 0;
        motorStepCommandOut.stopMotorCommand = true;
        this->motorStepCommandOutMsg.write(&motorStepCommandOut, moduleID, callTime);
    }
}

void StepperMotorController::setStepAngle(const float stepAngleIn) { this->algorithm.setStepAngle(stepAngleIn); }

float StepperMotorController::getStepAngle() const { return this->algorithm.getStepAngle(); }

void StepperMotorController::setMotorAngleRange(const float minAngleIn, const float maxAngleIn) {
    this->algorithm.setMotorAngleRange(minAngleIn, maxAngleIn);
}

std::array<float, 2> StepperMotorController::getMotorAngleRange() const { return this->algorithm.getMotorAngleRange(); }

void StepperMotorController::setSettleCountMax(const uint32_t settleCountMaxIn) {
    this->algorithm.setSettleCountMax(settleCountMaxIn);
}

uint32_t StepperMotorController::getSettleCountMax() const { return this->algorithm.getSettleCountMax(); }

void StepperMotorController::setMinStepCommand(const uint32_t minStepCommandIn) {
    this->algorithm.setMinStepCommand(minStepCommandIn);
}

uint32_t StepperMotorController::getMinStepCommand() const { return this->algorithm.getMinStepCommand(); }
