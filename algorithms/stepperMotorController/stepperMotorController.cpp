#include "stepperMotorController.h"
#include "utilities/xmera/xmeraLifecycleException.h"
#include <stdexcept>

/*! Reset the module. Builds and validates the immutable configuration from the adapter's stored properties,
 (re)constructs the controller, and verifies the input messages are linked.
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

    const StepperMotorControllerConfig config =
        StepperMotorControllerConfig::create(this->stepAngle,
                                             StepperMotorAngleRange{this->minAngle, this->maxAngle},
                                             this->settleCountMax,
                                             this->minStepCommand);
    this->algorithm = std::make_unique<StepperMotorControllerAlgorithm>(config);
}

void StepperMotorController::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("StepperMotorController reset() has not been called.");
    }
    this->algorithm->reInitialize();
}

/*! Read input messages, run the state machine, and write output commands.
 @return void
 @param callTime [ns] Time the method is called
*/
void StepperMotorController::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("StepperMotorController reset() has not been called.");
    }

    float referenceAngle{};
    if (this->motorRefAngleInMsg.isWritten()) {
        const MotorAngleRefMsgF32Payload motorRefAngleIn = this->motorRefAngleInMsg();
        referenceAngle = motorRefAngleIn.theta;
    }

    int currentPosition = 0;
    bool isMotorMoving = false;
    if (this->stepperMotorInMsg.isWritten()) {
        const StepperMotorMsgPayload stepperMotorIn = this->stepperMotorInMsg();
        currentPosition = stepperMotorIn.motorPosition;
        isMotorMoving = stepperMotorIn.isMotorMoving;
    }

    const StepperMotorControllerOutput output = this->algorithm->update(currentPosition, referenceAngle, isMotorMoving);

    if (output.commandType == StepperMotorCommandType::MOVE) {
        MotorStepCommandMsgPayload motorStepCommandOut{};
        motorStepCommandOut.stepsCommanded = output.stepsToMove;
        motorStepCommandOut.stopMotorCommand = false;
        this->motorStepCommandOutMsg.write(motorStepCommandOut, moduleID, callTime);
    } else if (output.commandType == StepperMotorCommandType::STOP) {
        MotorStepCommandMsgPayload motorStepCommandOut{};
        motorStepCommandOut.stepsCommanded = 0;
        motorStepCommandOut.stopMotorCommand = true;
        this->motorStepCommandOutMsg.write(motorStepCommandOut, moduleID, callTime);
    }
}
