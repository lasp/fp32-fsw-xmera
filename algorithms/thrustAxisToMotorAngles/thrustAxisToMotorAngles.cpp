#include "thrustAxisToMotorAngles.h"

#include <memory>
#include <stdexcept>

#include "utilities/xmera/xmeraLifecycleException.h"

/*! This method checks the input message to ensure it is linked and builds the algorithm from the
configured parameters.
 @return void
 @param currentSimNanos [ns] Time the method is called
*/
void ThrustAxisToMotorAngles::reset(uint64_t currentSimNanos) {
    if (!this->twoAxisGimbalInMsg.isLinked()) {
        throw std::invalid_argument("thrustAxisToMotorAngles.twoAxisGimbalInMsg wasn't connected.");
    }

    const auto config = ThrustAxisToMotorAnglesConfig::create(StepperMotorAngleRange{this->minAngle, this->maxAngle},
                                                              this->gimbalToMotor1AngleData,
                                                              this->gimbalToMotor2AngleData,
                                                              GimbalToMotorAngleTableLayout{this->rowStartStrideIndices,
                                                                                            this->rowStartColIndices,
                                                                                            this->tipColIdxOffset,
                                                                                            this->tiltRowIdxOffset,
                                                                                            this->tableStepAngle});
    this->algorithm = std::make_unique<ThrustAxisToMotorAnglesAlgorithm>(config);
    this->previousWrittenTime = -1.0;
}

ThrustAxisToMotorAnglesConfig ThrustAxisToMotorAngles::toConfig() const {
    return ThrustAxisToMotorAnglesConfig::create(StepperMotorAngleRange{this->minAngle, this->maxAngle},
                                                 this->gimbalToMotor1AngleData,
                                                 this->gimbalToMotor2AngleData,
                                                 GimbalToMotorAngleTableLayout{this->rowStartStrideIndices,
                                                                               this->rowStartColIndices,
                                                                               this->tipColIdxOffset,
                                                                               this->tiltRowIdxOffset,
                                                                               this->tableStepAngle});
}

void ThrustAxisToMotorAngles::reconfigure() const {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrustAxisToMotorAngles reset() has not been called.");
    }

    this->algorithm->setConfig(this->toConfig());
}

void ThrustAxisToMotorAngles::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrustAxisToMotorAngles reset() has not been called.");
    }
    this->algorithm->reInitialize();
}

/*! This method reads the incoming requested gimbal angles, delegates the gimbal and stepper motor
angle computation to the algorithm, and writes the resulting motor angles to the output messages.
 @return void
 @param currentSimNanos [ns] The current time of simulation
*/
void ThrustAxisToMotorAngles::updateState(uint64_t currentSimNanos) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ThrustAxisToMotorAngles reset() has not been called.");
    }

    // Read the input message
    if (this->twoAxisGimbalInMsg.isWritten() && (this->previousWrittenTime < this->twoAxisGimbalInMsg.timeWritten())) {
        // Update the previous written time to the current message time written
        this->previousWrittenTime = this->twoAxisGimbalInMsg.timeWritten();

        // Store the commanded gimbal tip and tilt angles
        const auto twoAxisGimbalIn = this->twoAxisGimbalInMsg();
        const float gimbalAngle1 = twoAxisGimbalIn.theta1;
        const float gimbalAngle2 = twoAxisGimbalIn.theta2;

        // Determine motor angles corresponding to the gimbal angles
        const ThrustAxisToMotorAnglesOutput motorAngles = this->algorithm->update(gimbalAngle1, gimbalAngle2);

        // Write the module output messages
        auto motor1AngleOut = HingedRigidBodyMsgF32Payload();
        motor1AngleOut.theta = motorAngles.motorAngle1;
        this->motor1AngleOutMsg.write(motor1AngleOut, moduleID, currentSimNanos);

        auto motor2AngleOut = HingedRigidBodyMsgF32Payload();
        motor2AngleOut.theta = motorAngles.motorAngle2;
        this->motor2AngleOutMsg.write(motor2AngleOut, moduleID, currentSimNanos);
    }
}
