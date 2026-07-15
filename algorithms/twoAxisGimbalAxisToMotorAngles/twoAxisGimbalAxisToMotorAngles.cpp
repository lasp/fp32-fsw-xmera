// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "twoAxisGimbalAxisToMotorAngles.h"

#include <memory>
#include <stdexcept>

#include "architecture/utilities/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"

/*! This method checks the input message to ensure it is linked and builds the algorithm from the
configured parameters.
 @return void
 @param currentSimNanos [ns] Time the method is called
*/
void TwoAxisGimbalAxisToMotorAngles::reset(uint64_t currentSimNanos) {
    if (!this->thrustDirectionInMsg.isLinked()) {
        throw std::invalid_argument("twoAxisGimbalAxisToMotorAngles.thrustDirectionInMsg wasn't connected.");
    }

    const auto config =
        TwoAxisGimbalAxisToMotorAnglesConfig::create(this->dcm_MB, this->gimbalToMotor1Data, this->gimbalToMotor2Data);
    this->algorithm = std::make_unique<TwoAxisGimbalAxisToMotorAnglesAlgorithm>(config);
    this->previousWrittenTime = -1.0;
}

TwoAxisGimbalAxisToMotorAnglesConfig TwoAxisGimbalAxisToMotorAngles::toConfig() const {
    return TwoAxisGimbalAxisToMotorAnglesConfig::create(
        this->dcm_MB, this->gimbalToMotor1Data, this->gimbalToMotor2Data);
}

void TwoAxisGimbalAxisToMotorAngles::reconfigure() const {
    if (!this->algorithm) {
        throw XmeraLifecycleException("TwoAxisGimbalAxisToMotorAngles reset() has not been called.");
    }

    this->algorithm->setConfig(this->toConfig());
}

/*! This method reads the commanded body-frame thrust direction message, delegates the gimbal and stepper motor
angle computation to the algorithm, and writes the resulting gimbal tip/tilt angles and the two motor angles to the
output messages.
 @return void
 @param currentSimNanos [ns] The current time of simulation
*/
void TwoAxisGimbalAxisToMotorAngles::updateState(uint64_t currentSimNanos) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("TwoAxisGimbalAxisToMotorAngles reset() has not been called.");
    }

    // Read the input message
    if (this->thrustDirectionInMsg.isWritten() &&
        (this->previousWrittenTime < this->thrustDirectionInMsg.timeWritten())) {
        // Update the previous written time to the current message time written
        this->previousWrittenTime = this->thrustDirectionInMsg.timeWritten();

        // Store the thrust direction command vector in body frame components
        const auto thrustDirectionIn = this->thrustDirectionInMsg();
        const Eigen::Vector3f thrustDirHat_B = cArrayToEigenVector3<float>(thrustDirectionIn.rHat_XB_B);

        // Determine the gimbal and motor angles corresponding to the thrust direction
        const TwoAxisGimbalAxisToMotorAnglesOutput motorAngles = this->algorithm->update(thrustDirHat_B);

        // Write the module output messages
        auto motor1AngleOut = HingedRigidBodyMsgF32Payload();
        motor1AngleOut.theta = motorAngles.motorAngle1;
        this->motor1AngleOutMsg.write(motor1AngleOut, moduleID, currentSimNanos);

        auto motor2AngleOut = HingedRigidBodyMsgF32Payload();
        motor2AngleOut.theta = motorAngles.motorAngle2;
        this->motor2AngleOutMsg.write(motor2AngleOut, moduleID, currentSimNanos);

        auto twoAxisGimbalOut = TwoAxisGimbalMsgF32Payload();
        twoAxisGimbalOut.theta1 = motorAngles.gimbalTipAngle;
        twoAxisGimbalOut.theta2 = motorAngles.gimbalTiltAngle;
        this->twoAxisGimbalOutMsg.write(twoAxisGimbalOut, moduleID, currentSimNanos);
    }
}
