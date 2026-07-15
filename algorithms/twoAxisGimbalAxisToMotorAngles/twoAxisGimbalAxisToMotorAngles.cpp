// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "twoAxisGimbalAxisToMotorAngles.h"

#include <stdexcept>

#include "architecture/utilities/eigenSupport.h"

/*! Module constructor. The gimbal-to-motor interpolation table data must be specified.
 @param gimbalToMotor1Data Gimbal-to-motor 1 angle data table
 @param gimbalToMotor2Data Gimbal-to-motor 2 angle data table
*/
TwoAxisGimbalAxisToMotorAngles::TwoAxisGimbalAxisToMotorAngles(
    const std::array<std::array<double, NUM_GIMBAL_TO_MOTOR_TABLE_COLS>, NUM_GIMBAL_TO_MOTOR_TABLE_ROWS>&
        gimbalToMotor1Data,
    const std::array<std::array<double, NUM_GIMBAL_TO_MOTOR_TABLE_COLS>, NUM_GIMBAL_TO_MOTOR_TABLE_ROWS>&
        gimbalToMotor2Data)
    : algorithm(gimbalToMotor1Data, gimbalToMotor2Data) {}

/*! This method checks the input message to ensure it is linked. This method also resets module parameters to
default values.
 @return void
 @param currentSimNanos [ns] Time the method is called
*/
void TwoAxisGimbalAxisToMotorAngles::reset(uint64_t currentSimNanos) {
    if (!this->thrustDirectionInMsg.isLinked()) {
        throw std::invalid_argument("twoAxisGimbalAxisToMotorAngles.thrustDirectionInMsg wasn't connected.");
    }

    this->previousWrittenTime = -1.0;
}

/*! This method reads the commanded body-frame thrust direction message, delegates the gimbal and stepper motor
angle computation to the algorithm, and writes the resulting gimbal tip/tilt angles and the two motor angles to the
output messages.
 @return void
 @param currentSimNanos [ns] The current time of simulation
*/
void TwoAxisGimbalAxisToMotorAngles::updateState(uint64_t currentSimNanos) {
    // Read the input message
    if (this->thrustDirectionInMsg.isWritten() &&
        (this->previousWrittenTime < this->thrustDirectionInMsg.timeWritten())) {
        // Update the previous written time to the current message time written
        this->previousWrittenTime = this->thrustDirectionInMsg.timeWritten();

        // Store the thrust direction command vector in body frame components
        const auto thrustDirectionIn = this->thrustDirectionInMsg();
        const Eigen::Vector3d thrustDirHat_B = cArrayToEigenVector3(thrustDirectionIn.rHat_XB_B);

        // Determine the gimbal and motor angles corresponding to the thrust direction
        const TwoAxisGimbalAxisToMotorAnglesOutput motorAngles = this->algorithm.update(thrustDirHat_B);

        // Write the module output messages
        auto motor1AngleOut = HingedRigidBodyMsgPayload();
        motor1AngleOut.theta = motorAngles.motorAngle1;
        this->motor1AngleOutMsg.write(motor1AngleOut, moduleID, currentSimNanos);

        auto motor2AngleOut = HingedRigidBodyMsgPayload();
        motor2AngleOut.theta = motorAngles.motorAngle2;
        this->motor2AngleOutMsg.write(motor2AngleOut, moduleID, currentSimNanos);

        auto twoAxisGimbalOut = TwoAxisGimbalMsgPayload();
        twoAxisGimbalOut.theta1 = motorAngles.gimbalTipAngle;
        twoAxisGimbalOut.theta2 = motorAngles.gimbalTiltAngle;
        this->twoAxisGimbalOutMsg.write(twoAxisGimbalOut, moduleID, currentSimNanos);
    }
}

/*!  Setter method for dcm_MB (DCM from body frame to gimbal mount frame).
 @return void
 @param dcm_MB DCM from body frame to gimbal mount frame
*/
void TwoAxisGimbalAxisToMotorAngles::setDcmMB(const Eigen::Matrix3d dcm_MB) { this->algorithm.setDcmMB(dcm_MB); }

/*! Getter method for dcm_MB (DCM from body frame to gimbal mount frame).
 @return const Eigen::Matrix3d
*/
const Eigen::Matrix3d& TwoAxisGimbalAxisToMotorAngles::getDcmMB() const { return this->algorithm.getDcmMB(); }
