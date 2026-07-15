// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "twoAxisGimbalAxisToMotorAngles.h"

#include <math.h>
#include <cassert>

#include "architecture/utilities/eigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"

/*! Module constructor. A TwoAxisGimbalLookupTables object must be passed to the constructor.
 @return void
 @param gimbalLookupTables TwoAxisGimbalLookupTables class object
*/
TwoAxisGimbalAxisToMotorAngles::TwoAxisGimbalAxisToMotorAngles(const TwoAxisGimbalLookupTables& gimbalLookupTables)
    : gimbalLookupTables(gimbalLookupTables) {}

/*! This method checks the input message to ensure it is linked. This method also resets module parameters to
default values.
 @return void
 @param currentSimNanos [ns] Time the method is called
*/
void TwoAxisGimbalAxisToMotorAngles::reset(uint64_t currentSimNanos) {
    if (!this->thrustDirectionInMsg.isLinked()) {
        this->bskLogger->bskLog(BSK_ERROR, "TwoAxisGimbalAxisToMotorAngles.thrustDirectionInMsg wasn't connected.");
    }

    this->gimbalTipAngle = 0.0;
    this->gimbalTiltAngle = 0.0;
    this->previousWrittenTime = -1.0;
}

/*! This method determines the gimbal sequential tip and tilt angles corresponding to the given thrust direction vector
in spacecraft body frame components.
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
        auto thrustDirectionIn = this->thrustDirectionInMsg();
        Eigen::Vector3d thrustDirHat_B = cArrayToEigenVector3(thrustDirectionIn.rHat_XB_B);

        // Convert the commanded thrust direction vector to gimbal mount frame (hub-fixed) components
        Eigen::Vector3d thrustDirHat_M = this->dcm_MB * thrustDirHat_B;

        // Determine the corresponding gimbal tip and tilt angles
        this->gimbalTipAngle = atan(-thrustDirHat_M[1] / thrustDirHat_M[2]);
        this->gimbalTiltAngle = asin(thrustDirHat_M[0]);

        // Interpolate the motor angles given the gimbal angles
        MotorAngles motorAngles =
            this->gimbalLookupTables.gimbalAnglesToMotorAngles(this->gimbalTipAngle, this->gimbalTiltAngle);
        assert(motorAngles.isValidInterpolation);

        // Write the module output messages
        auto motor1AngleOut = HingedRigidBodyMsgPayload();
        motor1AngleOut.theta = motorAngles.angle1;
        this->motor1AngleOutMsg.write(motor1AngleOut, moduleID, currentSimNanos);

        auto motor2AngleOut = HingedRigidBodyMsgPayload();
        motor2AngleOut.theta = motorAngles.angle2;
        this->motor2AngleOutMsg.write(motor2AngleOut, moduleID, currentSimNanos);

        auto twoAxisGimbalOut = TwoAxisGimbalMsgPayload();
        twoAxisGimbalOut.theta1 = this->gimbalTipAngle;
        twoAxisGimbalOut.theta2 = this->gimbalTiltAngle;
        this->twoAxisGimbalOutMsg.write(twoAxisGimbalOut, moduleID, currentSimNanos);
    }
}

/*!  Setter method for dcm_MB (DCM from body frame to gimbal mount frame).
 @return void
 @param dcm_MB DCM from body frame to gimbal mount frame
*/
void TwoAxisGimbalAxisToMotorAngles::setDcmMB(const Eigen::Matrix3d dcm_MB) { this->dcm_MB = dcm_MB; }

/*! Getter method for dcm_MB (DCM from body frame to gimbal mount frame).
 @return const Eigen::Matrix3d
*/
const Eigen::Matrix3d& TwoAxisGimbalAxisToMotorAngles::getDcmMB() const { return this->dcm_MB; }
