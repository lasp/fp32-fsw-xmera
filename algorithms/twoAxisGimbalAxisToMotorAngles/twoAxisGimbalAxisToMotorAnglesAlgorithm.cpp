// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "twoAxisGimbalAxisToMotorAnglesAlgorithm.h"

#include <math.h>

#include "architecture/utilities/bilinearInterpolation.hpp"
#include "architecture/utilities/linearInterpolation.hpp"
#include "utilities/fsw/safeMath.h"

const int tipAngleIdxOffset = 38;
const int tiltAngleIdxOffset = 55;

/*! Algorithm constructor.
 @param config Validated configuration (DCM and gimbal-to-motor interpolation tables)
*/
TwoAxisGimbalAxisToMotorAnglesAlgorithm::TwoAxisGimbalAxisToMotorAnglesAlgorithm(
    const TwoAxisGimbalAxisToMotorAnglesConfig& config)
    : cfg(config) {
    setConfig(config);
}

/*! Replaces the algorithm's configuration for runtime reconfiguration.
 @param config Validated configuration (DCM and gimbal-to-motor interpolation tables)
*/
void TwoAxisGimbalAxisToMotorAnglesAlgorithm::setConfig(const TwoAxisGimbalAxisToMotorAnglesConfig& config) {
    this->cfg = config;
}

/*! This method determines the gimbal sequential tip and tilt angles corresponding to the given thrust direction vector
in spacecraft body frame components, then interpolates the corresponding stepper motor angles.
 @return TwoAxisGimbalAxisToMotorAnglesOutput
 @param thrustDirHat_B Commanded thrust direction unit vector in body frame components
*/
TwoAxisGimbalAxisToMotorAnglesOutput TwoAxisGimbalAxisToMotorAnglesAlgorithm::update(
    const Eigen::Vector3f& thrustDirHat_B) const {
    // Convert the commanded thrust direction vector to gimbal mount frame (hub-fixed) components
    const Eigen::Vector3f thrustDirHat_M = this->cfg.getDcmMB() * thrustDirHat_B;

    // Determine the corresponding gimbal tip and tilt angles
    const float gimbalTipAngle = safeAtanf(-thrustDirHat_M[1] / thrustDirHat_M[2]);
    const float gimbalTiltAngle = safeAsinf(thrustDirHat_M[0]);

    // Interpolate the motor angles given the gimbal angles
    const MotorAngles motorAngles = this->gimbalAnglesToMotorAngles(gimbalTipAngle, gimbalTiltAngle);

    TwoAxisGimbalAxisToMotorAnglesOutput output{};
    output.gimbalTipAngle = gimbalTipAngle;
    output.gimbalTiltAngle = gimbalTiltAngle;
    output.motorAngle1 = motorAngles.angle1;
    output.motorAngle2 = motorAngles.angle2;
    output.isValidInterpolation = motorAngles.isValidInterpolation;

    return output;
}

/*! This method determines the stepper motor angles given the gimbal sequential tip and tilt angles.
 @return MotorAngles
 @param gimbalTipAngle [rad] Gimbal tip angle
 @param gimbalTiltAngle [rad] Gimbal tilt angle
*/
MotorAngles TwoAxisGimbalAxisToMotorAnglesAlgorithm::gimbalAnglesToMotorAngles(const float gimbalTipAngle,
                                                                               const float gimbalTiltAngle) const {
    MotorAngles motorAngles{};
    if (this->bilinearInterpolationRequired(gimbalTipAngle, gimbalTiltAngle)) {
        motorAngles = this->bilinearlyInterpolateAngles(gimbalTipAngle, gimbalTiltAngle);
    } else if (this->noInterpolationRequired(gimbalTipAngle, gimbalTiltAngle)) {
        motorAngles = this->pullAngles(gimbalTipAngle, gimbalTiltAngle);
    } else if (this->linearInterpolationRequired(gimbalTipAngle)) {
        motorAngles = this->linearlyInterpolateAngles(gimbalTipAngle, gimbalTiltAngle, FixedAngle::ANGLE_1_FIXED);
    } else {
        motorAngles = this->linearlyInterpolateAngles(gimbalTipAngle, gimbalTiltAngle, FixedAngle::ANGLE_2_FIXED);
    }

    return motorAngles;
}

/*! This method pulls the requested angles from the provided interpolation table.
 @return MotorAngles
 @param gimbalAngle1 [rad]
 @param gimbalAngle2 [rad]
*/
MotorAngles TwoAxisGimbalAxisToMotorAnglesAlgorithm::pullAngles(float gimbalAngle1, float gimbalAngle2) const {
    gimbalAngle1 += static_cast<float>(tipAngleIdxOffset) * this->tableStepAngle;
    gimbalAngle2 += static_cast<float>(tiltAngleIdxOffset) * this->tableStepAngle;
    const auto index1 = static_cast<int>(roundf(gimbalAngle1 / this->tableStepAngle));
    const auto index2 = static_cast<int>(roundf(gimbalAngle2 / this->tableStepAngle));

    const float motor1Angle = this->cfg.getGimbalToMotor1Data()[index2][index1];
    const float motor2Angle = this->cfg.getGimbalToMotor2Data()[index2][index1];

    MotorAngles motorAngles{};
    motorAngles.angle1 = motor1Angle;
    motorAngles.angle2 = motor2Angle;
    motorAngles.isValidInterpolation = true;

    return motorAngles;
}

/*! This method determines if bilinear interpolation is required to obtain the desired angles.
 @return bool
 @param gimbalAngle1 [rad]
 @param gimbalAngle2 [rad]
*/
bool TwoAxisGimbalAxisToMotorAnglesAlgorithm::bilinearInterpolationRequired(const float gimbalAngle1,
                                                                            const float gimbalAngle2) const {
    const float motor1Rounded = roundf(fabsf(gimbalAngle1 / this->tableStepAngle));
    const float motor1Exact = fabsf(gimbalAngle1 / this->tableStepAngle);
    const float motor1Remainder = fabsf(motor1Exact - motor1Rounded);

    const float motor2Rounded = roundf(fabsf(gimbalAngle2 / this->tableStepAngle));
    const float motor2Exact = fabsf(gimbalAngle2 / this->tableStepAngle);
    const float motor2Remainder = fabsf(motor2Exact - motor2Rounded);

    return motor1Remainder >= kInterpolationRemainderTolerance && motor2Remainder >= kInterpolationRemainderTolerance;
}

/*! This method determines if no interpolation is required to obtain the desired angles.
 @return bool
 @param gimbalAngle1 [rad]
 @param gimbalAngle2 [rad]
*/
bool TwoAxisGimbalAxisToMotorAnglesAlgorithm::noInterpolationRequired(const float gimbalAngle1,
                                                                      const float gimbalAngle2) const {
    const float motor1Rounded = roundf(fabsf(gimbalAngle1 / this->tableStepAngle));
    const float motor1Exact = fabsf(gimbalAngle1 / this->tableStepAngle);
    const float motor1Remainder = fabsf(motor1Exact - motor1Rounded);

    const float motor2Rounded = roundf(fabsf(gimbalAngle2 / this->tableStepAngle));
    const float motor2Exact = fabsf(gimbalAngle2 / this->tableStepAngle);
    const float motor2Remainder = fabsf(motor2Exact - motor2Rounded);

    return motor1Remainder < kInterpolationRemainderTolerance && motor2Remainder < kInterpolationRemainderTolerance;
}

/*! This method determines if linear interpolation is required to obtain the desired angles.
 @return bool
 @param angle [rad]
*/
bool TwoAxisGimbalAxisToMotorAnglesAlgorithm::linearInterpolationRequired(const float angle) const {
    const float rounded = roundf(fabsf(angle / this->tableStepAngle));
    const float exact = fabsf(angle / this->tableStepAngle);
    const float remainder = fabsf(exact - rounded);

    return remainder < kInterpolationRemainderTolerance;
}

/*! This method bilinearly interpolates the motor angles from the four surrounding interpolation-table entries. If
any of the four bounding motor 1 angles is negative (an out-of-range table entry), the interpolation is flagged as
invalid.
 @return MotorAngles
 @param gimbalAngle1 [rad]
 @param gimbalAngle2 [rad]
*/
MotorAngles TwoAxisGimbalAxisToMotorAnglesAlgorithm::bilinearlyInterpolateAngles(const float gimbalAngle1,
                                                                                 const float gimbalAngle2) const {
    // Find the upper and lower interpolation table angle bounds using the given angles
    const float gimbalAngle1LBound = this->tableStepAngle * floorf(gimbalAngle1 / this->tableStepAngle);
    const float gimbalAngle1UBound = this->tableStepAngle * ceilf(gimbalAngle1 / this->tableStepAngle);
    const float gimbalAngle2LBound = this->tableStepAngle * floorf(gimbalAngle2 / this->tableStepAngle);
    const float gimbalAngle2UBound = this->tableStepAngle * ceilf(gimbalAngle2 / this->tableStepAngle);

    // Determine the bounding angles
    const MotorAngles motorLLBounds = this->pullAngles(gimbalAngle1LBound, gimbalAngle2LBound);
    const MotorAngles motorLUBounds = this->pullAngles(gimbalAngle1LBound, gimbalAngle2UBound);
    const MotorAngles motorULBounds = this->pullAngles(gimbalAngle1UBound, gimbalAngle2LBound);
    const MotorAngles motorUUBounds = this->pullAngles(gimbalAngle1UBound, gimbalAngle2UBound);
    const float motor1AngleLLBound = motorLLBounds.angle1;
    const float motor1AngleLUBound = motorLUBounds.angle1;
    const float motor1AngleULBound = motorULBounds.angle1;
    const float motor1AngleUUBound = motorUUBounds.angle1;
    const float motor2AngleLLBound = motorLLBounds.angle2;
    const float motor2AngleLUBound = motorLUBounds.angle2;
    const float motor2AngleULBound = motorULBounds.angle2;
    const float motor2AngleUUBound = motorUUBounds.angle2;

    float motor1Angle{};
    float motor2Angle{};
    bool validInterpolation{};
    if (motor1AngleLLBound >= 0.0F && motor1AngleLUBound >= 0.0F && motor1AngleULBound >= 0.0F &&
        motor1AngleUUBound >= 0.0F) {
        motor1Angle = static_cast<float>(bilinearInterpolation(gimbalAngle1LBound,
                                                               gimbalAngle1UBound,
                                                               gimbalAngle2LBound,
                                                               gimbalAngle2UBound,
                                                               motor1AngleLLBound,
                                                               motor1AngleLUBound,
                                                               motor1AngleULBound,
                                                               motor1AngleUUBound,
                                                               gimbalAngle1,
                                                               gimbalAngle2));
        motor2Angle = static_cast<float>(bilinearInterpolation(gimbalAngle1LBound,
                                                               gimbalAngle1UBound,
                                                               gimbalAngle2LBound,
                                                               gimbalAngle2UBound,
                                                               motor2AngleLLBound,
                                                               motor2AngleLUBound,
                                                               motor2AngleULBound,
                                                               motor2AngleUUBound,
                                                               gimbalAngle1,
                                                               gimbalAngle2));
        validInterpolation = true;
    }

    MotorAngles motorAngles{};
    motorAngles.angle1 = motor1Angle;
    motorAngles.angle2 = motor2Angle;
    motorAngles.isValidInterpolation = validInterpolation;

    return motorAngles;
}

/*! This method calls the linear interpolation function to interpolate using a fixed angle and a bounded angle.
For the gimbal-to-motor interpolation case where the gimbal angles are at the edges of the interpolation table,
the logic determines the appropriate motor angles to return. If 2/2 of the pulled motor angles are valid, linear
interpolation is used to determine the motor angle. If 1/2 motor angles are valid, the valid motor angle is directly
used as the result.
 @return MotorAngles
 @param gimbalAngle1 [rad] Angle 1 for linear interpolation
 @param gimbalAngle2 [rad] Angle 2 for linear interpolation
 @param fixedAngle Angle that is fixed for linear interpolation
*/
MotorAngles TwoAxisGimbalAxisToMotorAnglesAlgorithm::linearlyInterpolateAngles(const float gimbalAngle1,
                                                                               const float gimbalAngle2,
                                                                               const FixedAngle fixedAngle) const {
    // Use the provided fixed angle to save the bounded angle (The bounded angle is the non-fixed angle)
    float boundedAngle{};
    if (fixedAngle == FixedAngle::ANGLE_1_FIXED) {
        boundedAngle = gimbalAngle2;
    } else {
        boundedAngle = gimbalAngle1;
    }

    // Find the upper and lower interpolation table bounds for the bounded angle
    const float gimbalAngleLBound = this->tableStepAngle * floorf(boundedAngle / this->tableStepAngle);
    const float gimbalAngleUBound = this->tableStepAngle * ceilf(boundedAngle / this->tableStepAngle);

    // Determine the bounding angles for linear interpolation
    MotorAngles lowerMotorBounds{};
    MotorAngles upperMotorBounds{};
    if (fixedAngle == FixedAngle::ANGLE_1_FIXED) {
        lowerMotorBounds = this->pullAngles(gimbalAngle1, gimbalAngleLBound);
        upperMotorBounds = this->pullAngles(gimbalAngle1, gimbalAngleUBound);
    } else {
        lowerMotorBounds = this->pullAngles(gimbalAngleLBound, gimbalAngle2);
        upperMotorBounds = this->pullAngles(gimbalAngleUBound, gimbalAngle2);
    }

    const float motor1AngleLBound = lowerMotorBounds.angle1;
    const float motor1AngleUBound = upperMotorBounds.angle1;
    const float motor2AngleLBound = lowerMotorBounds.angle2;
    const float motor2AngleUBound = upperMotorBounds.angle2;

    // Linearly interpolate if the pulled angles are valid
    float motor1Angle{};
    float motor2Angle{};
    bool validInterpolation{};
    if (motor1AngleLBound >= 0.0F && motor1AngleUBound >= 0.0F) {
        motor1Angle = static_cast<float>(linearInterpolation(
            gimbalAngleLBound, gimbalAngleUBound, motor1AngleLBound, motor1AngleUBound, boundedAngle));
        motor2Angle = static_cast<float>(linearInterpolation(
            gimbalAngleLBound, gimbalAngleUBound, motor2AngleLBound, motor2AngleUBound, boundedAngle));
        validInterpolation = true;
    }

    MotorAngles motorAngles{};
    motorAngles.angle1 = motor1Angle;
    motorAngles.angle2 = motor2Angle;
    motorAngles.isValidInterpolation = validInterpolation;

    return motorAngles;
}
