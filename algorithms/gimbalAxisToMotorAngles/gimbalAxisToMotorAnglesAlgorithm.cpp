#include "gimbalAxisToMotorAnglesAlgorithm.h"

#include <math.h>

#include "architecture/utilities/bilinearInterpolation.hpp"
#include "architecture/utilities/linearInterpolation.hpp"
#include "utilities/fsw/safeMath.h"

const int tipAngleIdxOffset = 38;
const int tiltAngleIdxOffset = 55;

GimbalAxisToMotorAnglesAlgorithm::GimbalAxisToMotorAnglesAlgorithm(const GimbalAxisToMotorAnglesConfig& config)
    : cfg(config) {
    setConfig(config);
}

/*! Replaces the algorithm's configuration for runtime reconfiguration.
 @param config Validated configuration (DCM and gimbal-to-motor interpolation tables)
*/
void GimbalAxisToMotorAnglesAlgorithm::setConfig(const GimbalAxisToMotorAnglesConfig& config) { this->cfg = config; }

/*! This method determines the gimbal sequential tip and tilt angles corresponding to the given thrust direction vector
in spacecraft body frame components, then interpolates the corresponding stepper motor angles.
 @return GimbalAxisToMotorAnglesOutput
 @param thrustHat_B Commanded thrust direction unit vector in body frame components
*/
GimbalAxisToMotorAnglesOutput GimbalAxisToMotorAnglesAlgorithm::update(const Eigen::Vector3f& thrustHat_B) const {
    /*! Set default output */
    GimbalAxisToMotorAnglesOutput output{};

    /*! Motor angles are only resolveable if the incoming thrust direction is not zero. */
    const bool isThrustHatResolved = thrustHat_B.stableNorm() != 0.0F;

    if (isThrustHatResolved) {
        // Determine the required gimbal tip and tilt angles
        const Eigen::Vector3f thrustDirHat_M = this->cfg.getDcmMB() * thrustHat_B;
        const float gimbalTipAngle = safeAtanf(-thrustDirHat_M[1] / thrustDirHat_M[2]);
        const float gimbalTiltAngle = safeAsinf(thrustDirHat_M[0]);

        // Determine the required motor angles
        const MotorAngles motorAngles = this->gimbalAnglesToMotorAngles(gimbalTipAngle, gimbalTiltAngle);

        output.gimbalTipAngle = gimbalTipAngle;
        output.gimbalTiltAngle = gimbalTiltAngle;
        output.motorAngle1 = motorAngles.angle1;
        output.motorAngle2 = motorAngles.angle2;
        output.isValidInterpolation = motorAngles.isValidInterpolation;
    }

    return output;
}

/*! This method determines the stepper motor angles given the gimbal sequential tip and tilt angles.
 @return MotorAngles
 @param gimbalTipAngle [rad] Gimbal tip angle
 @param gimbalTiltAngle [rad] Gimbal tilt angle
*/
MotorAngles GimbalAxisToMotorAnglesAlgorithm::gimbalAnglesToMotorAngles(const float gimbalTipAngle,
                                                                        const float gimbalTiltAngle) const {
    MotorAngles motorAngles{};
    if (this->isBilinearInterpolationRequired(gimbalTipAngle, gimbalTiltAngle)) {
        motorAngles = this->bilinearlyInterpolateMotorAngles(gimbalTipAngle, gimbalTiltAngle);
    } else if (this->isNoInterpolationRequired(gimbalTipAngle, gimbalTiltAngle)) {
        motorAngles = this->pullAngles(gimbalTipAngle, gimbalTiltAngle);
    } else if (this->isLinearInterpolationRequired(gimbalTipAngle)) {
        motorAngles = this->linearlyInterpolateMotorAngles(gimbalTipAngle, gimbalTiltAngle, FixedAngle::ANGLE_1_FIXED);
    } else {
        motorAngles = this->linearlyInterpolateMotorAngles(gimbalTipAngle, gimbalTiltAngle, FixedAngle::ANGLE_2_FIXED);
    }

    return motorAngles;
}

/*! This method pulls the motor angles from the provided interpolation tables.
 @return MotorAngles
 @param gimbalAngle1 [rad]
 @param gimbalAngle2 [rad]
*/
MotorAngles GimbalAxisToMotorAnglesAlgorithm::pullAngles(float gimbalAngle1, float gimbalAngle2) const {
    gimbalAngle1 += static_cast<float>(tipAngleIdxOffset) * this->tableStepAngle;
    gimbalAngle2 += static_cast<float>(tiltAngleIdxOffset) * this->tableStepAngle;
    const auto index1 = static_cast<int>(roundf(gimbalAngle1 / this->tableStepAngle));
    const auto index2 = static_cast<int>(roundf(gimbalAngle2 / this->tableStepAngle));

    const float motor1Angle = this->cfg.getGimbalToMotor1AngleTable()[index2][index1];
    const float motor2Angle = this->cfg.getGimbalToMotor2AngleTable()[index2][index1];

    MotorAngles motorAngles{};
    motorAngles.angle1 = motor1Angle;
    motorAngles.angle2 = motor2Angle;
    motorAngles.isValidInterpolation = true;

    return motorAngles;
}

/*! This method determines if bilinear interpolation is required to obtain the motor angles.
 @return bool
 @param gimbalAngle1 [rad]
 @param gimbalAngle2 [rad]
*/
bool GimbalAxisToMotorAnglesAlgorithm::isBilinearInterpolationRequired(const float gimbalAngle1,
                                                                       const float gimbalAngle2) const {
    const float motor1Rounded = roundf(fabsf(gimbalAngle1 / this->tableStepAngle));
    const float motor1Exact = fabsf(gimbalAngle1 / this->tableStepAngle);
    const float motor1Remainder = fabsf(motor1Exact - motor1Rounded);

    const float motor2Rounded = roundf(fabsf(gimbalAngle2 / this->tableStepAngle));
    const float motor2Exact = fabsf(gimbalAngle2 / this->tableStepAngle);
    const float motor2Remainder = fabsf(motor2Exact - motor2Rounded);

    return motor1Remainder >= kInterpolationRemainderTolerance && motor2Remainder >= kInterpolationRemainderTolerance;
}

/*! This method determines if no interpolation is required to obtain the motor angles.
 @return bool
 @param gimbalAngle1 [rad]
 @param gimbalAngle2 [rad]
*/
bool GimbalAxisToMotorAnglesAlgorithm::isNoInterpolationRequired(const float gimbalAngle1,
                                                                 const float gimbalAngle2) const {
    const float motor1Rounded = roundf(fabsf(gimbalAngle1 / this->tableStepAngle));
    const float motor1Exact = fabsf(gimbalAngle1 / this->tableStepAngle);
    const float motor1Remainder = fabsf(motor1Exact - motor1Rounded);

    const float motor2Rounded = roundf(fabsf(gimbalAngle2 / this->tableStepAngle));
    const float motor2Exact = fabsf(gimbalAngle2 / this->tableStepAngle);
    const float motor2Remainder = fabsf(motor2Exact - motor2Rounded);

    return motor1Remainder < kInterpolationRemainderTolerance && motor2Remainder < kInterpolationRemainderTolerance;
}

/*! This method determines if linear interpolation is required to obtain the motor angles.
 @return bool
 @param angle [rad]
*/
bool GimbalAxisToMotorAnglesAlgorithm::isLinearInterpolationRequired(const float angle) const {
    const float rounded = roundf(fabsf(angle / this->tableStepAngle));
    const float exact = fabsf(angle / this->tableStepAngle);
    const float remainder = fabsf(exact - rounded);

    return remainder < kInterpolationRemainderTolerance;
}

/*! This method bilinearly interpolates the motor angles from the four surrounding interpolation-table entries. If
any of the bounding motor angles are negative (an out-of-range table entry), the interpolation is flagged as
invalid and zero motor angles are returned.
 @return MotorAngles
 @param gimbalAngle1 [rad]
 @param gimbalAngle2 [rad]
*/
MotorAngles GimbalAxisToMotorAnglesAlgorithm::bilinearlyInterpolateMotorAngles(const float gimbalAngle1,
                                                                               const float gimbalAngle2) const {
    // Determine the bounding gimbal angles
    const float gimbalAngle1LBound = this->tableStepAngle * floorf(gimbalAngle1 / this->tableStepAngle);
    const float gimbalAngle1UBound = this->tableStepAngle * ceilf(gimbalAngle1 / this->tableStepAngle);
    const float gimbalAngle2LBound = this->tableStepAngle * floorf(gimbalAngle2 / this->tableStepAngle);
    const float gimbalAngle2UBound = this->tableStepAngle * ceilf(gimbalAngle2 / this->tableStepAngle);

    // Determine the bounding motor angles
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
    // Interpolate the motor angles if all bounding angles are valid
    if (motor1AngleLLBound >= 0.0F && motor1AngleLUBound >= 0.0F && motor1AngleULBound >= 0.0F &&
        motor1AngleUUBound >= 0.0F && motor2AngleLLBound >= 0.0F && motor2AngleLUBound >= 0.0F &&
        motor2AngleULBound >= 0.0F && motor2AngleUUBound >= 0.0F) {
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

/*! This method calls the linear interpolation function to interpolate the motor angles, given one fixed and one
bounded gimbal angle. If both pulled motor angles are valid, linear interpolation is used to determine the motor
angles. Otherwise, the interpolation is flagged as invalid and zero motor angles are returned.
 @return MotorAngles
 @param gimbalAngle1 [rad] Angle 1 for linear interpolation
 @param gimbalAngle2 [rad] Angle 2 for linear interpolation
 @param fixedAngle Angle that is fixed for linear interpolation
*/
MotorAngles GimbalAxisToMotorAnglesAlgorithm::linearlyInterpolateMotorAngles(const float gimbalAngle1,
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

    float motor1Angle{};
    float motor2Angle{};
    bool validInterpolation{};
    // Linearly interpolate if the pulled angles are valid
    if (motor1AngleLBound >= 0.0F && motor1AngleUBound >= 0.0F && motor2AngleLBound >= 0.0F &&
        motor2AngleUBound >= 0.0F) {
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
