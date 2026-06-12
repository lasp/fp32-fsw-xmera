/*
 ISC License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

*/

#include "twoAxisGimbalLookupTables.h"
#include <math.h>

#include "architecture/utilities/bilinearInterpolation.hpp"
#include "architecture/utilities/linearInterpolation.hpp"

const int tipAngleIdxOffset = 38;
const int tiltAngleIdxOffset = 55;

/*! This method determines the stepper motor angles given the gimbal sequential tip and tilt angles.
 @return InterpolatedAngles
 @param gimbalTipAngle [rad] Gimbal tip angle
 @param gimbalTiltAngle [rad] Gimbal tilt angle
*/
InterpolatedAngles TwoAxisGimbalLookupTables::gimbalAnglesToMotorAngles(double gimbalTipAngle, double gimbalTiltAngle) {
    InterpolatedAngles motorAngles{};

    if (this->bilinearInterpolationRequired(gimbalTipAngle, gimbalTiltAngle)) {
        motorAngles = this->bilinearlyInterpolateAngles(
            gimbalTipAngle, gimbalTiltAngle, InterpolationType::GIMBAL_ANGLES_TO_MOTOR_ANGLES);
    } else if (this->noInterpolationRequired(gimbalTipAngle, gimbalTiltAngle)) {
        double motor1Angle =
            this->pullAngle(gimbalTipAngle, gimbalTiltAngle, InterpolationTableType::GIMBAL_ANGLES_TO_MOTOR_1_ANGLES);
        double motor2Angle =
            this->pullAngle(gimbalTipAngle, gimbalTiltAngle, InterpolationTableType::GIMBAL_ANGLES_TO_MOTOR_2_ANGLES);
        motorAngles.angle1 = motor1Angle;
        motorAngles.angle2 = motor2Angle;
        motorAngles.isValidInterpolation = true;
    } else if (this->linearInterpolationRequired(gimbalTipAngle)) {
        motorAngles = this->linearlyInterpolateAngles(gimbalTipAngle,
                                                      gimbalTiltAngle,
                                                      InterpolationType::GIMBAL_ANGLES_TO_MOTOR_ANGLES,
                                                      FixedAngle::ANGLE_1_FIXED);
    } else {
        motorAngles = this->linearlyInterpolateAngles(gimbalTipAngle,
                                                      gimbalTiltAngle,
                                                      InterpolationType::GIMBAL_ANGLES_TO_MOTOR_ANGLES,
                                                      FixedAngle::ANGLE_2_FIXED);
    }

    return motorAngles;
}

/*! This method determines the gimbal sequential tip and tilt angles given the stepper motor angles.
 @return InterpolatedAngles
 @param motor1Angle [rad] Stepper motor 1 angle
 @param motor2Angle [rad] Stepper motor 2 angle
*/
InterpolatedAngles TwoAxisGimbalLookupTables::motorAnglesToGimbalAngles(double motor1Angle, double motor2Angle) {
    InterpolatedAngles gimbalAngles{};

    if (this->bilinearInterpolationRequired(motor1Angle, motor2Angle)) {
        gimbalAngles = this->bilinearlyInterpolateAngles(
            motor1Angle, motor2Angle, InterpolationType::MOTOR_ANGLES_TO_GIMBAL_ANGLES);
    } else if (this->noInterpolationRequired(motor1Angle, motor2Angle)) {
        double gimbalTipAngle =
            this->pullAngle(motor1Angle, motor2Angle, InterpolationTableType::MOTOR_ANGLES_TO_GIMBAL_TIP_ANGLES);
        double gimbalTiltAngle =
            this->pullAngle(motor1Angle, motor2Angle, InterpolationTableType::MOTOR_ANGLES_TO_GIMBAL_TILT_ANGLES);
        gimbalAngles.angle1 = gimbalTipAngle;
        gimbalAngles.angle2 = gimbalTiltAngle;
        gimbalAngles.isValidInterpolation = true;
    } else if (this->linearInterpolationRequired(motor1Angle)) {
        gimbalAngles = this->linearlyInterpolateAngles(
            motor1Angle, motor2Angle, InterpolationType::MOTOR_ANGLES_TO_GIMBAL_ANGLES, FixedAngle::ANGLE_1_FIXED);
    } else {
        gimbalAngles = this->linearlyInterpolateAngles(
            motor1Angle, motor2Angle, InterpolationType::MOTOR_ANGLES_TO_GIMBAL_ANGLES, FixedAngle::ANGLE_2_FIXED);
    }

    return gimbalAngles;
}

/*! This method pulls the requested angles from the provided interpolation table.
 @return double
 @param angle1 [rad]
 @param angle2 [rad]
 @param interpolationTableType Enumeration indicating which interpolation table to pull the angles from
*/
double TwoAxisGimbalLookupTables::pullAngle(double angle1,
                                            double angle2,
                                            InterpolationTableType interpolationTableType) const {
    if (interpolationTableType == InterpolationTableType::GIMBAL_ANGLES_TO_MOTOR_1_ANGLES ||
        interpolationTableType == InterpolationTableType::GIMBAL_ANGLES_TO_MOTOR_2_ANGLES) {
        angle1 += tipAngleIdxOffset * this->tableStepAngle;
        angle2 += tiltAngleIdxOffset * this->tableStepAngle;
    }

    auto angle1Index = static_cast<int>(round(angle1 / this->tableStepAngle));
    auto angle2Index = static_cast<int>(round(angle2 / this->tableStepAngle));

    switch (interpolationTableType) {
        case InterpolationTableType::GIMBAL_ANGLES_TO_MOTOR_1_ANGLES:
            return this->gimbalAnglesToMotor1AngleData[angle2Index][angle1Index];

        case InterpolationTableType::GIMBAL_ANGLES_TO_MOTOR_2_ANGLES:
            return this->gimbalAnglesToMotor2AngleData[angle2Index][angle1Index];

        case InterpolationTableType::MOTOR_ANGLES_TO_GIMBAL_TIP_ANGLES:
            return this->motorToGimbalTipAngleData[angle2Index][angle1Index];

        case InterpolationTableType::MOTOR_ANGLES_TO_GIMBAL_TILT_ANGLES:
            return this->motorToGimbalTiltAngleData[angle2Index][angle1Index];
    }
}

/*! This method determines if bilinear interpolation is required to obtain the desired angles.
 @return bool
 @param angle1 [rad]
 @param angle2 [rad]
*/
bool TwoAxisGimbalLookupTables::bilinearInterpolationRequired(double angle1, double angle2) {
    double compare1a = round(fabs(angle1 / this->tableStepAngle));
    double compare2a = fabs(angle1 / this->tableStepAngle);
    double compare3a = fabs(compare2a - compare1a);

    double compare1b = round(fabs(angle2 / this->tableStepAngle));
    double compare2b = fabs(angle2 / this->tableStepAngle);
    double compare3b = fabs(compare2b - compare1b);

    return (!(compare3a < 1e-3) && !(compare3b < 1e-3));
}

/*! This method determines if no interpolation is required to obtain the desired angles.
 @return bool
 @param angle1 [rad]
 @param angle2 [rad]
*/
bool TwoAxisGimbalLookupTables::noInterpolationRequired(double angle1, double angle2) {
    double compare1a = round(fabs(angle1 / this->tableStepAngle));
    double compare2a = fabs(angle1 / this->tableStepAngle);
    double compare3a = fabs(compare2a - compare1a);

    double compare1b = round(fabs(angle2 / this->tableStepAngle));
    double compare2b = fabs(angle2 / this->tableStepAngle);
    double compare3b = fabs(compare2b - compare1b);

    return (compare3a < 1e-3 && compare3b < 1e-3);
}

/*! This method determines if linear interpolation is required to obtain the desired angles.
 @return bool
 @param angle [rad]
*/
bool TwoAxisGimbalLookupTables::linearInterpolationRequired(double angle) {
    double compare1 = round(fabs(angle / this->tableStepAngle));
    double compare2 = fabs(angle / this->tableStepAngle);
    double compare3 = fabs(compare2 - compare1);

    return (compare3 < 1e-3);
}

/*! This method calls the bilinear interpolation function to interpolate the desired angles from the given angles.
The case where the gimbal angles are at the edge of the interpolation table is checked and the method
computeTableEdgeCase() is called to determine the appropriate motor angles.
 @return void
*/
InterpolatedAngles TwoAxisGimbalLookupTables::bilinearlyInterpolateAngles(double angle1,
                                                                          double angle2,
                                                                          InterpolationType interpolationType) {
    // Find the upper and lower interpolation table angle bounds using the given angles
    double angle1LowerBound = this->tableStepAngle * floor(angle1 / this->tableStepAngle);
    double angle1UpperBound = this->tableStepAngle * ceil(angle1 / this->tableStepAngle);
    double angle2LowerBound = this->tableStepAngle * floor(angle2 / this->tableStepAngle);
    double angle2UpperBound = this->tableStepAngle * ceil(angle2 / this->tableStepAngle);

    // Use the provided interpolation type to save the interpolation table types
    InterpolationTableType interpolationTableType1;
    InterpolationTableType interpolationTableType2;
    switch (interpolationType) {
        case InterpolationType::GIMBAL_ANGLES_TO_MOTOR_ANGLES:
            interpolationTableType1 = InterpolationTableType::GIMBAL_ANGLES_TO_MOTOR_1_ANGLES;
            interpolationTableType2 = InterpolationTableType::GIMBAL_ANGLES_TO_MOTOR_2_ANGLES;
            break;

        case InterpolationType::MOTOR_ANGLES_TO_GIMBAL_ANGLES:
            interpolationTableType1 = InterpolationTableType::MOTOR_ANGLES_TO_GIMBAL_TIP_ANGLES;
            interpolationTableType2 = InterpolationTableType::MOTOR_ANGLES_TO_GIMBAL_TILT_ANGLES;
            break;
    }

    // Determine the bounding angles for the first angle
    double interpolatedAngle1LLBound = this->pullAngle(angle1LowerBound, angle2LowerBound, interpolationTableType1);
    double interpolatedAngle1LUBound = this->pullAngle(angle1LowerBound, angle2UpperBound, interpolationTableType1);
    double interpolatedAngle1ULBound = this->pullAngle(angle1UpperBound, angle2LowerBound, interpolationTableType1);
    double interpolatedAngle1UUBound = this->pullAngle(angle1UpperBound, angle2UpperBound, interpolationTableType1);

    // Determine the bounding angles for the second angle
    double interpolatedAngle2LLBound = this->pullAngle(angle1LowerBound, angle2LowerBound, interpolationTableType2);
    double interpolatedAngle2LUBound = this->pullAngle(angle1LowerBound, angle2UpperBound, interpolationTableType2);
    double interpolatedAngle2ULBound = this->pullAngle(angle1UpperBound, angle2LowerBound, interpolationTableType2);
    double interpolatedAngle2UUBound = this->pullAngle(angle1UpperBound, angle2UpperBound, interpolationTableType2);

    double interpolatedAngle1{};
    double interpolatedAngle2{};
    bool validInterpolation{};
    if ((interpolatedAngle1LLBound >= 0.0 && interpolatedAngle1LUBound >= 0.0 && interpolatedAngle1ULBound >= 0.0 &&
         interpolatedAngle1UUBound >= 0.0) ||
        (interpolationType == InterpolationType::MOTOR_ANGLES_TO_GIMBAL_ANGLES)) {
        interpolatedAngle1 = bilinearInterpolation(angle1LowerBound,
                                                   angle1UpperBound,
                                                   angle2LowerBound,
                                                   angle2UpperBound,
                                                   interpolatedAngle1LLBound,
                                                   interpolatedAngle1LUBound,
                                                   interpolatedAngle1ULBound,
                                                   interpolatedAngle1UUBound,
                                                   angle1,
                                                   angle2);

        interpolatedAngle2 = bilinearInterpolation(angle1LowerBound,
                                                   angle1UpperBound,
                                                   angle2LowerBound,
                                                   angle2UpperBound,
                                                   interpolatedAngle2LLBound,
                                                   interpolatedAngle2LUBound,
                                                   interpolatedAngle2ULBound,
                                                   interpolatedAngle2UUBound,
                                                   angle1,
                                                   angle2);
        validInterpolation = true;
    }

    InterpolatedAngles interpolatedAngles;
    interpolatedAngles.angle1 = interpolatedAngle1;
    interpolatedAngles.angle2 = interpolatedAngle2;
    interpolatedAngles.isValidInterpolation = validInterpolation;

    return interpolatedAngles;
}

/*! This method calls the linear interpolation function to interpolate using a fixed angle and a bounded angle.
For the gimbal-to-motor interpolation case where the gimbal angles are at the edges of the interpolation table,
the logic determines the appropriate motor angles to return. If 2/2 of the pulled motor angles are valid, linear
interpolation is used to determine the motor angle. If 1/2 motor angles are valid, the valid motor angle is directly
used as the result.
 @return InterpolatedAngles
 @param angle1 [rad] Angle 1 for linear interpolation
 @param angle2 [rad] Angle 2 for linear interpolation
 @param interpolationType Enumeration indicating the direction of interpolation
 @param fixedAngle Angle that is fixed for linear interpolation
*/
InterpolatedAngles TwoAxisGimbalLookupTables::linearlyInterpolateAngles(double angle1,
                                                                        double angle2,
                                                                        InterpolationType interpolationType,
                                                                        FixedAngle fixedAngle) {
    // Use the provided fixed angle to save the bounded angle (The bounded angle is the non-fixed angle)
    double boundedAngle{};
    if (fixedAngle == FixedAngle::ANGLE_1_FIXED) {
        boundedAngle = angle2;
    } else {
        boundedAngle = angle1;
    }

    // Find the upper and lower interpolation table bounds for the bounded angle
    double angleLowerBound = this->tableStepAngle * floor(boundedAngle / this->tableStepAngle);
    double angleUpperBound = this->tableStepAngle * ceil(boundedAngle / this->tableStepAngle);

    // Use the provided interpolation type to save the interpolation table types
    InterpolationTableType interpolationTableType1;
    InterpolationTableType interpolationTableType2;
    if (interpolationType == InterpolationType::GIMBAL_ANGLES_TO_MOTOR_ANGLES) {
        interpolationTableType1 = InterpolationTableType::GIMBAL_ANGLES_TO_MOTOR_1_ANGLES;
        interpolationTableType2 = InterpolationTableType::GIMBAL_ANGLES_TO_MOTOR_2_ANGLES;
    } else {
        interpolationTableType1 = InterpolationTableType::MOTOR_ANGLES_TO_GIMBAL_TIP_ANGLES;
        interpolationTableType2 = InterpolationTableType::MOTOR_ANGLES_TO_GIMBAL_TILT_ANGLES;
    }

    // Determine the bounding angles for linear interpolation
    double interpolatedAngle1LowerBound{};
    double interpolatedAngle1UpperBound{};
    double interpolatedAngle2LowerBound{};
    double interpolatedAngle2UpperBound{};
    switch (fixedAngle) {
        case FixedAngle::ANGLE_1_FIXED:
            interpolatedAngle1LowerBound = this->pullAngle(angle1, angleLowerBound, interpolationTableType1);
            interpolatedAngle1UpperBound = this->pullAngle(angle1, angleUpperBound, interpolationTableType1);
            interpolatedAngle2LowerBound = this->pullAngle(angle1, angleLowerBound, interpolationTableType2);
            interpolatedAngle2UpperBound = this->pullAngle(angle1, angleUpperBound, interpolationTableType2);
            break;
        case FixedAngle::ANGLE_2_FIXED:
            interpolatedAngle1LowerBound = this->pullAngle(angleLowerBound, angle2, interpolationTableType1);
            interpolatedAngle1UpperBound = this->pullAngle(angleUpperBound, angle2, interpolationTableType1);
            interpolatedAngle2LowerBound = this->pullAngle(angleLowerBound, angle2, interpolationTableType2);
            interpolatedAngle2UpperBound = this->pullAngle(angleUpperBound, angle2, interpolationTableType2);
            break;
    }

    // Linearly interpolate if the pulled angles are valid
    double interpolatedAngle1{};
    double interpolatedAngle2{};
    bool validInterpolation{};
    if ((interpolatedAngle1LowerBound >= 0.0 && interpolatedAngle1UpperBound >= 0.0) ||
        (interpolationType == InterpolationType::MOTOR_ANGLES_TO_GIMBAL_ANGLES)) {
        interpolatedAngle1 = linearInterpolation(
            angleLowerBound, angleUpperBound, interpolatedAngle1LowerBound, interpolatedAngle1UpperBound, boundedAngle);
        interpolatedAngle2 = linearInterpolation(
            angleLowerBound, angleUpperBound, interpolatedAngle2LowerBound, interpolatedAngle2UpperBound, boundedAngle);
        validInterpolation = true;
    }

    InterpolatedAngles interpolatedAngles;
    interpolatedAngles.angle1 = interpolatedAngle1;
    interpolatedAngles.angle2 = interpolatedAngle2;
    interpolatedAngles.isValidInterpolation = validInterpolation;

    return interpolatedAngles;
}
