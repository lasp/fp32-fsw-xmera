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
 @return MotorAngles
 @param gimbalTipAngle [rad] Gimbal tip angle
 @param gimbalTiltAngle [rad] Gimbal tilt angle
*/
MotorAngles TwoAxisGimbalLookupTables::gimbalAnglesToMotorAngles(double gimbalTipAngle, double gimbalTiltAngle) {
    MotorAngles motorAngles{};

    if (this->bilinearInterpolationRequired(gimbalTipAngle, gimbalTiltAngle)) {
        motorAngles = this->bilinearlyInterpolateAngles(gimbalTipAngle, gimbalTiltAngle);
    } else if (this->noInterpolationRequired(gimbalTipAngle, gimbalTiltAngle)) {
        double motor1Angle =
            this->pullAngle(gimbalTipAngle, gimbalTiltAngle, InterpolationTableType::GIMBAL_ANGLES_TO_MOTOR_1_ANGLES);
        double motor2Angle =
            this->pullAngle(gimbalTipAngle, gimbalTiltAngle, InterpolationTableType::GIMBAL_ANGLES_TO_MOTOR_2_ANGLES);
        motorAngles.angle1 = motor1Angle;
        motorAngles.angle2 = motor2Angle;
        motorAngles.isValidInterpolation = true;
    } else if (this->linearInterpolationRequired(gimbalTipAngle)) {
        motorAngles = this->linearlyInterpolateAngles(gimbalTipAngle, gimbalTiltAngle, FixedAngle::ANGLE_1_FIXED);
    } else {
        motorAngles = this->linearlyInterpolateAngles(gimbalTipAngle, gimbalTiltAngle, FixedAngle::ANGLE_2_FIXED);
    }

    return motorAngles;
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
    angle1 += tipAngleIdxOffset * this->tableStepAngle;
    angle2 += tiltAngleIdxOffset * this->tableStepAngle;

    auto angle1Index = static_cast<int>(round(angle1 / this->tableStepAngle));
    auto angle2Index = static_cast<int>(round(angle2 / this->tableStepAngle));

    switch (interpolationTableType) {
        case InterpolationTableType::GIMBAL_ANGLES_TO_MOTOR_1_ANGLES:
            return this->gimbalAnglesToMotor1AngleData[angle2Index][angle1Index];

        case InterpolationTableType::GIMBAL_ANGLES_TO_MOTOR_2_ANGLES:
            return this->gimbalAnglesToMotor2AngleData[angle2Index][angle1Index];
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
MotorAngles TwoAxisGimbalLookupTables::bilinearlyInterpolateAngles(double angle1, double angle2) {
    // Find the upper and lower interpolation table angle bounds using the given angles
    double angle1LowerBound = this->tableStepAngle * floor(angle1 / this->tableStepAngle);
    double angle1UpperBound = this->tableStepAngle * ceil(angle1 / this->tableStepAngle);
    double angle2LowerBound = this->tableStepAngle * floor(angle2 / this->tableStepAngle);
    double angle2UpperBound = this->tableStepAngle * ceil(angle2 / this->tableStepAngle);

    // Determine the bounding angles for the first angle
    double motor1AngleLLBound =
        this->pullAngle(angle1LowerBound, angle2LowerBound, InterpolationTableType::GIMBAL_ANGLES_TO_MOTOR_1_ANGLES);
    double motor1AngleLUBound =
        this->pullAngle(angle1LowerBound, angle2UpperBound, InterpolationTableType::GIMBAL_ANGLES_TO_MOTOR_1_ANGLES);
    double motor1AngleULBound =
        this->pullAngle(angle1UpperBound, angle2LowerBound, InterpolationTableType::GIMBAL_ANGLES_TO_MOTOR_1_ANGLES);
    double motor1AngleUUBound =
        this->pullAngle(angle1UpperBound, angle2UpperBound, InterpolationTableType::GIMBAL_ANGLES_TO_MOTOR_1_ANGLES);

    // Determine the bounding angles for the second angle
    double motor2AngleLLBound =
        this->pullAngle(angle1LowerBound, angle2LowerBound, InterpolationTableType::GIMBAL_ANGLES_TO_MOTOR_2_ANGLES);
    double motor2AngleLUBound =
        this->pullAngle(angle1LowerBound, angle2UpperBound, InterpolationTableType::GIMBAL_ANGLES_TO_MOTOR_2_ANGLES);
    double motor2AngleULBound =
        this->pullAngle(angle1UpperBound, angle2LowerBound, InterpolationTableType::GIMBAL_ANGLES_TO_MOTOR_2_ANGLES);
    double motor2AngleUUBound =
        this->pullAngle(angle1UpperBound, angle2UpperBound, InterpolationTableType::GIMBAL_ANGLES_TO_MOTOR_2_ANGLES);

    double motor1Angle{};
    double motor2Angle{};
    bool validInterpolation{};
    if (motor1AngleLLBound >= 0.0 && motor1AngleLUBound >= 0.0 && motor1AngleULBound >= 0.0 &&
        motor1AngleUUBound >= 0.0) {
        motor1Angle = bilinearInterpolation(angle1LowerBound,
                                            angle1UpperBound,
                                            angle2LowerBound,
                                            angle2UpperBound,
                                            motor1AngleLLBound,
                                            motor1AngleLUBound,
                                            motor1AngleULBound,
                                            motor1AngleUUBound,
                                            angle1,
                                            angle2);

        motor2Angle = bilinearInterpolation(angle1LowerBound,
                                            angle1UpperBound,
                                            angle2LowerBound,
                                            angle2UpperBound,
                                            motor2AngleLLBound,
                                            motor2AngleLUBound,
                                            motor2AngleULBound,
                                            motor2AngleUUBound,
                                            angle1,
                                            angle2);
        validInterpolation = true;
    }

    MotorAngles MotorAngles;
    MotorAngles.angle1 = motor1Angle;
    MotorAngles.angle2 = motor2Angle;
    MotorAngles.isValidInterpolation = validInterpolation;

    return MotorAngles;
}

/*! This method calls the linear interpolation function to interpolate using a fixed angle and a bounded angle.
For the gimbal-to-motor interpolation case where the gimbal angles are at the edges of the interpolation table,
the logic determines the appropriate motor angles to return. If 2/2 of the pulled motor angles are valid, linear
interpolation is used to determine the motor angle. If 1/2 motor angles are valid, the valid motor angle is directly
used as the result.
 @return MotorAngles
 @param angle1 [rad] Angle 1 for linear interpolation
 @param angle2 [rad] Angle 2 for linear interpolation
 @param fixedAngle Angle that is fixed for linear interpolation
*/
MotorAngles TwoAxisGimbalLookupTables::linearlyInterpolateAngles(double angle1, double angle2, FixedAngle fixedAngle) {
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

    // Determine the bounding angles for linear interpolation
    double motor1AngleLowerBound{};
    double motor1AngleUpperBound{};
    double motor2AngleLowerBound{};
    double motor2AngleUpperBound{};
    switch (fixedAngle) {
        case FixedAngle::ANGLE_1_FIXED:
            motor1AngleLowerBound =
                this->pullAngle(angle1, angleLowerBound, InterpolationTableType::GIMBAL_ANGLES_TO_MOTOR_1_ANGLES);
            motor1AngleUpperBound =
                this->pullAngle(angle1, angleUpperBound, InterpolationTableType::GIMBAL_ANGLES_TO_MOTOR_1_ANGLES);
            motor2AngleLowerBound =
                this->pullAngle(angle1, angleLowerBound, InterpolationTableType::GIMBAL_ANGLES_TO_MOTOR_2_ANGLES);
            motor2AngleUpperBound =
                this->pullAngle(angle1, angleUpperBound, InterpolationTableType::GIMBAL_ANGLES_TO_MOTOR_2_ANGLES);
            break;
        case FixedAngle::ANGLE_2_FIXED:
            motor1AngleLowerBound =
                this->pullAngle(angleLowerBound, angle2, InterpolationTableType::GIMBAL_ANGLES_TO_MOTOR_1_ANGLES);
            motor1AngleUpperBound =
                this->pullAngle(angleUpperBound, angle2, InterpolationTableType::GIMBAL_ANGLES_TO_MOTOR_1_ANGLES);
            motor2AngleLowerBound =
                this->pullAngle(angleLowerBound, angle2, InterpolationTableType::GIMBAL_ANGLES_TO_MOTOR_2_ANGLES);
            motor2AngleUpperBound =
                this->pullAngle(angleUpperBound, angle2, InterpolationTableType::GIMBAL_ANGLES_TO_MOTOR_2_ANGLES);
            break;
    }

    // Linearly interpolate if the pulled angles are valid
    double motor1Angle{};
    double motor2Angle{};
    bool validInterpolation{};
    if (motor1AngleLowerBound >= 0.0 && motor1AngleUpperBound >= 0.0) {
        motor1Angle = linearInterpolation(
            angleLowerBound, angleUpperBound, motor1AngleLowerBound, motor1AngleUpperBound, boundedAngle);
        motor2Angle = linearInterpolation(
            angleLowerBound, angleUpperBound, motor2AngleLowerBound, motor2AngleUpperBound, boundedAngle);
        validInterpolation = true;
    }

    MotorAngles MotorAngles;
    MotorAngles.angle1 = motor1Angle;
    MotorAngles.angle2 = motor2Angle;
    MotorAngles.isValidInterpolation = validInterpolation;

    return MotorAngles;
}
