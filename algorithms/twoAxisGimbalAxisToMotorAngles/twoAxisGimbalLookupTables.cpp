#include "twoAxisGimbalLookupTables.h"
#include <math.h>

#include "architecture/utilities/bilinearInterpolation.hpp"
#include "architecture/utilities/linearInterpolation.hpp"

const int tipAngleIdxOffset = 38;
const int tiltAngleIdxOffset = 55;

/*! Module constructor. The path to the interpolation table data files must be specified.
 @param gimbalToMotor1Data Gimbal-to-motor 1 angle data table
 @param gimbalToMotor2Data Gimbal-to-motor 1 angle data table
*/
TwoAxisGimbalLookupTables::TwoAxisGimbalLookupTables(
    const std::array<std::array<double, NUM_GIMBAL_TO_MOTOR_TABLE_COLS>, NUM_GIMBAL_TO_MOTOR_TABLE_ROWS>&
        gimbalToMotor1Data,
    const std::array<std::array<double, NUM_GIMBAL_TO_MOTOR_TABLE_COLS>, NUM_GIMBAL_TO_MOTOR_TABLE_ROWS>&
        gimbalToMotor2Data) {
    this->gimbalAnglesToMotor1AngleData = gimbalToMotor1Data;
    this->gimbalAnglesToMotor2AngleData = gimbalToMotor2Data;
    ;
}

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
MotorAngles TwoAxisGimbalLookupTables::pullAngles(double gimbalAngle1, double gimbalAngle2) const {
    gimbalAngle1 += tipAngleIdxOffset * this->tableStepAngle;
    gimbalAngle2 += tiltAngleIdxOffset * this->tableStepAngle;

    auto index1 = static_cast<int>(round(gimbalAngle1 / this->tableStepAngle));
    auto index2 = static_cast<int>(round(gimbalAngle2 / this->tableStepAngle));

    double motor1Angle = this->gimbalAnglesToMotor1AngleData[index2][index1];
    double motor2Angle = this->gimbalAnglesToMotor2AngleData[index2][index1];

    MotorAngles MotorAngles;
    MotorAngles.angle1 = motor1Angle;
    MotorAngles.angle2 = motor2Angle;
    MotorAngles.isValidInterpolation = true;

    return MotorAngles;
}

/*! This method determines if bilinear interpolation is required to obtain the desired angles.
 @return bool
 @param gimbalAngle1 [rad]
 @param gimbalAngle2 [rad]
*/
bool TwoAxisGimbalLookupTables::bilinearInterpolationRequired(double gimbalAngle1, double gimbalAngle2) {
    double motor1Rounded = round(fabs(gimbalAngle1 / this->tableStepAngle));
    double motor1Exact = fabs(gimbalAngle1 / this->tableStepAngle);
    double motor1Remainder = fabs(motor1Exact - motor1Rounded);

    double motor2Rounded = round(fabs(gimbalAngle2 / this->tableStepAngle));
    double motor2Exact = fabs(gimbalAngle2 / this->tableStepAngle);
    double motor2Remainder = fabs(motor2Exact - motor2Rounded);

    return motor1Remainder >= 1e-3 && motor2Remainder >= 1e-3;
}

/*! This method determines if no interpolation is required to obtain the desired angles.
 @return bool
 @param gimbalAngle1 [rad]
 @param gimbalAngle2 [rad]
*/
bool TwoAxisGimbalLookupTables::noInterpolationRequired(double gimbalAngle1, double gimbalAngle2) {
    double motor1Rounded = round(fabs(gimbalAngle1 / this->tableStepAngle));
    double motor1Exact = fabs(gimbalAngle1 / this->tableStepAngle);
    double motor1Remainder = fabs(motor1Exact - motor1Rounded);

    double motor2Rounded = round(fabs(gimbalAngle2 / this->tableStepAngle));
    double motor2Exact = fabs(gimbalAngle2 / this->tableStepAngle);
    double motor2Remainder = fabs(motor2Exact - motor2Rounded);

    return motor1Remainder < 1e-3 && motor2Remainder < 1e-3;
}

/*! This method determines if linear interpolation is required to obtain the desired angles.
 @return bool
 @param angle [rad]
*/
bool TwoAxisGimbalLookupTables::linearInterpolationRequired(double angle) {
    double rounded = round(fabs(angle / this->tableStepAngle));
    double exact = fabs(angle / this->tableStepAngle);
    double remainder = fabs(exact - rounded);

    return remainder < 1e-3;
}

/*! This method calls the bilinear interpolation function to interpolate the desired angles from the given angles.
The case where the gimbal angles are at the edge of the interpolation table is checked and the method
computeTableEdgeCase() is called to determine the appropriate motor angles.
 @return void
*/
MotorAngles TwoAxisGimbalLookupTables::bilinearlyInterpolateAngles(double gimbalAngle1, double gimbalAngle2) {
    // Find the upper and lower interpolation table angle bounds using the given angles
    double gimbalAngle1LBound = this->tableStepAngle * floor(gimbalAngle1 / this->tableStepAngle);
    double gimbalAngle1UBound = this->tableStepAngle * ceil(gimbalAngle1 / this->tableStepAngle);
    double gimbalAngle2LBound = this->tableStepAngle * floor(gimbalAngle2 / this->tableStepAngle);
    double gimbalAngle2UBound = this->tableStepAngle * ceil(gimbalAngle2 / this->tableStepAngle);

    // Determine the bounding angles
    MotorAngles motorLLBounds = this->pullAngles(gimbalAngle1LBound, gimbalAngle2LBound);
    MotorAngles motorLUBounds = this->pullAngles(gimbalAngle1LBound, gimbalAngle2UBound);
    MotorAngles motorULBounds = this->pullAngles(gimbalAngle1UBound, gimbalAngle2LBound);
    MotorAngles motorUUBounds = this->pullAngles(gimbalAngle1UBound, gimbalAngle2UBound);

    double motor1AngleLLBound = motorLLBounds.angle1;
    double motor1AngleLUBound = motorLUBounds.angle1;
    double motor1AngleULBound = motorULBounds.angle1;
    double motor1AngleUUBound = motorUUBounds.angle1;

    double motor2AngleLLBound = motorLLBounds.angle2;
    double motor2AngleLUBound = motorLUBounds.angle2;
    double motor2AngleULBound = motorULBounds.angle2;
    double motor2AngleUUBound = motorUUBounds.angle2;

    double motor1Angle{};
    double motor2Angle{};
    bool validInterpolation{};
    if (motor1AngleLLBound >= 0.0 && motor1AngleLUBound >= 0.0 && motor1AngleULBound >= 0.0 &&
        motor1AngleUUBound >= 0.0) {
        motor1Angle = bilinearInterpolation(gimbalAngle1LBound,
                                            gimbalAngle1UBound,
                                            gimbalAngle2LBound,
                                            gimbalAngle2UBound,
                                            motor1AngleLLBound,
                                            motor1AngleLUBound,
                                            motor1AngleULBound,
                                            motor1AngleUUBound,
                                            gimbalAngle1,
                                            gimbalAngle2);

        motor2Angle = bilinearInterpolation(gimbalAngle1LBound,
                                            gimbalAngle1UBound,
                                            gimbalAngle2LBound,
                                            gimbalAngle2UBound,
                                            motor2AngleLLBound,
                                            motor2AngleLUBound,
                                            motor2AngleULBound,
                                            motor2AngleUUBound,
                                            gimbalAngle1,
                                            gimbalAngle2);
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
 @param gimbalAngle1 [rad] Angle 1 for linear interpolation
 @param gimbalAngle2 [rad] Angle 2 for linear interpolation
 @param fixedAngle Angle that is fixed for linear interpolation
*/
MotorAngles TwoAxisGimbalLookupTables::linearlyInterpolateAngles(double gimbalAngle1,
                                                                 double gimbalAngle2,
                                                                 FixedAngle fixedAngle) {
    // Use the provided fixed angle to save the bounded angle (The bounded angle is the non-fixed angle)
    double boundedAngle{};
    if (fixedAngle == FixedAngle::ANGLE_1_FIXED) {
        boundedAngle = gimbalAngle2;
    } else {
        boundedAngle = gimbalAngle1;
    }

    // Find the upper and lower interpolation table bounds for the bounded angle
    double gimbalAngleLBound = this->tableStepAngle * floor(boundedAngle / this->tableStepAngle);
    double gimbalAngleUBound = this->tableStepAngle * ceil(boundedAngle / this->tableStepAngle);

    // Determine the bounding angles for linear interpolation
    MotorAngles lowerMotorBounds{};
    MotorAngles upperMotorBounds{};
    switch (fixedAngle) {
        case FixedAngle::ANGLE_1_FIXED:
            lowerMotorBounds = this->pullAngles(gimbalAngle1, gimbalAngleLBound);
            upperMotorBounds = this->pullAngles(gimbalAngle1, gimbalAngleUBound);

            break;
        case FixedAngle::ANGLE_2_FIXED:
            lowerMotorBounds = this->pullAngles(gimbalAngleLBound, gimbalAngle2);
            upperMotorBounds = this->pullAngles(gimbalAngleUBound, gimbalAngle2);

            break;
    }

    double motor1AngleLBound = lowerMotorBounds.angle1;
    double motor1AngleUBound = upperMotorBounds.angle1;

    double motor2AngleLBound = lowerMotorBounds.angle2;
    double motor2AngleUBound = upperMotorBounds.angle2;

    // Linearly interpolate if the pulled angles are valid
    double motor1Angle{};
    double motor2Angle{};
    bool validInterpolation{};
    if (motor1AngleLBound >= 0.0 && motor1AngleUBound >= 0.0) {
        motor1Angle = linearInterpolation(
            gimbalAngleLBound, gimbalAngleUBound, motor1AngleLBound, motor1AngleUBound, boundedAngle);
        motor2Angle = linearInterpolation(
            gimbalAngleLBound, gimbalAngleUBound, motor2AngleLBound, motor2AngleUBound, boundedAngle);
        validInterpolation = true;
    }

    MotorAngles MotorAngles;
    MotorAngles.angle1 = motor1Angle;
    MotorAngles.angle2 = motor2Angle;
    MotorAngles.isValidInterpolation = validInterpolation;

    return MotorAngles;
}
