#include "thrustAxisToMotorAnglesAlgorithm.h"

#include "utilities/fsw/bilinearInterpolation.h"
#include <math.h>

ThrustAxisToMotorAnglesAlgorithm::ThrustAxisToMotorAnglesAlgorithm(const ThrustAxisToMotorAnglesConfig& config)
    : cfg(config) {
    setConfig(config);
}

/*! Replaces the algorithm's configuration for runtime reconfiguration.
 @param config Validated configuration
*/
void ThrustAxisToMotorAnglesAlgorithm::setConfig(const ThrustAxisToMotorAnglesConfig& config) { this->cfg = config; }

/*! This method determines the stepper motor angles corresponding to the incoming gimbal tip and tilt angles.
 @return ThrustAxisToMotorAnglesOutput
 @param gimbalTipAngle [rad]
 @param gimbalTiltAngle [rad]
*/
ThrustAxisToMotorAnglesOutput ThrustAxisToMotorAnglesAlgorithm::update(float gimbalTipAngle,
                                                                       float gimbalTiltAngle) const {
    // Determine the required motor angles
    const MotorAngles motorAngles = this->gimbalAnglesToMotorAngles(gimbalTipAngle, gimbalTiltAngle);

    ThrustAxisToMotorAnglesOutput output{};
    output.motorAngle1 = motorAngles.angle1;
    output.motorAngle2 = motorAngles.angle2;
    output.isValidInterpolation = motorAngles.isValidInterpolation;

    return output;
}

/*! This method determines the stepper motor angles given the gimbal sequential tip and tilt angles.
 @return MotorAngles
 @param gimbalAngle1 [rad] Gimbal tip angle
 @param gimbalAngle2 [rad] Gimbal tilt angle
*/
MotorAngles ThrustAxisToMotorAnglesAlgorithm::gimbalAnglesToMotorAngles(const float gimbalAngle1,
                                                                        const float gimbalAngle2) const {
    // Determine the bounding gimbal angles
    const float gimbalAngle1LBound =
        this->cfg.getTableLayout().tableStepAngle * floorf(gimbalAngle1 / this->cfg.getTableLayout().tableStepAngle);
    const float gimbalAngle1UBound =
        this->cfg.getTableLayout().tableStepAngle * ceilf(gimbalAngle1 / this->cfg.getTableLayout().tableStepAngle);
    const float gimbalAngle2LBound =
        this->cfg.getTableLayout().tableStepAngle * floorf(gimbalAngle2 / this->cfg.getTableLayout().tableStepAngle);
    const float gimbalAngle2UBound =
        this->cfg.getTableLayout().tableStepAngle * ceilf(gimbalAngle2 / this->cfg.getTableLayout().tableStepAngle);

    // Determine the bounding motor angles
    const MotorAngles motorLLBounds = this->pullAngles(gimbalAngle1LBound, gimbalAngle2LBound);
    const MotorAngles motorLUBounds = this->pullAngles(gimbalAngle1LBound, gimbalAngle2UBound);
    const MotorAngles motorULBounds = this->pullAngles(gimbalAngle1UBound, gimbalAngle2LBound);
    const MotorAngles motorUUBounds = this->pullAngles(gimbalAngle1UBound, gimbalAngle2UBound);

    // Interpolate the motor angles if all bounding angles are valid
    MotorAngles motorAngles{.angle1 = kDefaultMotorAngle, .angle2 = kDefaultMotorAngle};
    if (motorLLBounds.isValidInterpolation && motorLUBounds.isValidInterpolation &&
        motorULBounds.isValidInterpolation && motorUUBounds.isValidInterpolation) {
        const std::optional<float> motor1Angle = bilinearInterpolation(gimbalAngle1LBound,
                                                                       gimbalAngle1UBound,
                                                                       gimbalAngle2LBound,
                                                                       gimbalAngle2UBound,
                                                                       motorLLBounds.angle1,
                                                                       motorLUBounds.angle1,
                                                                       motorULBounds.angle1,
                                                                       motorUUBounds.angle1,
                                                                       gimbalAngle1,
                                                                       gimbalAngle2);
        const std::optional<float> motor2Angle = bilinearInterpolation(gimbalAngle1LBound,
                                                                       gimbalAngle1UBound,
                                                                       gimbalAngle2LBound,
                                                                       gimbalAngle2UBound,
                                                                       motorLLBounds.angle2,
                                                                       motorLUBounds.angle2,
                                                                       motorULBounds.angle2,
                                                                       motorUUBounds.angle2,
                                                                       gimbalAngle1,
                                                                       gimbalAngle2);

        if (motor1Angle.has_value() && motor2Angle.has_value()) {
            motorAngles.angle1 = *motor1Angle;
            motorAngles.angle2 = *motor2Angle;
            motorAngles.isValidInterpolation = true;
        }
    }

    return motorAngles;
}

/*! This method pulls the motor angles from the provided interpolation tables.
 @return MotorAngles
 @param gimbalAngle1 [rad] Gimbal tip angle
 @param gimbalAngle2 [rad] Gimbal tilt angle
*/
MotorAngles ThrustAxisToMotorAnglesAlgorithm::pullAngles(float gimbalAngle1, float gimbalAngle2) const {
    // Shift the gimbal angles because the diamond is positioned at the center of the data table
    gimbalAngle1 +=
        static_cast<float>(this->cfg.getTableLayout().tipColIdxOffset) * this->cfg.getTableLayout().tableStepAngle;
    gimbalAngle2 +=
        static_cast<float>(this->cfg.getTableLayout().tiltRowIdxOffset) * this->cfg.getTableLayout().tableStepAngle;

    // Determine row and column table indices for the gimbal angles
    const auto colIdx = static_cast<int>(roundf(gimbalAngle1 / this->cfg.getTableLayout().tableStepAngle));
    const auto rowIdx = static_cast<int>(roundf(gimbalAngle2 / this->cfg.getTableLayout().tableStepAngle));

    // Default returned motor angles
    MotorAngles motorAngles{.angle1 = kDefaultMotorAngle, .angle2 = kDefaultMotorAngle};

    // Use the table indices to determine the single index required to pull data from the data table storage arrays
    const std::optional<int> arrayIndex = this->getArrayIndex(rowIdx, colIdx);
    if (arrayIndex.has_value()) {
        const float motor1Angle = this->cfg.getGimbalToMotor1AngleData()[*arrayIndex];
        const float motor2Angle = this->cfg.getGimbalToMotor2AngleData()[*arrayIndex];
        motorAngles.angle1 = motor1Angle;
        motorAngles.angle2 = motor2Angle;
        motorAngles.isValidInterpolation = true;
    }

    return motorAngles;
}

/*! This method determines the index required to extract the queried angle from the interpolation data table arrays
 @return std::optional<int>
 @param rowIdx [-]
 @param colIdx [-]
*/
std::optional<int> ThrustAxisToMotorAnglesAlgorithm::getArrayIndex(const int rowIdx, const int colIdx) const {
    // Invalid if either rowIdx or colIdx exceeds the table bounds
    if (colIdx < 0 || colIdx >= kNumTableCols || rowIdx < 0 || rowIdx >= kNumTableRows) {
        return std::nullopt;
    }

    // Determine the length of the row corresponding the queried value
    int rowLength{};
    if (rowIdx != NUM_GIMBAL_TO_MOTOR_TABLE_ROWS - 1) {
        rowLength = this->cfg.getTableLayout().rowStartStrideIndices[rowIdx + 1] -
                    this->cfg.getTableLayout().rowStartStrideIndices[rowIdx];
    } else {
        rowLength = NUM_GIMBAL_TO_MOTOR_TABLE_ELEMENTS - this->cfg.getTableLayout().rowStartStrideIndices[rowIdx];
    }

    // Determine the offset index of the queried value from the start of the row
    const int offsetIndex = colIdx - this->cfg.getTableLayout().rowStartColIndices[rowIdx];

    // Invalid if the offset index is greater than or equal to the row length
    if (offsetIndex >= rowLength || offsetIndex < 0) {
        return std::nullopt;
    }

    // Determine the index of the queried value in the data table arrays
    const int rowStartTableIndex = this->cfg.getTableLayout().rowStartStrideIndices[rowIdx];
    const int arrayIndex = rowStartTableIndex + offsetIndex;

    return arrayIndex;
}
