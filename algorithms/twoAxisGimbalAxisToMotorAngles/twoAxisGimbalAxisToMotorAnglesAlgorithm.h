// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef F32XMERA_TWO_AXIS_GIMBAL_AXIS_TO_MOTOR_ANGLES_ALGORITHM_H
#define F32XMERA_TWO_AXIS_GIMBAL_AXIS_TO_MOTOR_ANGLES_ALGORITHM_H

#include <Eigen/Core>
#include <array>
#include <numbers>

#define NUM_GIMBAL_TO_MOTOR_TABLE_ROWS 111
#define NUM_GIMBAL_TO_MOTOR_TABLE_COLS 76

const double DEG2RAD = std::numbers::pi / 180.0;

enum class FixedAngle { ANGLE_1_FIXED, ANGLE_2_FIXED };

struct MotorAngles {
    double angle1;
    double angle2;
    bool isValidInterpolation;
};

/*! @brief Output of the twoAxisGimbalAxisToMotorAngles algorithm. */
struct TwoAxisGimbalAxisToMotorAnglesOutput {
    double gimbalTipAngle{};      //!< [rad] Gimbal tip angle (sequential angle 1)
    double gimbalTiltAngle{};     //!< [rad] Gimbal tilt angle (sequential angle 2)
    double motorAngle1{};         //!< [rad] Motor 1 angle
    double motorAngle2{};         //!< [rad] Motor 2 angle
    bool isValidInterpolation{};  //!< Whether the interpolation produced a valid result
};

/*! @brief Pure algorithm: converts a commanded body-frame thrust direction into the gimbal
 * sequential tip and tilt angles and interpolates the corresponding stepper motor angles from the
 * gimbal-to-motor lookup tables. */
class TwoAxisGimbalAxisToMotorAnglesAlgorithm final {
   public:
    TwoAxisGimbalAxisToMotorAnglesAlgorithm(
        const std::array<std::array<double, NUM_GIMBAL_TO_MOTOR_TABLE_COLS>, NUM_GIMBAL_TO_MOTOR_TABLE_ROWS>&
            gimbalToMotor1Data,
        const std::array<std::array<double, NUM_GIMBAL_TO_MOTOR_TABLE_COLS>, NUM_GIMBAL_TO_MOTOR_TABLE_ROWS>&
            gimbalToMotor2Data);  //!< Constructor; populates the gimbal-to-motor interpolation tables

    TwoAxisGimbalAxisToMotorAnglesOutput update(
        const Eigen::Vector3d& thrustDirHat_B) const;  //!< Determine the gimbal and motor angles for a thrust direction

    void setDcmMB(const Eigen::Matrix3d& dcm_MB);  //!< Setter for dcm_MB (DCM from body frame to gimbal mount frame)
    const Eigen::Matrix3d& getDcmMB() const;       //!< Getter for dcm_MB (DCM from body frame to gimbal mount frame)

   private:
    MotorAngles gimbalAnglesToMotorAngles(double gimbalTipAngle, double gimbalTiltAngle)
        const;  //!< Method to determine the stepper motor angles given the gimbal sequential tip and tilt angles
    MotorAngles pullAngles(double gimbalAngle1, double gimbalAngle2) const;
    bool bilinearInterpolationRequired(double gimbalAngle1, double gimbalAngle2)
        const;  //!< Method to determine if bilinear interpolation is required
    bool noInterpolationRequired(double gimbalAngle1,
                                 double gimbalAngle2) const;  //!< Method to determine if no interpolation is required
    bool linearInterpolationRequired(double angle) const;  //!< Method to determine if linear interpolation is required
    MotorAngles bilinearlyInterpolateAngles(double gimbalAngle1, double gimbalAngle2) const;
    MotorAngles linearlyInterpolateAngles(double gimbalAngle1, double gimbalAngle2, FixedAngle fixedAngle) const;

    Eigen::Matrix3d dcm_MB;  //!< Attitude DCM for the gimbal mount frame (hub-fixed) relative to the hub body B frame
    double tableStepAngle{0.5 * DEG2RAD};  //!< [rad] Interpolation table motor discretization angle
    std::array<std::array<double, NUM_GIMBAL_TO_MOTOR_TABLE_COLS>, NUM_GIMBAL_TO_MOTOR_TABLE_ROWS>
        gimbalAnglesToMotor1AngleData;  //!< [rad] Gimbal-to-motor 1 angle interpolation table storage array
    std::array<std::array<double, NUM_GIMBAL_TO_MOTOR_TABLE_COLS>, NUM_GIMBAL_TO_MOTOR_TABLE_ROWS>
        gimbalAnglesToMotor2AngleData;  //!< [rad] Gimbal-to-motor 2 angle interpolation table storage array
};

#endif /* F32XMERA_TWO_AXIS_GIMBAL_AXIS_TO_MOTOR_ANGLES_ALGORITHM_H */
