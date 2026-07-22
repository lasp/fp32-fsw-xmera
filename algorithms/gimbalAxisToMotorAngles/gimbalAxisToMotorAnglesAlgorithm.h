// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef F32XMERA_GIMBAL_AXIS_TO_MOTOR_ANGLES_ALGORITHM_H
#define F32XMERA_GIMBAL_AXIS_TO_MOTOR_ANGLES_ALGORITHM_H

#include <math.h>
#include <Eigen/Core>
#include <array>
#include <numbers>

#include "gimbalAxisToMotorAnglesTypes.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/validDcmCheck.h"

const float DEG2RAD = std::numbers::pi_v<float> / 180.0F;

enum class FixedAngle { ANGLE_1_FIXED, ANGLE_2_FIXED };

struct MotorAngles {
    float angle1;
    float angle2;
    bool isValidInterpolation;
};

//!< Gimbal-to-motor interpolation table storage type
using GimbalToMotorAngleTable =
    std::array<std::array<float, NUM_GIMBAL_TO_MOTOR_TABLE_COLS>, NUM_GIMBAL_TO_MOTOR_TABLE_ROWS>;

/*! @brief Validated configuration for GimbalAxisToMotorAnglesAlgorithm. Holds the gimbal
 * mount-frame DCM and the two gimbal-to-motor interpolation tables. */
class GimbalAxisToMotorAnglesConfig final {
   public:
    static GimbalAxisToMotorAnglesConfig create(const Eigen::Matrix3f& dcm_MB,
                                                const GimbalToMotorAngleTable& gimbalToMotor1AngleTable,
                                                const GimbalToMotorAngleTable& gimbalToMotor2AngleTable) {
        if (!isValidDcmMB(dcm_MB)) {
            FSW_THROW_INVALID_ARGUMENT("gimbalAxisToMotorAngles: dcm_MB must be a valid DCM");
        }
        if (!isValidTable(gimbalToMotor1AngleTable)) {
            FSW_THROW_INVALID_ARGUMENT("gimbalAxisToMotorAngles: gimbalToMotor1AngleTable must be finite");
        }
        if (!isValidTable(gimbalToMotor2AngleTable)) {
            FSW_THROW_INVALID_ARGUMENT("gimbalAxisToMotorAngles: gimbalToMotor2AngleTable must be finite");
        }
        return {dcm_MB, gimbalToMotor1AngleTable, gimbalToMotor2AngleTable};
    }

    static bool isValidDcmMB(const Eigen::Matrix3f& dcm_MB) { return isValidDcm(dcm_MB); }

    static bool isValidTable(const GimbalToMotorAngleTable& table) {
        for (const auto& row : table) {
            for (const float value : row) {
                if (isfinite(value) == 0) {
                    return false;
                }
            }
        }
        return true;
    }

    const Eigen::Matrix3f& getDcmMB() const { return this->dcm_MB; }
    const GimbalToMotorAngleTable& getGimbalToMotor1AngleTable() const { return this->gimbalToMotor1AngleTable; }
    const GimbalToMotorAngleTable& getGimbalToMotor2AngleTable() const { return this->gimbalToMotor2AngleTable; }

   private:
    GimbalAxisToMotorAnglesConfig(const Eigen::Matrix3f& dcm_MB,
                                  const GimbalToMotorAngleTable& gimbalToMotor1AngleTable,
                                  const GimbalToMotorAngleTable& gimbalToMotor2AngleTable)
        : dcm_MB(dcm_MB),
          gimbalToMotor1AngleTable(gimbalToMotor1AngleTable),
          gimbalToMotor2AngleTable(gimbalToMotor2AngleTable) {}

    Eigen::Matrix3f dcm_MB;                            //!< DCM from body frame to gimbal mount frame
    GimbalToMotorAngleTable gimbalToMotor1AngleTable;  //!< [rad] Gimbal-to-motor 1 angle interpolation table
    GimbalToMotorAngleTable gimbalToMotor2AngleTable;  //!< [rad] Gimbal-to-motor 2 angle interpolation table
};

/*! @brief Pure algorithm: converts a commanded body-frame thrust direction into the gimbal
 * sequential tip and tilt angles and interpolates the corresponding stepper motor angles from the
 * gimbal-to-motor lookup tables. */
class GimbalAxisToMotorAnglesAlgorithm final {
   public:
    explicit GimbalAxisToMotorAnglesAlgorithm(const GimbalAxisToMotorAnglesConfig& config);

    void setConfig(const GimbalAxisToMotorAnglesConfig& config);  //!< Runtime reconfiguration

    GimbalAxisToMotorAnglesOutput update(
        const Eigen::Vector3f& thrustDirHat_B) const;  //!< Determine the gimbal and motor angles for a thrust direction

   private:
    MotorAngles gimbalAnglesToMotorAngles(float gimbalTipAngle, float gimbalTiltAngle)
        const;  //!< Method to determine the stepper motor angles given the gimbal sequential tip and tilt angles
    MotorAngles pullAngles(float gimbalAngle1, float gimbalAngle2) const;
    bool bilinearInterpolationRequired(float gimbalAngle1, float gimbalAngle2)
        const;  //!< Method to determine if bilinear interpolation is required
    bool noInterpolationRequired(float gimbalAngle1,
                                 float gimbalAngle2) const;  //!< Method to determine if no interpolation is required
    bool linearInterpolationRequired(float angle) const;  //!< Method to determine if linear interpolation is required
    MotorAngles bilinearlyInterpolateAngles(float gimbalAngle1, float gimbalAngle2) const;
    MotorAngles linearlyInterpolateAngles(float gimbalAngle1, float gimbalAngle2, FixedAngle fixedAngle) const;

    static constexpr float kTableStepAngleDeg = 0.5F;  //!< [deg] Interpolation table motor discretization step
    static constexpr float kInterpolationRemainderTolerance =
        1e-3F;  //!< Tolerance for treating a normalized gimbal angle as landing exactly on a table node

    float tableStepAngle{kTableStepAngleDeg * DEG2RAD};  //!< [rad] Interpolation table motor discretization angle
    GimbalAxisToMotorAnglesConfig cfg;                   //!< Validated configuration (DCM + interpolation tables)
};

#endif /* F32XMERA_GIMBAL_AXIS_TO_MOTOR_ANGLES_ALGORITHM_H */
