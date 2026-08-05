#ifndef F32XMERA_THRUST_AXIS_TO_MOTOR_ANGLES_ALGORITHM_H
#define F32XMERA_THRUST_AXIS_TO_MOTOR_ANGLES_ALGORITHM_H

#include <math.h>
#include <Eigen/Core>
#include <array>
#include <numbers>

#include "thrustAxisToMotorAnglesTypes.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include "utilities/fsw/validDcmCheck.h"

enum class FixedAngle { ANGLE_1_FIXED, ANGLE_2_FIXED };

struct MotorAngles {
    float angle1;
    float angle2;
    bool isValidInterpolation;
};

/*! @brief Motor angular travel range in body-frame radians. */
struct StepperMotorAngleRange {
    float minAngle{0.0F};                              //!< [rad] lower bound of the motor travel range
    float maxAngle{2.0F * std::numbers::pi_v<float>};  //!< [rad] upper bound of the motor travel range
};

//!< Gimbal-to-motor interpolation table storage type
using GimbalToMotorAngleTable =
    std::array<std::array<float, NUM_GIMBAL_TO_MOTOR_TABLE_COLS>, NUM_GIMBAL_TO_MOTOR_TABLE_ROWS>;

/*! @brief Validated configuration for ThrustAxisToMotorAnglesAlgorithm. Holds the gimbal
 * mount-frame DCM and the two gimbal-to-motor interpolation tables. */
class ThrustAxisToMotorAnglesConfig final {
   public:
    static ThrustAxisToMotorAnglesConfig create(const Eigen::Matrix3f& dcm_MB,
                                                const StepperMotorAngleRange& angleRange,
                                                const GimbalToMotorAngleTable& gimbalToMotor1AngleTable,
                                                const GimbalToMotorAngleTable& gimbalToMotor2AngleTable) {
        if (!isValidDcmMB(dcm_MB)) {
            FSW_THROW_INVALID_ARGUMENT("thrustAxisToMotorAngles: dcm_MB must be a valid DCM");
        }
        if (!isValidAngleRange(angleRange)) {
            FSW_THROW_INVALID_ARGUMENT(
                "thrustAxisToMotorAngles: minAngle and maxAngle must be in [0, 2*pi] with minAngle strictly less "
                "than maxAngle.");
        }
        if (!isValidTable(gimbalToMotor1AngleTable, angleRange)) {
            FSW_THROW_INVALID_ARGUMENT("thrustAxisToMotorAngles: gimbalToMotor1AngleTable data is not valid");
        }
        if (!isValidTable(gimbalToMotor2AngleTable, angleRange)) {
            FSW_THROW_INVALID_ARGUMENT("thrustAxisToMotorAngles: gimbalToMotor2AngleTable data is not valid");
        }
        return {dcm_MB, angleRange, gimbalToMotor1AngleTable, gimbalToMotor2AngleTable};
    }

    static bool isValidDcmMB(const Eigen::Matrix3f& dcm_MB) { return isValidDcm(dcm_MB); }
    static bool isValidAngleRange(const StepperMotorAngleRange& angleRange) {
        constexpr float twoPi = 2.0F * std::numbers::pi_v<float>;
        return angleRange.minAngle >= 0.0F && angleRange.minAngle <= twoPi && angleRange.maxAngle >= 0.0F &&
               angleRange.maxAngle <= twoPi && angleRange.minAngle < angleRange.maxAngle;
    }
    static bool isValidTable(const GimbalToMotorAngleTable& table, const StepperMotorAngleRange& angleRange) {
        for (const auto& row : table) {
            for (const float value : row) {
                if (!fsw::is_finite(value) ||
                    (value != -1.0F && (value < angleRange.minAngle || value > angleRange.maxAngle))) {
                    return false;
                }
            }
        }
        return true;
    }

    const Eigen::Matrix3f& getDcmMB() const { return this->dcm_MB; }
    const StepperMotorAngleRange& getAngleRange() const { return this->angleRange; }
    const GimbalToMotorAngleTable& getGimbalToMotor1AngleTable() const { return this->gimbalToMotor1AngleTable; }
    const GimbalToMotorAngleTable& getGimbalToMotor2AngleTable() const { return this->gimbalToMotor2AngleTable; }

   private:
    ThrustAxisToMotorAnglesConfig(const Eigen::Matrix3f& dcm_MB,
                                  const StepperMotorAngleRange& angleRange,
                                  const GimbalToMotorAngleTable& gimbalToMotor1AngleTable,
                                  const GimbalToMotorAngleTable& gimbalToMotor2AngleTable)
        : dcm_MB(dcm_MB),
          angleRange(angleRange),
          gimbalToMotor1AngleTable(gimbalToMotor1AngleTable),
          gimbalToMotor2AngleTable(gimbalToMotor2AngleTable) {}

    Eigen::Matrix3f dcm_MB;                            //!< DCM from body frame to gimbal mount frame
    StepperMotorAngleRange angleRange;                 //!< [rad] motor travel range
    GimbalToMotorAngleTable gimbalToMotor1AngleTable;  //!< [rad] Gimbal-to-motor 1 angle interpolation table
    GimbalToMotorAngleTable gimbalToMotor2AngleTable;  //!< [rad] Gimbal-to-motor 2 angle interpolation table
};

/*! @brief Pure algorithm: converts a commanded body-frame thrust direction into the gimbal
 * sequential tip and tilt angles and interpolates the corresponding stepper motor angles from the
 * gimbal-to-motor lookup tables. */
class ThrustAxisToMotorAnglesAlgorithm final {
   public:
    explicit ThrustAxisToMotorAnglesAlgorithm(const ThrustAxisToMotorAnglesConfig& config);
    void setConfig(const ThrustAxisToMotorAnglesConfig& config);
    ThrustAxisToMotorAnglesOutput update(const Eigen::Vector3f& thrustHat_B) const;

   private:
    MotorAngles gimbalAnglesToMotorAngles(float gimbalTipAngle, float gimbalTiltAngle) const;
    MotorAngles pullAngles(float gimbalAngle1, float gimbalAngle2) const;
    static bool isBilinearInterpolationRequired(float gimbalAngle1, float gimbalAngle2);
    static bool isNoInterpolationRequired(float gimbalAngle1, float gimbalAngle2);
    static bool isLinearInterpolationRequired(float angle);
    MotorAngles bilinearlyInterpolateMotorAngles(float gimbalAngle1, float gimbalAngle2) const;
    MotorAngles linearlyInterpolateMotorAngles(float gimbalAngle1, float gimbalAngle2, FixedAngle fixedAngle) const;

    static constexpr int kNumTableRows = NUM_GIMBAL_TO_MOTOR_TABLE_ROWS;
    static constexpr int kNumTableCols = NUM_GIMBAL_TO_MOTOR_TABLE_COLS;
    static constexpr int kTipColIdxOffset = 38;
    static constexpr int kTiltRowIdxOffset = 55;
    static constexpr float kTableStepAngle =
        0.5F * std::numbers::pi_v<float> / 180.0F;  //!< [rad] Interpolation table motor discretization step
    static constexpr float kInterpolationRemainderTolerance =
        1e-3F;  //!< Tolerance for treating a normalized gimbal angle as landing exactly on a table node
    static constexpr float kDefaultMotorAngle = 103.2242F * std::numbers::pi_v<float> / 180.0F;
    ThrustAxisToMotorAnglesConfig cfg;
};

#endif /* F32XMERA_THRUST_AXIS_TO_MOTOR_ANGLES_ALGORITHM_H */
