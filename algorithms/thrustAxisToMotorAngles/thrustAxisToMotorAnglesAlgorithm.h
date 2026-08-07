#ifndef F32XMERA_THRUST_AXIS_TO_MOTOR_ANGLES_ALGORITHM_H
#define F32XMERA_THRUST_AXIS_TO_MOTOR_ANGLES_ALGORITHM_H

#include <math.h>
#include <Eigen/Core>
#include <array>
#include <numbers>
#include <optional>

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

/*! @brief Type alias for gimbal-to-motor angle table data storage. */
using GimbalToMotorAngleTable = std::array<float, NUM_GIMBAL_TO_MOTOR_TABLE_ELEMENTS>;

/*! @brief Type alias for table row layout information. */
using GimbalToMotorAngleTableRowLayout = std::array<float, NUM_GIMBAL_TO_MOTOR_TABLE_ROWS>;

static constexpr int kNumTableCols = NUM_GIMBAL_TO_MOTOR_TABLE_COLS;

/*! @brief Validated configuration for ThrustAxisToMotorAnglesAlgorithm. Holds the gimbal
 * mount-frame DCM and the two gimbal-to-motor interpolation tables. */
class ThrustAxisToMotorAnglesConfig final {
   public:
    static ThrustAxisToMotorAnglesConfig create(const Eigen::Matrix3f& dcm_MB,
                                                const StepperMotorAngleRange& angleRange,
                                                const GimbalToMotorAngleTable& gimbalToMotor1AngleData,
                                                const GimbalToMotorAngleTable& gimbalToMotor2AngleData,
                                                const GimbalToMotorAngleTableRowLayout rowStartStrideIndices,
                                                const GimbalToMotorAngleTableRowLayout rowStartColIndices) {
        if (!isValidDcmMB(dcm_MB)) {
            FSW_THROW_INVALID_ARGUMENT("thrustAxisToMotorAngles: dcm_MB must be a valid DCM");
        }
        if (!isValidAngleRange(angleRange)) {
            FSW_THROW_INVALID_ARGUMENT(
                "thrustAxisToMotorAngles: minAngle and maxAngle must be in [0, 2*pi] with minAngle strictly less "
                "than maxAngle.");
        }
        if (!isValidTable(gimbalToMotor1AngleData, angleRange)) {
            FSW_THROW_INVALID_ARGUMENT("thrustAxisToMotorAngles: gimbalToMotor1AngleData data is not valid");
        }
        if (!isValidTable(gimbalToMotor2AngleData, angleRange)) {
            FSW_THROW_INVALID_ARGUMENT("thrustAxisToMotorAngles: gimbalToMotor2AngleData data is not valid");
        }
        if (!isValidRowStartStrideIndices(rowStartStrideIndices)) {
            FSW_THROW_INVALID_ARGUMENT("thrustAxisToMotorAngles: rowStartStrideIndices data is not valid");
        }
        if (!isValidRowStartColIndices(rowStartColIndices)) {
            FSW_THROW_INVALID_ARGUMENT("thrustAxisToMotorAngles: rowStartColIndices data is not valid");
        }
        return {dcm_MB,
                angleRange,
                gimbalToMotor1AngleData,
                gimbalToMotor2AngleData,
                rowStartStrideIndices,
                rowStartColIndices};
    }

    static bool isValidDcmMB(const Eigen::Matrix3f& dcm_MB) { return isValidDcm(dcm_MB); }
    static bool isValidAngleRange(const StepperMotorAngleRange& angleRange) {
        constexpr float twoPi = 2.0F * std::numbers::pi_v<float>;
        return angleRange.minAngle >= 0.0F && angleRange.minAngle <= twoPi && angleRange.maxAngle >= 0.0F &&
               angleRange.maxAngle <= twoPi && angleRange.minAngle < angleRange.maxAngle;
    }
    static bool isValidTable(const GimbalToMotorAngleTable& table, const StepperMotorAngleRange& angleRange) {
        for (const auto& value : table) {
            if (!fsw::is_finite(value) || value < angleRange.minAngle || value > angleRange.maxAngle) {
                return false;
            }
        }
        return true;
    }
    static bool isValidRowStartStrideIndices(const GimbalToMotorAngleTableRowLayout& rowStartStrideIndices) {
        if (!fsw::is_finite(rowStartStrideIndices[0])) {
            return false;
        }
        for (std::size_t value = 1; value < rowStartStrideIndices.size(); ++value) {
            if (!fsw::is_finite(rowStartStrideIndices[value]) ||
                rowStartStrideIndices[value] <= rowStartStrideIndices[value - 1]) {
                return false;
            }
        }
        return true;
    }
    static bool isValidRowStartColIndices(const GimbalToMotorAngleTableRowLayout& rowStartColIndices) {
        for (const auto& value : rowStartColIndices) {
            if (!fsw::is_finite(value) || value >= kNumTableCols) {
                return false;
            }
        }
        return true;
    }

    const Eigen::Matrix3f& getDcmMB() const { return this->dcm_MB; }
    const StepperMotorAngleRange& getAngleRange() const { return this->angleRange; }
    const GimbalToMotorAngleTable& getGimbalToMotor1AngleData() const { return this->gimbalToMotor1AngleData; }
    const GimbalToMotorAngleTable& getGimbalToMotor2AngleData() const { return this->gimbalToMotor2AngleData; }
    const GimbalToMotorAngleTableRowLayout& getRowStartStrideIndices() const { return this->rowStartStrideIndices; }
    const GimbalToMotorAngleTableRowLayout& getRowStartColIndices() const { return this->rowStartColIndices; }

   private:
    ThrustAxisToMotorAnglesConfig(const Eigen::Matrix3f& dcm_MB,
                                  const StepperMotorAngleRange& angleRange,
                                  const GimbalToMotorAngleTable& gimbalToMotor1AngleData,
                                  const GimbalToMotorAngleTable& gimbalToMotor2AngleData,
                                  const GimbalToMotorAngleTableRowLayout rowStartStrideIndices,
                                  const GimbalToMotorAngleTableRowLayout rowStartColIndices)
        : dcm_MB(dcm_MB),
          angleRange(angleRange),
          gimbalToMotor1AngleData(gimbalToMotor1AngleData),
          gimbalToMotor2AngleData(gimbalToMotor2AngleData),
          rowStartStrideIndices(rowStartStrideIndices),
          rowStartColIndices(rowStartColIndices) {}

    Eigen::Matrix3f dcm_MB;                           //!< DCM from body frame to gimbal mount frame
    StepperMotorAngleRange angleRange;                //!< [rad] motor travel range
    GimbalToMotorAngleTable gimbalToMotor1AngleData;  //!< [rad] Gimbal-to-motor 1 angle interpolation table
    GimbalToMotorAngleTable gimbalToMotor2AngleData;  //!< [rad] Gimbal-to-motor 2 angle interpolation table
    GimbalToMotorAngleTableRowLayout
        rowStartStrideIndices;  //!< [-] Stride indices for the starting location of the table rows
    GimbalToMotorAngleTableRowLayout
        rowStartColIndices;  //!< [-] Column indices for the starting location of the table rows
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
    std::optional<int> getArrayIndex(const int rowIdx, const int colIdx) const;
    static bool isBilinearInterpolationRequired(float gimbalAngle1, float gimbalAngle2);
    static bool isNoInterpolationRequired(float gimbalAngle1, float gimbalAngle2);
    static bool isLinearInterpolationRequired(float angle);
    MotorAngles bilinearlyInterpolateMotorAngles(float gimbalAngle1, float gimbalAngle2) const;
    MotorAngles linearlyInterpolateMotorAngles(float gimbalAngle1, float gimbalAngle2, FixedAngle fixedAngle) const;

    static constexpr int kNumTableRows = NUM_GIMBAL_TO_MOTOR_TABLE_ROWS;
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
