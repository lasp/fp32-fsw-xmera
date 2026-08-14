#ifndef F32XMERA_THRUST_AXIS_TO_MOTOR_ANGLES_ALGORITHM_H
#define F32XMERA_THRUST_AXIS_TO_MOTOR_ANGLES_ALGORITHM_H

#include <math.h>
#include <stdlib.h>
#include <array>
#include <numbers>
#include <optional>

#include "thrustAxisToMotorAnglesTypes.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"

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
using GimbalToMotorAngleTableRowLayout = std::array<int, NUM_GIMBAL_TO_MOTOR_TABLE_ROWS>;
struct GimbalToMotorAngleTableLayout {
    GimbalToMotorAngleTableRowLayout rowStartStrideIndices;
    GimbalToMotorAngleTableRowLayout rowStartColIndices;
    int tipColIdxOffset{38};
    int tiltRowIdxOffset{55};
    float tableStepAngle{0.5F * std::numbers::pi_v<float> /
                         180.0F};  //!< [rad] Interpolation table motor discretization step
};

static constexpr int kNumTableCols = NUM_GIMBAL_TO_MOTOR_TABLE_COLS;
static constexpr int kNumTableRows = NUM_GIMBAL_TO_MOTOR_TABLE_ROWS;
static constexpr int kNumArrayElements = NUM_GIMBAL_TO_MOTOR_TABLE_ELEMENTS;

/*! @brief Validated configuration for ThrustAxisToMotorAnglesAlgorithm. Holds the motor travel
 * range and the two gimbal-to-motor interpolation tables. */
class ThrustAxisToMotorAnglesConfig final {
   public:
    static ThrustAxisToMotorAnglesConfig create(const StepperMotorAngleRange& angleRange,
                                                const GimbalToMotorAngleTable& gimbalToMotor1AngleData,
                                                const GimbalToMotorAngleTable& gimbalToMotor2AngleData,
                                                const GimbalToMotorAngleTableLayout tableLayout) {
        if (!isValidAngleRange(angleRange)) {
            FSW_THROW_INVALID_ARGUMENT(
                "thrustAxisToMotorAngles: minAngle and maxAngle must be in [0, 2*pi] with minAngle strictly less "
                "than maxAngle.");
        }
        if (!isValidTable(angleRange, gimbalToMotor1AngleData, tableLayout)) {
            FSW_THROW_INVALID_ARGUMENT("thrustAxisToMotorAngles: gimbalToMotor1AngleData data is not valid");
        }
        if (!isValidTable(angleRange, gimbalToMotor2AngleData, tableLayout)) {
            FSW_THROW_INVALID_ARGUMENT("thrustAxisToMotorAngles: gimbalToMotor2AngleData data is not valid");
        }

        return {angleRange, gimbalToMotor1AngleData, gimbalToMotor2AngleData, tableLayout};
    }

    static bool isValidAngleRange(const StepperMotorAngleRange& angleRange) {
        constexpr float twoPi = 2.0F * std::numbers::pi_v<float>;
        return angleRange.minAngle >= 0.0F && angleRange.minAngle <= twoPi && angleRange.maxAngle >= 0.0F &&
               angleRange.maxAngle <= twoPi && angleRange.minAngle < angleRange.maxAngle;
    }
    static bool isValidTable(const StepperMotorAngleRange& angleRange,
                             const GimbalToMotorAngleTable& table,
                             const GimbalToMotorAngleTableLayout& tableLayout) {
        // Table values must be finite and comply with provided motor angle range
        for (const auto& value : table) {
            if (!fsw::is_finite(value) || value < angleRange.minAngle || value > angleRange.maxAngle) {
                return false;
            }
        }

        // |tiltRowIdxOffset| and |tipColIdxOffset| cannot exceed the number of table rows/columns, respectively
        if (abs(tableLayout.tiltRowIdxOffset) >= kNumTableRows || abs(tableLayout.tipColIdxOffset) >= kNumTableCols) {
            return false;
        }

        // Gimbal angle ranges cannot exceed +- 90 degrees
        const float maxGimbalAngle = 90.0F * std::numbers::pi_v<float> / 180.0F;  // [rad]
        int maxNumData = static_cast<int>(roundf(maxGimbalAngle / tableLayout.tableStepAngle));
        if (tableLayout.tiltRowIdxOffset > maxNumData &&
            kNumTableRows - tableLayout.tiltRowIdxOffset - 1 > maxNumData) {
            return false;
        }
        if (tableLayout.tipColIdxOffset > maxNumData && kNumTableCols - tableLayout.tipColIdxOffset - 1 > maxNumData) {
            return false;
        }

        // Every rowStartStrideIndices array value must be greater than the previous and cannot be negative
        if (tableLayout.rowStartStrideIndices[0] < 0) {
            return false;
        }
        for (std::size_t value = 1; value < tableLayout.rowStartStrideIndices.size(); ++value) {
            if (tableLayout.rowStartStrideIndices[value] <= tableLayout.rowStartStrideIndices[value - 1]) {
                return false;
            }
        }

        // rowStartColIndices array values cannot be negative or exceed number of table columns
        for (const auto& value : tableLayout.rowStartColIndices) {
            if (value < 0 || value >= kNumTableCols) {
                return false;
            }
        }

        // tableStepAngle must be finite and greater than zero
        if (!fsw::is_finite(tableLayout.tableStepAngle) || tableLayout.tableStepAngle <= 0.0F) {
            return false;
        }

        return true;
    }

    const StepperMotorAngleRange& getAngleRange() const { return this->angleRange; }
    const GimbalToMotorAngleTable& getGimbalToMotor1AngleData() const { return this->gimbalToMotor1AngleData; }
    const GimbalToMotorAngleTable& getGimbalToMotor2AngleData() const { return this->gimbalToMotor2AngleData; }
    const GimbalToMotorAngleTableLayout& getTableLayout() const { return this->tableLayout; }

   private:
    ThrustAxisToMotorAnglesConfig(const StepperMotorAngleRange& angleRange,
                                  const GimbalToMotorAngleTable& gimbalToMotor1AngleData,
                                  const GimbalToMotorAngleTable& gimbalToMotor2AngleData,
                                  const GimbalToMotorAngleTableLayout tableLayout)
        : angleRange(angleRange),
          gimbalToMotor1AngleData(gimbalToMotor1AngleData),
          gimbalToMotor2AngleData(gimbalToMotor2AngleData),
          tableLayout(tableLayout) {}

    StepperMotorAngleRange angleRange;                //!< [rad] motor travel range
    GimbalToMotorAngleTable gimbalToMotor1AngleData;  //!< [rad] Gimbal-to-motor 1 angle interpolation table
    GimbalToMotorAngleTable gimbalToMotor2AngleData;  //!< [rad] Gimbal-to-motor 2 angle interpolation table
    GimbalToMotorAngleTableLayout tableLayout;        //!< [-] Interpolation table layout data
};

/*! @brief Pure algorithm: interpolates the stepper motor angles corresponding to the commanded
 * gimbal tip and tilt angles from the gimbal-to-motor lookup tables. */
class ThrustAxisToMotorAnglesAlgorithm final {
   public:
    explicit ThrustAxisToMotorAnglesAlgorithm(const ThrustAxisToMotorAnglesConfig& config);
    void setConfig(const ThrustAxisToMotorAnglesConfig& config);
    ThrustAxisToMotorAnglesOutput update(float gimbalAngle1, float gimbalAngle2) const;

   private:
    MotorAngles gimbalAnglesToMotorAngles(float gimbalAngle1, float gimbalAngle2) const;
    MotorAngles pullAngles(float gimbalAngle1, float gimbalAngle2) const;
    std::optional<int> getArrayIndex(const int rowIdx, const int colIdx) const;

    static constexpr float kInterpolationRemainderTolerance =
        1e-3F;  //!< Tolerance for treating a normalized gimbal angle as landing exactly on a table node
    static constexpr float kDefaultMotorAngle = 103.2242F * std::numbers::pi_v<float> / 180.0F;
    ThrustAxisToMotorAnglesConfig cfg;
};

#endif /* F32XMERA_THRUST_AXIS_TO_MOTOR_ANGLES_ALGORITHM_H */
