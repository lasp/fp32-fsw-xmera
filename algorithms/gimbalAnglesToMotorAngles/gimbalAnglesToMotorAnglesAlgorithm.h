#ifndef F32XMERA_GIMBAL_ANGLES_TO_MOTOR_ANGLES_ALGORITHM_H
#define F32XMERA_GIMBAL_ANGLES_TO_MOTOR_ANGLES_ALGORITHM_H

#include <math.h>
#include <stdlib.h>
#include <array>
#include <numbers>
#include <optional>

#include "gimbalAnglesToMotorAnglesTypes.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"

static constexpr float kDefaultMotorAngle = 103.2242F * std::numbers::pi_v<float> / 180.0F;
static constexpr int kNumTableCols = NUM_GIMBAL_TO_MOTOR_TABLE_COLS;
static constexpr int kNumTableRows = NUM_GIMBAL_TO_MOTOR_TABLE_ROWS;
static constexpr int kNumArrayElements = NUM_GIMBAL_TO_MOTOR_TABLE_ELEMENTS;

/*! @brief Structure containing the algorithm output. */
struct GimbalAnglesToMotorAnglesOutput {
    float motorAngle1{kDefaultMotorAngle};  //!< [rad] Motor 1 angle
    float motorAngle2{kDefaultMotorAngle};  //!< [rad] Motor 2 angle
};

/*! @brief Structure containing the motor angles and boolean indicating whether the angles are valid. */
struct MotorAngles {
    float angle1;               //!< [rad] Motor 1 angle
    float angle2;               //!< [rad] Motor 2 angle
    bool isValidInterpolation;  //!< Boolean indicating whether the motor angles are valid
};

/*! @brief Structure containing the motor angular travel range. */
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
    int tipColIdxOffset{37};
    int tiltRowIdxOffset{54};
    float tableStepAngle{0.5F * std::numbers::pi_v<float> /
                         180.0F};  //!< [rad] Interpolation table motor discretization step
};

/*! @brief Validated configuration for the gimbal angles to motor angles algorithm.
 * Holds the motor travel range, lookup table data for each motor, and table layout information. */
class GimbalAnglesToMotorAnglesConfig final {
   public:
    static GimbalAnglesToMotorAnglesConfig create(const StepperMotorAngleRange& angleRange,
                                                  const GimbalToMotorAngleTable& gimbalToMotor1AngleData,
                                                  const GimbalToMotorAngleTable& gimbalToMotor2AngleData,
                                                  const GimbalToMotorAngleTableLayout tableLayout) {
        if (!isValidAngleRange(angleRange)) {
            FSW_THROW_INVALID_ARGUMENT(
                "gimbalAnglesToMotorAngles: minAngle and maxAngle must be in [0, 2*pi] with minAngle strictly less "
                "than maxAngle.");
        }
        if (!isValidTable(angleRange, gimbalToMotor1AngleData, tableLayout)) {
            FSW_THROW_INVALID_ARGUMENT("gimbalAnglesToMotorAngles: gimbalToMotor1AngleData data is not valid");
        }
        if (!isValidTable(angleRange, gimbalToMotor2AngleData, tableLayout)) {
            FSW_THROW_INVALID_ARGUMENT("gimbalAnglesToMotorAngles: gimbalToMotor2AngleData data is not valid");
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
    GimbalAnglesToMotorAnglesConfig(const StepperMotorAngleRange& angleRange,
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

/*! @brief Pure algorithm: determines the stepper motor angles corresponding to the incoming
 * gimbal tip and tilt angles. */
class GimbalAnglesToMotorAnglesAlgorithm final {
   public:
    explicit GimbalAnglesToMotorAnglesAlgorithm(const GimbalAnglesToMotorAnglesConfig& config);
    void setConfig(const GimbalAnglesToMotorAnglesConfig& config);
    void reInitialize();
    GimbalAnglesToMotorAnglesOutput update(float gimbalAngle1, float gimbalAngle2);

   private:
    MotorAngles pullAngles(float gimbalAngle1, float gimbalAngle2) const;
    std::optional<int> getArrayIndex(const int rowIdx, const int colIdx) const;
    GimbalAnglesToMotorAnglesOutput previousValidOutput{};  //!< [rad] Last valid algorithm output
    GimbalAnglesToMotorAnglesConfig cfg;
};

#endif /* F32XMERA_GIMBAL_ANGLES_TO_MOTOR_ANGLES_ALGORITHM_H */
