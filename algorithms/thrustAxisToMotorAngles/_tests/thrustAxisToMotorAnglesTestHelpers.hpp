#ifndef TEST_THRUSTAXISTOMOTORANGLES_H
#define TEST_THRUSTAXISTOMOTORANGLES_H

#include "thrustAxisToMotorAnglesAlgorithm.h"
#include "utilities/fsw/_tests/utilitiesHelpers.hpp"
#include "utilities/fsw/bilinearInterpolation.h"
#include "utilities/fsw/safeMath.h"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <architecture/utilities/rigidBodyKinematics.hpp>
#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <string>

constexpr float kReferenceDefaultMotorAngle = 103.2242F * std::numbers::pi_v<float> / 180.0F;
constexpr float degToRad = std::numbers::pi_v<float> / 180.0F;

/*! This method reads a provided CSV file into a fixed-size array. Each CSV file holds a
single line of comma-separated values, one value per array entry.
 @return std::array<ElementType, numValues>
 @param fileName [-] CSV file name relative to the interpolation table data directory
*/
template <typename ElementType, std::size_t numElements>
std::array<ElementType, numElements> readCsv(const std::string& fileName) {
    // THRUST_AXIS_TABLE_DATA_DIR is the directory holding the CSV files, supplied by CMake at configure time
    const std::string filePath = std::string{THRUST_AXIS_TABLE_DATA_DIR} + "/" + fileName;
    std::ifstream file(filePath);
    EXPECT_TRUE(file.is_open()) << "could not open " << filePath;

    // Read at most numElements values so that a longer than expected file cannot overrun the array
    std::array<ElementType, numElements> dataArray{};
    std::string valueText;
    std::size_t numValuesRead = 0;
    while (numValuesRead < numElements && std::getline(file, valueText, ',')) {
        dataArray[numValuesRead] = static_cast<ElementType>(std::stod(valueText));
        ++numValuesRead;
    }
    EXPECT_EQ(numValuesRead, numElements)
        << filePath << " holds " << numValuesRead << " values, expected " << numElements;

    return dataArray;
}

// This method returns an algorithm configuration for tests which import the table data csv files.
inline ThrustAxisToMotorAnglesConfig makeConfig() {
    // Read the csv files into storage arrays
    GimbalToMotorAngleTable gimbalToMotor1AngleTable =
        readCsv<float, NUM_GIMBAL_TO_MOTOR_TABLE_ELEMENTS>("gimbalToMotor1Angles.csv");
    GimbalToMotorAngleTable gimbalToMotor2AngleTable =
        readCsv<float, NUM_GIMBAL_TO_MOTOR_TABLE_ELEMENTS>("gimbalToMotor2Angles.csv");
    const GimbalToMotorAngleTableRowLayout rowStartStrideIndices =
        readCsv<int, NUM_GIMBAL_TO_MOTOR_TABLE_ROWS>("rowStartStrideIndices.csv");
    const GimbalToMotorAngleTableRowLayout rowStartColIndices =
        readCsv<int, NUM_GIMBAL_TO_MOTOR_TABLE_ROWS>("rowStartColIndices.csv");

    // Convert the table data to radians
    for (std::size_t idx = 0; idx < gimbalToMotor1AngleTable.size(); ++idx) {
        gimbalToMotor1AngleTable[idx] *= degToRad;
        gimbalToMotor2AngleTable[idx] *= degToRad;
    }

    constexpr int tipColIdxOffset = 37;
    constexpr int tiltRowIdxOffset = 54;
    constexpr float tableStepAngle = 0.5F * degToRad;  // [rad]

    const GimbalToMotorAngleTableLayout tableLayout{
        rowStartStrideIndices, rowStartColIndices, tipColIdxOffset, tiltRowIdxOffset, tableStepAngle};

    constexpr float minAngle = 0.0F;                              // [rad]
    constexpr float maxAngle = 2.0F * std::numbers::pi_v<float>;  // [rad]

    return ThrustAxisToMotorAnglesConfig::create(
        StepperMotorAngleRange{minAngle, maxAngle}, gimbalToMotor1AngleTable, gimbalToMotor2AngleTable, tableLayout);
}

// Test table layout. Every row holds kTestRowLength elements, and the first kTestLongRows rows hold one extra
// element so that the rows fill all NUM_GIMBAL_TO_MOTOR_TABLE_ELEMENTS entries of the tables
constexpr int kTestRowLength = NUM_GIMBAL_TO_MOTOR_TABLE_ELEMENTS / NUM_GIMBAL_TO_MOTOR_TABLE_ROWS;
constexpr int kTestLongRows = NUM_GIMBAL_TO_MOTOR_TABLE_ELEMENTS - (NUM_GIMBAL_TO_MOTOR_TABLE_ROWS * kTestRowLength);

/*! This method returns the number of elements held by the given test table row.
 @return int
 @param row [-]
*/
inline int testRowLength(const int row) { return (row < kTestLongRows) ? kTestRowLength + 1 : kTestRowLength; }

/*! This method returns the test table value at the given row and column. The value varies linearly with both the
row and the column so that bilinear interpolation reproduces it exactly.
 @return float
 @param coeffs [-] Table coefficients in the range [0, 1]
 @param row [-]
 @param col [-]
*/
inline float testTableValue(const Eigen::Vector3f& coeffs, const int row, const int col) {
    return 0.5F + coeffs[0] + 0.02F * coeffs[1] * static_cast<float>(col) + 0.02F * coeffs[2] * static_cast<float>(row);
}

// Reference getArrayIndex method
inline std::optional<int> referenceGetArrayIndex(const int rowIdx,
                                                 const int colIdx,
                                                 const GimbalToMotorAngleTableRowLayout& rowStartStrideIndices,
                                                 const GimbalToMotorAngleTableRowLayout& rowStartColIndices) {
    // Invalid if either rowIdx or colIdx exceeds the table bounds
    if (colIdx < 0 || colIdx >= kNumTableCols || rowIdx < 0 || rowIdx >= kNumTableRows) {
        return std::nullopt;
    }

    // Determine the length of the row corresponding the queried value
    int rowLength{};
    if (rowIdx != NUM_GIMBAL_TO_MOTOR_TABLE_ROWS - 1) {
        rowLength = rowStartStrideIndices[rowIdx + 1] - rowStartStrideIndices[rowIdx];
    } else {
        rowLength = NUM_GIMBAL_TO_MOTOR_TABLE_ELEMENTS - rowStartStrideIndices[rowIdx];
    }

    // Determine the offset index of the queried value from the start of the row
    const int offsetIndex = colIdx - rowStartColIndices[rowIdx];

    // Invalid if the offset index is greater than or equal to the row length
    if (offsetIndex >= rowLength || offsetIndex < 0) {
        return std::nullopt;
    }

    // Determine the index of the queried value in the data table arrays
    const int rowStartTableIndex = rowStartStrideIndices[rowIdx];
    const int arrayIndex = rowStartTableIndex + offsetIndex;

    return arrayIndex;
}

// Reference pullAngles method
inline MotorAngles referencePullAngles(float gimbalAngle1,
                                       float gimbalAngle2,
                                       const GimbalToMotorAngleTable& gimbalToMotor1AngleTable,
                                       const GimbalToMotorAngleTable& gimbalToMotor2AngleTable,
                                       const GimbalToMotorAngleTableRowLayout& rowStartStrideIndices,
                                       const GimbalToMotorAngleTableRowLayout& rowStartColIndices,
                                       const int tipColIdxOffset,
                                       const int tiltRowIdxOffset,
                                       const float tableStepAngle) {
    // Shift the gimbal angles because the diamond is positioned at the center of the data table
    gimbalAngle1 += static_cast<float>(tipColIdxOffset) * tableStepAngle;
    gimbalAngle2 += static_cast<float>(tiltRowIdxOffset) * tableStepAngle;

    // Determine row and column table indices for the gimbal angles
    const auto colIdx = static_cast<int>(roundf(gimbalAngle1 / tableStepAngle));
    const auto rowIdx = static_cast<int>(roundf(gimbalAngle2 / tableStepAngle));

    // Default returned motor angles
    MotorAngles motorAngles{.angle1 = kReferenceDefaultMotorAngle, .angle2 = kReferenceDefaultMotorAngle};

    // Use the table indices to determine the single index required to pull data from the data table storage arrays
    const std::optional<int> arrayIndex =
        referenceGetArrayIndex(rowIdx, colIdx, rowStartStrideIndices, rowStartColIndices);
    if (arrayIndex.has_value()) {
        const float motor1Angle = gimbalToMotor1AngleTable[*arrayIndex];
        const float motor2Angle = gimbalToMotor2AngleTable[*arrayIndex];
        motorAngles.angle1 = motor1Angle;
        motorAngles.angle2 = motor2Angle;
        motorAngles.isValidInterpolation = true;
    }

    return motorAngles;
}

// Reference implementation of the thrust axis to motor angles algorithm
inline ThrustAxisToMotorAnglesOutput referenceThrustAxisToMotorAngles(
    const float gimbalAngle1,
    const float gimbalAngle2,
    const float minAngle,
    const float maxAngle,
    const GimbalToMotorAngleTable& gimbalToMotor1AngleTable,
    const GimbalToMotorAngleTable& gimbalToMotor2AngleTable,
    const GimbalToMotorAngleTableRowLayout& rowStartStrideIndices,
    const GimbalToMotorAngleTableRowLayout& rowStartColIndices,
    const int tipColIdxOffset,
    const int tiltRowIdxOffset,
    const float tableStepAngle) {
    /*! Set default output */
    ThrustAxisToMotorAnglesOutput output{.motorAngle1 = kReferenceDefaultMotorAngle,
                                         .motorAngle2 = kReferenceDefaultMotorAngle};

    // Determine the bounding gimbal angles
    const float gimbalAngle1LBound = tableStepAngle * floorf(gimbalAngle1 / tableStepAngle);
    const float gimbalAngle1UBound = tableStepAngle * ceilf(gimbalAngle1 / tableStepAngle);
    const float gimbalAngle2LBound = tableStepAngle * floorf(gimbalAngle2 / tableStepAngle);
    const float gimbalAngle2UBound = tableStepAngle * ceilf(gimbalAngle2 / tableStepAngle);

    // Determine the bounding motor angles
    const MotorAngles motorLLBounds = referencePullAngles(gimbalAngle1LBound,
                                                          gimbalAngle2LBound,
                                                          gimbalToMotor1AngleTable,
                                                          gimbalToMotor2AngleTable,
                                                          rowStartStrideIndices,
                                                          rowStartColIndices,
                                                          tipColIdxOffset,
                                                          tiltRowIdxOffset,
                                                          tableStepAngle);
    const MotorAngles motorLUBounds = referencePullAngles(gimbalAngle1LBound,
                                                          gimbalAngle2UBound,
                                                          gimbalToMotor1AngleTable,
                                                          gimbalToMotor2AngleTable,
                                                          rowStartStrideIndices,
                                                          rowStartColIndices,
                                                          tipColIdxOffset,
                                                          tiltRowIdxOffset,
                                                          tableStepAngle);
    const MotorAngles motorULBounds = referencePullAngles(gimbalAngle1UBound,
                                                          gimbalAngle2LBound,
                                                          gimbalToMotor1AngleTable,
                                                          gimbalToMotor2AngleTable,
                                                          rowStartStrideIndices,
                                                          rowStartColIndices,
                                                          tipColIdxOffset,
                                                          tiltRowIdxOffset,
                                                          tableStepAngle);
    const MotorAngles motorUUBounds = referencePullAngles(gimbalAngle1UBound,
                                                          gimbalAngle2UBound,
                                                          gimbalToMotor1AngleTable,
                                                          gimbalToMotor2AngleTable,
                                                          rowStartStrideIndices,
                                                          rowStartColIndices,
                                                          tipColIdxOffset,
                                                          tiltRowIdxOffset,
                                                          tableStepAngle);

    // Interpolate the motor angles if all bounding angles are valid
    MotorAngles motorAngles{.angle1 = kReferenceDefaultMotorAngle, .angle2 = kReferenceDefaultMotorAngle};
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

    output.motorAngle1 = motorAngles.angle1;
    output.motorAngle2 = motorAngles.angle2;

    return output;
}

// ---------------------------------------------------------------------------
// Regression test helper function
// ---------------------------------------------------------------------------

inline void testThrustAxisToMotorAnglesRegression(const float gimbalAngle1,
                                                  const float gimbalAngle2,
                                                  const Eigen::Vector3f& tableCoeffs1,
                                                  const Eigen::Vector3f& tableCoeffs2,
                                                  const int tipColIdxOffset,
                                                  const int tiltRowIdxOffset,
                                                  const float tableStepAngle) {
    // Place the block of table data so that it holds the column corresponding to a zero tip angle
    const int maxRowLength = kTestRowLength + 1;
    int rowStartCol = tipColIdxOffset - kTestRowLength / 2;
    rowStartCol = (rowStartCol < 0) ? 0 : rowStartCol;
    rowStartCol = (rowStartCol > kNumTableCols - maxRowLength) ? kNumTableCols - maxRowLength : rowStartCol;

    // Build the table layout. Every row starts at the same column
    GimbalToMotorAngleTableRowLayout rowStartStrideIndices{};
    GimbalToMotorAngleTableRowLayout rowStartColIndices{};
    int strideIndex = 0;
    for (int row = 0; row < NUM_GIMBAL_TO_MOTOR_TABLE_ROWS; ++row) {
        rowStartStrideIndices[row] = strideIndex;
        rowStartColIndices[row] = rowStartCol;
        strideIndex += testRowLength(row);
    }

    // Build the tables
    GimbalToMotorAngleTable gimbalToMotor1AngleTable{};
    GimbalToMotorAngleTable gimbalToMotor2AngleTable{};
    for (int row = 0; row < NUM_GIMBAL_TO_MOTOR_TABLE_ROWS; ++row) {
        for (int offset = 0; offset < testRowLength(row); ++offset) {
            const int col = rowStartCol + offset;
            const int arrayIndex = rowStartStrideIndices[row] + offset;
            gimbalToMotor1AngleTable[arrayIndex] = testTableValue(tableCoeffs1, row, col);
            gimbalToMotor2AngleTable[arrayIndex] = testTableValue(tableCoeffs2, row, col);
        }
    }

    const GimbalToMotorAngleTableLayout tableLayout{
        rowStartStrideIndices, rowStartColIndices, tipColIdxOffset, tiltRowIdxOffset, tableStepAngle};
    constexpr float minAngle = 0.0F;
    constexpr float maxAngle = 2.0F * std::numbers::pi_v<float>;

    auto config = ThrustAxisToMotorAnglesConfig::create(
        StepperMotorAngleRange{minAngle, maxAngle}, gimbalToMotor1AngleTable, gimbalToMotor2AngleTable, tableLayout);
    ThrustAxisToMotorAnglesAlgorithm alg(config);
    const ThrustAxisToMotorAnglesOutput result = alg.update(gimbalAngle1, gimbalAngle2);
    const ThrustAxisToMotorAnglesOutput expected = referenceThrustAxisToMotorAngles(gimbalAngle1,
                                                                                    gimbalAngle2,
                                                                                    minAngle,
                                                                                    maxAngle,
                                                                                    gimbalToMotor1AngleTable,
                                                                                    gimbalToMotor2AngleTable,
                                                                                    rowStartStrideIndices,
                                                                                    rowStartColIndices,
                                                                                    tableLayout.tipColIdxOffset,
                                                                                    tableLayout.tiltRowIdxOffset,
                                                                                    tableLayout.tableStepAngle);

    constexpr float tol = 1e-5F;
    EXPECT_NEAR(result.motorAngle1, expected.motorAngle1, tol);
    EXPECT_NEAR(result.motorAngle2, expected.motorAngle2, tol);
}

// ---------------------------------------------------------------------------
// Property test helper functions
// ---------------------------------------------------------------------------

// All output components are finite for valid inputs.
inline void propertyOutputIsFinite(const float gimbalAngle1,
                                   const float gimbalAngle2,
                                   const Eigen::Vector3f& tableCoeffs1,
                                   const Eigen::Vector3f& tableCoeffs2,
                                   const int tipColIdxOffset,
                                   const int tiltRowIdxOffset,
                                   const float tableStepAngle) {
    // Place the block of table data so that it holds the column corresponding to a zero tip angle
    const int maxRowLength = kTestRowLength + 1;
    int rowStartCol = tipColIdxOffset - kTestRowLength / 2;
    rowStartCol = (rowStartCol < 0) ? 0 : rowStartCol;
    rowStartCol = (rowStartCol > kNumTableCols - maxRowLength) ? kNumTableCols - maxRowLength : rowStartCol;

    // Build the table layout. Every row starts at the same column
    GimbalToMotorAngleTableRowLayout rowStartStrideIndices{};
    GimbalToMotorAngleTableRowLayout rowStartColIndices{};
    int strideIndex = 0;
    for (int row = 0; row < NUM_GIMBAL_TO_MOTOR_TABLE_ROWS; ++row) {
        rowStartStrideIndices[row] = strideIndex;
        rowStartColIndices[row] = rowStartCol;
        strideIndex += testRowLength(row);
    }

    // Build the tables
    GimbalToMotorAngleTable gimbalToMotor1AngleTable{};
    GimbalToMotorAngleTable gimbalToMotor2AngleTable{};
    for (int row = 0; row < NUM_GIMBAL_TO_MOTOR_TABLE_ROWS; ++row) {
        for (int offset = 0; offset < testRowLength(row); ++offset) {
            const int col = rowStartCol + offset;
            const int arrayIndex = rowStartStrideIndices[row] + offset;
            gimbalToMotor1AngleTable[arrayIndex] = testTableValue(tableCoeffs1, row, col);
            gimbalToMotor2AngleTable[arrayIndex] = testTableValue(tableCoeffs2, row, col);
        }
    }

    const GimbalToMotorAngleTableLayout tableLayout{
        rowStartStrideIndices, rowStartColIndices, tipColIdxOffset, tiltRowIdxOffset, tableStepAngle};
    constexpr float minAngle = 0.0F;
    constexpr float maxAngle = 2.0F * std::numbers::pi_v<float>;

    auto config = ThrustAxisToMotorAnglesConfig::create(
        StepperMotorAngleRange{minAngle, maxAngle}, gimbalToMotor1AngleTable, gimbalToMotor2AngleTable, tableLayout);
    ThrustAxisToMotorAnglesAlgorithm alg(config);
    const ThrustAxisToMotorAnglesOutput result = alg.update(gimbalAngle1, gimbalAngle2);

    EXPECT_TRUE(std::isfinite(result.motorAngle1));
    EXPECT_TRUE(std::isfinite(result.motorAngle2));
}

// Motor angles should be bounded for any inputs
inline void propertyMotorAnglesBounded(const float gimbalAngle1,
                                       const float gimbalAngle2,
                                       const Eigen::Vector3f& tableCoeffs1,
                                       const Eigen::Vector3f& tableCoeffs2,
                                       const int tipColIdxOffset,
                                       const int tiltRowIdxOffset,
                                       const float tableStepAngle) {
    // Place the block of table data so that it holds the column corresponding to a zero tip angle
    const int maxRowLength = kTestRowLength + 1;
    int rowStartCol = tipColIdxOffset - kTestRowLength / 2;
    rowStartCol = (rowStartCol < 0) ? 0 : rowStartCol;
    rowStartCol = (rowStartCol > kNumTableCols - maxRowLength) ? kNumTableCols - maxRowLength : rowStartCol;

    // Build the table layout. Every row starts at the same column
    GimbalToMotorAngleTableRowLayout rowStartStrideIndices{};
    GimbalToMotorAngleTableRowLayout rowStartColIndices{};
    int strideIndex = 0;
    for (int row = 0; row < NUM_GIMBAL_TO_MOTOR_TABLE_ROWS; ++row) {
        rowStartStrideIndices[row] = strideIndex;
        rowStartColIndices[row] = rowStartCol;
        strideIndex += testRowLength(row);
    }

    // Build the tables
    GimbalToMotorAngleTable gimbalToMotor1AngleTable{};
    GimbalToMotorAngleTable gimbalToMotor2AngleTable{};
    for (int row = 0; row < NUM_GIMBAL_TO_MOTOR_TABLE_ROWS; ++row) {
        for (int offset = 0; offset < testRowLength(row); ++offset) {
            const int col = rowStartCol + offset;
            const int arrayIndex = rowStartStrideIndices[row] + offset;
            gimbalToMotor1AngleTable[arrayIndex] = testTableValue(tableCoeffs1, row, col);
            gimbalToMotor2AngleTable[arrayIndex] = testTableValue(tableCoeffs2, row, col);
        }
    }

    const GimbalToMotorAngleTableLayout tableLayout{
        rowStartStrideIndices, rowStartColIndices, tipColIdxOffset, tiltRowIdxOffset, tableStepAngle};
    constexpr float minAngle = 0.0F;
    constexpr float maxAngle = 2.0F * std::numbers::pi_v<float>;

    auto config = ThrustAxisToMotorAnglesConfig::create(
        StepperMotorAngleRange{minAngle, maxAngle}, gimbalToMotor1AngleTable, gimbalToMotor2AngleTable, tableLayout);
    ThrustAxisToMotorAnglesAlgorithm alg(config);
    const ThrustAxisToMotorAnglesOutput result = alg.update(gimbalAngle1, gimbalAngle2);

    // Check motor angles do not exceed the set motor bounds
    EXPECT_GE(result.motorAngle1, minAngle - 1e-6F);
    EXPECT_LE(result.motorAngle1, maxAngle + 1e-6F);
    EXPECT_GE(result.motorAngle2, minAngle - 1e-6F);
    EXPECT_LE(result.motorAngle2, maxAngle + 1e-6F);

    // Check motor angles do not exceed tighter interpolation bounds
    // Determine the bounding gimbal angles
    const float gimbalAngle1LBound = tableStepAngle * floorf(gimbalAngle1 / tableStepAngle);
    const float gimbalAngle1UBound = tableStepAngle * ceilf(gimbalAngle1 / tableStepAngle);
    const float gimbalAngle2LBound = tableStepAngle * floorf(gimbalAngle2 / tableStepAngle);
    const float gimbalAngle2UBound = tableStepAngle * ceilf(gimbalAngle2 / tableStepAngle);

    // Determine the bounding motor angles
    const MotorAngles motorLLBounds = referencePullAngles(gimbalAngle1LBound,
                                                          gimbalAngle2LBound,
                                                          gimbalToMotor1AngleTable,
                                                          gimbalToMotor2AngleTable,
                                                          rowStartStrideIndices,
                                                          rowStartColIndices,
                                                          tipColIdxOffset,
                                                          tiltRowIdxOffset,
                                                          tableStepAngle);
    const MotorAngles motorLUBounds = referencePullAngles(gimbalAngle1LBound,
                                                          gimbalAngle2UBound,
                                                          gimbalToMotor1AngleTable,
                                                          gimbalToMotor2AngleTable,
                                                          rowStartStrideIndices,
                                                          rowStartColIndices,
                                                          tipColIdxOffset,
                                                          tiltRowIdxOffset,
                                                          tableStepAngle);
    const MotorAngles motorULBounds = referencePullAngles(gimbalAngle1UBound,
                                                          gimbalAngle2LBound,
                                                          gimbalToMotor1AngleTable,
                                                          gimbalToMotor2AngleTable,
                                                          rowStartStrideIndices,
                                                          rowStartColIndices,
                                                          tipColIdxOffset,
                                                          tiltRowIdxOffset,
                                                          tableStepAngle);
    const MotorAngles motorUUBounds = referencePullAngles(gimbalAngle1UBound,
                                                          gimbalAngle2UBound,
                                                          gimbalToMotor1AngleTable,
                                                          gimbalToMotor2AngleTable,
                                                          rowStartStrideIndices,
                                                          rowStartColIndices,
                                                          tipColIdxOffset,
                                                          tiltRowIdxOffset,
                                                          tableStepAngle);

    const bool allAnglesValid = motorLLBounds.isValidInterpolation && motorLUBounds.isValidInterpolation &&
                                motorULBounds.isValidInterpolation && motorUUBounds.isValidInterpolation;
    constexpr float tol = 1e-6F;
    if (allAnglesValid) {
        // Determine the tighter bounds on the interpolated motor angles based on the pulled table data
        const float minMotorAngle1 =
            std::min({motorLLBounds.angle1, motorLUBounds.angle1, motorULBounds.angle1, motorUUBounds.angle1});
        const float maxMotorAngle1 =
            std::max({motorLLBounds.angle1, motorLUBounds.angle1, motorULBounds.angle1, motorUUBounds.angle1});
        const float minMotorAngle2 =
            std::min({motorLLBounds.angle2, motorLUBounds.angle2, motorULBounds.angle2, motorUUBounds.angle2});
        const float maxMotorAngle2 =
            std::max({motorLLBounds.angle2, motorLUBounds.angle2, motorULBounds.angle2, motorUUBounds.angle2});

        // Check that the motor angles are within the tighter bounds
        EXPECT_GE(result.motorAngle1, minMotorAngle1 - tol);
        EXPECT_LE(result.motorAngle1, maxMotorAngle1 + tol);
        EXPECT_GE(result.motorAngle2, minMotorAngle2 - tol);
        EXPECT_LE(result.motorAngle2, maxMotorAngle2 + tol);
    } else {
        // If the request falls outside the table, the default motor angles corresponding to the gimbal (0,0) home
        // position are returned
        EXPECT_NEAR(result.motorAngle1, kReferenceDefaultMotorAngle, tol);
        EXPECT_NEAR(result.motorAngle2, kReferenceDefaultMotorAngle, tol);
    }
}

#endif  // TEST_THRUSTAXISTOMOTORANGLES_H
