#include "../thrustAxisToMotorAnglesAlgorithm.h"
#include "thrustAxisToMotorAnglesTestHelpers.hpp"

#include <gtest/gtest.h>

// ---------------------------------------------------------------------------
// Regression test
// ---------------------------------------------------------------------------

TEST(ThrustAxisToMotorAnglesTest, RegressionTest) {
    const float gimbalAngle1 = 1.7F * degToRad;
    const float gimbalAngle2 = -2.1F * degToRad;
    const Eigen::Vector3f tableCoeffs1{0.5F, 0.5F, 0.5F};
    const Eigen::Vector3f tableCoeffs2{0.25F, 0.75F, 0.25F};
    const int tipColIdxOffset = 37;
    const int tiltRowIdxOffset = 54;
    const float tableStepAngle = 0.5F * degToRad;

    testThrustAxisToMotorAnglesRegression(
        gimbalAngle1, gimbalAngle2, tableCoeffs1, tableCoeffs2, tipColIdxOffset, tiltRowIdxOffset, tableStepAngle);
}

// ---------------------------------------------------------------------------
// Setup tests (setter validation + round-trip)
// ---------------------------------------------------------------------------

TEST(ThrustAxisToMotorAnglesTest, SetupTest) {
    constexpr float twoPi = 2.0F * std::numbers::pi_v<float>;
    const StepperMotorAngleRange fullRange{0.0F, twoPi};
    // Create valid lookup table
    GimbalToMotorAngleTable gimbalToMotorAngleTable{};
    for (std::size_t i = 0; i < gimbalToMotorAngleTable.size(); ++i) {
        const float fraction = static_cast<float>(i) / static_cast<float>(gimbalToMotorAngleTable.size() - 1);
        gimbalToMotorAngleTable[i] = fullRange.minAngle + fraction * (fullRange.maxAngle - fullRange.minAngle);
    }

    // Create valid array for rowStartStrideIndices
    int validTipColIdxOffset = 37;
    GimbalToMotorAngleTableRowLayout validRowStartStrideIndices{};
    GimbalToMotorAngleTableRowLayout validRowStartColIndices{};
    for (int row = 0; row < NUM_GIMBAL_TO_MOTOR_TABLE_ROWS; ++row) {
        validRowStartStrideIndices[row] = row * row;
        validRowStartColIndices[row] = (row < validTipColIdxOffset) ? validTipColIdxOffset - row : 0;
    }

    // Create valid GimbalToMotorAngleTableLayout
    int validTiltRowIdxOffset = 54;
    float validTableStepAngle = 0.5F * degToRad;
    const GimbalToMotorAngleTableLayout validTableLayout{validRowStartStrideIndices,
                                                         validRowStartColIndices,
                                                         validTipColIdxOffset,
                                                         validTiltRowIdxOffset,
                                                         validTableStepAngle};

    // Valid config should not throw
    EXPECT_NO_THROW(ThrustAxisToMotorAnglesConfig::create(
        fullRange, gimbalToMotorAngleTable, gimbalToMotorAngleTable, validTableLayout));

    // ---------------------------------------------------------------------------
    // Checks for config parameter angleRange
    // ---------------------------------------------------------------------------

    // Invalid min/max motor angles should throw
    EXPECT_THROW(
        ThrustAxisToMotorAnglesConfig::create(
            StepperMotorAngleRange{-0.1F, 1.0F}, gimbalToMotorAngleTable, gimbalToMotorAngleTable, validTableLayout),
        fsw::invalid_argument);
    EXPECT_THROW(
        ThrustAxisToMotorAnglesConfig::create(
            StepperMotorAngleRange{-twoPi, 1.0F}, gimbalToMotorAngleTable, gimbalToMotorAngleTable, validTableLayout),
        fsw::invalid_argument);
    EXPECT_THROW(ThrustAxisToMotorAnglesConfig::create(StepperMotorAngleRange{0.0F, twoPi + 0.1F},
                                                       gimbalToMotorAngleTable,
                                                       gimbalToMotorAngleTable,
                                                       validTableLayout),
                 fsw::invalid_argument);
    EXPECT_THROW(
        ThrustAxisToMotorAnglesConfig::create(
            StepperMotorAngleRange{1.0F, 0.5F}, gimbalToMotorAngleTable, gimbalToMotorAngleTable, validTableLayout),
        fsw::invalid_argument);
    EXPECT_THROW(
        ThrustAxisToMotorAnglesConfig::create(
            StepperMotorAngleRange{1.0F, 1.0F}, gimbalToMotorAngleTable, gimbalToMotorAngleTable, validTableLayout),
        fsw::invalid_argument);

    // ----------------------------------------------------------------------------------------------
    // Checks for config parameters gimbalToMotor1AngleData, gimbalToMotor2AngleData, and tableLayout
    // ----------------------------------------------------------------------------------------------

    // Nonfinite table entries should throw
    GimbalToMotorAngleTable nonFiniteGimbalToMotorAngleTable = gimbalToMotorAngleTable;
    nonFiniteGimbalToMotorAngleTable.back() = std::numeric_limits<float>::infinity();
    EXPECT_THROW(ThrustAxisToMotorAnglesConfig::create(
                     fullRange, gimbalToMotorAngleTable, nonFiniteGimbalToMotorAngleTable, validTableLayout),
                 fsw::invalid_argument);

    // Tables with values outside of motor actuation limits should throw
    GimbalToMotorAngleTable aboveMotorRangeGimbalToMotorAngleTable = gimbalToMotorAngleTable;
    aboveMotorRangeGimbalToMotorAngleTable.back() = twoPi + 0.1F;
    EXPECT_THROW(ThrustAxisToMotorAnglesConfig::create(
                     fullRange, gimbalToMotorAngleTable, aboveMotorRangeGimbalToMotorAngleTable, validTableLayout),
                 fsw::invalid_argument);
    GimbalToMotorAngleTable belowMotorRangeGimbalToMotorAngleTable = gimbalToMotorAngleTable;
    belowMotorRangeGimbalToMotorAngleTable.back() = -0.1F;
    EXPECT_THROW(ThrustAxisToMotorAnglesConfig::create(
                     fullRange, gimbalToMotorAngleTable, belowMotorRangeGimbalToMotorAngleTable, validTableLayout),
                 fsw::invalid_argument);

    // Invalid values for tipColIdxOffset and tiltRowIdxOffset should throw
    // (Cannot exceed number of table rows/cols)
    EXPECT_THROW(ThrustAxisToMotorAnglesConfig::create(fullRange,
                                                       gimbalToMotorAngleTable,
                                                       gimbalToMotorAngleTable,
                                                       GimbalToMotorAngleTableLayout{validRowStartStrideIndices,
                                                                                     validRowStartColIndices,
                                                                                     kNumTableCols,
                                                                                     validTiltRowIdxOffset,
                                                                                     validTableStepAngle}),
                 fsw::invalid_argument);
    EXPECT_THROW(ThrustAxisToMotorAnglesConfig::create(fullRange,
                                                       gimbalToMotorAngleTable,
                                                       gimbalToMotorAngleTable,
                                                       GimbalToMotorAngleTableLayout{validRowStartStrideIndices,
                                                                                     validRowStartColIndices,
                                                                                     validTipColIdxOffset,
                                                                                     kNumTableRows,
                                                                                     validTableStepAngle}),
                 fsw::invalid_argument);

    // Tables with gimbal angle range greater than +- 90 degrees should throw
    GimbalToMotorAngleTableLayout badGimbalTipRangeTableLayout = validTableLayout;
    badGimbalTipRangeTableLayout.tableStepAngle = 2.0F * degToRad;
    badGimbalTipRangeTableLayout.tipColIdxOffset = 50;
    EXPECT_THROW(ThrustAxisToMotorAnglesConfig::create(
                     fullRange, gimbalToMotorAngleTable, gimbalToMotorAngleTable, badGimbalTipRangeTableLayout),
                 fsw::invalid_argument);
    GimbalToMotorAngleTableLayout badGimbalTiltRangeTableLayout = validTableLayout;
    badGimbalTiltRangeTableLayout.tableStepAngle = 2.0F * degToRad;
    badGimbalTipRangeTableLayout.tiltRowIdxOffset = 20;
    EXPECT_THROW(ThrustAxisToMotorAnglesConfig::create(
                     fullRange, gimbalToMotorAngleTable, gimbalToMotorAngleTable, badGimbalTiltRangeTableLayout),
                 fsw::invalid_argument);
    badGimbalTipRangeTableLayout.tiltRowIdxOffset = 50;
    EXPECT_THROW(ThrustAxisToMotorAnglesConfig::create(
                     fullRange, gimbalToMotorAngleTable, gimbalToMotorAngleTable, badGimbalTiltRangeTableLayout),
                 fsw::invalid_argument);

    // rowStartStrideIndices with negative values should throw
    GimbalToMotorAngleTableRowLayout negativeRowStartStrideIndices = validRowStartStrideIndices;
    negativeRowStartStrideIndices[0] = -3;
    EXPECT_THROW(ThrustAxisToMotorAnglesConfig::create(fullRange,
                                                       gimbalToMotorAngleTable,
                                                       gimbalToMotorAngleTable,
                                                       GimbalToMotorAngleTableLayout{negativeRowStartStrideIndices,
                                                                                     validRowStartColIndices,
                                                                                     validTipColIdxOffset,
                                                                                     validTiltRowIdxOffset,
                                                                                     validTableStepAngle}),
                 fsw::invalid_argument);

    // rowStartStrideIndices not in ascending order should throw
    GimbalToMotorAngleTableRowLayout descendingRowStartStrideIndices = validRowStartStrideIndices;
    descendingRowStartStrideIndices[5] = descendingRowStartStrideIndices[1];
    EXPECT_THROW(ThrustAxisToMotorAnglesConfig::create(fullRange,
                                                       gimbalToMotorAngleTable,
                                                       gimbalToMotorAngleTable,
                                                       GimbalToMotorAngleTableLayout{descendingRowStartStrideIndices,
                                                                                     validRowStartColIndices,
                                                                                     validTipColIdxOffset,
                                                                                     validTiltRowIdxOffset,
                                                                                     validTableStepAngle}),
                 fsw::invalid_argument);

    // rowStartColIndices with negative values should throw
    GimbalToMotorAngleTableRowLayout negativeRowStartColIndices = validRowStartColIndices;
    negativeRowStartColIndices.back() = -2;
    EXPECT_THROW(ThrustAxisToMotorAnglesConfig::create(fullRange,
                                                       gimbalToMotorAngleTable,
                                                       gimbalToMotorAngleTable,
                                                       GimbalToMotorAngleTableLayout{validRowStartStrideIndices,
                                                                                     negativeRowStartColIndices,
                                                                                     validTipColIdxOffset,
                                                                                     validTiltRowIdxOffset,
                                                                                     validTableStepAngle}),
                 fsw::invalid_argument);

    // rowStartColIndices values greater than fixed number of table columns should throw
    GimbalToMotorAngleTableRowLayout outOfTableBoundsRowStartColIndices = validRowStartColIndices;
    outOfTableBoundsRowStartColIndices[outOfTableBoundsRowStartColIndices.size() - 2] = kNumTableCols;
    outOfTableBoundsRowStartColIndices.back() = kNumTableCols + 1;
    EXPECT_THROW(ThrustAxisToMotorAnglesConfig::create(fullRange,
                                                       gimbalToMotorAngleTable,
                                                       gimbalToMotorAngleTable,
                                                       GimbalToMotorAngleTableLayout{validRowStartStrideIndices,
                                                                                     outOfTableBoundsRowStartColIndices,
                                                                                     validTipColIdxOffset,
                                                                                     validTiltRowIdxOffset,
                                                                                     validTableStepAngle}),
                 fsw::invalid_argument);

    // Invalid tableStepAngle should throw (Cannot be <= 0, must be finite)
    EXPECT_THROW(ThrustAxisToMotorAnglesConfig::create(fullRange,
                                                       gimbalToMotorAngleTable,
                                                       gimbalToMotorAngleTable,
                                                       GimbalToMotorAngleTableLayout{validRowStartStrideIndices,
                                                                                     validRowStartColIndices,
                                                                                     validTipColIdxOffset,
                                                                                     validTiltRowIdxOffset,
                                                                                     0.0F}),
                 fsw::invalid_argument);
    EXPECT_THROW(ThrustAxisToMotorAnglesConfig::create(fullRange,
                                                       gimbalToMotorAngleTable,
                                                       gimbalToMotorAngleTable,
                                                       GimbalToMotorAngleTableLayout{validRowStartStrideIndices,
                                                                                     validRowStartColIndices,
                                                                                     validTipColIdxOffset,
                                                                                     validTiltRowIdxOffset,
                                                                                     -1.0F}),
                 fsw::invalid_argument);
    EXPECT_THROW(
        ThrustAxisToMotorAnglesConfig::create(fullRange,
                                              gimbalToMotorAngleTable,
                                              gimbalToMotorAngleTable,
                                              GimbalToMotorAngleTableLayout{validRowStartStrideIndices,
                                                                            validRowStartColIndices,
                                                                            validTipColIdxOffset,
                                                                            validTiltRowIdxOffset,
                                                                            std::numeric_limits<float>::infinity()}),
        fsw::invalid_argument);
    EXPECT_THROW(ThrustAxisToMotorAnglesConfig::create(fullRange,
                                                       gimbalToMotorAngleTable,
                                                       gimbalToMotorAngleTable,
                                                       GimbalToMotorAngleTableLayout{validRowStartStrideIndices,
                                                                                     validRowStartColIndices,
                                                                                     validTipColIdxOffset,
                                                                                     validTiltRowIdxOffset,
                                                                                     std::nanf("")}),
                 fsw::invalid_argument);

    // -----------------
    // Config round-trip
    // -----------------
    auto config = ThrustAxisToMotorAnglesConfig::create(
        fullRange, gimbalToMotorAngleTable, gimbalToMotorAngleTable, validTableLayout);
    EXPECT_EQ(config.getAngleRange().minAngle, fullRange.minAngle);
    EXPECT_EQ(config.getAngleRange().maxAngle, fullRange.maxAngle);
    EXPECT_EQ(config.getGimbalToMotor1AngleData(), gimbalToMotorAngleTable);
    EXPECT_EQ(config.getGimbalToMotor2AngleData(), gimbalToMotorAngleTable);
    EXPECT_EQ(config.getTableLayout().rowStartStrideIndices, validTableLayout.rowStartStrideIndices);
    EXPECT_EQ(config.getTableLayout().rowStartColIndices, validTableLayout.rowStartColIndices);
    EXPECT_EQ(config.getTableLayout().tipColIdxOffset, validTableLayout.tipColIdxOffset);
    EXPECT_EQ(config.getTableLayout().tiltRowIdxOffset, validTableLayout.tiltRowIdxOffset);
    EXPECT_EQ(config.getTableLayout().tableStepAngle, validTableLayout.tableStepAngle);
}

// ---------------------------------------------------------------------------
// Property tests
// ---------------------------------------------------------------------------

// All output components are finite for valid inputs.
TEST(ThrustAxisToMotorAnglesTest, OutputIsFinite) {
    const float gimbalAngle1 = 1.7F * degToRad;
    const float gimbalAngle2 = -2.1F * degToRad;
    const Eigen::Vector3f tableCoeffs1{0.5F, 0.5F, 0.5F};
    const Eigen::Vector3f tableCoeffs2{0.25F, 0.75F, 0.25F};
    const int tipColIdxOffset = 37;
    const int tiltRowIdxOffset = 54;
    const float tableStepAngle = 0.5F * degToRad;

    propertyOutputIsFinite(
        gimbalAngle1, gimbalAngle2, tableCoeffs1, tableCoeffs2, tipColIdxOffset, tiltRowIdxOffset, tableStepAngle);
}

// Motor angles should be bounded for any inputs
TEST(ThrustAxisToMotorAnglesTest, MotorAnglesBounded) {
    const float gimbalAngle1 = 1.7F * degToRad;
    const float gimbalAngle2 = -2.1F * degToRad;
    const Eigen::Vector3f tableCoeffs1{0.5F, 0.5F, 0.5F};
    const Eigen::Vector3f tableCoeffs2{0.25F, 0.75F, 0.25F};
    const int tipColIdxOffset = 37;
    const int tiltRowIdxOffset = 54;
    const float tableStepAngle = 0.5F * degToRad;

    propertyMotorAnglesBounded(
        gimbalAngle1, gimbalAngle2, tableCoeffs1, tableCoeffs2, tipColIdxOffset, tiltRowIdxOffset, tableStepAngle);
}

// ---------------------------------------------------------------------------
// Edge-case tests
// ---------------------------------------------------------------------------

// Invalid interpolation on the first update call returns the default motor angles
TEST(ThrustAxisToMotorAnglesTest, InvalidFirstCallReturnsDefaultAngles) {
    const float invalidGimbalAngle1 = 30.0F * degToRad;
    const float invalidGimbalAngle2 = 0.0F;

    ThrustAxisToMotorAnglesAlgorithm alg(makeConfig());
    const ThrustAxisToMotorAnglesOutput result = alg.update(invalidGimbalAngle1, invalidGimbalAngle2);

    EXPECT_NEAR(result.motorAngle1, kReferenceDefaultMotorAngle, 1e-6F);
    EXPECT_NEAR(result.motorAngle2, kReferenceDefaultMotorAngle, 1e-6F);
}

// Invalid interpolation returns the motor angles determined by the previous update call
TEST(ThrustAxisToMotorAnglesTest, InvalidReturnsPreviousAngles) {
    const float validGimbalAngle1 = 1.7F * degToRad;
    const float validGimbalAngle2 = -2.1F * degToRad;

    ThrustAxisToMotorAnglesAlgorithm alg(makeConfig());
    const ThrustAxisToMotorAnglesOutput validResult = alg.update(validGimbalAngle1, validGimbalAngle2);

    // The valid update call must not return the default motor angles
    ASSERT_GT(std::abs(validResult.motorAngle1 - kReferenceDefaultMotorAngle), 1e-6F);
    ASSERT_GT(std::abs(validResult.motorAngle2 - kReferenceDefaultMotorAngle), 1e-6F);

    const float invalidGimbalAngle1 = 30.0F * degToRad;
    const float invalidGimbalAngle2 = 0.0F;

    const ThrustAxisToMotorAnglesOutput firstResult = alg.update(invalidGimbalAngle1, invalidGimbalAngle2);
    const ThrustAxisToMotorAnglesOutput secondResult = alg.update(invalidGimbalAngle1, invalidGimbalAngle2);

    EXPECT_NEAR(firstResult.motorAngle1, validResult.motorAngle1, 1e-6F);
    EXPECT_NEAR(firstResult.motorAngle2, validResult.motorAngle2, 1e-6F);
    EXPECT_NEAR(secondResult.motorAngle1, validResult.motorAngle1, 1e-6F);
    EXPECT_NEAR(secondResult.motorAngle2, validResult.motorAngle2, 1e-6F);
}

// Calling reInitialize() sets the motor angles in previousValidOutput back to the default motor angles
TEST(ThrustAxisToMotorAnglesTest, ReInitializeReturnsDefaultAngles) {
    const float validGimbalAngle1 = 1.7F * degToRad;
    const float validGimbalAngle2 = -2.1F * degToRad;

    ThrustAxisToMotorAnglesAlgorithm alg(makeConfig());
    alg.update(validGimbalAngle1, validGimbalAngle2);
    alg.reInitialize();

    const float invalidGimbalAngle1 = 30.0F * degToRad;
    const float invalidGimbalAngle2 = 0.0F;

    const ThrustAxisToMotorAnglesOutput result = alg.update(invalidGimbalAngle1, invalidGimbalAngle2);

    EXPECT_NEAR(result.motorAngle1, kReferenceDefaultMotorAngle, 1e-6F);
    EXPECT_NEAR(result.motorAngle2, kReferenceDefaultMotorAngle, 1e-6F);
}
