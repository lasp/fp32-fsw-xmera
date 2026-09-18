#include "cssWeightedLeastSquaresTestHelpers.hpp"
#include "utilities/fsw/freestandingInvalidArgument.h"

#include <gtest/gtest.h>

// ---------------------------------------------------------------------------
// Regression tests — update() against the independent fp64 reference.
// ---------------------------------------------------------------------------

TEST(CssWeightedLeastSquaresTest, RegressionOverDeterminedFit) {
    runRegressionCase(referenceInputs(), readingsFor(Eigen::Vector3d{1.0, 0.0, 0.0}));
}

TEST(CssWeightedLeastSquaresTest, RegressionWeightedFit) {
    ConstellationInputs inputs = referenceInputs();
    inputs.useWeights = true;
    runRegressionCase(inputs, readingsFor(Eigen::Vector3d{0.0, 1.0, 0.0}));
}

// A heading 40.68 degrees off +z in the x-z plane lights only two sensors, which takes the minimum norm
// branch instead of the weighted one.
TEST(CssWeightedLeastSquaresTest, RegressionMinimumNormFit) {
    const double latitude = 40.68 * M_PI / 180.0;
    runRegressionCase(referenceInputs(), readingsFor(Eigen::Vector3d{std::sin(latitude), 0.0, std::cos(latitude)}));
}

// ---------------------------------------------------------------------------
// Configuration tests — what the factory accepts and what it rejects.
// ---------------------------------------------------------------------------

TEST(CssWeightedLeastSquaresTest, ConfigAcceptsTheReferenceSetup) {
    const ConstellationInputs inputs = referenceInputs();
    BuiltConfig built{};
    ASSERT_TRUE(buildConfig(inputs, built));

    const CssWeightedLeastSquaresConfig config = makeConfig(inputs, built);
    EXPECT_FALSE(config.getUseWeights());
    EXPECT_FLOAT_EQ(config.getSensorUseThresh(), kSensorUseThresh);
    EXPECT_FLOAT_EQ(config.getControlPeriod(), kControlPeriod);

    // The factory normalizes the boresights so the estimator can rely on exact unit vectors.
    for (int i = 0; i < kMaxNumCssSensors; ++i) {
        EXPECT_NEAR(config.getCssNHat_B().row(i).norm(), 1.0F, 1e-6F) << "boresight " << i;
    }
}

TEST(CssWeightedLeastSquaresTest, ConfigRejectsBoresightThatIsNotUnit) {
    const ConstellationInputs inputs = referenceInputs();
    BuiltConfig built{};
    ASSERT_TRUE(buildConfig(inputs, built));

    built.cssSensors.at(2).nHat_B *= 1.1F;  // outside the 1e-3 tolerance on the norm
    EXPECT_THROW((void)makeConfig(inputs, built), fsw::invalid_argument);

    built.cssSensors.at(2).nHat_B.setZero();
    EXPECT_THROW((void)makeConfig(inputs, built), fsw::invalid_argument);
}

TEST(CssWeightedLeastSquaresTest, ConfigAcceptsAnUnavailableSensor) {
    const ConstellationInputs inputs = referenceInputs();
    BuiltConfig built{};
    ASSERT_TRUE(buildConfig(inputs, built));

    built.cssSensors.at(1).availability = fsw::DeviceAvailability::Unavailable;
    EXPECT_NO_THROW((void)makeConfig(inputs, built));

    // An unavailable sensor never reaches the fit, so its boresight is not held to the unit rule.
    built.cssSensors.at(1).nHat_B.setZero();
    EXPECT_NO_THROW((void)makeConfig(inputs, built));
}

TEST(CssWeightedLeastSquaresTest, ConfigRejectsEverySensorUnavailable) {
    const ConstellationInputs inputs = referenceInputs();
    BuiltConfig built{};
    ASSERT_TRUE(buildConfig(inputs, built));

    for (auto& sensor : built.cssSensors) {
        sensor.availability = fsw::DeviceAvailability::Unavailable;
    }
    EXPECT_THROW((void)makeConfig(inputs, built), fsw::invalid_argument);
}

TEST(CssWeightedLeastSquaresTest, ConfigRejectsUseThresholdOutOfRange) {
    ConstellationInputs inputs = referenceInputs();
    BuiltConfig built{};
    ASSERT_TRUE(buildConfig(inputs, built));

    // A sensor cannot report a negative cosine, so a negative threshold would only admit the sensors that
    // see no sun at all.
    for (const float threshold : {-0.1F, 1.1F, std::numeric_limits<float>::quiet_NaN()}) {
        inputs.sensorUseThresh = threshold;
        EXPECT_THROW((void)makeConfig(inputs, built), fsw::invalid_argument) << "threshold " << threshold;
    }

    for (const float threshold : {0.0F, 1.0F}) {
        inputs.sensorUseThresh = threshold;
        EXPECT_NO_THROW((void)makeConfig(inputs, built)) << "threshold " << threshold;
    }
}

TEST(CssWeightedLeastSquaresTest, ConfigRejectsControlPeriodThatIsNotPositive) {
    ConstellationInputs inputs = referenceInputs();
    BuiltConfig built{};
    ASSERT_TRUE(buildConfig(inputs, built));

    for (const float period :
         {0.0F, -0.5F, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
        inputs.controlPeriod = period;
        EXPECT_THROW((void)makeConfig(inputs, built), fsw::invalid_argument) << "period " << period;
    }
}

// ---------------------------------------------------------------------------
// Property tests — fixed representative inputs exercising invariants. The same
// helpers are re-run under fuzz inputs in test_cssWeightedLeastSquares_fuzz.cpp.
// ---------------------------------------------------------------------------

TEST(CssWeightedLeastSquaresTest, PropertyOutputIsFinite) {
    propertyOutputIsFinite(referenceInputs(), readingsFor(Eigen::Vector3d{0.3, -0.5, 0.8}.normalized()));
}

TEST(CssWeightedLeastSquaresTest, PropertyHeadingIsUnitOrZero) {
    propertyHeadingIsUnitOrZero(referenceInputs(), readingsFor(Eigen::Vector3d{1.0, 0.0, 0.0}));
}

TEST(CssWeightedLeastSquaresTest, PropertyResidualsPaddedWithZeros) {
    propertyResidualsPaddedWithZeros(referenceInputs(), readingsFor(Eigen::Vector3d{1.0, 0.0, 0.0}));
}

TEST(CssWeightedLeastSquaresTest, PropertyDisabledSensorIgnored) {
    propertyDisabledSensorIgnored(referenceInputs(), readingsFor(Eigen::Vector3d{1.0, 0.0, 0.0}), 0U);
}

TEST(CssWeightedLeastSquaresTest, PropertyRotationEquivariance) {
    propertyRotationEquivariance(
        referenceInputs(), readingsFor(Eigen::Vector3d{1.0, 0.0, 0.0}), Eigen::Vector3f{0.2F, -0.4F, 0.9F});
}

TEST(CssWeightedLeastSquaresTest, PropertyRateOrthogonalToHeading) {
    propertyRateOrthogonalToHeading(
        referenceInputs(), readingsFor(Eigen::Vector3d{1.0, 0.0, 0.0}), readingsFor(Eigen::Vector3d{0.0, 1.0, 0.0}));
}

// ---------------------------------------------------------------------------
// Edge-case tests — the boundaries of the fit, the rate and the sensor set.
// ---------------------------------------------------------------------------

namespace {

// Builds the estimator over the reference constellation with the given tuning.
CssWeightedLeastSquaresAlgorithm makeReferenceAlgorithm(bool useWeights = false,
                                                        const std::vector<bool>& available = allAvailable()) {
    ConstellationInputs inputs = referenceInputs();
    inputs.useWeights = useWeights;
    inputs.available = available;
    BuiltConfig built{};
    EXPECT_TRUE(buildConfig(inputs, built));
    return CssWeightedLeastSquaresAlgorithm{makeConfig(inputs, built)};
}

}  // namespace

TEST(CssWeightedLeastSquaresTest, NoReadingAboveThresholdGivesNoHeading) {
    CssWeightedLeastSquaresAlgorithm algorithm = makeReferenceAlgorithm();
    const CssWeightedLeastSquaresOutput out =
        algorithm.update(makeReadings(std::vector<float>(kMaxNumCssSensors, 0.0F)));

    EXPECT_EQ(out.numCssViewingSun, 0U);
    EXPECT_TRUE(out.sunHeading_B.isZero());
    EXPECT_TRUE(out.omega_BN_B.isZero());
    EXPECT_TRUE(out.postFitResiduals.isZero());
}

// One reading fixes only the cone about that boresight, so the estimator reports the boresight itself.
TEST(CssWeightedLeastSquaresTest, SingleSensorReturnsItsBoresight) {
    CssWeightedLeastSquaresAlgorithm algorithm = makeReferenceAlgorithm();
    std::vector<float> readings(kMaxNumCssSensors, 0.0F);
    readings[3] = 0.84F;
    const CssWeightedLeastSquaresOutput out = algorithm.update(makeReadings(readings));

    ASSERT_EQ(out.numCssViewingSun, 1U);
    const Eigen::Vector3d boresight = referenceConstellation()[3];
    EXPECT_LT((out.sunHeading_B.cast<double>() - boresight).norm(), 1e-5);
}

// Two readings leave the system underdetermined, so the minimum norm solution bisects the two boresights
// when both read the same cosine.
TEST(CssWeightedLeastSquaresTest, TwoSensorsGiveTheMinimumNormSolution) {
    CssWeightedLeastSquaresAlgorithm algorithm = makeReferenceAlgorithm();
    std::vector<float> readings(kMaxNumCssSensors, 0.0F);
    readings[0] = 0.5F;
    readings[3] = 0.5F;
    const CssWeightedLeastSquaresOutput out = algorithm.update(makeReadings(readings));

    ASSERT_EQ(out.numCssViewingSun, 2U);
    const Eigen::Vector3d bisector = (referenceConstellation()[0] + referenceConstellation()[3]).normalized();
    EXPECT_LT((out.sunHeading_B.cast<double>() - bisector).norm(), 1e-5);
}

// Two boresights that point the same way fix no plane for the minimum norm solution to lie in, so there is
// no heading to report.
TEST(CssWeightedLeastSquaresTest, TwoParallelBoresightsGiveNoHeading) {
    ConstellationInputs inputs = referenceInputs();
    inputs.boresights = referenceBoresightVector();
    // Point sensor 3 a milliradian off sensor 0. The separation is small enough that the pair fixes no
    // plane, and large enough that single precision still tells the two boresights apart, so the rule is
    // what rejects the pair rather than the arithmetic collapsing on its own.
    inputs.boresights[9] = inputs.boresights[0];
    inputs.boresights[10] = inputs.boresights[1] + 1e-3F;
    inputs.boresights[11] = inputs.boresights[2];
    BuiltConfig built{};
    ASSERT_TRUE(buildConfig(inputs, built));

    CssWeightedLeastSquaresAlgorithm algorithm{makeConfig(inputs, built)};
    std::vector<float> readings(kMaxNumCssSensors, 0.0F);
    readings[0] = 0.8F;
    readings[3] = 0.8F;
    const CssWeightedLeastSquaresOutput out = algorithm.update(makeReadings(readings));

    EXPECT_EQ(out.numCssViewingSun, 2U);
    EXPECT_TRUE(out.sunHeading_B.isZero());
}

// Three collinear boresights span one direction, so the normal matrix is singular and there is no fit to
// report. The estimator returns no heading rather than an arbitrary one.
TEST(CssWeightedLeastSquaresTest, CollinearBoresightsGiveNoHeading) {
    ConstellationInputs inputs = referenceInputs();
    inputs.boresights = {1.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F,
                         1.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F};
    BuiltConfig built{};
    ASSERT_TRUE(buildConfig(inputs, built));

    CssWeightedLeastSquaresAlgorithm algorithm{makeConfig(inputs, built)};
    const CssWeightedLeastSquaresOutput out = algorithm.update(makeReadings({0.9F, 0.9F, 0.9F}));

    EXPECT_EQ(out.numCssViewingSun, 3U);
    EXPECT_TRUE(out.sunHeading_B.isZero());
}

// The fit forms its products over the full-width operands, which is only correct while a sensor that takes
// no part this cycle carries a weight of zero. A cycle with fewer lit sensors than the one before it is
// what would expose a weight left behind, so run the busy cycle first and check the lean one that follows.
TEST(CssWeightedLeastSquaresTest, CoverageDroppingBetweenCyclesLeavesNoStaleTail) {
    ConstellationInputs inputs = referenceInputs();
    inputs.useWeights = true;
    BuiltConfig built{};
    ASSERT_TRUE(buildConfig(inputs, built));
    CssWeightedLeastSquaresAlgorithm algorithm{makeConfig(inputs, built)};

    // Both cycles must stay on the three-or-more branch, which is the one that forms its products over the
    // full-width operands. The two-sensor branch reads only the rows it fills, so it could not show a tail.
    std::vector<float> manyLit(kMaxNumCssSensors, 0.0F);
    for (size_t i = 0U; i < 5U; ++i) {
        manyLit[i] = 0.6F;
    }
    const CssWeightedLeastSquaresOutput busy = algorithm.update(makeReadings(manyLit));
    ASSERT_EQ(busy.numCssViewingSun, 5U);

    std::vector<float> fewLit(kMaxNumCssSensors, 0.0F);
    for (size_t i = 0U; i < 3U; ++i) {
        fewLit[i] = 0.6F;
    }
    const CssWeightedLeastSquaresOutput lean = algorithm.update(makeReadings(fewLit));
    ASSERT_EQ(lean.numCssViewingSun, 3U);

    // Hold the lean cycle to the conditions that define its fit rather than to a second run of the
    // estimator. A tail hoisted out of update() would be shared by every instance, so two runs of the same
    // code would agree with each other and hide the fault.
    const ActiveSystem system = activeSystem(built.boresights,
                                             built.available,
                                             inputs.useWeights,
                                             static_cast<double>(inputs.sensorUseThresh),
                                             toDouble(fewLit));
    ASSERT_TRUE(system.resolvable);
    expectFitIsOptimal(system, lean.sunHeading_B, lean.postFitResiduals);

    for (uint32_t k = lean.numCssViewingSun; k < static_cast<uint32_t>(kMaxNumCssSensors); ++k) {
        EXPECT_EQ(lean.postFitResiduals(static_cast<Eigen::Index>(k)), 0.0F) << "residual slot " << k;
    }
}

// ---------------------------------------------------------------------------
// Rate edge cases
// ---------------------------------------------------------------------------

TEST(CssWeightedLeastSquaresTest, FirstCycleReportsNoRate) {
    CssWeightedLeastSquaresAlgorithm algorithm = makeReferenceAlgorithm();
    const CssWeightedLeastSquaresOutput out = algorithm.update(makeReadings(readingsFor({1.0, 0.0, 0.0})));
    EXPECT_TRUE(out.omega_BN_B.isZero());
}

// A slow slew puts the two headings a milliradian apart, where the cosine of the angle rounds to one in
// single precision. Taking the angle from the cross product as well keeps its significant digits.
TEST(CssWeightedLeastSquaresTest, SlowSlewRateKeepsItsPrecision) {
    CssWeightedLeastSquaresAlgorithm algorithm = makeReferenceAlgorithm();
    const double slewAngle = 1.0e-3;  // [r] heading change across one control period

    algorithm.update(makeReadings(readingsFor({1.0, 0.0, 0.0})));
    const CssWeightedLeastSquaresOutput out =
        algorithm.update(makeReadings(readingsFor({std::cos(slewAngle), std::sin(slewAngle), 0.0})));

    const Eigen::Vector3d expected{0.0, 0.0, -slewAngle / static_cast<double>(kControlPeriod)};
    EXPECT_LT((out.omega_BN_B.cast<double>() - expected).norm(), 1e-3 * expected.norm());
}

// Two opposed headings lie on infinitely many great circles, so they fix a rotation angle but no axis to
// apply it about. Reporting a full rate about a direction round-off chose would be worse than reporting
// none, so the estimator reports none.
TEST(CssWeightedLeastSquaresTest, ReversedHeadingReportsNoRate) {
    CssWeightedLeastSquaresAlgorithm algorithm = makeReferenceAlgorithm();
    const double residualAngle = 5.0e-7;  // [r] how far the reversal misses being exact

    algorithm.update(makeReadings(readingsFor({1.0, 0.0, 0.0})));
    const CssWeightedLeastSquaresOutput out =
        algorithm.update(makeReadings(readingsFor({-std::cos(residualAngle), std::sin(residualAngle), 0.0})));

    EXPECT_TRUE(out.omega_BN_B.isZero());
}

// Far enough from the reversal the axis is determined again, and the rate comes back.
TEST(CssWeightedLeastSquaresTest, NearReversalStillReportsARate) {
    CssWeightedLeastSquaresAlgorithm algorithm = makeReferenceAlgorithm();
    const double residualAngle = 1.0e-2;  // [r] well above the tolerance on the cross product

    algorithm.update(makeReadings(readingsFor({1.0, 0.0, 0.0})));
    const CssWeightedLeastSquaresOutput out =
        algorithm.update(makeReadings(readingsFor({-std::cos(residualAngle), std::sin(residualAngle), 0.0})));

    EXPECT_FALSE(out.omega_BN_B.isZero());
    const double expectedMagnitude = (M_PI - residualAngle) / static_cast<double>(kControlPeriod);
    EXPECT_NEAR(out.omega_BN_B.norm(), expectedMagnitude, 1e-2);
}

TEST(CssWeightedLeastSquaresTest, ReInitializeDropsThePriorHeading) {
    CssWeightedLeastSquaresAlgorithm algorithm = makeReferenceAlgorithm();
    algorithm.update(makeReadings(readingsFor({1.0, 0.0, 0.0})));
    algorithm.reInitialize();

    const CssWeightedLeastSquaresOutput out = algorithm.update(makeReadings(readingsFor({0.0, 1.0, 0.0})));
    EXPECT_TRUE(out.omega_BN_B.isZero());  // nothing to difference the new heading against
    EXPECT_FALSE(out.sunHeading_B.isZero());
}

// A cycle with no sun drops the prior heading too, so the step across the gap is not differenced into a rate.
TEST(CssWeightedLeastSquaresTest, LosingTheSunDropsThePriorHeading) {
    CssWeightedLeastSquaresAlgorithm algorithm = makeReferenceAlgorithm();
    algorithm.update(makeReadings(readingsFor({1.0, 0.0, 0.0})));
    algorithm.update(makeReadings(std::vector<float>(kMaxNumCssSensors, 0.0F)));

    const CssWeightedLeastSquaresOutput out = algorithm.update(makeReadings(readingsFor({0.0, 1.0, 0.0})));
    EXPECT_TRUE(out.omega_BN_B.isZero());
}

// Installing a configuration replaces the parameters without disturbing an estimate already in progress,
// so the prior heading survives and the next cycle still produces a rate.
TEST(CssWeightedLeastSquaresTest, SetConfigKeepsTheRuntimeState) {
    ConstellationInputs inputs = referenceInputs();
    BuiltConfig built{};
    ASSERT_TRUE(buildConfig(inputs, built));
    CssWeightedLeastSquaresAlgorithm algorithm{makeConfig(inputs, built)};

    algorithm.update(makeReadings(readingsFor({1.0, 0.0, 0.0})));

    inputs.sensorUseThresh = 0.3F;
    algorithm.setConfig(makeConfig(inputs, built));

    const CssWeightedLeastSquaresOutput out = algorithm.update(makeReadings(readingsFor({0.0, 1.0, 0.0})));
    EXPECT_FALSE(out.omega_BN_B.isZero());
}
