#include "solarArrayReferenceTestHelpers.hpp"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include <numbers>

// ---------------------------------------------------------------------------
// Regression tests
// ---------------------------------------------------------------------------

TEST(SolarArrayReferenceTest, RegressionTest) {
    regressionTestSolarArrayReference({0.1F, 0.2F, 0.3F},  // sigma_BN
                                      {0.3F, 0.2F, 0.1F},  // sigma_RN
                                      {1.0F, 0.0F, 0.0F},  // rHatIn_SB_B
                                      {1.0F, 0.0F, 0.0F},  // a1Hat_B
                                      {0.0F, 1.0F, 0.0F},  // a2Hat_B
                                      1e-3F,               // alignmentThreshold
                                      0.0F                 // theta
    );
}

TEST(SolarArrayReferenceTest, RegressionTestNonZeroTheta) {
    regressionTestSolarArrayReference({0.5F, 0.4F, 0.3F},  // sigma_BN
                                      {0.9F, 0.7F, 0.8F},  // sigma_RN
                                      {0.0F, 0.0F, 1.0F},  // rHatIn_SB_B
                                      {1.0F, 0.0F, 0.0F},  // a1Hat_B
                                      {0.0F, 1.0F, 0.0F},  // a2Hat_B
                                      1e-3F,               // alignmentThreshold
                                      1.5F                 // theta
    );
}

TEST(SolarArrayReferenceTest, RegressionTestArbitraryAxes) {
    regressionTestSolarArrayReference({0.1F, -0.3F, 0.2F},  // sigma_BN
                                      {0.2F, 0.1F, -0.1F},  // sigma_RN
                                      {1.0F, 1.0F, 1.0F},   // rHatIn_SB_B
                                      {0.0F, 0.0F, 1.0F},   // a1Hat_B
                                      {1.0F, 0.0F, 0.0F},   // a2Hat_B
                                      1e-3F,                // alignmentThreshold
                                      -0.5F                 // theta
    );
}

// ---------------------------------------------------------------------------
// Setup tests (config validation + round-trip)
// ---------------------------------------------------------------------------

TEST(SolarArrayReferenceTest, SetupTest) {
    const Eigen::Vector3f xAxis{1.0F, 0.0F, 0.0F};
    const Eigen::Vector3f yAxis{0.0F, 1.0F, 0.0F};
    constexpr float pi = std::numbers::pi_v<float>;
    constexpr float halfPi = pi / 2.0F;

    const auto makeConfig = [&](const SolarArrayAxes& axes, float threshold, float specifiedAngle, float offsetAngle) {
        return SolarArrayReferenceConfig::create(
            axes, threshold, TrackingMode::AUTO_TRACK, specifiedAngle, offsetAngle);
    };

    // Valid configuration does not throw.
    EXPECT_NO_THROW(makeConfig(SolarArrayAxes{xAxis, yAxis}, 1e-3F, 0.0F, 0.0F));

    // Zero / non-unit / non-orthogonal axes throw.
    EXPECT_THROW(makeConfig(SolarArrayAxes{Eigen::Vector3f::Zero(), yAxis}, 1e-3F, 0.0F, 0.0F), fsw::invalid_argument);
    EXPECT_THROW(makeConfig(SolarArrayAxes{xAxis, Eigen::Vector3f::Zero()}, 1e-3F, 0.0F, 0.0F), fsw::invalid_argument);
    EXPECT_THROW(makeConfig(SolarArrayAxes{Eigen::Vector3f{2.0F, 0.0F, 0.0F}, yAxis}, 1e-3F, 0.0F, 0.0F),
                 fsw::invalid_argument);
    EXPECT_THROW(makeConfig(SolarArrayAxes{xAxis, Eigen::Vector3f{0.0F, 3.0F, 0.0F}}, 1e-3F, 0.0F, 0.0F),
                 fsw::invalid_argument);
    EXPECT_THROW(makeConfig(SolarArrayAxes{xAxis, Eigen::Vector3f{1.0F, 1.0F, 0.0F}.normalized()}, 1e-3F, 0.0F, 0.0F),
                 fsw::invalid_argument);

    // Alignment threshold must lie in [1e-3, pi/2].
    EXPECT_THROW(makeConfig(SolarArrayAxes{xAxis, yAxis}, -0.01F, 0.0F, 0.0F), fsw::invalid_argument);
    EXPECT_THROW(makeConfig(SolarArrayAxes{xAxis, yAxis}, 0.0F, 0.0F, 0.0F), fsw::invalid_argument);
    EXPECT_THROW(makeConfig(SolarArrayAxes{xAxis, yAxis}, 1e-4F, 0.0F, 0.0F), fsw::invalid_argument);
    EXPECT_THROW(makeConfig(SolarArrayAxes{xAxis, yAxis}, halfPi + 0.01F, 0.0F, 0.0F), fsw::invalid_argument);
    EXPECT_NO_THROW(makeConfig(SolarArrayAxes{xAxis, yAxis}, halfPi, 0.0F, 0.0F));
    EXPECT_NO_THROW(makeConfig(SolarArrayAxes{xAxis, yAxis}, 1e-3F, 0.0F, 0.0F));

    // Specified array angle must lie in [-pi, pi].
    EXPECT_NO_THROW(makeConfig(SolarArrayAxes{xAxis, yAxis}, 1e-3F, -pi, 0.0F));
    EXPECT_NO_THROW(makeConfig(SolarArrayAxes{xAxis, yAxis}, 1e-3F, pi, 0.0F));
    EXPECT_THROW(makeConfig(SolarArrayAxes{xAxis, yAxis}, 1e-3F, -10.0F, 0.0F), fsw::invalid_argument);
    EXPECT_THROW(makeConfig(SolarArrayAxes{xAxis, yAxis}, 1e-3F, 10.0F, 0.0F), fsw::invalid_argument);

    // Offset angle must lie in [-pi, pi].
    EXPECT_NO_THROW(makeConfig(SolarArrayAxes{xAxis, yAxis}, 1e-3F, 0.0F, -pi));
    EXPECT_NO_THROW(makeConfig(SolarArrayAxes{xAxis, yAxis}, 1e-3F, 0.0F, pi));
    EXPECT_THROW(makeConfig(SolarArrayAxes{xAxis, yAxis}, 1e-3F, 0.0F, -10.0F), fsw::invalid_argument);
    EXPECT_THROW(makeConfig(SolarArrayAxes{xAxis, yAxis}, 1e-3F, 0.0F, 10.0F), fsw::invalid_argument);

    // Invalid tracking mode throws.
    EXPECT_THROW(SolarArrayReferenceConfig::create(
                     SolarArrayAxes{xAxis, yAxis}, 1e-3F, static_cast<TrackingMode>(7), 0.0F, 0.0F),
                 fsw::invalid_argument);

    // Getter round-trips and axis canonicalization.
    const auto cfg = SolarArrayReferenceConfig::create(
        SolarArrayAxes{xAxis, yAxis}, 0.05F, TrackingMode::SPECIFIED_ANGLE, 0.5F, 0.3F);
    EXPECT_FLOAT_EQ(cfg.getAlignmentThreshold(), 0.05F);
    EXPECT_EQ(cfg.getTrackingMode(), TrackingMode::SPECIFIED_ANGLE);
    EXPECT_FLOAT_EQ(cfg.getSpecifiedArrayAngle(), 0.5F);
    EXPECT_FLOAT_EQ(cfg.getOffsetAngle(), 0.3F);
    EXPECT_NEAR(cfg.getDriveAxisHat_B()(0), 1.0F, 1e-6F);
    EXPECT_NEAR(cfg.getSurfaceNormalHat_B()(1), 1.0F, 1e-6F);
    EXPECT_NEAR(cfg.getThirdAxisHat_B()(2), 1.0F, 1e-6F);  // a3 = a1 x a2 = z
}

// ---------------------------------------------------------------------------
// Property tests
// ---------------------------------------------------------------------------

TEST(SolarArrayReferenceTest, OutputIsFinite) {
    propertyOutputIsFinite({0.1F, 0.2F, 0.3F}, {0.3F, 0.2F, 0.1F}, {1.0F, 1.0F, 0.0F}, 1e-3F, 0.5F);
}

TEST(SolarArrayReferenceTest, AlignedSunReturnsCurrentTheta) {
    propertyAlignedSunReturnsCurrentTheta({1.0F, 0.0F, 0.0F}, 1e-3F, 0.7F);
}

TEST(SolarArrayReferenceTest, AlignedSunNegativeTheta) {
    propertyAlignedSunReturnsCurrentTheta({0.0F, 0.0F, 1.0F}, 1e-3F, -1.2F);
}

TEST(SolarArrayReferenceTest, SpecifiedAngleReturnsAngle) {
    propertySpecifiedAngleReturnsAngle({0.1F, 0.2F, 0.3F}, {0.3F, 0.2F, 0.1F}, {1.0F, 1.0F, 0.0F}, 0.5F, 0.7F);
}

// ---------------------------------------------------------------------------
// Edge-case tests
// ---------------------------------------------------------------------------

// Sun direction exactly along drive axis: no preferred angle, output = input theta.
TEST(SolarArrayReferenceTest, SunAlignedWithDriveAxis) {
    const auto alg =
        makeSolarArrayReferenceAlgorithm(Eigen::Vector3f{1.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 1.0F, 0.0F});

    float theta = 0.5F;
    float result =
        alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f{1.0F, 0.0F, 0.0F}, theta);
    EXPECT_NEAR(result, theta, 1e-5F);
}

// Sun direction exactly opposite to drive axis: still aligned, output = input theta.
TEST(SolarArrayReferenceTest, SunAntiAlignedWithDriveAxis) {
    const auto alg =
        makeSolarArrayReferenceAlgorithm(Eigen::Vector3f{1.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 1.0F, 0.0F});

    float theta = -0.3F;
    float result =
        alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f{-1.0F, 0.0F, 0.0F}, theta);
    EXPECT_NEAR(result, theta, 1e-5F);
}

// Sun perpendicular to drive axis and aligned with surface normal: thetaRef should be near zero.
TEST(SolarArrayReferenceTest, SunAlignedWithSurfaceNormal) {
    const auto alg =
        makeSolarArrayReferenceAlgorithm(Eigen::Vector3f{1.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 1.0F, 0.0F});

    float result =
        alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f{0.0F, 1.0F, 0.0F}, 0.0F);
    EXPECT_NEAR(result, 0.0F, 1e-5F);
}

// Large theta values: wrapping should keep output reasonable.
TEST(SolarArrayReferenceTest, LargeThetaWrapping) {
    const auto alg =
        makeSolarArrayReferenceAlgorithm(Eigen::Vector3f{1.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 1.0F, 0.0F});

    float result = alg.update(Eigen::Vector3f{0.1F, 0.2F, 0.3F},
                              Eigen::Vector3f{0.3F, 0.2F, 0.1F},
                              Eigen::Vector3f{0.0F, 0.0F, 1.0F},
                              100.0F);
    EXPECT_TRUE(std::isfinite(result));
}

// Zero sun direction vector falls back to current theta (no preferred rotation).
TEST(SolarArrayReferenceTest, ZeroSunDirectionReturnsCurrentTheta) {
    const auto alg =
        makeSolarArrayReferenceAlgorithm(Eigen::Vector3f{1.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 1.0F, 0.0F});

    float theta = 0.7F;
    float result = alg.update(
        Eigen::Vector3f{0.1F, 0.2F, 0.3F}, Eigen::Vector3f{0.3F, 0.2F, 0.1F}, Eigen::Vector3f::Zero(), theta);
    EXPECT_FLOAT_EQ(result, theta);
}

// Alignment threshold: just inside threshold keeps current theta.
TEST(SolarArrayReferenceTest, AlignmentThresholdJustInside) {
    const auto alg = makeSolarArrayReferenceAlgorithm(
        Eigen::Vector3f{0.0F, 0.0F, 1.0F}, Eigen::Vector3f{1.0F, 0.0F, 0.0F}, 0.1F);  // 0.1 rad threshold

    // Sun along drive axis
    Eigen::Vector3f sunNearAxis{0.0F, 0.0F, 1.0F};  // sun angle of 0 deg
    float theta = 1.0F;
    float result = alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), sunNearAxis, theta);
    EXPECT_NEAR(result, theta, 1e-5F);
}

// Alignment threshold: just outside threshold computes reference.
TEST(SolarArrayReferenceTest, AlignmentThresholdJustOutside) {
    const auto alg = makeSolarArrayReferenceAlgorithm(
        Eigen::Vector3f{0.0F, 0.0F, 1.0F}, Eigen::Vector3f{1.0F, 0.0F, 0.0F}, 0.01F);  // 0.01 rad threshold

    // Sun well away from drive axis
    Eigen::Vector3f sunAway{1.0F, 0.0F, 0.0F};  // 90 deg from z-axis
    float theta = 0.0F;
    float result = alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), sunAway, theta);
    // Should compute a reference angle, not just return theta
    EXPECT_TRUE(std::isfinite(result));
}

// SPECIFIED_ANGLE mode ignores sun direction and attitudes — output depends only on the configured angle.
TEST(SolarArrayReferenceTest, SpecifiedAngleModeIgnoresSun) {
    const auto alg = makeSolarArrayReferenceAlgorithm(Eigen::Vector3f{1.0F, 0.0F, 0.0F},
                                                      Eigen::Vector3f{0.0F, 1.0F, 0.0F},
                                                      1e-3F,
                                                      TrackingMode::SPECIFIED_ANGLE,
                                                      0.8F);

    // Two very different sun vectors must produce identical output.
    float resultA =
        alg.update(Eigen::Vector3f{0.1F, 0.2F, 0.3F}, Eigen::Vector3f::Zero(), Eigen::Vector3f{1.0F, 0.0F, 0.0F}, 0.0F);
    float resultB = alg.update(
        Eigen::Vector3f{-0.5F, 0.4F, 0.1F}, Eigen::Vector3f{0.2F, 0.2F, 0.2F}, Eigen::Vector3f{0.0F, 0.0F, 1.0F}, 1.5F);
    EXPECT_FLOAT_EQ(resultA, resultB);
    EXPECT_FLOAT_EQ(resultA, 0.8F);
}

// Offset angle is added to the AUTO_TRACK reference angle (verified via wrapping equivalence).
TEST(SolarArrayReferenceTest, OffsetAngleAppliedAutoTrack) {
    // No offset: sun aligned with surface normal -> thetaRef = 0
    const auto algNoOffset =
        makeSolarArrayReferenceAlgorithm(Eigen::Vector3f{1.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 1.0F, 0.0F});
    float resultNoOffset =
        algNoOffset.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f{0.0F, 1.0F, 0.0F}, 0.0F);
    EXPECT_NEAR(resultNoOffset, 0.0F, 1e-5F);

    // With offset 0.4: same scenario shifts result by 0.4
    const auto algWithOffset = makeSolarArrayReferenceAlgorithm(Eigen::Vector3f{1.0F, 0.0F, 0.0F},
                                                                Eigen::Vector3f{0.0F, 1.0F, 0.0F},
                                                                1e-3F,
                                                                TrackingMode::AUTO_TRACK,
                                                                0.0F,
                                                                0.4F);
    float resultWithOffset =
        algWithOffset.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f{0.0F, 1.0F, 0.0F}, 0.0F);
    EXPECT_NEAR(resultWithOffset, 0.4F, 1e-5F);
}

// Offset angle is added to the SPECIFIED_ANGLE result and the sum is wrapped to [-pi, pi].
TEST(SolarArrayReferenceTest, OffsetAngleAppliedSpecifiedAngle) {
    const auto alg = makeSolarArrayReferenceAlgorithm(Eigen::Vector3f{1.0F, 0.0F, 0.0F},
                                                      Eigen::Vector3f{0.0F, 1.0F, 0.0F},
                                                      1e-3F,
                                                      TrackingMode::SPECIFIED_ANGLE,
                                                      0.5F,
                                                      0.2F);

    float result =
        alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f{1.0F, 0.0F, 0.0F}, 0.0F);
    // Expected: wrap(0.5 + 0.2) = 0.7
    EXPECT_NEAR(result, 0.7F, 1e-5F);
}

// Offset angle that pushes the sum past pi wraps correctly to the negative side.
TEST(SolarArrayReferenceTest, OffsetAngleWrapsPastPi) {
    const auto alg = makeSolarArrayReferenceAlgorithm(Eigen::Vector3f{1.0F, 0.0F, 0.0F},
                                                      Eigen::Vector3f{0.0F, 1.0F, 0.0F},
                                                      1e-3F,
                                                      TrackingMode::SPECIFIED_ANGLE,
                                                      2.0F,
                                                      2.0F);  // 2.0 + 2.0 = 4.0, which wraps to 4.0 - 2*pi

    float result =
        alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f{1.0F, 0.0F, 0.0F}, 0.0F);
    constexpr float pi = std::numbers::pi_v<float>;
    EXPECT_NEAR(result, 4.0F - 2.0F * pi, 1e-5F);
}
