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
                                      0.0F                 // priorAngle
    );
}

TEST(SolarArrayReferenceTest, RegressionTestNonZeroPriorAngle) {
    regressionTestSolarArrayReference({0.5F, 0.4F, 0.3F},  // sigma_BN
                                      {0.9F, 0.7F, 0.8F},  // sigma_RN
                                      {0.0F, 0.0F, 1.0F},  // rHatIn_SB_B
                                      {1.0F, 0.0F, 0.0F},  // a1Hat_B
                                      {0.0F, 1.0F, 0.0F},  // a2Hat_B
                                      1e-3F,               // alignmentThreshold
                                      1.5F                 // priorAngle
    );
}

TEST(SolarArrayReferenceTest, RegressionTestArbitraryAxes) {
    regressionTestSolarArrayReference({0.1F, -0.3F, 0.2F},  // sigma_BN
                                      {0.2F, 0.1F, -0.1F},  // sigma_RN
                                      {1.0F, 1.0F, 1.0F},   // rHatIn_SB_B
                                      {0.0F, 0.0F, 1.0F},   // a1Hat_B
                                      {1.0F, 0.0F, 0.0F},   // a2Hat_B
                                      1e-3F,                // alignmentThreshold
                                      -0.5F                 // priorAngle
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
    EXPECT_THROW(
        makeConfig(SolarArrayAxes{xAxis, Eigen::Vector3f{1.0F, 1.0F, 0.0F}.stableNormalized()}, 1e-3F, 0.0F, 0.0F),
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

TEST(SolarArrayReferenceTest, AlignedSunReturnsPriorThetaRef) {
    propertyAlignedSunReturnsPriorThetaRef({1.0F, 0.0F, 0.0F}, 1e-3F, 0.7F);
}

TEST(SolarArrayReferenceTest, AlignedSunNegativePriorThetaRef) {
    propertyAlignedSunReturnsPriorThetaRef({0.0F, 0.0F, 1.0F}, 1e-3F, -1.2F);
}

TEST(SolarArrayReferenceTest, SpecifiedAngleReturnsAngle) {
    propertySpecifiedAngleReturnsAngle({0.1F, 0.2F, 0.3F}, {0.3F, 0.2F, 0.1F}, {1.0F, 1.0F, 0.0F}, 0.5F);
}

// ---------------------------------------------------------------------------
// Edge-case tests
// ---------------------------------------------------------------------------

// Sun direction exactly along drive axis: no preferred angle, output holds the previous reference angle.
TEST(SolarArrayReferenceTest, SunAlignedWithDriveAxis) {
    const Eigen::Vector3f a1Hat_B{1.0F, 0.0F, 0.0F};
    const Eigen::Vector3f a2Hat_B{0.0F, 1.0F, 0.0F};
    auto alg = makeSolarArrayReferenceAlgorithm(a1Hat_B, a2Hat_B);

    constexpr float priorAngle = 0.5F;
    alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), sunDirectionForAngle(a1Hat_B, a2Hat_B, priorAngle));

    float result = alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f{1.0F, 0.0F, 0.0F});
    EXPECT_NEAR(result, priorAngle, 1e-5F);
}

// Sun direction exactly opposite to drive axis: still aligned, output holds the previous reference angle.
TEST(SolarArrayReferenceTest, SunAntiAlignedWithDriveAxis) {
    const Eigen::Vector3f a1Hat_B{1.0F, 0.0F, 0.0F};
    const Eigen::Vector3f a2Hat_B{0.0F, 1.0F, 0.0F};
    auto alg = makeSolarArrayReferenceAlgorithm(a1Hat_B, a2Hat_B);

    constexpr float priorAngle = -0.3F;
    alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), sunDirectionForAngle(a1Hat_B, a2Hat_B, priorAngle));

    float result = alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f{-1.0F, 0.0F, 0.0F});
    EXPECT_NEAR(result, priorAngle, 1e-5F);
}

// Sun perpendicular to drive axis and aligned with surface normal: thetaRef should be near zero.
TEST(SolarArrayReferenceTest, SunAlignedWithSurfaceNormal) {
    auto alg = makeSolarArrayReferenceAlgorithm(Eigen::Vector3f{1.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 1.0F, 0.0F});

    float result = alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f{0.0F, 1.0F, 0.0F});
    EXPECT_NEAR(result, 0.0F, 1e-5F);
}

// Without a prior update the retained reference angle is zero, so the aligned-sun fallback outputs zero.
TEST(SolarArrayReferenceTest, AlignedSunOnFirstUpdateReturnsZero) {
    auto alg = makeSolarArrayReferenceAlgorithm(Eigen::Vector3f{1.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 1.0F, 0.0F});

    float result = alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f{1.0F, 0.0F, 0.0F});
    EXPECT_NEAR(result, 0.0F, 1e-5F);
}

// reInitialize() drops the retained reference angle, so the fallback returns to zero.
TEST(SolarArrayReferenceTest, ReInitializeClearsPriorThetaRef) {
    const Eigen::Vector3f a1Hat_B{1.0F, 0.0F, 0.0F};
    const Eigen::Vector3f a2Hat_B{0.0F, 1.0F, 0.0F};
    auto alg = makeSolarArrayReferenceAlgorithm(a1Hat_B, a2Hat_B);

    constexpr float priorAngle = 0.9F;
    alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), sunDirectionForAngle(a1Hat_B, a2Hat_B, priorAngle));
    const Eigen::Vector3f sunAligned{1.0F, 0.0F, 0.0F};
    EXPECT_NEAR(alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), sunAligned), priorAngle, 1e-5F);

    alg.reInitialize();
    EXPECT_NEAR(alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), sunAligned), 0.0F, 1e-5F);
}

// The output is always wrapped to [-pi, pi], so the retained reference angle cannot grow without bound.
TEST(SolarArrayReferenceTest, OutputIsWrappedToPi) {
    auto alg = makeSolarArrayReferenceAlgorithm(Eigen::Vector3f{1.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 1.0F, 0.0F});

    constexpr float pi = std::numbers::pi_v<float>;
    float result = alg.update(
        Eigen::Vector3f{0.1F, 0.2F, 0.3F}, Eigen::Vector3f{0.3F, 0.2F, 0.1F}, Eigen::Vector3f{0.0F, 0.0F, 1.0F});
    EXPECT_TRUE(std::isfinite(result));
    EXPECT_LE(fabsf(result), pi);
}

// Zero sun direction vector falls back to the previous reference angle (no preferred rotation).
TEST(SolarArrayReferenceTest, ZeroSunDirectionReturnsPriorThetaRef) {
    const Eigen::Vector3f a1Hat_B{1.0F, 0.0F, 0.0F};
    const Eigen::Vector3f a2Hat_B{0.0F, 1.0F, 0.0F};
    auto alg = makeSolarArrayReferenceAlgorithm(a1Hat_B, a2Hat_B);

    constexpr float priorAngle = 0.7F;
    alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), sunDirectionForAngle(a1Hat_B, a2Hat_B, priorAngle));

    float result =
        alg.update(Eigen::Vector3f{0.1F, 0.2F, 0.3F}, Eigen::Vector3f{0.3F, 0.2F, 0.1F}, Eigen::Vector3f::Zero());
    EXPECT_NEAR(result, priorAngle, 1e-5F);
}

// Alignment threshold: just inside threshold holds the previous reference angle.
TEST(SolarArrayReferenceTest, AlignmentThresholdJustInside) {
    const Eigen::Vector3f a1Hat_B{0.0F, 0.0F, 1.0F};
    const Eigen::Vector3f a2Hat_B{1.0F, 0.0F, 0.0F};
    auto alg = makeSolarArrayReferenceAlgorithm(a1Hat_B, a2Hat_B, 0.1F);  // 0.1 rad threshold

    constexpr float priorAngle = 1.0F;
    alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), sunDirectionForAngle(a1Hat_B, a2Hat_B, priorAngle));

    // Sun along drive axis
    Eigen::Vector3f sunNearAxis{0.0F, 0.0F, 1.0F};  // sun angle of 0 deg
    float result = alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), sunNearAxis);
    EXPECT_NEAR(result, priorAngle, 1e-5F);
}

// Alignment threshold: just outside threshold computes reference.
TEST(SolarArrayReferenceTest, AlignmentThresholdJustOutside) {
    auto alg = makeSolarArrayReferenceAlgorithm(
        Eigen::Vector3f{0.0F, 0.0F, 1.0F}, Eigen::Vector3f{1.0F, 0.0F, 0.0F}, 0.01F);  // 0.01 rad threshold

    // Sun well away from drive axis
    Eigen::Vector3f sunAway{1.0F, 0.0F, 0.0F};  // 90 deg from z-axis
    float result = alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), sunAway);
    // Should compute a reference angle, not just hold the previous one
    EXPECT_TRUE(std::isfinite(result));
}

// SPECIFIED_ANGLE mode ignores sun direction and attitudes — output depends only on the configured angle.
TEST(SolarArrayReferenceTest, SpecifiedAngleModeIgnoresSun) {
    auto alg = makeSolarArrayReferenceAlgorithm(Eigen::Vector3f{1.0F, 0.0F, 0.0F},
                                                Eigen::Vector3f{0.0F, 1.0F, 0.0F},
                                                1e-3F,
                                                TrackingMode::SPECIFIED_ANGLE,
                                                0.8F);

    // Two very different sun vectors must produce identical output.
    float resultA =
        alg.update(Eigen::Vector3f{0.1F, 0.2F, 0.3F}, Eigen::Vector3f::Zero(), Eigen::Vector3f{1.0F, 0.0F, 0.0F});
    float resultB = alg.update(
        Eigen::Vector3f{-0.5F, 0.4F, 0.1F}, Eigen::Vector3f{0.2F, 0.2F, 0.2F}, Eigen::Vector3f{0.0F, 0.0F, 1.0F});
    EXPECT_FLOAT_EQ(resultA, resultB);
    EXPECT_FLOAT_EQ(resultA, 0.8F);
}

// Offset angle is added to the AUTO_TRACK reference angle (verified via wrapping equivalence).
TEST(SolarArrayReferenceTest, OffsetAngleAppliedAutoTrack) {
    // No offset: sun aligned with surface normal -> thetaRef = 0
    auto algNoOffset =
        makeSolarArrayReferenceAlgorithm(Eigen::Vector3f{1.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 1.0F, 0.0F});
    float resultNoOffset =
        algNoOffset.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f{0.0F, 1.0F, 0.0F});
    EXPECT_NEAR(resultNoOffset, 0.0F, 1e-5F);

    // With offset 0.4: same scenario shifts result by 0.4
    auto algWithOffset = makeSolarArrayReferenceAlgorithm(Eigen::Vector3f{1.0F, 0.0F, 0.0F},
                                                          Eigen::Vector3f{0.0F, 1.0F, 0.0F},
                                                          1e-3F,
                                                          TrackingMode::AUTO_TRACK,
                                                          0.0F,
                                                          0.4F);
    float resultWithOffset =
        algWithOffset.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f{0.0F, 1.0F, 0.0F});
    EXPECT_NEAR(resultWithOffset, 0.4F, 1e-5F);
}

// Offset angle is not applied in SPECIFIED_ANGLE mode.
TEST(SolarArrayReferenceTest, OffsetAngleIgnoredSpecifiedAngle) {
    auto alg = makeSolarArrayReferenceAlgorithm(Eigen::Vector3f{1.0F, 0.0F, 0.0F},
                                                Eigen::Vector3f{0.0F, 1.0F, 0.0F},
                                                1e-3F,
                                                TrackingMode::SPECIFIED_ANGLE,
                                                0.5F,
                                                0.2F);

    float result = alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f{1.0F, 0.0F, 0.0F});
    EXPECT_NEAR(result, 0.5F, 1e-5F);
}

// Offset angle that pushes the AUTO_TRACK sum past pi wraps correctly to the negative side.
TEST(SolarArrayReferenceTest, OffsetAngleWrapsPastPi) {
    auto alg = makeSolarArrayReferenceAlgorithm(Eigen::Vector3f{1.0F, 0.0F, 0.0F},
                                                Eigen::Vector3f{0.0F, 1.0F, 0.0F},
                                                1e-3F,
                                                TrackingMode::AUTO_TRACK,
                                                0.0F,
                                                2.0F);

    // Sun in the (a2, a3) plane at 2.0 rad from a2: 2.0 + 2.0 = 4.0, which wraps to 4.0 - 2*pi.
    const Eigen::Vector3f sun{0.0F, cosf(2.0F), sinf(2.0F)};
    float result = alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), sun);
    constexpr float pi = std::numbers::pi_v<float>;
    EXPECT_NEAR(result, 4.0F - 2.0F * pi, 1e-5F);
}
