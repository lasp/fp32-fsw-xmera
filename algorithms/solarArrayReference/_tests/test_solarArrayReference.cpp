#include "solarArrayReferenceTestHelpers.hpp"
#include <numbers>

// ---------------------------------------------------------------------------
// Regression tests
// ---------------------------------------------------------------------------

TEST(SolarArrayReferenceTest, RegressionTest) {
    regressionTestSolarArrayReference({0.1F, 0.2F, 0.3F},  // sigma_BN
                                      {0.3F, 0.2F, 0.1F},  // sigma_RN
                                      {1.0F, 0.0F, 0.0F},  // vehSunPntBdy
                                      {1.0F, 0.0F, 0.0F},  // a1Hat_B
                                      {0.0F, 1.0F, 0.0F},  // a2Hat_B
                                      0.0F                 // theta
    );
}

TEST(SolarArrayReferenceTest, RegressionTestNonZeroTheta) {
    regressionTestSolarArrayReference({0.5F, 0.4F, 0.3F},  // sigma_BN
                                      {0.9F, 0.7F, 0.8F},  // sigma_RN
                                      {0.0F, 0.0F, 1.0F},  // vehSunPntBdy
                                      {1.0F, 0.0F, 0.0F},  // a1Hat_B
                                      {0.0F, 1.0F, 0.0F},  // a2Hat_B
                                      1.5F                 // theta
    );
}

TEST(SolarArrayReferenceTest, RegressionTestArbitraryAxes) {
    regressionTestSolarArrayReference({0.1F, -0.3F, 0.2F},  // sigma_BN
                                      {0.2F, 0.1F, -0.1F},  // sigma_RN
                                      {1.0F, 1.0F, 1.0F},   // vehSunPntBdy
                                      {0.0F, 0.0F, 1.0F},   // a1Hat_B
                                      {1.0F, 0.0F, 0.0F},   // a2Hat_B
                                      -0.5F                 // theta
    );
}

// ---------------------------------------------------------------------------
// Setup tests (setter validation + round-trip)
// ---------------------------------------------------------------------------

TEST(SolarArrayReferenceTest, SetupTest) {
    SolarArrayReferenceAlgorithm alg{};

    // Zero drive axis should throw (norm far from 1.0)
    EXPECT_THROW(alg.setSolarArrayAxes_B(Eigen::Vector3f::Zero(), Eigen::Vector3f{0.0F, 1.0F, 0.0F}),
                 fsw::invalid_argument);

    // Zero surface normal should throw
    EXPECT_THROW(alg.setSolarArrayAxes_B(Eigen::Vector3f{1.0F, 0.0F, 0.0F}, Eigen::Vector3f::Zero()),
                 fsw::invalid_argument);

    // Non-unit drive axis (norm far from 1.0) should throw
    EXPECT_THROW(alg.setSolarArrayAxes_B(Eigen::Vector3f{2.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 1.0F, 0.0F}),
                 fsw::invalid_argument);

    // Non-unit surface normal should throw
    EXPECT_THROW(alg.setSolarArrayAxes_B(Eigen::Vector3f{1.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 3.0F, 0.0F}),
                 fsw::invalid_argument);

    // Non-orthogonal axes should throw
    EXPECT_THROW(
        alg.setSolarArrayAxes_B(Eigen::Vector3f{1.0F, 0.0F, 0.0F}, Eigen::Vector3f{1.0F, 1.0F, 0.0F}.normalized()),
        fsw::invalid_argument);

    // Alignment threshold: negative should throw
    EXPECT_THROW(alg.setAlignmentThreshold(-0.01F), fsw::invalid_argument);

    // Alignment threshold: zero should throw
    EXPECT_THROW(alg.setAlignmentThreshold(0.0F), fsw::invalid_argument);

    // Alignment threshold: below 1e-3 (fp32 precision floor) should throw
    EXPECT_THROW(alg.setAlignmentThreshold(1e-4F), fsw::invalid_argument);

    // Alignment threshold: above pi/2 should throw
    constexpr float halfPi = std::numbers::pi_v<float> / 2.0F;
    EXPECT_THROW(alg.setAlignmentThreshold(halfPi + 0.01F), fsw::invalid_argument);

    // Valid alignment threshold should not throw
    EXPECT_NO_THROW(alg.setAlignmentThreshold(halfPi));

    // Lower bound exactly at 1e-3 should not throw
    EXPECT_NO_THROW(alg.setAlignmentThreshold(1e-3F));

    // Alignment threshold round-trip
    alg.setAlignmentThreshold(0.05F);
    EXPECT_FLOAT_EQ(alg.getAlignmentThreshold(), 0.05F);

    // Valid orthogonal unit axes should not throw
    EXPECT_NO_THROW(alg.setSolarArrayAxes_B(Eigen::Vector3f{1.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 1.0F, 0.0F}));

    // Getter round-trip
    alg.setSolarArrayAxes_B(Eigen::Vector3f{1.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 1.0F, 0.0F});
    const auto axes = alg.getSolarArrayAxes_B();
    EXPECT_NEAR(axes[0](0), 1.0F, 1e-6F);
    EXPECT_NEAR(axes[0](1), 0.0F, 1e-6F);
    EXPECT_NEAR(axes[0](2), 0.0F, 1e-6F);
    EXPECT_NEAR(axes[1](0), 0.0F, 1e-6F);
    EXPECT_NEAR(axes[1](1), 1.0F, 1e-6F);
    EXPECT_NEAR(axes[1](2), 0.0F, 1e-6F);

    // Tracking mode default is AUTO_TRACK
    EXPECT_EQ(alg.getTrackingMode(), TrackingMode::AUTO_TRACK);

    // Tracking mode round-trip for both values
    alg.setTrackingMode(TrackingMode::SPECIFIED_ANGLE);
    EXPECT_EQ(alg.getTrackingMode(), TrackingMode::SPECIFIED_ANGLE);
    alg.setTrackingMode(TrackingMode::AUTO_TRACK);
    EXPECT_EQ(alg.getTrackingMode(), TrackingMode::AUTO_TRACK);

    // Specified array angle accepts any value (no setter validation; wrapped at update time)
    EXPECT_NO_THROW(alg.setSpecifiedArrayAngle(0.0F));
    EXPECT_NO_THROW(alg.setSpecifiedArrayAngle(-10.0F));
    EXPECT_NO_THROW(alg.setSpecifiedArrayAngle(10.0F));

    // Specified array angle round-trip stores the raw value
    alg.setSpecifiedArrayAngle(0.5F);
    EXPECT_FLOAT_EQ(alg.getSpecifiedArrayAngle(), 0.5F);
    alg.setSpecifiedArrayAngle(-2.5F);
    EXPECT_FLOAT_EQ(alg.getSpecifiedArrayAngle(), -2.5F);

    // Offset angle: default is zero
    EXPECT_FLOAT_EQ(alg.getOffsetAngle(), 0.0F);

    // Offset angle accepts any value (no setter validation; wrapped at update time)
    EXPECT_NO_THROW(alg.setOffsetAngle(0.0F));
    EXPECT_NO_THROW(alg.setOffsetAngle(-10.0F));
    EXPECT_NO_THROW(alg.setOffsetAngle(10.0F));

    // Offset angle round-trip stores the raw value
    alg.setOffsetAngle(0.3F);
    EXPECT_FLOAT_EQ(alg.getOffsetAngle(), 0.3F);
    alg.setOffsetAngle(-1.7F);
    EXPECT_FLOAT_EQ(alg.getOffsetAngle(), -1.7F);
}

// ---------------------------------------------------------------------------
// Property tests
// ---------------------------------------------------------------------------

TEST(SolarArrayReferenceTest, OutputIsFinite) {
    propertyOutputIsFinite({0.1F, 0.2F, 0.3F}, {0.3F, 0.2F, 0.1F}, {1.0F, 1.0F, 0.0F}, 0.5F);
}

TEST(SolarArrayReferenceTest, AlignedSunReturnsCurrentTheta) {
    propertyAlignedSunReturnsCurrentTheta({1.0F, 0.0F, 0.0F}, 0.7F);
}

TEST(SolarArrayReferenceTest, AlignedSunNegativeTheta) {
    propertyAlignedSunReturnsCurrentTheta({0.0F, 0.0F, 1.0F}, -1.2F);
}

TEST(SolarArrayReferenceTest, SpecifiedAngleReturnsAngle) {
    propertySpecifiedAngleReturnsAngle({0.1F, 0.2F, 0.3F}, {0.3F, 0.2F, 0.1F}, {1.0F, 1.0F, 0.0F}, 0.5F, 0.7F);
}

// ---------------------------------------------------------------------------
// Edge-case tests
// ---------------------------------------------------------------------------

// Sun direction exactly along drive axis: no preferred angle, output = input theta.
TEST(SolarArrayReferenceTest, SunAlignedWithDriveAxis) {
    SolarArrayReferenceAlgorithm alg{};
    alg.setSolarArrayAxes_B(Eigen::Vector3f{1.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 1.0F, 0.0F});

    float theta = 0.5F;
    float result =
        alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f{1.0F, 0.0F, 0.0F}, theta);
    EXPECT_NEAR(result, theta, 1e-5F);
}

// Sun direction exactly opposite to drive axis: still aligned, output = input theta.
TEST(SolarArrayReferenceTest, SunAntiAlignedWithDriveAxis) {
    SolarArrayReferenceAlgorithm alg{};
    alg.setSolarArrayAxes_B(Eigen::Vector3f{1.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 1.0F, 0.0F});

    float theta = -0.3F;
    float result =
        alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f{-1.0F, 0.0F, 0.0F}, theta);
    EXPECT_NEAR(result, theta, 1e-5F);
}

// Sun perpendicular to drive axis and aligned with surface normal: thetaRef should be near zero.
TEST(SolarArrayReferenceTest, SunAlignedWithSurfaceNormal) {
    SolarArrayReferenceAlgorithm alg{};
    alg.setSolarArrayAxes_B(Eigen::Vector3f{1.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 1.0F, 0.0F});

    float result =
        alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f{0.0F, 1.0F, 0.0F}, 0.0F);
    EXPECT_NEAR(result, 0.0F, 1e-5F);
}

// Large theta values: wrapping should keep output reasonable.
TEST(SolarArrayReferenceTest, LargeThetaWrapping) {
    SolarArrayReferenceAlgorithm alg{};
    alg.setSolarArrayAxes_B(Eigen::Vector3f{1.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 1.0F, 0.0F});

    float result = alg.update(Eigen::Vector3f{0.1F, 0.2F, 0.3F},
                              Eigen::Vector3f{0.3F, 0.2F, 0.1F},
                              Eigen::Vector3f{0.0F, 0.0F, 1.0F},
                              100.0F);
    EXPECT_TRUE(std::isfinite(result));
}

// Alignment threshold: just inside threshold keeps current theta.
TEST(SolarArrayReferenceTest, AlignmentThresholdJustInside) {
    SolarArrayReferenceAlgorithm alg{};
    alg.setSolarArrayAxes_B(Eigen::Vector3f{0.0F, 0.0F, 1.0F}, Eigen::Vector3f{1.0F, 0.0F, 0.0F});
    alg.setAlignmentThreshold(0.1F);  // 0.1 rad threshold

    // Sun along drive axis
    Eigen::Vector3f sunNearAxis{0.0F, 0.0F, 1.0F};  // sun angle of 0 deg
    float theta = 1.0F;
    float result = alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), sunNearAxis, theta);
    EXPECT_NEAR(result, theta, 1e-5F);
}

// Alignment threshold: just outside threshold computes reference.
TEST(SolarArrayReferenceTest, AlignmentThresholdJustOutside) {
    SolarArrayReferenceAlgorithm alg{};
    alg.setSolarArrayAxes_B(Eigen::Vector3f{0.0F, 0.0F, 1.0F}, Eigen::Vector3f{1.0F, 0.0F, 0.0F});
    alg.setAlignmentThreshold(0.01F);  // 0.01 rad threshold

    // Sun well away from drive axis
    Eigen::Vector3f sunAway{1.0F, 0.0F, 0.0F};  // 90 deg from z-axis
    float theta = 0.0F;
    float result = alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), sunAway, theta);
    // Should compute a reference angle, not just return theta
    EXPECT_TRUE(std::isfinite(result));
}

// SPECIFIED_ANGLE mode ignores sun direction and attitudes — output depends only on the configured angle.
TEST(SolarArrayReferenceTest, SpecifiedAngleModeIgnoresSun) {
    SolarArrayReferenceAlgorithm alg{};
    alg.setSolarArrayAxes_B(Eigen::Vector3f{1.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 1.0F, 0.0F});
    alg.setTrackingMode(TrackingMode::SPECIFIED_ANGLE);
    alg.setSpecifiedArrayAngle(0.8F);

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
    SolarArrayReferenceAlgorithm alg{};
    alg.setSolarArrayAxes_B(Eigen::Vector3f{1.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 1.0F, 0.0F});

    // No offset: sun aligned with surface normal -> thetaRef = 0
    float resultNoOffset =
        alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f{0.0F, 1.0F, 0.0F}, 0.0F);
    EXPECT_NEAR(resultNoOffset, 0.0F, 1e-5F);

    // With offset 0.4: same scenario shifts result by 0.4
    alg.setOffsetAngle(0.4F);
    float resultWithOffset =
        alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f{0.0F, 1.0F, 0.0F}, 0.0F);
    EXPECT_NEAR(resultWithOffset, 0.4F, 1e-5F);
}

// Offset angle is added to the SPECIFIED_ANGLE result and the sum is wrapped to [-pi, pi].
TEST(SolarArrayReferenceTest, OffsetAngleAppliedSpecifiedAngle) {
    SolarArrayReferenceAlgorithm alg{};
    alg.setSolarArrayAxes_B(Eigen::Vector3f{1.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 1.0F, 0.0F});
    alg.setTrackingMode(TrackingMode::SPECIFIED_ANGLE);
    alg.setSpecifiedArrayAngle(0.5F);
    alg.setOffsetAngle(0.2F);

    float result =
        alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f{1.0F, 0.0F, 0.0F}, 0.0F);
    // Expected: wrap(0.5 + 0.2) = 0.7
    EXPECT_NEAR(result, 0.7F, 1e-5F);
}

// Offset angle that pushes the sum past pi wraps correctly to the negative side.
TEST(SolarArrayReferenceTest, OffsetAngleWrapsPastPi) {
    SolarArrayReferenceAlgorithm alg{};
    alg.setSolarArrayAxes_B(Eigen::Vector3f{1.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 1.0F, 0.0F});
    alg.setTrackingMode(TrackingMode::SPECIFIED_ANGLE);
    alg.setSpecifiedArrayAngle(2.0F);
    alg.setOffsetAngle(2.0F);  // 2.0 + 2.0 = 4.0, which wraps to 4.0 - 2*pi ≈ -2.283

    float result =
        alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f{1.0F, 0.0F, 0.0F}, 0.0F);
    constexpr float pi = std::numbers::pi_v<float>;
    EXPECT_NEAR(result, 4.0F - 2.0F * pi, 1e-5F);
}
