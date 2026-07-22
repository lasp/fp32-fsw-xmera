#include "triadTestHelpers.hpp"
#include <gtest/gtest.h>

// ---------------------------------------------------------------------------
// Regression test
// ---------------------------------------------------------------------------

TEST(TriadTest, RegressionTest) {
    const Eigen::Vector3f rHat_SB_N = Eigen::Vector3f(1.0F, 1.0F, 0.0F).normalized();
    const Eigen::Vector3f thrustHat_B = Eigen::Vector3f::UnitY();
    const Eigen::Vector3f sadaHat_B = Eigen::Vector3f::UnitX();
    const Eigen::Vector3f thrustReqHat_N = Eigen::Vector3f(1.0F, -1.0F, -1.0F).normalized();
    const float signOfN3Hat_N = -1.0F;

    testTriadRegression(rHat_SB_N, thrustHat_B, sadaHat_B, thrustReqHat_N, signOfN3Hat_N);
}

// ---------------------------------------------------------------------------
// Setup tests (setter validation + round-trip)
// ---------------------------------------------------------------------------

TEST(TriadTest, SetupTest) {
    // Valid config should not throw
    EXPECT_NO_THROW(TriadConfig::create(Eigen::Vector3f::UnitX(), Eigen::Vector3f::UnitY(), 1.0F));

    // Zero or non-unit vector sadaHat_B should throw
    EXPECT_THROW(TriadConfig::create(Eigen::Vector3f::Zero(), Eigen::Vector3f::UnitX(), -1.0F), fsw::invalid_argument);
    EXPECT_THROW(TriadConfig::create(Eigen::Vector3f(1.0F, 2.0F, 3.0F), Eigen::Vector3f::UnitX(), -1.0F),
                 fsw::invalid_argument);

    // Zero or non-unit vector thrustReqHat_N should throw
    EXPECT_THROW(TriadConfig::create(Eigen::Vector3f::UnitX(), Eigen::Vector3f::Zero(), 2.0F), fsw::invalid_argument);
    EXPECT_THROW(TriadConfig::create(Eigen::Vector3f::UnitX(), Eigen::Vector3f(1.0F, 2.0F, 3.0F), 2.0F),
                 fsw::invalid_argument);

    // Zero signOfN3Hat_N should throw
    EXPECT_THROW(TriadConfig::create(Eigen::Vector3f::UnitX(), Eigen::Vector3f::UnitY(), 0.0F), fsw::invalid_argument);

    // Config round-trip
    const Eigen::Vector3f sadaHat_B = Eigen::Vector3f::UnitX();
    const Eigen::Vector3f thrustReqHat_N = Eigen::Vector3f::UnitZ();
    const float signOfN3Hat_N = -1.0F;
    auto config = TriadConfig::create(sadaHat_B, thrustReqHat_N, signOfN3Hat_N);
    EXPECT_EQ(config.getSadaHat_B(), sadaHat_B);
    EXPECT_EQ(config.getThrustReqHat_N(), thrustReqHat_N);
    EXPECT_EQ(config.getSignOfN3Hat_N(), signOfN3Hat_N);

    // Static validators
    EXPECT_TRUE(TriadConfig::isValidSadaHat_B(Eigen::Vector3f::UnitX()));
    EXPECT_FALSE(TriadConfig::isValidSadaHat_B(Eigen::Vector3f::Zero()));
    EXPECT_TRUE(TriadConfig::isValidThrustReqHat_N(Eigen::Vector3f::UnitX()));
    EXPECT_FALSE(TriadConfig::isValidThrustReqHat_N(Eigen::Vector3f::Zero()));
    EXPECT_TRUE(TriadConfig::isValidSignOfN3Hat_N(-2.0F));
    EXPECT_FALSE(TriadConfig::isValidSignOfN3Hat_N(0.0F));
}

// ---------------------------------------------------------------------------
// Property tests
// ---------------------------------------------------------------------------

// All output components are finite for valid inputs.
TEST(TriadTest, OutputIsFinite) {
    const Eigen::Vector3f rHat_SB_N = Eigen::Vector3f(1.0F, 1.0F, 0.0F).normalized();
    const Eigen::Vector3f thrustHat_B = Eigen::Vector3f::UnitX();
    const Eigen::Vector3f sadaHat_B = Eigen::Vector3f::UnitY();
    const Eigen::Vector3f thrustReqHat_N = Eigen::Vector3f::UnitZ();
    const float signOfN3Hat_N = 1.0F;

    propertyOutputIsFinite(rHat_SB_N, thrustHat_B, sadaHat_B, thrustReqHat_N, signOfN3Hat_N);
}

// Thrust body axis should align with thrust inertial heading direction
TEST(TriadTest, ThrustBodyHeadingAlignedToThrustInertialHeading) {
    const Eigen::Vector3f rHat_SB_N = Eigen::Vector3f(1.0F, 1.0F, 0.0F).normalized();
    const Eigen::Vector3f thrustHat_B = Eigen::Vector3f::UnitX();
    const Eigen::Vector3f sadaHat_B = Eigen::Vector3f::UnitY();
    const Eigen::Vector3f thrustReqHat_N = Eigen::Vector3f::UnitZ();
    const float signOfN3Hat_N = 1.0F;

    propertyThrustBodyHeadingAlignedToThrustInertialHeading(
        rHat_SB_N, thrustHat_B, sadaHat_B, thrustReqHat_N, signOfN3Hat_N);
}

// sigma_RN norm is bounded by 1 (inner MRP set) for any inputs
TEST(TriadTest, SigmaRnNormBounded) {
    const Eigen::Vector3f rHat_SB_N = Eigen::Vector3f(1.0F, 1.0F, 0.0F).normalized();
    const Eigen::Vector3f thrustHat_B = Eigen::Vector3f::UnitX();
    const Eigen::Vector3f sadaHat_B = Eigen::Vector3f::UnitY();
    const Eigen::Vector3f thrustReqHat_N = Eigen::Vector3f::UnitZ();
    const float signOfN3Hat_N = 1.0F;

    propertySigmaNormBounded(rHat_SB_N, thrustHat_B, sadaHat_B, thrustReqHat_N, signOfN3Hat_N);
}

// The solar array offset angle from the Sun is bounded by the body thrust vector offset angle from the plane normal
// to the sada axis
TEST(TriadTest, SolarArraySunOffsetBoundedByBodyThrustOffset) {
    const Eigen::Vector3f rHat_SB_N = Eigen::Vector3f(1.0F, 1.0F, 0.0F).normalized();
    const float thrustYZOffsetAngleRad = 15.0F * std::numbers::pi_v<float> / 180.0F;
    const Eigen::Vector3f thrustHat_B =
        Eigen::Vector3f(safeSinf(thrustYZOffsetAngleRad), 0.0F, -safeCosf(thrustYZOffsetAngleRad));
    const Eigen::Vector3f sadaHat_B = Eigen::Vector3f::UnitX();
    const Eigen::Vector3f thrustReqHat_N = Eigen::Vector3f::UnitY();
    const float signOfN3Hat_N = 1.0F;

    propertySolarArraySunOffsetBoundedByBodyThrustOffset(
        rHat_SB_N, thrustHat_B, sadaHat_B, thrustReqHat_N, signOfN3Hat_N);
}

// ---------------------------------------------------------------------------
// Algorithm checks with solutions computed by hand
// ---------------------------------------------------------------------------

// Comparing algorithm output to pre-computed known output
TEST(TriadTest, ReturnedOutputMatchesPrecomputedReference) {
    const Eigen::Vector3f rHat_SB_N = Eigen::Vector3f(0.0F, 0.0F, -1.0F);
    const Eigen::Vector3f thrustHat_B = Eigen::Vector3f(0.0F, 1.0F, 1.0F).normalized();
    const Eigen::Vector3f sadaHat_B = Eigen::Vector3f::UnitY();
    const Eigen::Vector3f thrustReqHat_N = Eigen::Vector3f::UnitY();
    const float signOfN3Hat_N = 1.0F;

    auto config = TriadConfig::create(sadaHat_B, thrustReqHat_N, signOfN3Hat_N);
    TriadAlgorithm alg(config);
    const Eigen::Vector3f result = alg.update(rHat_SB_N, thrustHat_B);

    // Compute known output
    Eigen::Matrix3f dcm_BD;
    dcm_BD.col(0) = Eigen::Vector3f(0.0F, 1.0F, -1.0F).normalized();
    dcm_BD.col(1) = thrustHat_B;
    dcm_BD.col(2) = Eigen::Vector3f::UnitX();

    Eigen::Matrix3f dcm_ND;
    dcm_ND.col(0) = Eigen::Vector3f::UnitX();
    dcm_ND.col(1) = thrustReqHat_N;
    dcm_ND.col(2) = Eigen::Vector3f::UnitZ();

    const Eigen::Matrix3f dcm_RN = dcm_BD * dcm_ND.transpose();
    const Eigen::Vector3f expected = dcmToMrp(dcm_RN);

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(result(i), expected(i), 1e-6F);
    }
}

// Comparing final inertial sada axis with pre-computed known inertial sada axis direction
TEST(TriadTest, InertialSadaAxisMatchesPrecomputedAxis) {
    const Eigen::Vector3f rHat_SB_N = Eigen::Vector3f(-1.0F, 0.0F, 1.0F).normalized();
    const float thrustYZOffsetAngleRad = 15.0F * std::numbers::pi_v<float> / 180.0F;
    const Eigen::Vector3f thrustHat_B =
        Eigen::Vector3f(safeSinf(thrustYZOffsetAngleRad), 0.0F, -safeCosf(thrustYZOffsetAngleRad));
    const Eigen::Vector3f sadaHat_B = Eigen::Vector3f::UnitX();
    const Eigen::Vector3f thrustReqHat_N = -Eigen::Vector3f::UnitZ();
    const float signOfN3Hat_N = 1.0F;

    auto config = TriadConfig::create(sadaHat_B, thrustReqHat_N, signOfN3Hat_N);
    TriadAlgorithm alg(config);
    const Eigen::Vector3f sigma_RN = alg.update(rHat_SB_N, thrustHat_B);

    // Compute SADA axis in inertial frame components
    const Eigen::Matrix3f dcm_RN = mrpToDcm(sigma_RN);
    Eigen::Vector3f sadaHatResult_N = (dcm_RN.transpose() * sadaHat_B).stableNormalized();

    Eigen::Vector3f sadaHatExpected_N =
        Eigen::Vector3f(0.0F, -safeCosf(thrustYZOffsetAngleRad), -safeSinf(thrustYZOffsetAngleRad));

    EXPECT_NEAR(fabsf(sadaHatResult_N.dot(sadaHatExpected_N)), 1.0F, 1e-6F);
}

// ---------------------------------------------------------------------------
// Edge-case tests
// ---------------------------------------------------------------------------

// When the body thrust direction message is zero, the zero MRP is returned
TEST(TriadTest, ZeroThrustDirectionReturnsZero) {
    const Eigen::Vector3f rHat_SB_N = Eigen::Vector3f(1.0F, 1.0F, 0.0F).normalized();
    const Eigen::Vector3f thrustHat_B = Eigen::Vector3f::Zero();
    const Eigen::Vector3f sadaHat_B = Eigen::Vector3f::UnitY();
    const Eigen::Vector3f thrustReqHat_N = Eigen::Vector3f::UnitZ();
    const float signOfN3Hat_N = 1.0F;

    auto config = TriadConfig::create(sadaHat_B, thrustReqHat_N, signOfN3Hat_N);
    TriadAlgorithm alg(config);

    auto result = alg.update(rHat_SB_N, thrustHat_B);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(result(i), 0.0F, 1e-6F);
    }
}

// When the Sun direction message is zero, the zero MRP is returned
TEST(TriadTest, ZeroSunDirectionReturnsZero) {
    const Eigen::Vector3f rHat_SB_N = Eigen::Vector3f::Zero();
    const Eigen::Vector3f thrustHat_B = Eigen::Vector3f::UnitX();
    const Eigen::Vector3f sadaHat_B = Eigen::Vector3f::UnitY();
    const Eigen::Vector3f thrustReqHat_N = Eigen::Vector3f::UnitZ();
    const float signOfN3Hat_N = 1.0F;

    auto config = TriadConfig::create(sadaHat_B, thrustReqHat_N, signOfN3Hat_N);
    TriadAlgorithm alg(config);

    auto result = alg.update(rHat_SB_N, thrustHat_B);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(result(i), 0.0F, 1e-6F);
    }
}

// When the solar array drive axis is aligned with body thrust direction message, the zero MRP is returned
TEST(TriadTest, SadaAlignedBodyThrustReturnsZero) {
    const Eigen::Vector3f rHat_SB_N = Eigen::Vector3f(1.0F, 1.0F, 0.0F).normalized();
    const Eigen::Vector3f thrustHat_B = Eigen::Vector3f::UnitX();
    const Eigen::Vector3f sadaHat_B = Eigen::Vector3f::UnitX();
    const Eigen::Vector3f thrustReqHat_N = Eigen::Vector3f::UnitY();
    const float signOfN3Hat_N = 1.0F;

    auto config = TriadConfig::create(sadaHat_B, thrustReqHat_N, signOfN3Hat_N);
    TriadAlgorithm alg(config);

    auto result = alg.update(rHat_SB_N, thrustHat_B);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(result(i), 0.0F, 1e-6F);
    }
}

// When the Sun direction is aligned with the thrust inertial reference, the fallback inertial Z-axis is used in
// the triad frame computation
TEST(TriadTest, SunAlignedWithThrustRefUsesFallbackZAxis) {
    const Eigen::Vector3f rHat_SB_N = Eigen::Vector3f::UnitY();
    const Eigen::Vector3f thrustHat_B = Eigen::Vector3f::UnitX();
    const Eigen::Vector3f sadaHat_B = Eigen::Vector3f::UnitZ();
    const Eigen::Vector3f thrustReqHat_N = Eigen::Vector3f::UnitY();
    const float signOfN3Hat_N = 1.0F;

    auto config = TriadConfig::create(sadaHat_B, thrustReqHat_N, signOfN3Hat_N);
    TriadAlgorithm alg(config);

    auto result = alg.update(rHat_SB_N, thrustHat_B);
    Eigen::Vector3f expected = referenceTriad(rHat_SB_N, thrustHat_B, sadaHat_B, thrustReqHat_N, signOfN3Hat_N);

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(result(i), expected(i), 1e-6F);
    }
}

// When the sada axis is orthogonal to the body thrust direction, the array-Sun orthogonality constraint is met
TEST(TriadTest, SadaOrthogonalToSunWhenOrthogonalToBodyThrust) {
    const Eigen::Vector3f rHat_SB_N = Eigen::Vector3f(1.0F, 1.0F, 0.0F).normalized();
    const Eigen::Vector3f thrustHat_B = Eigen::Vector3f::UnitZ();
    const Eigen::Vector3f sadaHat_B = Eigen::Vector3f::UnitX();
    const Eigen::Vector3f thrustReqHat_N = Eigen::Vector3f::UnitZ();
    const float signOfN3Hat_N = 1.0F;

    auto config = TriadConfig::create(sadaHat_B, thrustReqHat_N, signOfN3Hat_N);
    TriadAlgorithm alg(config);

    const Eigen::Vector3f sigma_RN = alg.update(rHat_SB_N, thrustHat_B);

    // Compute Sun direction vector in reference body frame components
    const Eigen::Matrix3f dcm_RN = mrpToDcm(sigma_RN);
    Eigen::Vector3f rHat_SB_R = (dcm_RN * rHat_SB_N).stableNormalized();

    EXPECT_NEAR(sadaHat_B.dot(rHat_SB_R), 0.0F, 1e-6F);
}
