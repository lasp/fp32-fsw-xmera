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
