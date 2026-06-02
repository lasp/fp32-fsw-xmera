#include "triadTestHelpers.hpp"
#include <gtest/gtest.h>

// ---------------------------------------------------------------------------
// Regression test
// ---------------------------------------------------------------------------

TEST(TriadTest, RegressionTest) {
    const Eigen::Vector3f sigma_BN{0.1F, -0.2F, 0.3F};
    const Eigen::Vector3f rHat_SB_B = Eigen::Vector3f(1.0F, 1.0F, 0.0F).normalized();
    const Eigen::Vector3f thrustHat_B = Eigen::Vector3f::UnitY();
    const Eigen::Vector3f sadaHat_B = Eigen::Vector3f::UnitX();
    const Eigen::Vector3f thrustReqHat_N = Eigen::Vector3f(1.0F, -1.0F, -1.0F).normalized();
    const float signOfZHat_N = -1.0F;

    testTriadRegression(sigma_BN, rHat_SB_B, thrustHat_B, sadaHat_B, thrustReqHat_N, signOfZHat_N);
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

    // Zero signOfZHat_N should throw
    EXPECT_THROW(TriadConfig::create(Eigen::Vector3f::UnitX(), Eigen::Vector3f::UnitY(), 0.0F), fsw::invalid_argument);

    // Config round-trip
    const Eigen::Vector3f sadaHat_B = Eigen::Vector3f::UnitX();
    const Eigen::Vector3f thrustReqHat_N = Eigen::Vector3f::UnitZ();
    const float signOfZHat_N = -1.0F;
    auto config = TriadConfig::create(sadaHat_B, thrustReqHat_N, signOfZHat_N);
    EXPECT_EQ(config.getSadaHat_B(), sadaHat_B);
    EXPECT_EQ(config.getThrustReqHat_N(), thrustReqHat_N);
    EXPECT_EQ(config.getSignOfZHat_N(), signOfZHat_N);

    // Static validators
    EXPECT_TRUE(TriadConfig::isValidSadaHat_B(Eigen::Vector3f::UnitX()));
    EXPECT_FALSE(TriadConfig::isValidSadaHat_B(Eigen::Vector3f::Zero()));
    EXPECT_TRUE(TriadConfig::isValidThrustReqHat_N(Eigen::Vector3f::UnitX()));
    EXPECT_FALSE(TriadConfig::isValidThrustReqHat_N(Eigen::Vector3f::Zero()));
    EXPECT_TRUE(TriadConfig::isValidSignOfZHat_N(-2.0F));
    EXPECT_FALSE(TriadConfig::isValidSignOfZHat_N(0.0F));
}


// ---------------------------------------------------------------------------
// Property tests
// ---------------------------------------------------------------------------

// All output components are finite for valid inputs.
TEST(TriadTest, OutputIsFinite) {
    const Eigen::Vector3f sigma_BN{0.1F, -0.2F, 0.3F};
    const Eigen::Vector3f rHat_SB_B = Eigen::Vector3f(1.0F, 1.0F, 0.0F).normalized();
    const Eigen::Vector3f thrustHat_B = Eigen::Vector3f::UnitX();
    const Eigen::Vector3f sadaHat_B = Eigen::Vector3f::UnitY();
    const Eigen::Vector3f thrustReqHat_N = Eigen::Vector3f::UnitZ();
    const float signOfZHat_N = 1.0F;

    propertyOutputIsFinite(sigma_BN, rHat_SB_B, thrustHat_B, sadaHat_B, thrustReqHat_N, signOfZHat_N);
}

// Thrust body axis should align with thrust inertial heading direction
TEST(TriadTest, BodyHeadingAlignedToInertialHeading) {
    const Eigen::Vector3f sigma_BN{0.1F, -0.2F, 0.3F};
    const Eigen::Vector3f rHat_SB_B = Eigen::Vector3f(1.0F, 1.0F, 0.0F).normalized();
    const Eigen::Vector3f thrustHat_B = Eigen::Vector3f::UnitX();
    const Eigen::Vector3f sadaHat_B = Eigen::Vector3f::UnitY();
    const Eigen::Vector3f thrustReqHat_N = Eigen::Vector3f::UnitZ();
    const float signOfZHat_N = 1.0F;

    propertyBodyHeadingAlignedToInertialHeading(sigma_BN, rHat_SB_B, thrustHat_B, sadaHat_B, thrustReqHat_N, signOfZHat_N);
}

// sigma_RN norm is bounded by 1 (inner MRP set) for any inputs
TEST(TriadTest, SigmaRnNormBounded) {
    const Eigen::Vector3f sigma_BN{0.1F, -0.2F, 0.3F};
    const Eigen::Vector3f rHat_SB_B = Eigen::Vector3f(1.0F, 1.0F, 0.0F).normalized();
    const Eigen::Vector3f thrustHat_B = Eigen::Vector3f::UnitX();
    const Eigen::Vector3f sadaHat_B = Eigen::Vector3f::UnitY();
    const Eigen::Vector3f thrustReqHat_N = Eigen::Vector3f::UnitZ();
    const float signOfZHat_N = 1.0F;

    propertySigmaNormBounded(sigma_BN, rHat_SB_B, thrustHat_B, sadaHat_B, thrustReqHat_N, signOfZHat_N);
}

TEST(TriadTest, ParallelVectorsThrows) {
    const Eigen::Vector3f sigma_BN{0.0F, 0.0F, 0.0};
    const Eigen::Vector3f sadaHat_B = Eigen::Vector3f::UnitX();
    const Eigen::Vector3f thrustHat_B = Eigen::Vector3f::UnitY();
    // rHat_SB_B and heading nearly parallel (SPE < 0.5 degrees)
    const Eigen::Vector3f rHat_SB_B = Eigen::Vector3f::UnitZ();
    const Eigen::Vector3f thrustReqHat_N = Eigen::Vector3f::UnitZ();
    const float signOfZHat_N = -1.0F;

    auto config = TriadConfig::create(sadaHat_B, thrustReqHat_N, signOfZHat_N);
    TriadAlgorithm alg(config);
    EXPECT_THROW(alg.update(sigma_BN, rHat_SB_B, thrustHat_B), std::runtime_error);
}