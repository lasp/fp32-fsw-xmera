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

TEST(TriadTest, SetupTest) { testTriadSetup(); }

TEST(TriadTest, OutputIsFinite) {
    const Eigen::Vector3f sigma_BN{0.0F, 0.0F, 0.0};
    const Eigen::Vector3f sadaHat_B = Eigen::Vector3f::UnitX();
    const Eigen::Vector3f thrustHat_B = Eigen::Vector3f::UnitY();
    const Eigen::Vector3f rHat_SB_B = Eigen::Vector3f(1.0F, 1.0F, 0.0F).normalized();
    const Eigen::Vector3f thrustReqHat_N = Eigen::Vector3f(0.0F, 0.0F, 1.0F).normalized();
    const float signOfZHat_N = -1.0F;

    auto config = TriadConfig::create(sadaHat_B, thrustReqHat_N, signOfZHat_N);
    TriadAlgorithm alg(config);
    const Eigen::Vector3f result = alg.update(sigma_BN, rHat_SB_B, thrustHat_B);

    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(result[i]));
    }
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