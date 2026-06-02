#include "triadTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(TriadTest, RegressionCase1) {
    // SPE below 90 degrees
    const Eigen::Vector3f sigma_BN{0.0F, 0.0F, 0.0};
    const Eigen::Vector3f sadaHat_B = Eigen::Vector3f::UnitX();
    const Eigen::Vector3f thrustHat_B = Eigen::Vector3f::UnitY();
    const Eigen::Vector3f rHat_SB_B = Eigen::Vector3f(2.12024926e+11F, 2.12239088e+11F, 6.60583756e-01F).normalized();
    const Eigen::Vector3f thrustReqHat_N = Eigen::Vector3f(3.43401015e+11F, 2.76561597e+11F, 2.78825040e+10F).normalized();
    const float signOfZHat_N = -1.0F;

    testTriadRegression(sigma_BN, sadaHat_B, thrustHat_B, rHat_SB_B, thrustReqHat_N, signOfZHat_N);
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