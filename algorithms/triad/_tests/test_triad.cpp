#include "triadTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(TriadTest, RegressionCase1) {
    // SPE below 90 degrees
    const Eigen::Vector3f sigma_BN{0.0F, 0.0F, 0.0};
    const Eigen::Vector3f a1Hat_B = Eigen::Vector3f::UnitX();
    const Eigen::Vector3f h1Hat_B = Eigen::Vector3f::UnitY();
    const Eigen::Vector3f sh_B = Eigen::Vector3f(2.12024926e+11F, 2.12239088e+11F, 6.60583756e-01F).normalized();
    const Eigen::Vector3f eh_N = Eigen::Vector3f(3.43401015e+11F, 2.76561597e+11F, 2.78825040e+10F).normalized();
    const float signOfZHat_N = -1.0F;

    testTriadRegression(sigma_BN, a1Hat_B, h1Hat_B, sh_B, eh_N, signOfZHat_N);
}

TEST(TriadTest, RegressionCase2) {
    // SPE above 90 degrees
    const Eigen::Vector3f sigma_BN{0.0F, 0.0F, 0.0};
    const Eigen::Vector3f a1Hat_B = Eigen::Vector3f::UnitX();
    const Eigen::Vector3f h1Hat_B = Eigen::Vector3f::UnitY();
    const Eigen::Vector3f sh_B = Eigen::Vector3f(-7.47993852e+10F, -3.03274801e+08F, -6.16397545e-01F).normalized();
    const Eigen::Vector3f eh_N = Eigen::Vector3f(5.65767033e+10F, 6.40192339e+10F, 2.78825040e+10F).normalized();
    const float signOfZHat_N = -1.0F;

    testTriadRegression(sigma_BN, a1Hat_B, h1Hat_B, sh_B, eh_N, signOfZHat_N);
}

TEST(TriadTest, SetupTest) { testTriadSetup(); }

TEST(TriadTest, OutputIsFinite) {
    const Eigen::Vector3f sigma_BN{0.0F, 0.0F, 0.0};
    const Eigen::Vector3f a1 = Eigen::Vector3f::UnitX();
    const Eigen::Vector3f h1 = Eigen::Vector3f::UnitY();
    const Eigen::Vector3f sun = Eigen::Vector3f(1.0F, 1.0F, 0.0F).normalized();
    const Eigen::Vector3f earth = Eigen::Vector3f(0.0F, 0.0F, 1.0F).normalized();
    const float signOfZHat_N = -1.0F;

    auto config = TriadConfig::create(a1, earth, signOfZHat_N);
    TriadAlgorithm alg(config);
    const Eigen::Vector3f result = alg.update(sigma_BN, sun, h1);

    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(result[i]));
    }
}

TEST(TriadTest, ParallelVectorsThrows) {
    const Eigen::Vector3f sigma_BN{0.0F, 0.0F, 0.0};
    const Eigen::Vector3f a1 = Eigen::Vector3f::UnitX();
    const Eigen::Vector3f h1 = Eigen::Vector3f::UnitY();
    // Sun and heading nearly parallel (SPE < 0.5 degrees)
    const Eigen::Vector3f sun = Eigen::Vector3f::UnitZ();
    const Eigen::Vector3f earth = Eigen::Vector3f::UnitZ();
    const float signOfZHat_N = -1.0F;

    auto config = TriadConfig::create(a1, earth, signOfZHat_N);
    TriadAlgorithm alg(config);
    EXPECT_THROW(alg.update(sigma_BN, sun, h1), std::runtime_error);
}

TEST(TriadTest, BodyHeadingAlignedToInertialHeading) {
    // When the algorithm runs, the reference frame's y-axis (r2 = h1Hat_B direction)
    // should be aligned with the inertial heading direction
    const Eigen::Vector3f sigma_BN{0.0F, 0.0F, 0.0};
    const Eigen::Vector3f a1 = Eigen::Vector3f::UnitX();
    const Eigen::Vector3f h1 = Eigen::Vector3f::UnitY();
    const Eigen::Vector3f sun = Eigen::Vector3f(1.0F, 0.0F, 0.0F);
    const Eigen::Vector3f earth = Eigen::Vector3f(0.0F, 1.0F, 0.0F);
    const float signOfZHat_N = -1.0F;

    auto config = TriadConfig::create(a1, earth, signOfZHat_N);
    TriadAlgorithm alg(config);
    const Eigen::Vector3f sigma_RN = alg.update(sigma_BN, sun, h1);

    // Convert MRP to DCM and check y-axis alignment
    const Eigen::Matrix3f RN = mrpToDcm(sigma_RN);
    const float dot = RN.col(1).dot(earth);
    EXPECT_NEAR(dot, 1.0F, 1e-5F);
}

TEST(TriadTest, ConfigSetConfig) {
    auto config1 = TriadConfig::create(Eigen::Vector3f::UnitX(), Eigen::Vector3f::UnitY(), 1.0F);
    TriadAlgorithm alg(config1);

    auto config2 = TriadConfig::create(Eigen::Vector3f::UnitZ(), Eigen::Vector3f::UnitY(), -1.0F);
    EXPECT_NO_THROW(alg.setConfig(config2));
}
