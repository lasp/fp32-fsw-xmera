#include "triadTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(TriadTest, RegressionCase1) {
    // SPE below 90 degrees
    const Eigen::Vector3f a1Hat_B = Eigen::Vector3f::UnitX();
    const Eigen::Vector3f h1Hat_B = Eigen::Vector3f::UnitY();
    const Eigen::Vector3f sh_N = Eigen::Vector3f(2.12024926e+11F, 2.12239088e+11F, 6.60583756e-01F).normalized();
    const Eigen::Vector3f eh_N = Eigen::Vector3f(3.43401015e+11F, 2.76561597e+11F, 2.78825040e+10F).normalized();

    testTriadRegression(a1Hat_B, h1Hat_B, sh_N, eh_N);
}

TEST(TriadTest, RegressionCase2) {
    // SPE above 90 degrees
    const Eigen::Vector3f a1Hat_B = Eigen::Vector3f::UnitX();
    const Eigen::Vector3f h1Hat_B = Eigen::Vector3f::UnitY();
    const Eigen::Vector3f sh_N = Eigen::Vector3f(-7.47993852e+10F, -3.03274801e+08F, -6.16397545e-01F).normalized();
    const Eigen::Vector3f eh_N = Eigen::Vector3f(5.65767033e+10F, 6.40192339e+10F, 2.78825040e+10F).normalized();

    testTriadRegression(a1Hat_B, h1Hat_B, sh_N, eh_N);
}

TEST(TriadTest, SetupTest) { testTriadSetup(); }
