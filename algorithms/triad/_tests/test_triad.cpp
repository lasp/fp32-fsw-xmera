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
