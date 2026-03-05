#include "sunlineEphemTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(SunlineEphemTest, ReferenceTest) {
    testSunlineEphem(std::vector<double>{1.0e6, 2.0e6, 3.0e6},   // sun position
                     std::vector<double>{1.0e3, -2.0e3, 0.5e3},  // spacecraft position
                     std::vector<float>{0.1f, 0.2f, -0.3f}       // non-trivial attitude
    );
}

TEST(SunlineEphemTest, Colocation) {
    // When sun and spacecraft are at the same position, the output should be a zero vector
    SunlineEphemAlgorithm alg{};

    Eigen::Vector3d r_SN_N(1.0, 2.0, 3.0);
    Eigen::Vector3d r_BN_N(1.0, 2.0, 3.0);
    Eigen::Vector3f sigma_BN(0.1f, 0.2f, -0.3f);

    Eigen::Vector3f out;
    EXPECT_NO_THROW(out = alg.update(r_SN_N, r_BN_N, sigma_BN));

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out[i], 0.0f, 1e-6);
    }
}
