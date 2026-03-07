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

TEST(SunlineEphemTest, IdentityAttitude) {
    // With identity attitude (sigma_BN = 0), DCM is identity,
    // so the body-frame output sun direction should equal the sun direction in inertial-frame components
    SunlineEphemAlgorithm alg{};

    Eigen::Vector3d r_SN_N(1.0e6, 2.0e6, 3.0e6);
    Eigen::Vector3d r_BN_N(1.0e3, -2.0e3, 0.5e3);
    Eigen::Vector3f sigma_BN = Eigen::Vector3f::Zero();

    Eigen::Vector3f out;
    EXPECT_NO_THROW(out = alg.update(r_SN_N, r_BN_N, sigma_BN));

    // Expected: normalized (r_SN_N - r_BN_N) cast to float
    Eigen::Vector3f expected = (r_SN_N - r_BN_N).normalized().cast<float>();

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out[i], expected[i], 1e-6);
    }
}
