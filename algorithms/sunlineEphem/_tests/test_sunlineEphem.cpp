#include "sunlineEphemTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(SunlineEphemTest, ReferenceTest) {
    testSunlineEphem(Eigen::Vector3d{1.0e6, 2.0e6, 3.0e6},   // sun position
                     Eigen::Vector3d{1.0e3, -2.0e3, 0.5e3},  // spacecraft position
                     Eigen::Vector3f{0.1F, 0.2F, -0.3F}      // non-trivial attitude
    );
}

TEST(SunlineEphemTest, Colocation) {
    // When sun and spacecraft are at the same position, the output should be a zero vector
    SunlineEphemAlgorithm alg{SunlineEphemConfig::create()};

    Eigen::Vector3d r_SN_N(1.0, 2.0, 3.0);
    Eigen::Vector3d r_BN_N(1.0, 2.0, 3.0);
    Eigen::Vector3f sigma_BN(0.1f, 0.2f, -0.3f);

    Eigen::Vector3f out;
    EXPECT_NO_THROW(out = alg.update(r_SN_N, r_BN_N, sigma_BN));

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out[i], 0.0f, 1e-6);
    }
}

TEST(SunlineEphemTest, TinyRelativePosition) {
    // Scale reasonable positions by a tiny factor so the separation norm falls far below the
    // squaring underflow threshold (~1e-154). Squaring the components underflows to zero, so
    // regular norm() returns zero and normalized() produces NaN. stableNormalized() scales
    // before squaring and should still return the same unit direction.
    SunlineEphemAlgorithm alg{SunlineEphemConfig::create()};

    const Eigen::Vector3d r_SN_N_base(1.0, 2.0, 3.0);
    const Eigen::Vector3d r_BN_N_base(4.0, 5.0, 6.0);
    const Eigen::Vector3f sigma_BN = Eigen::Vector3f::Zero();
    const Eigen::Vector3f expected = (r_SN_N_base - r_BN_N_base).normalized().cast<float>();

    const double scale = 1.0e-200;
    Eigen::Vector3f out;
    EXPECT_NO_THROW(out = alg.update(scale * r_SN_N_base, scale * r_BN_N_base, sigma_BN));

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out[i], expected[i], 1e-6f);
    }
}

TEST(SunlineEphemTest, HugeRelativePosition) {
    // Scale reasonable positions by a huge factor so squaring the components overflows to inf.
    // Regular norm() returns inf and normalized() produces NaN.
    // stableNormalized() scales by the max component first and should still return the same unit direction.
    SunlineEphemAlgorithm alg{SunlineEphemConfig::create()};

    const Eigen::Vector3d r_SN_N_base(1.0, 2.0, 3.0);
    const Eigen::Vector3d r_BN_N_base(4.0, 5.0, 6.0);
    const Eigen::Vector3f sigma_BN = Eigen::Vector3f::Zero();
    const Eigen::Vector3f expected = (r_SN_N_base - r_BN_N_base).normalized().cast<float>();

    const double scale = 1.0e200;
    Eigen::Vector3f out;
    EXPECT_NO_THROW(out = alg.update(scale * r_SN_N_base, scale * r_BN_N_base, sigma_BN));

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out[i], expected[i], 1e-6f);
    }
}

TEST(SunlineEphemTest, IdentityAttitude) {
    // With identity attitude (sigma_BN = 0), DCM is identity,
    // so the body-frame output sun direction should equal the sun direction in inertial-frame components
    SunlineEphemAlgorithm alg{SunlineEphemConfig::create()};

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
