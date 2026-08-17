#include "hillPointTestHelpers.hpp"
#include <gtest/gtest.h>
#include <numbers>

TEST(HillPointTest, Setup) { testHillPointSetup(); }

TEST(HillPointTest, ReferenceTestPlanetAtOrigin) {
    // Circular equatorial orbit at ~17,800 km radius, planet at the origin
    testHillPointRegression(Eigen::Vector3d{8.92344e6, 1.54618e7, 0.0},  // r_BN_N
                            Eigen::Vector3d{-5.46e3, 3.15e3, 0.0},       // v_BN_N
                            Eigen::Vector3d::Zero(),                     // r_PN_N
                            Eigen::Vector3d::Zero()                      // v_PN_N
    );
}

TEST(HillPointTest, ReferenceTestPlanetOffset) {
    // Spacecraft orbits a planet that itself has nonzero inertial state
    testHillPointRegression(
        Eigen::Vector3d{1.5e11 + 7.0e6, 0.0, 0.0},    // r_BN_N (heliocentric + LEO offset)
        Eigen::Vector3d{0.0, 29800.0 + 7700.0, 0.0},  // v_BN_N (Earth heliocentric + SC relative velocity)
        Eigen::Vector3d{1.5e11, 0.0, 0.0},            // r_PN_N (Earth heliocentric)
        Eigen::Vector3d{0.0, 29800.0, 0.0}            // v_PN_N (Earth heliocentric velocity)
    );
}

TEST(HillPointTest, BelowThresholdRadius) {
    // Relative orbital radius below the 1 m robustness threshold: attitude and rates must be zero
    HillPointAlgorithm alg;

    HillPointOutput out;
    EXPECT_NO_THROW(out = alg.update(Eigen::Vector3d{0.5, 0.0, 0.0},  // r_BN_N: 0.5 m radius
                                     Eigen::Vector3d{0.0, 0.1, 0.0},  // small v_BN_N
                                     Eigen::Vector3d::Zero(),
                                     Eigen::Vector3d::Zero()));

    // Attitude and rates should be exactly zero in this branch
    EXPECT_FLOAT_EQ(out.sigma_RN[0], 0.0F);
    EXPECT_FLOAT_EQ(out.sigma_RN[1], 0.0F);
    EXPECT_FLOAT_EQ(out.sigma_RN[2], 0.0F);
    EXPECT_FLOAT_EQ(out.omega_RN_N[0], 0.0F);
    EXPECT_FLOAT_EQ(out.omega_RN_N[1], 0.0F);
    EXPECT_FLOAT_EQ(out.omega_RN_N[2], 0.0F);
    EXPECT_FLOAT_EQ(out.domega_RN_N[0], 0.0F);
    EXPECT_FLOAT_EQ(out.domega_RN_N[1], 0.0F);
    EXPECT_FLOAT_EQ(out.domega_RN_N[2], 0.0F);
}

TEST(HillPointTest, BelowSmallAngleThreshold) {
    // r and v here are collinear (v is just pointing along the same line as r), so h = r x v is
    // ~0 and there's no well-defined orbital plane.
    HillPointAlgorithm alg;

    HillPointOutput out;
    EXPECT_NO_THROW(out = alg.update(Eigen::Vector3d{8.0e6, 0.0, 0.0},  // r_BN_N: valid radius
                                     Eigen::Vector3d{100.0, 0.0, 0.0},  // v_BN_N: parallel to r_BN_N
                                     Eigen::Vector3d::Zero(),
                                     Eigen::Vector3d::Zero()));

    // Attitude and rates should be exactly zero in this branch
    EXPECT_FLOAT_EQ(out.sigma_RN[0], 0.0F);
    EXPECT_FLOAT_EQ(out.sigma_RN[1], 0.0F);
    EXPECT_FLOAT_EQ(out.sigma_RN[2], 0.0F);
    EXPECT_FLOAT_EQ(out.omega_RN_N[0], 0.0F);
    EXPECT_FLOAT_EQ(out.omega_RN_N[1], 0.0F);
    EXPECT_FLOAT_EQ(out.omega_RN_N[2], 0.0F);
    EXPECT_FLOAT_EQ(out.domega_RN_N[0], 0.0F);
    EXPECT_FLOAT_EQ(out.domega_RN_N[1], 0.0F);
    EXPECT_FLOAT_EQ(out.domega_RN_N[2], 0.0F);
}

TEST(HillPointTest, CircularOrbit) {
    testHillPointConicOrbit(
        0.0, 3.986004418e14, 1.0e7, 45.0 * std::numbers::pi_v<double> / 180.0);  // e = 0.0: circular orbit
}

TEST(HillPointTest, EllipticalOrbit) {
    testHillPointConicOrbit(
        0.3, 3.986004418e14, 1.0e7, 45.0 * std::numbers::pi_v<double> / 180.0);  // 0 < e < 1: elliptical orbit
}

TEST(HillPointTest, ParabolicOrbit) {
    testHillPointConicOrbit(
        1.0, 3.986004418e14, 1.0e7, 45.0 * std::numbers::pi_v<double> / 180.0);  // e = 1: parabolic orbit
}

TEST(HillPointTest, HyperbolicOrbit) {
    testHillPointConicOrbit(
        1.5, 3.986004418e14, 1.0e7, 45.0 * std::numbers::pi_v<double> / 180.0);  // e > 1: hyperbolic orbit
}

TEST(HillPointTest, OutputIsFinite) {
    propertyOutputIsFinite(Eigen::Vector3d{8.92344e6, 1.54618e7, 0.0},  // r_BN_N
                           Eigen::Vector3d{-5.46e3, 3.15e3, 0.0},       // v_BN_N
                           Eigen::Vector3d::Zero(),                     // r_PN_N
                           Eigen::Vector3d::Zero()                      // v_PN_N
    );
}

TEST(HillPointTest, SigmaNormBounded) {
    propertySigmaNormBounded(Eigen::Vector3d{8.92344e6, 1.54618e7, 0.0},  // r_BN_N
                             Eigen::Vector3d{-5.46e3, 3.15e3, 0.0},       // v_BN_N
                             Eigen::Vector3d::Zero(),                     // r_PN_N
                             Eigen::Vector3d::Zero()                      // v_PN_N
    );
}

TEST(HillPointTest, ZeroVelocityAtValidRadius) {
    // This test has v_BN_N as zero with a valid r_BN_N orbital radius. Since eigen normalizes a zero vector
    // to zero instead of producing a NaN, this case falls into the fallback branch, where attitude and
    // rates must be zero.
    testHillPointDegenerateFallback(Eigen::Vector3d{8.92344e6, 1.54618e7, 0.0},  // r_BN_N
                                    Eigen::Vector3d{0.0, 0.0, 0.0},              // v_BN_N
                                    Eigen::Vector3d::Zero(),                     // r_PN_N
                                    Eigen::Vector3d::Zero());                    // v_PN_N
}
