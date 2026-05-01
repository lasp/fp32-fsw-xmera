// SPDX-FileCopyrightText: 2025 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
// SPDX-License-Identifier: MIT

#include "hillPointTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(HillPointTest, Setup) { testHillPointSetup(); }

TEST(HillPointTest, ReferenceTestPlanetAtOrigin) {
    // Circular equatorial orbit at ~17,800 km radius, planet at the origin
    testHillPoint(Eigen::Vector3d{8.92344e6, 1.54618e7, 0.0},  // r_BN_N
                  Eigen::Vector3d{-5.46e3, 3.15e3, 0.0},       // v_BN_N
                  Eigen::Vector3d::Zero(),                     // r_planet_N
                  Eigen::Vector3d::Zero()                      // v_planet_N
    );
}

TEST(HillPointTest, ReferenceTestPlanetOffset) {
    // Spacecraft orbits a planet that itself has nonzero inertial state
    testHillPoint(Eigen::Vector3d{1.5e11 + 7.0e6, 0.0, 0.0},  // r_BN_N (heliocentric + LEO offset)
                  Eigen::Vector3d{0.0, 7700.0, 0.0},           // v_BN_N (orbital + tiny ignored)
                  Eigen::Vector3d{1.5e11, 0.0, 0.0},           // r_planet_N (Earth heliocentric)
                  Eigen::Vector3d{0.0, 0.0, 0.0}               // v_planet_N (ignored for this test)
    );
}

TEST(HillPointTest, BelowThresholdRadius) {
    // Relative orbital radius below the 1 m robustness threshold: rates must be zero and the
    // attitude output must remain finite (no NaN from divide-by-near-zero).
    HillPointAlgorithm alg(HillPointConfig::create());

    HillPointOutput out;
    EXPECT_NO_THROW(out = alg.update(Eigen::Vector3d{0.5, 0.0, 0.0},  // r_BN_N: 0.5 m radius
                                     Eigen::Vector3d{0.0, 0.1, 0.0},  // small v_BN_N
                                     Eigen::Vector3d::Zero(),
                                     Eigen::Vector3d::Zero()));

    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(out.sigma_RN[i]));
        EXPECT_TRUE(std::isfinite(out.omega_RN_N[i]));
        EXPECT_TRUE(std::isfinite(out.domega_RN_N[i]));
    }
    // Rates should be exactly zero in this branch
    EXPECT_FLOAT_EQ(out.omega_RN_N[0], 0.0F);
    EXPECT_FLOAT_EQ(out.omega_RN_N[1], 0.0F);
    EXPECT_FLOAT_EQ(out.omega_RN_N[2], 0.0F);
    EXPECT_FLOAT_EQ(out.domega_RN_N[0], 0.0F);
    EXPECT_FLOAT_EQ(out.domega_RN_N[1], 0.0F);
    EXPECT_FLOAT_EQ(out.domega_RN_N[2], 0.0F);
}

TEST(HillPointTest, CircularEquatorialOrbit) {
    // Truth values match the Python regression test: a = 2.8 R_E, e = 0, i = 0, true anomaly 60 deg.
    // Expected sigma_RN_z = tan(theta/4) where theta = 60 deg => sigma = (0, 0, 2 - sqrt(3)).
    constexpr double mu_E = 3.986004418e14;       // [m^3/s^2]
    constexpr double earth_radius = 6.378136e6;   // [m]
    const double a = earth_radius * 2.8;
    const double speed = std::sqrt(mu_E / a);
    const double f_rad = 60.0 * M_PI / 180.0;

    HillPointAlgorithm alg(HillPointConfig::create());
    HillPointOutput out;
    EXPECT_NO_THROW(out = alg.update(Eigen::Vector3d{a * std::cos(f_rad), a * std::sin(f_rad), 0.0},
                                     Eigen::Vector3d{-speed * std::sin(f_rad), speed * std::cos(f_rad), 0.0},
                                     Eigen::Vector3d::Zero(),
                                     Eigen::Vector3d::Zero()));

    constexpr float tol = 1e-5F;
    EXPECT_NEAR(out.sigma_RN[0], 0.0F, tol);
    EXPECT_NEAR(out.sigma_RN[1], 0.0F, tol);
    EXPECT_NEAR(out.sigma_RN[2], 2.0F - std::sqrt(3.0F), tol);

    // In a circular orbit ddot{f} = 0 (radial velocity is zero by construction).
    EXPECT_NEAR(out.domega_RN_N[0], 0.0F, tol);
    EXPECT_NEAR(out.domega_RN_N[1], 0.0F, tol);
    EXPECT_NEAR(out.domega_RN_N[2], 0.0F, tol);
}
