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
