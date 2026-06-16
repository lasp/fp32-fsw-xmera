#ifndef TEST_INERTIAL3D_H
#define TEST_INERTIAL3D_H

#include "inertial3DAlgorithm.h"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cmath>

// The fixed inertial-3D reference simply echoes the configured MRP.
inline void testInertial3D(const Eigen::Vector3f& sigma_RN) {
    Inertial3DAlgorithm alg{Inertial3DConfig::create(sigma_RN)};

    Eigen::Vector3f out;
    EXPECT_NO_THROW(out = alg.update());

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out[i], sigma_RN[i], 1e-6);
        EXPECT_TRUE(std::isfinite(out[i]));
    }
}

#endif  // TEST_INERTIAL3D_H
