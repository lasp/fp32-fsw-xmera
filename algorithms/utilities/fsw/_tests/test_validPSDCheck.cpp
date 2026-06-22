// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "../validPSDCheck.h"

#include <gtest/gtest.h>

#include <Eigen/Core>

// Three guards in isPositiveSemiDefinite: symmetric, eigen-solver succeeds,
// min eigenvalue >= -1e-12. Exercise each via a positive case and two
// negative cases that trip the two observable guards.
TEST(IsPositiveSemiDefinite, ContractChecks) {
    // (1) Accepts a symmetric PSD matrix constructed as L·L^T.
    {
        Eigen::Matrix3d L;
        L << 1.0, 0.0, 0.0, 0.5, 1.0, 0.0, 0.3, 0.4, 1.0;
        Eigen::Matrix3d const P = L * L.transpose();
        EXPECT_TRUE(isPositiveSemiDefinite<3>(P)) << "L·L^T should be PSD";
    }
    // (2) Rejects a non-symmetric matrix (fails symmetry guard).
    {
        Eigen::Matrix3d A;
        A << 1.0, 2.0, 3.0, 0.0, 1.0, 2.0, 0.0, 0.0, 1.0;
        EXPECT_FALSE(isPositiveSemiDefinite<3>(A)) << "non-symmetric should fail";
    }
    // (3) Rejects a symmetric matrix with a negative eigenvalue.
    {
        Eigen::Matrix3d N = Eigen::Matrix3d::Identity();
        N(0, 0) = -1.0;
        EXPECT_FALSE(isPositiveSemiDefinite<3>(N)) << "negative eigenvalue should fail";
    }
}
