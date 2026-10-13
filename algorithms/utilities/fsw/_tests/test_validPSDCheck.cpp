// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "../validPSDCheck.h"

#include <gtest/gtest.h>

#include <Eigen/Core>

// Three guards in isPositiveSemiDefinite: symmetric, eigen-solver succeeds,
// min eigenvalue >= -Size * epsilon * |largest eigenvalue|. Exercise each via
// positive cases and negative cases that trip the two observable guards.
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

// The tolerance is the eigen solver's roundoff band, Size * epsilon * |largest eigenvalue|.
TEST(IsPositiveSemiDefinite, ToleranceIsTheRoundoffBand) {
    // Accepts a roundoff-sized negative eigenvalue next to a large one (band 3 * 2.2e-16 * 3e7 = 2e-8).
    EXPECT_TRUE(isPositiveSemiDefinite<3>(Eigen::Matrix3d(Eigen::Vector3d(3e7, 1.0, -1e-10).asDiagonal())));
    // Accepts rank-deficient matrices whose zero eigenvalues come back with roundoff of either sign.
    {
        Eigen::Vector3d const v(1.0, 1.0 / 3.0, -2.0 / 7.0);
        EXPECT_TRUE(isPositiveSemiDefinite<3>(Eigen::Matrix3d(v * v.transpose()))) << "rank 1";
        Eigen::Matrix<double, 6, 2> B;
        B << 1.0, 0.1, 1.0 / 3.0, 2.0, -0.7, 1e-3, 5.0, -1.0 / 7.0, 0.2, 0.9, 1e2, 3.0;
        EXPECT_TRUE(isPositiveSemiDefinite<6>(Eigen::Matrix<double, 6, 6>(B * B.transpose()))) << "rank 2";
    }
    // Accepts the zero matrix (e.g. zero process noise): the band is zero and so are the eigenvalues.
    EXPECT_TRUE(isPositiveSemiDefinite<3>(Eigen::Matrix3d::Zero()));
    // Rejects tiny but genuinely negative eigenvalues: there is no absolute floor.
    EXPECT_FALSE(isPositiveSemiDefinite<3>(Eigen::Matrix3d(Eigen::Vector3d(-1e-14, 0.0, 0.0).asDiagonal())));
    EXPECT_FALSE(isPositiveSemiDefinite<3>(Eigen::Matrix3d(Eigen::Vector3d(1.0, 1.0, -1e-14).asDiagonal())));
    EXPECT_FALSE(isPositiveSemiDefinite<3>(Eigen::Matrix3d(Eigen::Vector3d(3e7, 1.0, -1e-3).asDiagonal())));
}
