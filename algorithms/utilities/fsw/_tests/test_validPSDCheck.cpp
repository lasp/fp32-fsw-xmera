// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "../validPSDCheck.h"

#include <gtest/gtest.h>

#include <Eigen/Core>

// Two guards in isPositiveSemiDefinite: eigen-solver succeeds, min eigenvalue >= -Size * epsilon *
// |largest eigenvalue|. Symmetry is the separate isSymmetric check.
TEST(IsPositiveSemiDefinite, ContractChecks) {
    // (1) Accepts a symmetric PSD matrix constructed as L·L^T.
    {
        Eigen::Matrix3d L;
        L << 1.0, 0.0, 0.0, 0.5, 1.0, 0.0, 0.3, 0.4, 1.0;
        Eigen::Matrix3d const P = L * L.transpose();
        EXPECT_TRUE(isPositiveSemiDefinite<3>(P)) << "L·L^T should be PSD";
    }
    // (2) Rejects a symmetric matrix with a negative eigenvalue.
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
    EXPECT_TRUE(isPositiveSemiDefinite<3>(Eigen::Matrix3d(Eigen::Matrix3d::Zero())));
    // Rejects tiny but genuinely negative eigenvalues: there is no absolute floor.
    EXPECT_FALSE(isPositiveSemiDefinite<3>(Eigen::Matrix3d(Eigen::Vector3d(-1e-14, 0.0, 0.0).asDiagonal())));
    EXPECT_FALSE(isPositiveSemiDefinite<3>(Eigen::Matrix3d(Eigen::Vector3d(1.0, 1.0, -1e-14).asDiagonal())));
    EXPECT_FALSE(isPositiveSemiDefinite<3>(Eigen::Matrix3d(Eigen::Vector3d(3e7, 1.0, -1e-3).asDiagonal())));
}

// Symmetry is checked entry by entry on the scale of the two variances, not against the whole matrix.
TEST(IsSymmetric, ChecksEachCrossTermOnTheScaleOfItsVariances) {
    // An asymmetric cross term between two small variances fails next to a large variance. A whole-matrix
    // relative check (|M - M^T| <= 1e-5 |M|) would let it through.
    {
        Eigen::Matrix3f P = Eigen::Vector3f(1e10F, 1e-4F, 1e-4F).asDiagonal();
        P(1, 2) = 5e-5F;
        P(2, 1) = 4e-5F;
        EXPECT_FALSE(isSymmetric<3>(P)) << "asymmetric small block";
    }
    // A symmetric double matrix narrowed to float stays exactly symmetric.
    {
        Eigen::Matrix3d L;
        L << 1e5, 0.0, 0.0, 1.0 / 3.0, 1e-3, 0.0, 2.0 / 7.0, 1e-4, 1e-6;
        Eigen::Matrix3d const P = L * L.transpose();
        Eigen::Matrix3f const Pf = P.cast<float>();
        EXPECT_TRUE(isSymmetric<3>(P)) << "double";
        EXPECT_TRUE(isSymmetric<3>(Pf)) << "narrowed to float";
    }
    // Rounding-sized asymmetry, a few ulps of the correlation scale, passes.
    {
        Eigen::Matrix2f P;
        P << 4.0F, 1.0F, 1.0F + 2.0F * std::numeric_limits<float>::epsilon(), 9.0F;
        EXPECT_TRUE(isSymmetric<2>(P));
    }
    // A cross term of a zero variance must be exactly symmetric.
    {
        Eigen::Matrix2f P;
        P << 0.0F, 1e-20F, 0.0F, 1.0F;
        EXPECT_FALSE(isSymmetric<2>(P)) << "zero variance";
        P(1, 0) = 1e-20F;
        EXPECT_TRUE(isSymmetric<2>(P)) << "zero variance, symmetric cross term";
    }
    // An upper triangle that disagrees with the lower one fails.
    {
        Eigen::Matrix3d A;
        A << 1.0, 2.0, 3.0, 0.0, 1.0, 2.0, 0.0, 0.0, 1.0;
        EXPECT_FALSE(isSymmetric<3>(A));
    }
    // A NaN entry fails.
    {
        Eigen::Matrix2f P = Eigen::Matrix2f::Identity();
        P(0, 1) = std::numeric_limits<float>::quiet_NaN();
        EXPECT_FALSE(isSymmetric<2>(P));
    }
}

// The float overload accepts the same PSD matrix and rejects a negative eigenvalue.
TEST(IsPositiveSemiDefinite, FloatContractChecks) {
    Eigen::Matrix3f L;
    L << 1.0F, 0.0F, 0.0F, 0.5F, 1.0F, 0.0F, 0.3F, 0.4F, 1.0F;
    Eigen::Matrix3f const P = L * L.transpose();
    EXPECT_TRUE(isPositiveSemiDefinite<3>(P)) << "L·L^T should be PSD";

    Eigen::Matrix3f N = Eigen::Matrix3f::Identity();
    N(0, 0) = -1.0F;
    EXPECT_FALSE(isPositiveSemiDefinite<3>(N)) << "negative eigenvalue should fail";
}
