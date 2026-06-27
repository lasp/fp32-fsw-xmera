// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

// Unit tests for filtering::SRuKF<State, Dynamics>.
//
// Sections:
//   * Static numerical helpers (forward/back substitution, Cholesky,
//     QR-just-R, Cholesky up/down-date, PSD check).
//   * Setters / getters / validity checks.
//   * timeUpdate(dt): rewind-to-anchor + propagate + process-noise growth.
//   * measurementUpdate(M): innovation, covariance shrink, anchor rolling.
//
// Time and measurement updates are the heavy ones; left as scaffolding.

#include <filteringCore/srukf.hpp>
#include <filteringCore/state.hpp>

#include <gtest/gtest.h>

#include <Eigen/Core>

namespace filtering {
namespace {

// Make a filter with zero dynamics
using TestState = StateVector<Position<3>>;
struct ZeroDynamics {
    TestState operator()(double, TestState const& s) const { return s.scale(0.0); }
};
using SRuKFType = SRuKF<TestState, ZeroDynamics>;

// Position measurement used by the measurementUpdate tests below.
struct PositionMeasurement {
    static constexpr int size = 3;
    Eigen::Vector3d observed = Eigen::Vector3d::Zero();
    Eigen::Matrix3d noiseCov = Eigen::Matrix3d::Identity();
    Eigen::Vector3d observation() const { return observed; }
    Eigen::Vector3d model(TestState const& s) const { return s.get<Position<3>>(); }
    Eigen::Matrix3d noise() const { return noiseCov; }
    Eigen::Vector3d subtract(Eigen::Vector3d const& a, Eigen::Vector3d const& b) const { return a - b; }
};
static_assert(Measurement<PositionMeasurement, TestState>);

}  // namespace

// Test all of the static functions
TEST(SrukfStatic, ForwardSubstitutionSolvesLowerTriangular) {
    Eigen::Matrix3d L;
    L << 2.0, 0.0, 0.0, 1.0, 3.0, 0.0, 0.5, 1.0, 4.0;
    Eigen::Vector3d const xTruth(1.0, 2.0, 3.0);
    Eigen::Matrix<double, 3, 1> const b = L * xTruth;

    Eigen::Matrix<double, 3, 1> const x = SRuKFType::forwardSubstitution<3, 1>(L, b);

    EXPECT_TRUE(x.col(0).isApprox(xTruth, 1e-12));
}

TEST(SrukfStatic, BackSubstitutionSolvesUpperTriangular) {
    Eigen::Matrix3d U;
    U << 2.0, 1.0, 0.5, 0.0, 3.0, 1.0, 0.0, 0.0, 4.0;
    Eigen::Vector3d const xTruth(1.0, 2.0, 3.0);
    Eigen::Matrix<double, 3, 1> const b = U * xTruth;

    Eigen::Matrix<double, 3, 1> const x = SRuKFType::backSubstitution<3, 1>(U, b);

    EXPECT_TRUE(x.col(0).isApprox(xTruth, 1e-12));
}

TEST(SrukfStatic, CholeskyDecompositionReconstructsP) {
    Eigen::Matrix3d P;
    P << 4.0, 2.0, 0.5, 2.0, 5.0, 1.0, 0.5, 1.0, 6.0;

    Eigen::Matrix3d const L = SRuKFType::choleskyDecomposition<3>(P);

    EXPECT_TRUE((L * L.transpose()).isApprox(P, 1e-10)) << "L·L^T must equal P";
    for (int i = 0; i < 3; ++i) {
        for (int j = i + 1; j < 3; ++j) {
            EXPECT_NEAR(L(i, j), 0.0, 1e-12) << "L must be lower-triangular at (" << i << "," << j << ")";
        }
    }
}

TEST(SrukfStatic, QrDecompositionJustRReturnsLowerTriangularSatisfyingNormalEquations) {
    Eigen::Matrix<double, 3, 9> A;
    A << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 2.0, -1.0, 0.0, 1.0,
        2.0, 3.0, 4.0, 5.0, 6.0;

    Eigen::Matrix3d const R = SRuKFType::qrDecompositionJustR<3, 9>(A);

    for (int i = 0; i < 3; ++i) {
        for (int j = i + 1; j < 3; ++j) {
            EXPECT_NEAR(R(i, j), 0.0, 1e-10) << "R must be lower-triangular at (" << i << "," << j << ")";
        }
    }
    EXPECT_TRUE((R * R.transpose()).isApprox(A * A.transpose(), 1e-9))
        << "R·R^T must equal A·A^T (normal equations preserved)";
}

TEST(SrukfStatic, CholeskyUpDownDateMatchesExplicitUpdate) {
    Eigen::Matrix3d P0;
    P0 << 4.0, 1.0, 0.0, 1.0, 3.0, 0.5, 0.0, 0.5, 2.0;
    Eigen::Matrix3d const S0 = SRuKFType::choleskyDecomposition<3>(P0);
    Eigen::Vector3d const v(0.1, -0.2, 0.3);

    // Up-date with +coef: chol(P + |coef|·v·v^T).
    {
        double const coef = 0.5;
        Eigen::Matrix3d const S1 = SRuKFType::choleskyUpDownDate<3>(S0, v, coef);
        Eigen::Matrix3d const P1 = P0 + coef * v * v.transpose();
        EXPECT_TRUE((S1 * S1.transpose()).isApprox(P1, 1e-9)) << "up-date";
    }
    // Down-date with -coef: chol(P - |coef|·v·v^T).
    {
        double const coef = -0.5;
        Eigen::Matrix3d const S1 = SRuKFType::choleskyUpDownDate<3>(S0, v, coef);
        Eigen::Matrix3d const P1 = P0 - 0.5 * v * v.transpose();
        EXPECT_TRUE((S1 * S1.transpose()).isApprox(P1, 1e-9)) << "down-date";
    }
}

// Test setters/getters and reset

// Setters/Getters
TEST(SrukfApi, SetGetRoundtripsForAlphaAndBeta) {
    SRuKFType filter;
    filter.setAlpha(0.5);
    filter.setBeta(2.0);
    EXPECT_DOUBLE_EQ(filter.getAlpha(), 0.5);
    EXPECT_DOUBLE_EQ(filter.getBeta(), 2.0);
}

// Valid inputs
TEST(SrukfApi, ValidityChecks) {
    SRuKFType filter;

    // (1) alphaIsValid: true iff alpha in [0, 1].
    {
        filter.setAlpha(0.5);
        EXPECT_TRUE(filter.alphaIsValid()) << "alpha=0.5 in range";
        filter.setAlpha(-0.1);
        EXPECT_FALSE(filter.alphaIsValid()) << "alpha<0";
        filter.setAlpha(1.5);
        EXPECT_FALSE(filter.alphaIsValid()) << "alpha>1";
    }
    // (2) betaIsValid: true iff beta in [0, 2].
    {
        filter.setBeta(1.0);
        EXPECT_TRUE(filter.betaIsValid()) << "beta=1 in range";
        filter.setBeta(-0.5);
        EXPECT_FALSE(filter.betaIsValid()) << "beta<0";
        filter.setBeta(2.5);
        EXPECT_FALSE(filter.betaIsValid()) << "beta>2";
    }
    // (3) initialCovarianceIsValid: true iff PSD.
    {
        filter.setInitialCovariance(Eigen::Matrix3d::Identity());
        EXPECT_TRUE(filter.initialCovarianceIsValid()) << "identity is PSD";

        Eigen::Matrix3d P_neg = -Eigen::Matrix3d::Identity();
        filter.setInitialCovariance(P_neg);
        EXPECT_FALSE(filter.initialCovarianceIsValid()) << "negative eigenvalue";
    }
    // (4) processNoiseIsValid: true iff PSD.
    {
        filter.setProcessNoise(Eigen::Matrix3d::Identity());
        EXPECT_TRUE(filter.processNoiseIsValid()) << "identity is PSD";

        Eigen::Matrix3d Q_neg = -Eigen::Matrix3d::Identity();
        filter.setProcessNoise(Q_neg);
        EXPECT_FALSE(filter.processNoiseIsValid()) << "negative eigenvalue";
    }
}

// Reset
TEST(SrukfApi, ResetCopiesInitialStateAndCovarianceToWorkingState) {
    SRuKFType filter;
    filter.setAlpha(0.5);
    filter.setBeta(2.0);

    TestState s0;
    Eigen::Vector<double, 3> const x0(1.0, 2.0, 3.0);
    s0.set<Position<3>>(x0);
    filter.setInitialState(s0);

    Eigen::Matrix3d const P0 = 2.0 * Eigen::Matrix3d::Identity();
    filter.setInitialCovariance(P0);
    filter.setProcessNoise(0.1 * Eigen::Matrix3d::Identity());

    filter.reset();
    filter.reConfigure();

    // Working state and last-measurement state are both stateInitial.
    EXPECT_TRUE(filter.getState().raw().isApprox(s0.raw(), 1e-12)) << "state";
    EXPECT_TRUE(filter.getStateAtLastMeasurement().raw().isApprox(s0.raw(), 1e-12)) << "anchor";

    // Working covariance is covarianceInitial.
    EXPECT_TRUE(filter.getCovariance().isApprox(P0, 1e-12)) << "covariance";
}

// timeUpdate tests

// Time update doesn't change the last-measurement state
TEST(SrukfTimeUpdate, RewindsToLastMeasurementMakingTimeUpdateIdempotent) {
    SRuKFType filter;
    filter.setAlpha(0.5);
    filter.setBeta(2.0);
    filter.setInitialCovariance(Eigen::Matrix3d::Identity());
    filter.setProcessNoise(0.1 * Eigen::Matrix3d::Identity());
    filter.reset();
    filter.reConfigure();

    // First timeUpdate from anchor produces stateA / covA.
    filter.timeUpdate(2.0);
    TestState const stateA = filter.getState();
    Eigen::Matrix3d const covA = filter.getCovariance();

    // Change the state, time update, and we should still overwrite the state thanks to the propagation
    TestState badState;
    badState.set<Position<3>>(Eigen::Vector3d(99.0, 99.0, 99.0));
    filter.setState(badState);

    filter.timeUpdate(2.0);
    EXPECT_TRUE(filter.getState().raw().isApprox(stateA.raw(), 1e-12)) << "state not rewound";
    EXPECT_TRUE(filter.getCovariance().isApprox(covA, 1e-12)) << "covariance not rewound";
}

// Time update increases the covariance
TEST(SrukfTimeUpdate, CovarianceUnderTimeUpdate) {
    SRuKFType filter;
    filter.setAlpha(0.5);
    filter.setBeta(2.0);
    Eigen::Matrix3d const P0 = Eigen::Matrix3d::Identity();
    filter.setInitialCovariance(P0);
    filter.setProcessNoise(0.1 * Eigen::Matrix3d::Identity());
    filter.reset();
    filter.reConfigure();

    filter.timeUpdate(0.0);
    EXPECT_TRUE(filter.getCovariance().isApprox(P0, 1e-10)) << "dt=0 should leave covariance unchanged";

    // Propagate 1s and add rocess noise. Covariance trace grows and stays symmetric PSD.
    filter.timeUpdate(1.0);
    Eigen::Matrix3d const P = filter.getCovariance();
    EXPECT_GT(P.trace(), P0.trace()) << "dt>0 should grow covariance";
    EXPECT_TRUE(P.isApprox(P.transpose(), 1e-10)) << "covariance not symmetric";
    EXPECT_TRUE(isPositiveSemiDefinite<3>(P)) << "covariance not PSD";
}

// measurementUpdate tests

// State updates and covariance shrinks
TEST(SrukfMeasurementUpdate, InformativeMeasurementUpdatesStateAndShrinksCovariance) {
    SRuKFType filter;
    filter.setAlpha(0.5);
    filter.setBeta(2.0);
    filter.setInitialCovariance(Eigen::Matrix3d::Identity());
    filter.setProcessNoise(Eigen::Matrix3d::Zero());
    filter.reset();
    filter.reConfigure();
    filter.timeUpdate(0.0);  // populates sigma points around the anchor

    PositionMeasurement m;
    m.observed = Eigen::Vector3d(1.0, 0.0, 0.0);
    m.noiseCov = 0.01 * Eigen::Matrix3d::Identity();

    auto const result = filter.measurementUpdate(m);

    // Pre-fit: observation - model(prior anchor at 0) = (1, 0, 0).
    EXPECT_TRUE(result.preFit.isApprox(Eigen::Vector3d(1.0, 0.0, 0.0), 1e-9))
        << "preFit should equal observation when prior state is zero";

    // Post-fit less than pre-fit
    EXPECT_LT(result.postFit.norm(), result.preFit.norm()) << "informative measurement should reduce residual";

    // Covariance shrunk (initial trace was 3.0).
    Eigen::Matrix3d const P = filter.getCovariance();
    EXPECT_LT(P.trace(), 3.0) << "covariance should shrink";
    EXPECT_TRUE(P.isApprox(P.transpose(), 1e-10)) << "covariance not symmetric";
    EXPECT_TRUE(isPositiveSemiDefinite<3>(P)) << "covariance not PSD";

    // Last-measurement state moved forward
    EXPECT_TRUE(filter.getStateAtLastMeasurement().raw().isApprox(filter.getState().raw(), 1e-12))
        << "anchor should track the post-update state";
}

// If the measurement noise is very high, the measurement should barely change the state
TEST(SrukfMeasurementUpdate, HighMeasurementNoiseLeavesStateNearlyUnchanged) {
    SRuKFType filter;
    filter.setAlpha(0.5);
    filter.setBeta(2.0);
    filter.setInitialCovariance(Eigen::Matrix3d::Identity());
    filter.setProcessNoise(Eigen::Matrix3d::Zero());
    filter.reset();
    filter.reConfigure();
    filter.timeUpdate(0.0);

    TestState const stateBefore = filter.getState();

    PositionMeasurement m;
    m.observed = Eigen::Vector3d(1.0, 0.0, 0.0);
    m.noiseCov = 1e8 * Eigen::Matrix3d::Identity();  // R ≫ P → Kalman gain ≈ 0

    auto const result = filter.measurementUpdate(m);

    EXPECT_LT((filter.getState().raw() - stateBefore.raw()).norm(), 1e-5)
        << "state should barely move when R much greater than P";
    EXPECT_TRUE(result.postFit.isApprox(result.preFit, 1e-5)) << "postFit ≈ preFit when R very large";
}

}  // namespace filtering
