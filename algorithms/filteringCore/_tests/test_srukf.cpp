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

#include <utilities/fsw/validPSDCheck.h>
#include <filteringCore/srukf.hpp>
#include <filteringCore/state.hpp>
#include <utilities/fsw/rigidBodyKinematics.hpp>

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <limits>

namespace filtering {

// Bound for comparing two float computations of the same quantity: a few operations of roundoff
// with the 3-10x margin of docs/PRECISION_GUIDELINES.md section 7 (10 * epsilon ~ 1.2e-6).
inline constexpr float kRoundoffTol = 10.0F * std::numeric_limits<float>::epsilon();

namespace {

// Make a filter with zero dynamics
using TestState = StateVector<Position<3>>;
struct ZeroDynamics {
    TestState operator()(float, TestState const& s) const { return s.scale(0.0); }
};
using SRuKFType = SRuKF<TestState, ZeroDynamics>;

// Position measurement used by the measurementUpdate tests below.
struct PositionMeasurement {
    static constexpr int size = 3;
    Eigen::Vector3f observed = Eigen::Vector3f::Zero();
    Eigen::Matrix3f noiseCov = Eigen::Matrix3f::Identity();
    Eigen::Vector3f observation() const { return observed; }
    Eigen::Vector3f model(TestState const& s) const { return s.get<Position<3>>(); }
    Eigen::Matrix3f noise() const { return noiseCov; }
    Eigen::Vector3f subtract(Eigen::Vector3f const& a, Eigen::Vector3f const& b) const { return a - b; }
};
static_assert(Measurement<PositionMeasurement, TestState>);

// MRP-attitude fixture. Unlike PositionMeasurement, subtract() composes rotations rather than
// differencing coordinates, and the two stop agreeing as |sigma| grows.
using MrpState = StateVector<MrpAttitude<3>>;
struct MrpZeroDynamics {
    MrpState operator()(float, MrpState const& s) const { return s.scale(0.0); }
};
using MrpSRuKF = SRuKF<MrpState, MrpZeroDynamics>;

struct MrpMeasurement {
    static constexpr int size = 3;
    Eigen::Vector3f observed = Eigen::Vector3f::Zero();
    Eigen::Matrix3f noiseCov = Eigen::Matrix3f::Identity();
    Eigen::Vector3f observation() const { return observed; }
    Eigen::Vector3f model(MrpState const& s) const { return s.get<MrpAttitude<3>>(); }
    Eigen::Matrix3f noise() const { return noiseCov; }
    Eigen::Vector3f subtract(Eigen::Vector3f const& a, Eigen::Vector3f const& b) const { return subMrp(a, b); }
};
static_assert(Measurement<MrpMeasurement, MrpState>);

}  // namespace

// Test all of the static functions
TEST(SrukfStatic, ForwardSubstitutionSolvesLowerTriangular) {
    Eigen::Matrix3f L;
    L << 2.0, 0.0, 0.0, 1.0, 3.0, 0.0, 0.5, 1.0, 4.0;
    Eigen::Vector3f const xTruth(1.0, 2.0, 3.0);
    Eigen::Matrix<float, 3, 1> const b = L * xTruth;

    Eigen::Matrix<float, 3, 1> const x = SRuKFType::forwardSubstitution<3, 1>(L, b);

    EXPECT_TRUE(x.col(0).isApprox(xTruth, kRoundoffTol));
}

TEST(SrukfStatic, BackSubstitutionSolvesUpperTriangular) {
    Eigen::Matrix3f U;
    U << 2.0, 1.0, 0.5, 0.0, 3.0, 1.0, 0.0, 0.0, 4.0;
    Eigen::Vector3f const xTruth(1.0, 2.0, 3.0);
    Eigen::Matrix<float, 3, 1> const b = U * xTruth;

    Eigen::Matrix<float, 3, 1> const x = SRuKFType::backSubstitution<3, 1>(U, b);

    EXPECT_TRUE(x.col(0).isApprox(xTruth, kRoundoffTol));
}

TEST(SrukfStatic, CholeskyDecompositionReconstructsP) {
    Eigen::Matrix3f P;
    P << 4.0, 2.0, 0.5, 2.0, 5.0, 1.0, 0.5, 1.0, 6.0;

    Eigen::Matrix3f const L = SRuKFType::choleskyDecomposition<3>(P);

    EXPECT_TRUE((L * L.transpose()).isApprox(P, kRoundoffTol)) << "L·L^T must equal P";
    for (int i = 0; i < 3; ++i) {
        for (int j = i + 1; j < 3; ++j) {
            EXPECT_NEAR(L(i, j), 0.0, 1e-12) << "L must be lower-triangular at (" << i << "," << j << ")";
        }
    }
}

TEST(SrukfStatic, QrDecompositionJustRReturnsLowerTriangularSatisfyingNormalEquations) {
    Eigen::Matrix<float, 3, 9> A;
    A << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 2.0, -1.0, 0.0, 1.0,
        2.0, 3.0, 4.0, 5.0, 6.0;

    Eigen::Matrix3f const R = SRuKFType::qrDecompositionJustR<3, 9>(A);

    for (int i = 0; i < 3; ++i) {
        for (int j = i + 1; j < 3; ++j) {
            EXPECT_NEAR(R(i, j), 0.0, 1e-10) << "R must be lower-triangular at (" << i << "," << j << ")";
        }
    }
    EXPECT_TRUE((R * R.transpose()).isApprox(A * A.transpose(), kRoundoffTol))
        << "R·R^T must equal A·A^T (normal equations preserved)";
}

TEST(SrukfStatic, CholeskyUpDownDateMatchesExplicitUpdate) {
    Eigen::Matrix3f P0;
    P0 << 4.0, 1.0, 0.0, 1.0, 3.0, 0.5, 0.0, 0.5, 2.0;
    Eigen::Matrix3f const S0 = SRuKFType::choleskyDecomposition<3>(P0);
    Eigen::Vector3f const v(0.1, -0.2, 0.3);

    // Up-date with +coef: chol(P + |coef|·v·v^T).
    {
        float const coef = 0.5;
        Eigen::Matrix3f const S1 = SRuKFType::choleskyUpDownDate<3>(S0, v, coef);
        Eigen::Matrix3f const P1 = P0 + coef * v * v.transpose();
        EXPECT_TRUE((S1 * S1.transpose()).isApprox(P1, kRoundoffTol)) << "up-date";
    }
    // Down-date with -coef: chol(P - |coef|·v·v^T).
    {
        float const coef = -0.5;
        Eigen::Matrix3f const S1 = SRuKFType::choleskyUpDownDate<3>(S0, v, coef);
        Eigen::Matrix3f const P1 = P0 - 0.5 * v * v.transpose();
        EXPECT_TRUE((S1 * S1.transpose()).isApprox(P1, kRoundoffTol)) << "down-date";
    }
}

// Test setters/getters and reset

// Setters/Getters
TEST(SrukfApi, SetGetRoundtripsForAlphaAndBeta) {
    SRuKFType filter;
    filter.setAlpha(0.5);
    filter.setBeta(2.0);
    EXPECT_FLOAT_EQ(filter.getAlpha(), 0.5);
    EXPECT_FLOAT_EQ(filter.getBeta(), 2.0);
}

// Valid inputs. The validators are static: they take the term to check and can be called
// without an SRuKF instance (e.g. from a filter's config validation in another file).
TEST(SrukfApi, ValidityChecks) {
    // (1) alphaIsValid: true iff alpha in (0, 1] -- zero excluded, one included.
    EXPECT_TRUE(SRuKFType::alphaIsValid(0.5)) << "alpha=0.5 in range";
    EXPECT_TRUE(SRuKFType::alphaIsValid(1.0)) << "alpha=1 is the inclusive upper bound";
    EXPECT_FALSE(SRuKFType::alphaIsValid(0.0)) << "alpha=0 excluded";
    EXPECT_FALSE(SRuKFType::alphaIsValid(-0.1)) << "alpha<0";
    EXPECT_FALSE(SRuKFType::alphaIsValid(1.5)) << "alpha>1";

    // (2) betaIsValid: true iff beta in [0, 2].
    EXPECT_TRUE(SRuKFType::betaIsValid(1.0)) << "beta=1 in range";
    EXPECT_FALSE(SRuKFType::betaIsValid(-0.5)) << "beta<0";
    EXPECT_FALSE(SRuKFType::betaIsValid(2.5)) << "beta>2";
}

// Reset
TEST(SrukfApi, ResetCopiesInitialStateAndCovarianceToWorkingState) {
    SRuKFType filter;
    filter.setAlpha(0.5);
    filter.setBeta(2.0);

    TestState s0;
    Eigen::Vector<float, 3> const x0(1.0, 2.0, 3.0);
    s0.set<Position<3>>(x0);
    filter.setInitialState(s0);

    Eigen::Matrix3f const P0 = 2.0 * Eigen::Matrix3f::Identity();
    filter.setInitialCovariance(P0);
    filter.setProcessNoise(0.1 * Eigen::Matrix3f::Identity());

    filter.reInitialize();
    filter.configure();

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
    filter.setInitialCovariance(Eigen::Matrix3f::Identity());
    filter.setProcessNoise(0.1 * Eigen::Matrix3f::Identity());
    filter.reInitialize();
    filter.configure();

    // First timeUpdate from anchor produces stateA / covA.
    filter.timeUpdate(2.0);
    TestState const stateA = filter.getState();
    Eigen::Matrix3f const covA = filter.getCovariance();

    // Change the state, time update, and we should still overwrite the state thanks to the propagation
    TestState badState;
    badState.set<Position<3>>(Eigen::Vector3f(99.0, 99.0, 99.0));
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
    Eigen::Matrix3f const P0 = Eigen::Matrix3f::Identity();
    filter.setInitialCovariance(P0);
    filter.setProcessNoise(0.1 * Eigen::Matrix3f::Identity());
    filter.reInitialize();
    filter.configure();

    filter.timeUpdate(0.0);
    EXPECT_TRUE(filter.getCovariance().isApprox(P0, 1e-10)) << "dt=0 should leave covariance unchanged";

    // Propagate 1s and add rocess noise. Covariance trace grows and stays symmetric PSD.
    filter.timeUpdate(1.0);
    Eigen::Matrix3f const P = filter.getCovariance();
    EXPECT_GT(P.trace(), P0.trace()) << "dt>0 should grow covariance";
    EXPECT_TRUE(P.isApprox(P.transpose(), kRoundoffTol)) << "covariance not symmetric";
    EXPECT_TRUE(isPositiveSemiDefinite<3>(P)) << "covariance not PSD";
}

// A covariance that overflows float must fail the time update: the prior and the process noise are finite, but
// their sum, 3e38 + 3e38, is above FLT_MAX (~3.4e38).
TEST(SrukfTimeUpdate, CovarianceOverflowReturnsFalse) {
    SRuKFType filter;
    filter.setAlpha(0.5);
    filter.setBeta(2.0);
    filter.setInitialCovariance(3e38F * Eigen::Matrix3f::Identity());
    filter.setProcessNoise(3e38F * Eigen::Matrix3f::Identity());
    filter.reInitialize();
    filter.configure();

    EXPECT_FALSE(filter.timeUpdate(1.0)) << "a covariance that overflows float should fail the time update";
}

// measurementUpdate tests

// State updates and covariance shrinks
TEST(SrukfMeasurementUpdate, InformativeMeasurementUpdatesStateAndShrinksCovariance) {
    SRuKFType filter;
    filter.setAlpha(0.5);
    filter.setBeta(2.0);
    filter.setInitialCovariance(Eigen::Matrix3f::Identity());
    filter.setProcessNoise(Eigen::Matrix3f::Zero());
    filter.reInitialize();
    filter.configure();
    filter.timeUpdate(0.0);  // populates sigma points around the anchor

    PositionMeasurement m;
    m.observed = Eigen::Vector3f(1.0, 0.0, 0.0);
    m.noiseCov = 0.01 * Eigen::Matrix3f::Identity();

    // A nominal update returns the pre/post-fit residuals.
    auto const result = filter.measurementUpdate(m);
    ASSERT_TRUE(result.has_value()) << "measurement update should be valid";

    // Pre-fit: observation - model(prior anchor at 0) = (1, 0, 0).
    EXPECT_TRUE(result->preFit.isApprox(Eigen::Vector3f(1.0, 0.0, 0.0), kRoundoffTol))
        << "preFit should equal observation when prior state is zero";

    // Post-fit less than pre-fit
    EXPECT_LT(result->postFit.stableNorm(), result->preFit.stableNorm())
        << "informative measurement should reduce residual";

    // Covariance shrunk (initial trace was 3.0).
    Eigen::Matrix3f const P = filter.getCovariance();
    EXPECT_LT(P.trace(), 3.0) << "covariance should shrink";
    EXPECT_TRUE(P.isApprox(P.transpose(), kRoundoffTol)) << "covariance not symmetric";
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
    filter.setInitialCovariance(Eigen::Matrix3f::Identity());
    filter.setProcessNoise(Eigen::Matrix3f::Zero());
    filter.reInitialize();
    filter.configure();
    filter.timeUpdate(0.0);

    TestState const stateBefore = filter.getState();

    PositionMeasurement m;
    m.observed = Eigen::Vector3f(1.0, 0.0, 0.0);
    m.noiseCov = 1e8 * Eigen::Matrix3f::Identity();  // R ≫ P → Kalman gain ≈ 0

    auto const result = filter.measurementUpdate(m);
    ASSERT_TRUE(result.has_value()) << "measurement update should be valid";

    EXPECT_LT((filter.getState().raw() - stateBefore.raw()).stableNorm(), 1e-5)
        << "state should barely move when R much greater than P";
    EXPECT_TRUE(result->postFit.isApprox(result->preFit, 1e-5)) << "postFit ≈ preFit when R very large";
}

// NaNs in the measurement propagate into the state, so measurementUpdate returns no value.
TEST(SrukfMeasurementUpdate, NaNMeasurementReturnsNoValue) {
    SRuKFType filter;
    filter.setAlpha(0.5);
    filter.setBeta(2.0);
    filter.setInitialCovariance(Eigen::Matrix3f::Identity());
    filter.setProcessNoise(Eigen::Matrix3f::Zero());
    filter.reInitialize();
    filter.configure();
    filter.timeUpdate(0.0);

    PositionMeasurement m;
    m.observed = Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN());
    m.noiseCov = 0.01 * Eigen::Matrix3f::Identity();

    EXPECT_FALSE(filter.measurementUpdate(m).has_value())
        << "measurementUpdate should not return a value for a bad (NaN) update";
}

// An MRP measurement must still pull the state onto the observation when |sigma| is large.
// The residuals that build the gain have to use the measurement's own subtract(); differencing
// the MRP coordinates instead mis-scales the correction by roughly (1 + |sigma|^2), which leaves
// the filter unable to converge at large attitudes while remaining perfectly well behaved near
// the origin.
TEST(SrukfMeasurementUpdate, MrpMeasurementConvergesAtLargeAttitude) {
    for (float const magnitude : {0.1, 0.5, 0.85}) {
        Eigen::Vector3f const truth = magnitude * Eigen::Vector3f(1.0, 0.0, 0.0);
        Eigen::Vector3f const start = truth + Eigen::Vector3f(0.0, 0.02, 0.0);

        MrpSRuKF filter;
        filter.setAlpha(0.02);
        filter.setBeta(2.0);
        filter.setInitialState(MrpState(start));
        filter.setInitialCovariance(1e-2 * Eigen::Matrix3f::Identity());
        filter.setProcessNoise(1e-8 * Eigen::Matrix3f::Identity());
        filter.reInitialize();
        filter.configure();

        MrpMeasurement m;
        m.observed = truth;
        m.noiseCov = 1e-8 * Eigen::Matrix3f::Identity();

        for (int k = 0; k < 5; ++k) {
            filter.timeUpdate(0.0);
            ASSERT_TRUE(filter.measurementUpdate(m).has_value()) << "update should be valid at |sigma| = " << magnitude;
        }

        float const residual = subMrp(Eigen::Vector3f(filter.getState().raw()), truth).stableNorm();
        EXPECT_LT(residual, 1e-4) << "filter failed to converge onto the observation at |sigma| = " << magnitude;
    }
}

}  // namespace filtering
