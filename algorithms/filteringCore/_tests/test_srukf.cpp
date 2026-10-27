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

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include <array>
#include <cmath>
#include <limits>

namespace filtering {

// Bound for comparing two float computations of the same quantity: a few operations of roundoff
// with a 3-10x margin (10 * epsilon ~ 1.2e-6).
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

// Nonlinear dynamics and measurement model, so that the sigma points have a nonzero mean offset.
struct QuadraticDynamics {
    TestState operator()(float, TestState const& s) const {
        Eigen::Vector3f const x = s.raw();
        return TestState(Eigen::Vector3f(x(1) * x(2), -x(0) * x(2), 0.5F * x(0) * x(1)));
    }
};
using QuadraticSRuKF = SRuKF<TestState, QuadraticDynamics>;
struct QuadraticMeasurement {
    static constexpr int size = 3;
    Eigen::Vector3f observed = Eigen::Vector3f::Zero();
    Eigen::Matrix3f noiseCov = Eigen::Matrix3f::Identity();
    Eigen::Vector3f observation() const { return observed; }
    Eigen::Vector3f model(TestState const& s) const {
        Eigen::Vector3f const x = s.raw();
        return {x(0) * x(0), x(0) * x(1), x(2) + x(1) * x(1)};
    }
    Eigen::Matrix3f noise() const { return noiseCov; }
    Eigen::Vector3f subtract(Eigen::Vector3f const& a, Eigen::Vector3f const& b) const { return a - b; }
};
static_assert(Measurement<QuadraticMeasurement, TestState>);

// The standard unscented transform in double, with the textbook weights (w^c_0 = w^m_0 + 1 - alpha^2 + beta).
struct UnscentedReference {
    double alpha;
    double beta;
    static constexpr int n = 3;
    double lambda() const { return n * (alpha * alpha - 1.0); }
    double wm(int i) const { return i == 0 ? lambda() / (n + lambda()) : 1.0 / (2.0 * (n + lambda())); }
    double wc(int i) const { return i == 0 ? wm(0) + 1.0 - alpha * alpha + beta : wm(i); }
    // Sigma points of (x, P), in float as the filter draws them.
    std::array<Eigen::Vector3f, 2 * n + 1> sigmaPoints(Eigen::Vector3f const& x, Eigen::Matrix3f const& P) const {
        Eigen::Matrix3f const S = Eigen::LLT<Eigen::Matrix3f>(P).matrixL();
        auto const eta = static_cast<float>(std::sqrt(n + lambda()));
        std::array<Eigen::Vector3f, 2 * n + 1> X;
        X[0] = x;
        for (int i = 0; i < n; ++i) {
            X[1 + i] = x + eta * S.col(i);
            X[1 + n + i] = x - eta * S.col(i);
        }
        return X;
    }
    Eigen::Vector3d mean(std::array<Eigen::Vector3f, 2 * n + 1> const& X) const {
        Eigen::Vector3d m = Eigen::Vector3d::Zero();
        for (int i = 0; i < 2 * n + 1; ++i) m += wm(i) * X[i].cast<double>();
        return m;
    }
    Eigen::Matrix3d crossCovariance(std::array<Eigen::Vector3f, 2 * n + 1> const& X,
                                    std::array<Eigen::Vector3f, 2 * n + 1> const& Y) const {
        Eigen::Vector3d const xBar = mean(X);
        Eigen::Vector3d const yBar = mean(Y);
        Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
        for (int i = 0; i < 2 * n + 1; ++i)
            C += wc(i) * (X[i].cast<double>() - xBar) * (Y[i].cast<double>() - yBar).transpose();
        return C;
    }
};
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

// A singular PSD matrix (zero process noise on some states, or a rank-deficient covariance) still factors: a
// zero pivot gives a zero column, and the factor of the remaining states is unaffected.
TEST(SrukfStatic, CholeskyDecompositionFactorsASingularPositiveSemiDefiniteMatrix) {
    Eigen::Matrix<float, 3, 2> B;
    B << 1.0F, 0.5F, -2.0F, 1.0F, 0.25F, 3.0F;
    Eigen::Matrix3f const zeroInTheMiddle = Eigen::Vector3f(4.0F, 0.0F, 9.0F).asDiagonal();
    Eigen::Matrix3f const zeroFirst = Eigen::Vector3f(0.0F, 4.0F, 9.0F).asDiagonal();
    Eigen::Matrix3f const rankTwo = B * B.transpose();

    for (Eigen::Matrix3f const& P : {zeroInTheMiddle, zeroFirst, rankTwo}) {
        Eigen::Matrix3f const L = SRuKFType::choleskyDecomposition<3>(P);
        ASSERT_TRUE(L.allFinite()) << "P =\n" << P;
        EXPECT_LE((L * L.transpose() - P).norm(), kRoundoffTol * P.norm()) << "L·L^T must equal P =\n" << P;
        EXPECT_TRUE(L.isLowerTriangular()) << "P =\n" << P;
        EXPECT_GE(L.diagonal().minCoeff(), 0.0F) << "P =\n" << P;
    }
    EXPECT_TRUE(SRuKFType::choleskyDecomposition<3>(Eigen::Matrix3f::Zero()).isZero()) << "zero process noise";
}

// A zero pivot is judged against that state's own variance, so small variances next to a much larger one are kept.
TEST(SrukfStatic, CholeskyDecompositionKeepsSmallVariancesNextToALargeOne) {
    Eigen::Matrix3f P = Eigen::Vector3f(1e10F, 1e-4F, 2e-4F).asDiagonal();
    P(1, 2) = P(2, 1) = 1e-4F;

    Eigen::Matrix3f const L = SRuKFType::choleskyDecomposition<3>(P);

    Eigen::Matrix2f const smallBlock = (L * L.transpose()).bottomRightCorner<2, 2>();
    EXPECT_TRUE(smallBlock.isApprox(P.bottomRightCorner<2, 2>(), kRoundoffTol)) << smallBlock;
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

// Adding a column to the QR decomposition keeps the small direction of an ill-conditioned factor: P = S0 S0^T has a
// variance of 1e-6 in one direction next to 1e4, a condition number of about 1e13. Forming P in float rounds the small
// direction away (2500 + 1e-6 == 2500); the QR works on the factor's entries and keeps it. This is how the filter adds
// the mean-offset term. The result is compared with a double-precision reference.
TEST(SrukfStatic, QrDecompositionKeepsTheSmallDirectionWhenAddingAColumn) {
    Eigen::Matrix3f S0;
    S0 << 100.0F, 0.0F, 0.0F, 50.0F, 1e-3F, 0.0F, 0.0F, 0.0F, 1.0F;
    Eigen::Vector3f const v(1.0F, 0.5F, 0.3F);
    float const coef = 0.5F;

    Eigen::Matrix<float, 3, 4> A;
    A << S0, sqrtf(coef) * v;
    Eigen::Matrix3f const S1 = SRuKFType::qrDecompositionJustR<3, 4>(A);

    Eigen::Matrix3d const S0d = S0.cast<double>();
    Eigen::Vector3d const vd = v.cast<double>();
    Eigen::Matrix3d const P1 = S0d * S0d.transpose() + static_cast<double>(coef) * vd * vd.transpose();
    Eigen::Matrix3d const expected = Eigen::LLT<Eigen::Matrix3d>(P1).matrixL();
    Eigen::Matrix3d const actual = S1.cast<double>();
    for (int i = 0; i < 3; ++i) {
        // Compare magnitudes: a valid factor may differ from the LLT one by the sign of a column.
        EXPECT_NEAR(std::abs(actual(i, i)), expected(i, i), 1e-4 * expected(i, i)) << "diagonal " << i;
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
// The measurement noise std must give R = std^2 * I positive definite: positive, with a square that neither
// underflows to zero nor overflows.
TEST(SrukfApi, MeasurementNoiseStdIsValidOnlyForAPositiveFiniteVariance) {
    EXPECT_TRUE(SRuKFType::measurementNoiseStdIsValid(1E-9F));
    EXPECT_TRUE(SRuKFType::measurementNoiseStdIsValid(1.0F));
    EXPECT_FALSE(SRuKFType::measurementNoiseStdIsValid(0.0F));
    EXPECT_FALSE(SRuKFType::measurementNoiseStdIsValid(-1.0F));
    EXPECT_FALSE(SRuKFType::measurementNoiseStdIsValid(1E-30F)) << "the variance underflows to zero";
    EXPECT_FALSE(SRuKFType::measurementNoiseStdIsValid(1E30F)) << "the variance overflows";
    EXPECT_FALSE(SRuKFType::measurementNoiseStdIsValid(std::numeric_limits<float>::infinity()));
    EXPECT_FALSE(SRuKFType::measurementNoiseStdIsValid(std::numeric_limits<float>::quiet_NaN()));
}

TEST(SrukfApi, ValidityChecks) {
    // alphaAndBetaAreValid: true iff alpha in (0, 1] and beta in [alpha^2, 2].
    EXPECT_TRUE(SRuKFType::alphaAndBetaAreValid(0.02F, 2.0F)) << "the flight tuning";
    EXPECT_TRUE(SRuKFType::alphaAndBetaAreValid(1.0F, 1.0F)) << "alpha = 1 and beta = alpha^2 are inclusive bounds";
    EXPECT_TRUE(SRuKFType::alphaAndBetaAreValid(0.5F, 0.25F)) << "beta = alpha^2 is the inclusive lower bound";
    EXPECT_FALSE(SRuKFType::alphaAndBetaAreValid(0.0F, 2.0F)) << "alpha = 0 excluded";
    EXPECT_FALSE(SRuKFType::alphaAndBetaAreValid(-0.1F, 2.0F)) << "alpha < 0";
    EXPECT_FALSE(SRuKFType::alphaAndBetaAreValid(1.5F, 2.0F)) << "alpha > 1";
    EXPECT_FALSE(SRuKFType::alphaAndBetaAreValid(0.5F, 2.5F)) << "beta > 2";
    EXPECT_FALSE(SRuKFType::alphaAndBetaAreValid(1.0F, 0.5F)) << "beta < alpha^2";
    EXPECT_FALSE(SRuKFType::alphaAndBetaAreValid(0.02F, 0.0F)) << "beta = 0 is below alpha^2 for any alpha";
    EXPECT_FALSE(SRuKFType::alphaAndBetaAreValid(std::numeric_limits<float>::quiet_NaN(), 2.0F)) << "NaN alpha";
    EXPECT_FALSE(SRuKFType::alphaAndBetaAreValid(0.5F, std::numeric_limits<float>::quiet_NaN())) << "NaN beta";
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
    EXPECT_TRUE(isSymmetric<3>(P)) << "covariance not symmetric";
    EXPECT_TRUE(isPositiveSemiDefinite<3>(P)) << "covariance not PSD";
}

// With nonlinear dynamics the propagated sigma points have a nonzero mean offset. The covariance after the time update
// matches the standard unscented transform in double, mean-offset term included; leaving that term out (beta = alpha^2
// in the reference) gives a covariance off by far more than the tolerance.
TEST(SrukfTimeUpdate, MatchesTheStandardUnscentedTransform) {
    float const alpha = 0.5F;
    float const beta = 2.0F;
    Eigen::Vector3f const x0(1.0F, 0.5F, -0.3F);
    Eigen::Matrix3f P0;
    P0 << 0.25F, 0.05F, 0.0F, 0.05F, 0.36F, -0.02F, 0.0F, -0.02F, 0.16F;
    Eigen::Matrix3f const Q = 1e-3F * Eigen::Matrix3f::Identity();
    float const dt = 1.0F;

    QuadraticSRuKF filter;
    filter.setAlpha(alpha);
    filter.setBeta(beta);
    filter.setInitialState(TestState(x0));
    filter.setInitialCovariance(P0);
    filter.setProcessNoise(Q);
    filter.reInitialize();
    filter.configure();
    ASSERT_TRUE(filter.timeUpdate(dt));

    UnscentedReference const reference{alpha, beta};
    auto X = reference.sigmaPoints(x0, P0);
    for (auto& Xi : X) {
        Xi = filtering::propagate(QuadraticDynamics{}, TestState(Xi), {0.0F, dt}).raw();
    }
    Eigen::Matrix3d const expected = reference.crossCovariance(X, X) + Q.cast<double>();
    Eigen::Matrix3d const actual = filter.getCovariance().cast<double>();
    EXPECT_LE((actual - expected).norm(), 1e-5 * expected.norm()) << "actual\n" << actual << "\nexpected\n" << expected;

    Eigen::Matrix3d const withoutMeanOffsetTerm =
        UnscentedReference{alpha, alpha * alpha}.crossCovariance(X, X) + Q.cast<double>();
    EXPECT_GT((withoutMeanOffsetTerm - expected).norm(), 100.0 * 1e-5 * expected.norm())
        << "the mean-offset term is too small for this test to see it";
}

// At beta == alpha^2 the mean-offset weight is zero, but beta - alpha * alpha can round (or be fused into one
// multiply-add) to -1 ulp: the weight is clamped, so the time update stays valid instead of taking sqrt(-1 ulp).
TEST(SrukfTimeUpdate, BetaEqualToAlphaSquaredKeepsAValidUpdate) {
    for (float const alpha : {0.357269615F, 0.1F, 0.3F, 0.7F, 1.0F}) {
        QuadraticSRuKF filter;
        filter.setAlpha(alpha);
        filter.setBeta(alpha * alpha);
        filter.setInitialState(TestState(Eigen::Vector3f(1.0F, 0.5F, -0.3F)));
        filter.setInitialCovariance(0.01F * Eigen::Matrix3f::Identity());
        filter.setProcessNoise(1e-4F * Eigen::Matrix3f::Identity());
        filter.reInitialize();
        filter.configure();
        EXPECT_TRUE(filter.timeUpdate(0.5F)) << "alpha " << alpha;
        EXPECT_TRUE(filter.getCovariance().allFinite()) << "alpha " << alpha;
    }
}

// A nonlinear measurement model gives a nonzero predicted-measurement offset, which enters S_y and the posterior. The
// state and covariance after the update match the standard unscented update in double.
TEST(SrukfMeasurementUpdate, MatchesTheStandardUnscentedUpdate) {
    float const alpha = 0.5F;
    float const beta = 2.0F;
    Eigen::Vector3f const x0(1.0F, 0.5F, -0.3F);
    Eigen::Matrix3f P0;
    P0 << 0.04F, 0.01F, 0.0F, 0.01F, 0.09F, -0.005F, 0.0F, -0.005F, 0.01F;

    QuadraticSRuKF filter;
    filter.setAlpha(alpha);
    filter.setBeta(beta);
    filter.setInitialState(TestState(x0));
    filter.setInitialCovariance(P0);
    filter.setProcessNoise(Eigen::Matrix3f::Zero());
    filter.reInitialize();
    filter.configure();
    ASSERT_TRUE(filter.timeUpdate(0.0F));

    QuadraticMeasurement m;
    m.observed = Eigen::Vector3f(1.1F, 0.6F, 0.0F);
    m.noiseCov = Eigen::Vector3f(1e-2F, 2e-2F, 5e-3F).asDiagonal();
    ASSERT_TRUE(filter.measurementUpdate(m).has_value());

    UnscentedReference const reference{alpha, beta};
    auto const X = reference.sigmaPoints(x0, P0);
    std::array<Eigen::Vector3f, 7> Y;
    for (int i = 0; i < 7; ++i) {
        Y[i] = m.model(TestState(X[i]));
    }
    Eigen::Matrix3d const pYY = reference.crossCovariance(Y, Y) + m.noiseCov.cast<double>();
    Eigen::Matrix3d const pXY = reference.crossCovariance(X, Y);
    Eigen::Matrix3d const K = pXY * pYY.inverse();
    Eigen::Vector3d const xExpected = reference.mean(X) + K * (m.observed.cast<double>() - reference.mean(Y));
    Eigen::Matrix3d const pExpected = reference.crossCovariance(X, X) - K * pYY * K.transpose();

    Eigen::Vector3d const xActual = filter.getState().raw().cast<double>();
    Eigen::Matrix3d const pActual = filter.getCovariance().cast<double>();
    EXPECT_LE((xActual - xExpected).norm(), 1e-5 * xExpected.norm())
        << xActual.transpose() << " vs " << xExpected.transpose();
    EXPECT_LE((pActual - pExpected).norm(), 1e-4 * pExpected.norm()) << "actual\n"
                                                                     << pActual << "\nexpected\n"
                                                                     << pExpected;
}

// Zero process noise on a state in the middle adds Q exactly: with zero dynamics the covariance after a time update
// is P0 + Q. A factor of Q that stopped at the zero pivot would add 0.3^2 instead of 0.3 to the last state.
TEST(SrukfTimeUpdate, PartlyZeroProcessNoiseAddsExactlyQ) {
    SRuKFType filter;
    filter.setAlpha(0.5);
    filter.setBeta(2.0);
    Eigen::Matrix3f const P0 = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f const Q = Eigen::Vector3f(0.1F, 0.0F, 0.3F).asDiagonal();
    filter.setInitialCovariance(P0);
    filter.setProcessNoise(Q);
    filter.reInitialize();
    filter.configure();

    ASSERT_TRUE(filter.timeUpdate(1.0));

    Eigen::Matrix3f const P = filter.getCovariance();
    EXPECT_TRUE(P.isApprox(P0 + Q, kRoundoffTol)) << P;
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
    EXPECT_TRUE(isSymmetric<3>(P)) << "covariance not symmetric";
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

// A measurement much more precise than the prior (R = 1e-10, P = 1) almost fully resolves the state. The posterior
// variance is P R / (P + R) ~ 1e-10. Removing K Pyy K^T from the prior subtracts two terms equal to 1 within 1e-10,
// which float cannot resolve; the Joseph form builds the posterior as a sum of squares and keeps it.
TEST(SrukfMeasurementUpdate, PreciseMeasurementKeepsAPositivePosteriorCovariance) {
    SRuKFType filter;
    filter.setAlpha(0.5);
    filter.setBeta(2.0);
    filter.setInitialCovariance(Eigen::Matrix3f::Identity());
    filter.setProcessNoise(Eigen::Matrix3f::Zero());
    filter.reInitialize();
    filter.configure();
    filter.timeUpdate(0.0);

    PositionMeasurement m;
    m.observed = Eigen::Vector3f(1.0, 0.0, 0.0);
    m.noiseCov = 1e-10F * Eigen::Matrix3f::Identity();

    ASSERT_TRUE(filter.measurementUpdate(m).has_value()) << "a precise measurement update should be valid";
    Eigen::Matrix3f const P = filter.getCovariance();
    float const expectedVariance = 1e-10F / (1.0F + 1e-10F);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(P(i, i), expectedVariance, 1e-3F * expectedVariance) << "posterior variance " << i;
    }
    EXPECT_TRUE(isSymmetric<3>(P)) << "covariance not symmetric";
    EXPECT_TRUE(isPositiveSemiDefinite<3>(P)) << "covariance not PSD";
}

// The process noise added by a propagating time update is in the prior factor but not in the propagated sigma
// points, so the gain does not see it and the Joseph form adds it back unchanged (sqrtNoiseOutsideSigmaPoints).
// With a precise measurement the sigma-point part is resolved to ~R, and the posterior is ~Q.
TEST(SrukfMeasurementUpdate, ProcessNoiseOutsideTheSigmaPointsCarriesThroughTheUpdate) {
    SRuKFType filter;
    filter.setAlpha(0.5);
    filter.setBeta(2.0);
    filter.setInitialCovariance(Eigen::Matrix3f::Identity());
    filter.setProcessNoise(0.1F * Eigen::Matrix3f::Identity());
    filter.reInitialize();
    filter.configure();
    ASSERT_TRUE(filter.timeUpdate(1.0));

    PositionMeasurement m;
    m.observed = Eigen::Vector3f(1.0, 0.0, 0.0);
    m.noiseCov = 1e-10F * Eigen::Matrix3f::Identity();

    ASSERT_TRUE(filter.measurementUpdate(m).has_value()) << "a precise measurement update should be valid";
    Eigen::Matrix3f const P = filter.getCovariance();
    EXPECT_TRUE(P.isApprox(0.1F * Eigen::Matrix3f::Identity(), 1e-4F)) << "posterior should be the process noise:\n"
                                                                       << P;
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

// A measurement noise that is not positive definite (here zero noise on one axis) is a bad update: no residuals,
// and the estimate is left as it was for the caller to clear.
TEST(SrukfMeasurementUpdate, SingularMeasurementNoiseReturnsNoValue) {
    SRuKFType filter;
    filter.setAlpha(0.5);
    filter.setBeta(2.0);
    filter.setInitialCovariance(Eigen::Matrix3f::Identity());
    filter.setProcessNoise(Eigen::Matrix3f::Zero());
    filter.reInitialize();
    filter.configure();
    filter.timeUpdate(0.0);

    PositionMeasurement m;
    m.observed = Eigen::Vector3f(0.1F, -0.2F, 0.3F);
    m.noiseCov = Eigen::Vector3f(0.01F, 0.0F, 0.01F).asDiagonal();

    EXPECT_FALSE(filter.measurementUpdate(m).has_value());
    EXPECT_TRUE(filter.getStateAtLastMeasurement().raw().isZero()) << "the last-measurement state is unchanged";
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
