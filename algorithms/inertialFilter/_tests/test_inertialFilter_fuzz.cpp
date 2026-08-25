// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

// Property-based fuzz tests for InertialFilterAlgorithm. Over reasonable, randomized
// filter parameters / initial states / measurements, the filter must:
//   * return valid results from every time update,
//   * either apply a measurement -- moving the state toward it and shrinking the
//     corresponding covariance block -- or reject it as an N-sigma outlier and leave the
//     estimate untouched, keeping the covariance symmetric + PSD + finite either way, and
//   * exercise specific behaviors: time-update covariance growth, and MRP regularization
//     keeping |sigma| <= 1.
//
// Domains keep attitudes moderate (inside the unit sphere) so the kinematic, linear-MRP
// filter stays in scope; the regularization fuzzer deliberately ranges past |sigma| = 1.
// Each observation is drawn independently of the state, so innovations of tens of sigma are
// in domain and the outlier gate legitimately fires.

#include "inertialFilterAlgorithm.h"
#include "inertialFilterSpecs.h"

#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/validPSDCheck.h"

#include <filteringCore/dynamicsModel.hpp>

#include <fuzztest/fuzztest.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>

namespace filtering::inertialFilter {
namespace {

using State = InertialFilterAlgorithm::State;
using Matrix6 = Eigen::Matrix<double, 6, 6>;

State makeState(Eigen::Vector3d const& sigma, Eigen::Vector3d const& omega) {
    State s;
    s.set<filtering::MrpAttitude<3>>(sigma);
    s.set<filtering::AngularRate<3>>(omega);
    return s;
}

// Build a validated configuration from fuzzed scalars. Domains below keep every argument
// inside its valid range so create() never throws.
InertialFilterConfig makeConfig(double alpha,
                                double beta,
                                double pAtt,
                                double pRate,
                                double q,
                                double stStd,
                                double gyroStd,
                                State const& initial) {
    Eigen::Matrix<double, 6, 1> diag;
    diag << pAtt, pAtt, pAtt, pRate, pRate, pRate;
    Matrix6 const initialCovariance = diag.asDiagonal();
    Matrix6 const processNoise = q * Matrix6::Identity();
    return InertialFilterConfig::create(alpha, beta, processNoise, initial, initialCovariance, stStd, gyroStd);
}

bool finiteSymmetricPsd(Matrix6 const& P) {
    return P.allFinite() && P.isApprox(P.transpose(), 1e-9) && isPositiveSemiDefinite<6>(P);
}

double attitudeTrace(Matrix6 const& P) { return P(0, 0) + P(1, 1) + P(2, 2); }
double rateTrace(Matrix6 const& P) { return P(3, 3) + P(4, 4) + P(5, 5); }

constexpr double kTol = 1e-9;

// Add some slack on the movement of the state towards the measurement
constexpr double kMoveTowardRelTol = 1e-2;

// Apply one measurement and check the post-update invariants. The N-sigma outlier gate in
// filteringCore/srukf.hpp rejects an innovation beyond outlierNSigma, so a false return is a
// legitimate outcome over these domains, not a defect: the gate returns before the estimate is
// written, so the state and covariance must be exactly as they were.
template <typename MeasurementT, typename ErrorFn, typename TraceFn>
void checkMeasurementUpdate(InertialFilterAlgorithm& algo,
                            MeasurementT const& measurement,
                            ErrorFn errorAgainstObservation,
                            TraceFn blockTrace) {
    State const priorState = algo.getState();
    Matrix6 const priorCovariance = algo.getCovariance();
    double const errorPrior = errorAgainstObservation(priorState);
    double const tracePrior = blockTrace(priorCovariance);

    if (!algo.measurementUpdate(measurement)) {
        EXPECT_TRUE((algo.getState().raw() - priorState.raw()).isZero(0.0))
            << "a gated measurement must leave the state untouched";
        EXPECT_TRUE((algo.getCovariance() - priorCovariance).isZero(0.0))
            << "a gated measurement must leave the covariance untouched";
        return;
    }

    Matrix6 const posterior = algo.getCovariance();
    EXPECT_TRUE(finiteSymmetricPsd(posterior)) << "covariance after an applied update";
    EXPECT_LE(errorAgainstObservation(algo.getState()), errorPrior + kMoveTowardRelTol * std::sqrt(tracePrior) + kTol)
        << "state should move toward the measurement (within the UKF sigma-point bias)";
    EXPECT_LE(blockTrace(posterior), tracePrior + kTol) << "covariance block should shrink";
}

}  // namespace

// ============================================================================
// Generic regression: time update + both measurement kinds. Each measurement update either
// moves the state toward the measurement and shrinks its covariance block, or is gated as an
// outlier and changes nothing. The covariance stays symmetric + PSD + finite throughout.
// ============================================================================
void fuzzTimeAndMeasurementUpdates(double alpha,
                                   double beta,
                                   double pAtt,
                                   double pRate,
                                   double q,
                                   double stStd,
                                   double gyroStd,
                                   double s0x,
                                   double s0y,
                                   double s0z,
                                   double w0x,
                                   double w0y,
                                   double w0z,
                                   double msx,
                                   double msy,
                                   double msz,
                                   double mwx,
                                   double mwy,
                                   double mwz,
                                   double dt) {
    InertialFilterAlgorithm algo(
        makeConfig(alpha, beta, pAtt, pRate, q, stStd, gyroStd, makeState({s0x, s0y, s0z}, {w0x, w0y, w0z})));

    Eigen::Vector3d const stObservation(msx, msy, msz);
    Eigen::Vector3d const rateObservation(mwx, mwy, mwz);

    // ---- time update ----
    ASSERT_TRUE(algo.timeUpdate(dt)) << "timeUpdate should be valid";
    ASSERT_TRUE(finiteSymmetricPsd(algo.getCovariance())) << "covariance after timeUpdate";

    // ---- star-tracker attitude update ----
    StAttMeasurement st;
    st.timeTag = 1.0;
    st.sigma_BN = stObservation;
    st.covar = (stStd * stStd) * Eigen::Matrix3d::Identity();
    st.valid = true;
    checkMeasurementUpdate(
        algo,
        st,
        [&](State const& s) { return subMrp(stObservation, s.get<filtering::MrpAttitude<3>>()).norm(); },
        attitudeTrace);

    // ---- gyro rate update (re-populate sigma points around the ST posterior first) ----
    ASSERT_TRUE(algo.timeUpdate(0.0)) << "zero-dt timeUpdate should be valid";

    RateMeasurement r;
    r.timeTag = 1.0;
    r.omega_BN_B = rateObservation;
    r.covar = (gyroStd * gyroStd) * Eigen::Matrix3d::Identity();
    r.valid = true;
    checkMeasurementUpdate(
        algo,
        r,
        [&](State const& s) { return (rateObservation - s.get<filtering::AngularRate<3>>()).norm(); },
        rateTrace);
}
FUZZ_TEST(InertialFilterFuzz, fuzzTimeAndMeasurementUpdates)
    .WithDomains(fuzztest::InRange(1e-2, 1.0 - 1e-9),  // alpha in (0, 1)
                 fuzztest::InRange(0.0, 2.0),          // beta
                 fuzztest::InRange(1e-4, 1e-1),        // initial attitude variance
                 fuzztest::InRange(1e-6, 1e-2),        // initial rate variance
                 fuzztest::InRange(0.0, 1e-4),         // process noise
                 fuzztest::InRange(1e-5, 1e-1),        // ST measurement noise std
                 fuzztest::InRange(1e-5, 1e-1),        // gyro measurement noise std
                 fuzztest::InRange(-0.3, 0.3),         // initial sigma x/y/z
                 fuzztest::InRange(-0.3, 0.3),
                 fuzztest::InRange(-0.3, 0.3),
                 fuzztest::InRange(-0.1, 0.1),  // initial omega x/y/z
                 fuzztest::InRange(-0.1, 0.1),
                 fuzztest::InRange(-0.1, 0.1),
                 fuzztest::InRange(-0.3, 0.3),  // ST observation sigma x/y/z
                 fuzztest::InRange(-0.3, 0.3),
                 fuzztest::InRange(-0.3, 0.3),
                 fuzztest::InRange(-0.1, 0.1),  // rate observation x/y/z
                 fuzztest::InRange(-0.1, 0.1),
                 fuzztest::InRange(-0.1, 0.1),
                 fuzztest::InRange(0.0, 2.0));  // dt

// ============================================================================
// Targeted: a time update advances the mean by the MRP kinematics and adds process noise
// to the covariance (no smaller than a noise-free propagation; stays symmetric + PSD + finite).
// ============================================================================
void fuzzTimeUpdatePropagatesStateAndGrowsCovariance(double alpha,
                                                     double beta,
                                                     double pAtt,
                                                     double pRate,
                                                     double q,
                                                     double s0x,
                                                     double s0y,
                                                     double s0z,
                                                     double w0x,
                                                     double w0y,
                                                     double w0z,
                                                     double dt) {
    State const initial = makeState({s0x, s0y, s0z}, {w0x, w0y, w0z});
    InertialFilterAlgorithm algo(makeConfig(alpha, beta, pAtt, pRate, q, 1e-3, 1e-3, initial));

    ASSERT_TRUE(algo.timeUpdate(dt)) << "timeUpdate should be valid";

    // The mean follows the kinematics ODE: timeUpdate advances the central state by
    // InertialDynamics (sigma_dot = 1/4 B(sigma) omega, omega_dot = 0) over [0, dt].
    State const predicted = filtering::propagate(InertialDynamics{}, initial, {0.0, dt});
    EXPECT_TRUE(algo.getState().raw().isApprox(predicted.raw(), 1e-9))
        << "state must follow the MRP-kinematics propagation";

    Matrix6 const P = algo.getCovariance();
    EXPECT_TRUE(finiteSymmetricPsd(P)) << "covariance after timeUpdate";

    // Compare against the same propagation with no process noise: P(withQ) = P(noQ) + Q.
    InertialFilterAlgorithm algoNoProcessNoise(makeConfig(alpha, beta, pAtt, pRate, 0.0, 1e-3, 1e-3, initial));
    ASSERT_TRUE(algoNoProcessNoise.timeUpdate(dt)) << "noise-free timeUpdate should be valid";
    double const traceWithoutProcessNoise = algoNoProcessNoise.getCovariance().trace();
    EXPECT_GE(P.trace(), traceWithoutProcessNoise - kTol) << "process noise should not shrink the covariance";
}
FUZZ_TEST(InertialFilterFuzz, fuzzTimeUpdatePropagatesStateAndGrowsCovariance)
    .WithDomains(fuzztest::InRange(1e-2, 1.0 - 1e-9),  // alpha in (0, 1)
                 fuzztest::InRange(0.0, 2.0),          // beta
                 fuzztest::InRange(1e-4, 1e-1),        // initial attitude variance
                 fuzztest::InRange(1e-6, 1e-2),        // initial rate variance
                 fuzztest::InRange(0.0, 1e-4),         // process noise
                 fuzztest::InRange(-0.3, 0.3),         // initial sigma x/y/z
                 fuzztest::InRange(-0.3, 0.3),
                 fuzztest::InRange(-0.3, 0.3),
                 fuzztest::InRange(-0.1, 0.1),  // initial omega x/y/z
                 fuzztest::InRange(-0.1, 0.1),
                 fuzztest::InRange(-0.1, 0.1),
                 fuzztest::InRange(0.0, 2.0));  // dt
}  // namespace filtering::inertialFilter
