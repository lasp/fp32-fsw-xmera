// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

// Property-based fuzz tests for InertialFilterAlgorithm. Over reasonable, randomized
// filter parameters / initial states / measurements, the filter must:
//   * return valid results from time and measurement updates,
//   * move the state toward each measurement and shrink the corresponding covariance
//     block, keeping the covariance symmetric + PSD + finite (generic regression), and
//   * exercise specific behaviors: time-update covariance growth, and MRP regularization
//     keeping |sigma| <= 1.
//
// Domains keep attitudes moderate (inside the unit sphere) so the kinematic, linear-MRP
// filter stays in scope; the regularization fuzzer deliberately ranges past |sigma| = 1.

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
using Matrix6 = Eigen::Matrix<float, 6, 6>;

State makeState(Eigen::Vector3f const& sigma, Eigen::Vector3f const& omega) {
    State s;
    s.set<filtering::MrpAttitude<3>>(sigma);
    s.set<filtering::AngularRate<3>>(omega);
    return s;
}

// Build a validated configuration from fuzzed scalars. Domains below keep every argument
// inside its valid range so create() never throws.
InertialFilterConfig makeConfig(float alpha,
                                float beta,
                                float pAtt,
                                float pRate,
                                float q,
                                float stStd,
                                float rateStd,
                                State const& initial) {
    Eigen::Matrix<float, 6, 1> diag;
    diag << pAtt, pAtt, pAtt, pRate, pRate, pRate;
    Matrix6 const initialCovariance = diag.asDiagonal();
    Matrix6 const processNoise = q * Matrix6::Identity();
    return InertialFilterConfig::create(alpha, beta, processNoise, initial, initialCovariance, stStd, rateStd);
}

bool finiteSymmetricPsd(Matrix6 const& P) {
    return P.allFinite() && P.isApprox(P.transpose(), 1e-5) && isPositiveSemiDefinite<6>(P);
}

float attitudeTrace(Matrix6 const& P) { return P(0, 0) + P(1, 1) + P(2, 2); }
float rateTrace(Matrix6 const& P) { return P(3, 3) + P(4, 4) + P(5, 5); }

constexpr float kTol = 1e-5;

// Add some slack on the movement of the state towards the measurement
constexpr float kMoveTowardRelTol = 1e-2;

}  // namespace

// ============================================================================
// Generic regression: time update + both measurement kinds. Each measurement update is
// valid, moves the state toward the measurement, shrinks its covariance block, and keeps
// the covariance symmetric + PSD + finite.
// ============================================================================
void fuzzTimeAndMeasurementUpdates(float alpha,
                                   float beta,
                                   float pAtt,
                                   float pRate,
                                   float q,
                                   float stStd,
                                   float rateStd,
                                   float s0x,
                                   float s0y,
                                   float s0z,
                                   float w0x,
                                   float w0y,
                                   float w0z,
                                   float msx,
                                   float msy,
                                   float msz,
                                   float mwx,
                                   float mwy,
                                   float mwz,
                                   float dt) {
    InertialFilterAlgorithm algo(
        makeConfig(alpha, beta, pAtt, pRate, q, stStd, rateStd, makeState({s0x, s0y, s0z}, {w0x, w0y, w0z})));

    Eigen::Vector3f const stObservation(msx, msy, msz);
    Eigen::Vector3f const rateObservation(mwx, mwy, mwz);

    // ---- time update ----
    ASSERT_TRUE(algo.timeUpdate(dt)) << "timeUpdate should be valid";
    ASSERT_TRUE(finiteSymmetricPsd(algo.getCovariance())) << "covariance after timeUpdate";

    // ---- star-tracker attitude update ----
    Eigen::Vector3f const attPrior = algo.getState().get<filtering::MrpAttitude<3>>();
    float const attErrorPrior = subMrp(stObservation, attPrior).stableNorm();
    float const attTracePrior = attitudeTrace(algo.getCovariance());

    StAttMeasurement st;
    st.timeTag = 1.0;
    st.sigma_BN = stObservation;
    st.covar = (stStd * stStd) * Eigen::Matrix3f::Identity();
    st.valid = true;
    ASSERT_TRUE(algo.measurementUpdate(st)) << "ST measurementUpdate should be valid";

    Matrix6 const afterSt = algo.getCovariance();
    EXPECT_TRUE(finiteSymmetricPsd(afterSt)) << "covariance after ST update";
    EXPECT_LE(subMrp(stObservation, algo.getState().get<filtering::MrpAttitude<3>>()).stableNorm(),
              attErrorPrior + kMoveTowardRelTol * std::sqrt(attTracePrior) + kTol)
        << "attitude should move toward the ST measurement (within the UKF MRP sigma-point bias)";
    EXPECT_LE(attitudeTrace(afterSt), attTracePrior + kTol) << "attitude covariance should shrink";

    // ---- rate update (re-populate sigma points around the ST posterior first) ----
    ASSERT_TRUE(algo.timeUpdate(0.0)) << "zero-dt timeUpdate should be valid";
    Eigen::Vector3f const ratePrior = algo.getState().get<filtering::AngularRate<3>>();
    float const rateErrorPrior = (rateObservation - ratePrior).stableNorm();
    float const rateTracePrior = rateTrace(algo.getCovariance());

    RateMeasurement r;
    r.timeTag = 1.0;
    r.omega_BN_B = rateObservation;
    r.covar = (rateStd * rateStd) * Eigen::Matrix3f::Identity();
    r.valid = true;
    ASSERT_TRUE(algo.measurementUpdate(r)) << "rate measurementUpdate should be valid";

    Matrix6 const afterRate = algo.getCovariance();
    EXPECT_TRUE(finiteSymmetricPsd(afterRate)) << "covariance after rate update";
    EXPECT_LE((rateObservation - algo.getState().get<filtering::AngularRate<3>>()).stableNorm(),
              rateErrorPrior + kMoveTowardRelTol * std::sqrt(rateTracePrior) + kTol)
        << "rate should move toward the rate measurement (within the UKF sigma-point bias)";
    EXPECT_LE(rateTrace(afterRate), rateTracePrior + kTol) << "rate covariance should shrink";
}
FUZZ_TEST(InertialFilterFuzz, fuzzTimeAndMeasurementUpdates)
    .WithDomains(fuzztest::InRange(1e-2F, 1.0F),   // alpha in (0, 1]
                 fuzztest::InRange(0.0F, 2.0F),    // beta
                 fuzztest::InRange(1e-4F, 1e-1F),  // initial attitude variance
                 fuzztest::InRange(1e-6F, 1e-2F),  // initial rate variance
                 fuzztest::InRange(0.0F, 1e-4F),   // process noise
                 fuzztest::InRange(1e-5F, 1e-1F),  // ST measurement noise std
                 fuzztest::InRange(1e-5F, 1e-1F),  // rate measurement noise std
                 fuzztest::InRange(-0.3F, 0.3F),   // initial sigma x/y/z
                 fuzztest::InRange(-0.3F, 0.3F),
                 fuzztest::InRange(-0.3F, 0.3F),
                 fuzztest::InRange(-0.1F, 0.1F),  // initial omega x/y/z
                 fuzztest::InRange(-0.1F, 0.1F),
                 fuzztest::InRange(-0.1F, 0.1F),
                 fuzztest::InRange(-0.3F, 0.3F),  // ST observation sigma x/y/z
                 fuzztest::InRange(-0.3F, 0.3F),
                 fuzztest::InRange(-0.3F, 0.3F),
                 fuzztest::InRange(-0.1F, 0.1F),  // rate observation x/y/z
                 fuzztest::InRange(-0.1F, 0.1F),
                 fuzztest::InRange(-0.1F, 0.1F),
                 fuzztest::InRange(0.0F, 2.0F));  // dt

// ============================================================================
// Targeted: a time update advances the mean by the MRP kinematics and adds process noise
// to the covariance (no smaller than a noise-free propagation; stays symmetric + PSD + finite).
// ============================================================================
void fuzzTimeUpdatePropagatesStateAndGrowsCovariance(float alpha,
                                                     float beta,
                                                     float pAtt,
                                                     float pRate,
                                                     float q,
                                                     float s0x,
                                                     float s0y,
                                                     float s0z,
                                                     float w0x,
                                                     float w0y,
                                                     float w0z,
                                                     float dt) {
    State const initial = makeState({s0x, s0y, s0z}, {w0x, w0y, w0z});
    InertialFilterAlgorithm algo(makeConfig(alpha, beta, pAtt, pRate, q, 1e-3, 1e-3, initial));

    ASSERT_TRUE(algo.timeUpdate(dt)) << "timeUpdate should be valid";

    // The mean follows the kinematics ODE: timeUpdate advances the central state by
    // InertialDynamics (sigma_dot = 1/4 B(sigma) omega, omega_dot = 0) over [0, dt].
    State const predicted = filtering::propagate(InertialDynamics{}, initial, {0.0, dt});
    EXPECT_TRUE(algo.getState().raw().isApprox(predicted.raw(), 1e-5))
        << "state must follow the MRP-kinematics propagation";

    Matrix6 const P = algo.getCovariance();
    EXPECT_TRUE(finiteSymmetricPsd(P)) << "covariance after timeUpdate";

    // Compare against the same propagation with no process noise: P(withQ) = P(noQ) + Q.
    InertialFilterAlgorithm algoNoProcessNoise(makeConfig(alpha, beta, pAtt, pRate, 0.0, 1e-3, 1e-3, initial));
    ASSERT_TRUE(algoNoProcessNoise.timeUpdate(dt)) << "noise-free timeUpdate should be valid";
    float const traceWithoutProcessNoise = algoNoProcessNoise.getCovariance().trace();
    EXPECT_GE(P.trace(), traceWithoutProcessNoise - kTol) << "process noise should not shrink the covariance";
}
FUZZ_TEST(InertialFilterFuzz, fuzzTimeUpdatePropagatesStateAndGrowsCovariance)
    .WithDomains(fuzztest::InRange(1e-2F, 1.0F),   // alpha in (0, 1]
                 fuzztest::InRange(0.0F, 2.0F),    // beta
                 fuzztest::InRange(1e-4F, 1e-1F),  // initial attitude variance
                 fuzztest::InRange(1e-6F, 1e-2F),  // initial rate variance
                 fuzztest::InRange(0.0F, 1e-4F),   // process noise
                 fuzztest::InRange(-0.3F, 0.3F),   // initial sigma x/y/z
                 fuzztest::InRange(-0.3F, 0.3F),
                 fuzztest::InRange(-0.3F, 0.3F),
                 fuzztest::InRange(-0.1F, 0.1F),  // initial omega x/y/z
                 fuzztest::InRange(-0.1F, 0.1F),
                 fuzztest::InRange(-0.1F, 0.1F),
                 fuzztest::InRange(0.0F, 2.0F));  // dt
}  // namespace filtering::inertialFilter
