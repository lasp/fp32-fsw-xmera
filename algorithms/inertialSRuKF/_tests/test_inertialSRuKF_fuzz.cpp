// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

// Property-based fuzz tests for InertialSRuKFAlgorithm. Over reasonable, randomized
// filter parameters / initial states / measurements, the filter must:
//   * return valid results from time and measurement updates,
//   * move the state toward each measurement and shrink the corresponding covariance
//     block, keeping the covariance symmetric + PSD + finite (generic regression), and
//   * exercise specific behaviors: time-update covariance growth, and MRP regularization
//     keeping |sigma| <= 1.
//
// Domains keep attitudes moderate (inside the unit sphere) so the kinematic, linear-MRP
// filter stays in scope; the regularization fuzzer deliberately ranges past |sigma| = 1.

#include "inertialSRuKFAlgorithm.h"
#include "inertialSRuKFSpecs.h"

#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/validPSDCheck.h"

#include <fuzztest/fuzztest.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

namespace filtering::inertialSRuKF {
namespace {

using State = InertialSRuKFAlgorithm::State;
using Matrix6 = Eigen::Matrix<double, 6, 6>;

State makeState(Eigen::Vector3d const& sigma, Eigen::Vector3d const& omega) {
    State s;
    s.set<filtering::MrpAttitude<3>>(sigma);
    s.set<filtering::AngularRate<3>>(omega);
    return s;
}

// Build a validated configuration from fuzzed scalars. Domains below keep every argument
// inside its valid range so create() never throws.
InertialSRuKFConfig makeConfig(double alpha, double beta, double pAtt, double pRate, double q, double stStd,
                               double gyroStd, State const& initial) {
    Eigen::Matrix<double, 6, 1> diag;
    diag << pAtt, pAtt, pAtt, pRate, pRate, pRate;
    Matrix6 const initialCovariance = diag.asDiagonal();
    Matrix6 const processNoise = q * Matrix6::Identity();
    return InertialSRuKFConfig::create(alpha, beta, processNoise, initial, initialCovariance, stStd, gyroStd);
}

bool finiteSymmetricPsd(Matrix6 const& P) {
    return P.allFinite() && P.isApprox(P.transpose(), 1e-9) && isPositiveSemiDefinite<6>(P);
}

double attitudeTrace(Matrix6 const& P) { return P(0, 0) + P(1, 1) + P(2, 2); }
double rateTrace(Matrix6 const& P) { return P(3, 3) + P(4, 4) + P(5, 5); }

constexpr double kTol = 1e-9;

}  // namespace

// ============================================================================
// Generic regression: time update + both measurement kinds. Each measurement update is
// valid, moves the state toward the measurement, shrinks its covariance block, and keeps
// the covariance symmetric + PSD + finite.
// ============================================================================
void fuzzTimeAndMeasurementUpdates(double alpha, double beta, double pAtt, double pRate, double q, double stStd,
                                   double gyroStd, double s0x, double s0y, double s0z, double w0x, double w0y,
                                   double w0z, double msx, double msy, double msz, double mwx, double mwy, double mwz,
                                   double dt) {
    InertialSRuKFAlgorithm algo(makeConfig(alpha, beta, pAtt, pRate, q, stStd, gyroStd,
                                           makeState({s0x, s0y, s0z}, {w0x, w0y, w0z})));

    Eigen::Vector3d const stObservation(msx, msy, msz);
    Eigen::Vector3d const rateObservation(mwx, mwy, mwz);

    // ---- time update ----
    ASSERT_TRUE(algo.timeUpdate(dt)) << "timeUpdate should be valid";
    ASSERT_TRUE(finiteSymmetricPsd(algo.getCovariance())) << "covariance after timeUpdate";

    // ---- star-tracker attitude update ----
    Eigen::Vector3d const attPrior = algo.getState().get<filtering::MrpAttitude<3>>();
    double const attErrorPrior = subMrp(stObservation, attPrior).norm();
    double const attTracePrior = attitudeTrace(algo.getCovariance());

    StAttMeasurement st;
    st.timeTag = 1.0;
    st.sigma_BN = stObservation;
    st.covar = (stStd * stStd) * Eigen::Matrix3d::Identity();
    st.valid = true;
    ASSERT_TRUE(algo.measurementUpdate(st)) << "ST measurementUpdate should be valid";

    Matrix6 const afterSt = algo.getCovariance();
    EXPECT_TRUE(finiteSymmetricPsd(afterSt)) << "covariance after ST update";
    EXPECT_LE(subMrp(stObservation, algo.getState().get<filtering::MrpAttitude<3>>()).norm(), attErrorPrior + kTol)
        << "attitude should move toward the ST measurement";
    EXPECT_LE(attitudeTrace(afterSt), attTracePrior + kTol) << "attitude covariance should shrink";

    // ---- gyro rate update (re-populate sigma points around the ST posterior first) ----
    ASSERT_TRUE(algo.timeUpdate(0.0)) << "zero-dt timeUpdate should be valid";
    Eigen::Vector3d const ratePrior = algo.getState().get<filtering::AngularRate<3>>();
    double const rateErrorPrior = (rateObservation - ratePrior).norm();
    double const rateTracePrior = rateTrace(algo.getCovariance());

    RateMeasurement r;
    r.timeTag = 1.0;
    r.omega_BN_B = rateObservation;
    r.covar = (gyroStd * gyroStd) * Eigen::Matrix3d::Identity();
    r.valid = true;
    ASSERT_TRUE(algo.measurementUpdate(r)) << "rate measurementUpdate should be valid";

    Matrix6 const afterRate = algo.getCovariance();
    EXPECT_TRUE(finiteSymmetricPsd(afterRate)) << "covariance after rate update";
    EXPECT_LE((rateObservation - algo.getState().get<filtering::AngularRate<3>>()).norm(), rateErrorPrior + kTol)
        << "rate should move toward the gyro measurement";
    EXPECT_LE(rateTrace(afterRate), rateTracePrior + kTol) << "rate covariance should shrink";
}
FUZZ_TEST(InertialSRuKFFuzz, fuzzTimeAndMeasurementUpdates)
    .WithDomains(fuzztest::InRange(1e-2, 1.0),    // alpha
                 fuzztest::InRange(0.0, 2.0),     // beta
                 fuzztest::InRange(1e-4, 1e-1),   // initial attitude variance
                 fuzztest::InRange(1e-6, 1e-2),   // initial rate variance
                 fuzztest::InRange(0.0, 1e-4),    // process noise
                 fuzztest::InRange(1e-5, 1e-1),   // ST measurement noise std
                 fuzztest::InRange(1e-5, 1e-1),   // gyro measurement noise std
                 fuzztest::InRange(-0.3, 0.3),    // initial sigma x/y/z
                 fuzztest::InRange(-0.3, 0.3),
                 fuzztest::InRange(-0.3, 0.3),
                 fuzztest::InRange(-0.1, 0.1),    // initial omega x/y/z
                 fuzztest::InRange(-0.1, 0.1),
                 fuzztest::InRange(-0.1, 0.1),
                 fuzztest::InRange(-0.3, 0.3),    // ST observation sigma x/y/z
                 fuzztest::InRange(-0.3, 0.3),
                 fuzztest::InRange(-0.3, 0.3),
                 fuzztest::InRange(-0.1, 0.1),    // rate observation x/y/z
                 fuzztest::InRange(-0.1, 0.1),
                 fuzztest::InRange(-0.1, 0.1),
                 fuzztest::InRange(0.0, 2.0));    // dt

// ============================================================================
// Targeted: a time update keeps the covariance symmetric + PSD + finite and does not
// shrink it (process noise can only grow it).
// ============================================================================
void fuzzTimeUpdateGrowsCovariance(double alpha, double beta, double pAtt, double pRate, double q, double s0x,
                                   double s0y, double s0z, double w0x, double w0y, double w0z, double dt) {
    InertialSRuKFAlgorithm algo(
        makeConfig(alpha, beta, pAtt, pRate, q, 1e-3, 1e-3, makeState({s0x, s0y, s0z}, {w0x, w0y, w0z})));

    double const tracePrior = algo.getCovariance().trace();
    ASSERT_TRUE(algo.timeUpdate(dt)) << "timeUpdate should be valid";

    Matrix6 const P = algo.getCovariance();
    EXPECT_TRUE(finiteSymmetricPsd(P)) << "covariance after timeUpdate";
    EXPECT_GE(P.trace(), tracePrior - kTol) << "process noise should not shrink the covariance";
}
FUZZ_TEST(InertialSRuKFFuzz, fuzzTimeUpdateGrowsCovariance)
    .WithDomains(fuzztest::InRange(1e-2, 1.0),    // alpha
                 fuzztest::InRange(0.0, 2.0),     // beta
                 fuzztest::InRange(1e-4, 1e-1),   // initial attitude variance
                 fuzztest::InRange(1e-6, 1e-2),   // initial rate variance
                 fuzztest::InRange(0.0, 1e-4),    // process noise
                 fuzztest::InRange(-0.3, 0.3),    // initial sigma x/y/z
                 fuzztest::InRange(-0.3, 0.3),
                 fuzztest::InRange(-0.3, 0.3),
                 fuzztest::InRange(-0.1, 0.1),    // initial omega x/y/z
                 fuzztest::InRange(-0.1, 0.1),
                 fuzztest::InRange(-0.1, 0.1),
                 fuzztest::InRange(0.0, 2.0));    // dt

// ============================================================================
// Targeted: regularization keeps the MRP attitude inside the unit sphere after update(),
// even when the initial attitude ranges past |sigma| = 1, and the state stays finite.
// ============================================================================
void fuzzRegularizationKeepsMrpBounded(double alpha, double beta, double pAtt, double pRate, double q, double stStd,
                                       double s0x, double s0y, double s0z, double msx, double msy, double msz,
                                       double dt) {
    InertialSRuKFAlgorithm algo(makeConfig(alpha, beta, pAtt, pRate, q, stStd, 1e-3,
                                           makeState({s0x, s0y, s0z}, Eigen::Vector3d::Zero())));

    StAttData st;
    st.timeTag = (dt > 0.0) ? dt : 1.0;
    st.sigma_BN = Eigen::Vector3d(msx, msy, msz);
    algo.update(st.timeTag, st, RateData{});

    EXPECT_TRUE(algo.getState().raw().allFinite()) << "state must stay finite";
    EXPECT_LE(algo.getState().get<filtering::MrpAttitude<3>>().norm(), 1.0 + kTol)
        << "regularization must keep |sigma| <= 1";
}
FUZZ_TEST(InertialSRuKFFuzz, fuzzRegularizationKeepsMrpBounded)
    .WithDomains(fuzztest::InRange(1e-2, 1.0),    // alpha
                 fuzztest::InRange(0.0, 2.0),     // beta
                 fuzztest::InRange(1e-4, 1e-1),   // initial attitude variance
                 fuzztest::InRange(1e-6, 1e-2),   // initial rate variance
                 fuzztest::InRange(0.0, 1e-4),    // process noise
                 fuzztest::InRange(1e-5, 1e-1),   // ST measurement noise std
                 fuzztest::InRange(-1.2, 1.2),    // initial sigma x/y/z (ranges past |sigma|=1)
                 fuzztest::InRange(-1.2, 1.2),
                 fuzztest::InRange(-1.2, 1.2),
                 fuzztest::InRange(-0.3, 0.3),    // ST observation sigma x/y/z
                 fuzztest::InRange(-0.3, 0.3),
                 fuzztest::InRange(-0.3, 0.3),
                 fuzztest::InRange(0.0, 2.0));    // dt

}  // namespace filtering::inertialSRuKF
