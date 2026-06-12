// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

// Property-based fuzz: measurementUpdate is a Kalman update — for any
// initial state, observation, and (P, R) configuration, the post-update
// state must move toward the observation (or not at all) and the
// covariance must remain symmetric PSD with non-increasing trace.

#include <utilities/validPSDCheck.h>
#include <filteringCore/srukf.hpp>
#include <filteringCore/state.hpp>

#include <fuzztest/fuzztest.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>

namespace filtering {
namespace {

using TestState = StateVector<Position<3>>;
struct ZeroDynamics {
    TestState operator()(double, TestState const& s) const { return s.scale(0.0); }
};
using SRuKFType = SRuKF<TestState, ZeroDynamics>;

struct PositionMeasurement {
    static constexpr int size = 3;
    Eigen::Vector3d observed = Eigen::Vector3d::Zero();
    Eigen::Matrix3d noiseCov = Eigen::Matrix3d::Identity();
    Eigen::Vector3d observation() const { return observed; }
    Eigen::Vector3d model(TestState const& s) const { return s.get<Position<3>>(); }
    Eigen::Matrix3d noise() const { return noiseCov; }
    Eigen::Vector3d subtract(Eigen::Vector3d const& a, Eigen::Vector3d const& b) const { return a - b; }
};

}  // namespace

// Vary only the x-component for state and observation. The Kalman is in the form
// K = P/(P+R), which lives in [0, 1]. We should see the covariance shrink
void fuzzMeasurementUpdateInvariants(double x0, double obs, double pDiag, double rDiag) {
    SRuKFType filter;
    filter.setAlpha(0.5);
    filter.setBeta(2.0);

    TestState s0;
    s0.set<Position<3>>(Eigen::Vector3d(x0, 0.0, 0.0));
    filter.setInitialState(s0);
    filter.setInitialCovariance(pDiag * Eigen::Matrix3d::Identity());
    filter.setProcessNoise(Eigen::Matrix3d::Zero());
    filter.reset();
    filter.timeUpdate(0.0);

    PositionMeasurement m;
    m.observed = Eigen::Vector3d(obs, 0.0, 0.0);
    m.noiseCov = rDiag * Eigen::Matrix3d::Identity();

    double const traceBefore = filter.getCovariance().trace();
    filter.measurementUpdate(m);
    double const newX = filter.getState().get<Position<3>>()(0);

    // State moves toward observation, or not at all.
    EXPECT_GE((obs - x0) * (newX - x0), -1e-9) << "state moved away from observation";
    EXPECT_LE(std::abs(newX - x0), std::abs(obs - x0) + 1e-9) << "state overshot";

    // Covariance stays symmetric PSD.
    Eigen::Matrix3d const P = filter.getCovariance();
    EXPECT_TRUE(P.isApprox(P.transpose(), 1e-9)) << "covariance not symmetric";
    EXPECT_TRUE(isPositiveSemiDefinite<3>(P)) << "covariance not PSD";

    // Covariance does not grow under a measurement update.
    EXPECT_LE(P.trace(), traceBefore + 1e-9) << "trace increased";
}
FUZZ_TEST(SrukfFuzz, fuzzMeasurementUpdateInvariants)
    .WithDomains(fuzztest::InRange(-1e3, 1e3),   // x0
                 fuzztest::InRange(-1e3, 1e3),   // obs
                 fuzztest::InRange(1e-3, 1e3),   // P_diag
                 fuzztest::InRange(1e-3, 1e3));  // R_diag

}  // namespace filtering
