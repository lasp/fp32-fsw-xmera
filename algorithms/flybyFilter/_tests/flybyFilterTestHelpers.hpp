#ifndef F32XMERA_FLYBYFILTER_TEST_HELPERS_H
#define F32XMERA_FLYBYFILTER_TEST_HELPERS_H

// Shared builders, a fuzzable config factory, and property-check helpers for the
// FlybyFilter tests. The property helpers each assert one filter invariant that
// must hold for any valid configuration and bounded inputs; every helper guards
// unusable inputs with an early return so the fuzz harness drops them silently.
// Both test_flybyFilter.cpp (fixed inputs) and test_flybyFilter_fuzz.cpp
// (fuzzed inputs) call the same helpers.
//
// All quantities are in the filter's internal units (km, km/s); the adapter
// handles SI<->internal.

#include "flybyFilterAlgorithm.h"
#include "flybyFilterSpecs.h"

#include "utilities/fsw/validPSDCheck.h"

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>
#include <optional>

namespace filtering::flybyFilter {

using TestState = FlybyFilterAlgorithm::State;
using Vector6 = Eigen::Matrix<double, 6, 1>;
using Matrix6 = Eigen::Matrix<double, 6, 6>;

// Mars gravitational parameter in internal units (km^3/s^2).
inline constexpr double kMu = 42828.314;
inline constexpr double kAlpha = 0.02;
inline constexpr double kBeta = 2.0;
inline constexpr double kHeadingStd = 1E-4;

inline TestState makeState(Eigen::Vector3d const& r, Eigen::Vector3d const& v) {
    TestState s;
    s.set<filtering::Position<3>>(r);
    s.set<filtering::Velocity<3>>(v);
    return s;
}

//! Build a diagonal covariance from position and velocity standard deviations (not variances).
inline Matrix6 diagCovariance(double posStd, double velStd) {
    Vector6 d;
    d << posStd * posStd, posStd * posStd, posStd * posStd, velStd * velStd, velStd * velStd, velStd * velStd;
    return d.asDiagonal();
}

inline Matrix6 smallProcessNoise() { return Matrix6::Identity() * 1E-12; }

inline Eigen::Vector3d headingOf(TestState const& s) {
    Eigen::Vector3d const r = s.get<filtering::Position<3>>();
    return r / r.norm();
}

// A representative flyby state (km, km/s).
inline TestState nominalTruth() { return makeState({3000.0, 1000.0, 500.0}, {1.0, -2.0, 0.5}); }

inline FlybyFilterConfig baseConfig(TestState const& initial, Matrix6 const& P) {
    return FlybyFilterConfig::create(kAlpha, kBeta, kMu, smallProcessNoise(), initial, P, kHeadingStd);
}

inline FlybyFilterConfig configWithProcessNoise(TestState const& initial,
                                                Matrix6 const& P,
                                                Matrix6 const& processNoise) {
    return FlybyFilterConfig::create(kAlpha, kBeta, kMu, processNoise, initial, P, kHeadingStd);
}

inline FlybyFilterConfig configWithHeadingStd(TestState const& initial, Matrix6 const& P, double headingStd) {
    return FlybyFilterConfig::create(kAlpha, kBeta, kMu, smallProcessNoise(), initial, P, headingStd);
}

//! Pack a heading observation with the noise covariance the filter would build from `headingStd`.
inline HeadingMeasurement makeHeadingMeasurement(double timeTag, Eigen::Vector3d const& rhat, double headingStd) {
    HeadingMeasurement m;
    m.timeTag = timeTag;
    m.rhat_BN_N = rhat;
    m.covar = (headingStd * headingStd) * Eigen::Matrix3d::Identity();
    m.valid = true;
    return m;
}

// A complete set of valid Config inputs; individual tests override one field.
struct ConfigInputs {
    double alpha = kAlpha;
    double beta = kBeta;
    double mu = kMu;
    Matrix6 processNoise = smallProcessNoise();
    TestState initialState = nominalTruth();
    Matrix6 initialCovariance = diagCovariance(100.0, 0.1);
    double headingStd = kHeadingStd;
};

inline FlybyFilterConfig buildConfig(ConfigInputs const& in) {
    return FlybyFilterConfig::create(
        in.alpha, in.beta, in.mu, in.processNoise, in.initialState, in.initialCovariance, in.headingStd);
}

//! Turn arbitrary fuzz inputs into a guaranteed-valid config, or nullopt when they cannot make one.
//! Keeps |r| away from zero so the two-body dynamics and the heading model stay well conditioned.
inline std::optional<FlybyFilterConfig> tryFuzzConfig(Eigen::Vector3d const& rOffset,
                                                      Eigen::Vector3d const& vOffset,
                                                      double mu,
                                                      Vector6 const& covDiagRaw,
                                                      double q,
                                                      double headingStd) {
    if (!rOffset.allFinite() || !vOffset.allFinite() || !covDiagRaw.allFinite() || !std::isfinite(mu) ||
        !std::isfinite(q) || !std::isfinite(headingStd) || mu <= 0.0 || q < 0.0 || headingStd < 0.0) {
        return std::nullopt;
    }
    TestState const initial = makeState(nominalTruth().get<filtering::Position<3>>() + rOffset,
                                        nominalTruth().get<filtering::Velocity<3>>() + vOffset);
    if (initial.get<filtering::Position<3>>().norm() < 1.0) {
        return std::nullopt;
    }
    // A strictly positive diagonal is symmetric positive definite, so the covariance always validates.
    Vector6 const covDiag = covDiagRaw.cwiseAbs() + Vector6::Constant(1E-6);
    return FlybyFilterConfig::create(
        kAlpha, kBeta, mu, Matrix6::Identity() * q, initial, covDiag.asDiagonal(), headingStd);
}

inline bool finiteSymmetricPsd(Matrix6 const& P) {
    return P.allFinite() && P.isApprox(P.transpose(), 1E-8) && isPositiveSemiDefinite<6>(P);
}

// ---- Property helpers: one filter invariant each, shared by the fixed-input and fuzzed suites. ----

//! For any valid config and finite, bounded inputs, one update() leaves the estimate physically
//! meaningful: state and covariance finite, covariance symmetric and positive semi-definite.
inline void propertyUpdateKeepsStateValidAndBounded(Eigen::Vector3d const& rOffset,
                                                    Eigen::Vector3d const& vOffset,
                                                    double mu,
                                                    Vector6 const& covDiagRaw,
                                                    double q,
                                                    Eigen::Vector3d const& rhatRaw,
                                                    double dt) {
    std::optional<FlybyFilterConfig> const cfg = tryFuzzConfig(rOffset, vOffset, mu, covDiagRaw, q, kHeadingStd);
    if (!cfg || !rhatRaw.allFinite() || rhatRaw.norm() < 1E-3 || !std::isfinite(dt) || dt < 0.0) {
        return;
    }
    FlybyFilterAlgorithm algo(*cfg);

    HeadingData heading;
    heading.timeTag = dt;
    heading.rhat_BN_N = rhatRaw.normalized();
    FlybyFilterOutput const out = algo.update(dt, heading);

    EXPECT_TRUE(algo.getState().raw().allFinite());
    EXPECT_TRUE(finiteSymmetricPsd(algo.getCovariance()));
    EXPECT_TRUE(out.filterState.state.isApprox(algo.getState().raw()));
    EXPECT_TRUE(out.filterState.covariance.isApprox(algo.getCovariance()));
}

//! A bad measurement can never corrupt the estimate: with a non-finite heading the SRuKF skips the
//! update (its finite-input guard), so the state stays finite and the covariance stays PSD.
inline void propertyArbitraryMeasurementsPreserveState(Eigen::Vector3d const& rOffset,
                                                       Eigen::Vector3d const& vOffset,
                                                       double mu,
                                                       Vector6 const& covDiagRaw,
                                                       double q,
                                                       Eigen::Vector3d const& rhat,
                                                       double dt) {
    std::optional<FlybyFilterConfig> const cfg = tryFuzzConfig(rOffset, vOffset, mu, covDiagRaw, q, kHeadingStd);
    if (!cfg || !std::isfinite(dt) || dt < 0.0) {
        return;
    }
    FlybyFilterAlgorithm algo(*cfg);

    HeadingData heading;
    heading.timeTag = dt;
    heading.rhat_BN_N = rhat;
    algo.update(dt, heading);

    EXPECT_TRUE(algo.getState().raw().allFinite());
    EXPECT_TRUE(finiteSymmetricPsd(algo.getCovariance()));
}

//! Folding in a measurement never increases the total uncertainty: with no time propagation between
//! them (timeUpdate(0) adds no process noise), the covariance trace after a heading measurementUpdate
//! is no larger than before. This is the defining information-gain property of a Kalman-family update.
inline void propertyMeasurementDoesNotIncreaseCovariance(Eigen::Vector3d const& rOffset,
                                                         Eigen::Vector3d const& vOffset,
                                                         double mu,
                                                         Vector6 const& covDiagRaw,
                                                         Eigen::Vector3d const& rhatRaw) {
    std::optional<FlybyFilterConfig> const cfg = tryFuzzConfig(rOffset, vOffset, mu, covDiagRaw, 0.0, kHeadingStd);
    if (!cfg || !rhatRaw.allFinite() || rhatRaw.norm() < 1E-3) {
        return;
    }
    FlybyFilterAlgorithm algo(*cfg);

    double const traceBefore = algo.getCovariance().trace();
    if (!algo.timeUpdate(0.0)) {
        return;
    }
    if (!algo.measurementUpdate(makeHeadingMeasurement(0.0, rhatRaw.normalized(), kHeadingStd))) {
        return;
    }
    double const traceAfter = algo.getCovariance().trace();
    EXPECT_LE(traceAfter, traceBefore + 1E-9 * (1.0 + traceBefore));
}

}  // namespace filtering::flybyFilter

#endif  // F32XMERA_FLYBYFILTER_TEST_HELPERS_H
