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
#include <limits>
#include <optional>

namespace filtering::flybyFilter {

using TestState = FlybyFilterAlgorithm::State;
using Vector8 = Eigen::Matrix<float, TestState::size, 1>;
using Matrix8 = Eigen::Matrix<float, TestState::size, TestState::size>;

// Bound for comparing two float computations of the same quantity: a few operations of roundoff
// with a 3-10x margin (10 * epsilon ~ 1.2e-6).
inline constexpr float kRoundoffTol = 10.0F * std::numeric_limits<float>::epsilon();

// Mars gravitational parameter in internal units (km^3/s^2).
inline constexpr float kMu = 42828.314;
inline constexpr float kAlpha = 0.02;
inline constexpr float kBeta = 2.0;
inline constexpr float kHeadingStd = 1E-4;
inline constexpr float kNoiseScale = 1.0;
inline constexpr float kBiasStd = 1E-3;  // [rad] prior standard deviation of each heading-bias angle
inline constexpr float kMaxBias = 1E-2;  // [rad] maximum magnitude of each heading-bias angle

//! Heading noise covariance a reading would carry (covar_N), isotropic with standard deviation `std`.
inline Eigen::Matrix3f headingCovariance(float std = kHeadingStd) { return (std * std) * Eigen::Matrix3f::Identity(); }

inline TestState makeState(Eigen::Vector3f const& r,
                           Eigen::Vector3f const& v,
                           Eigen::Vector2f const& bias = Eigen::Vector2f::Zero()) {
    TestState s;
    s.set<filtering::Position<3>>(r);
    s.set<filtering::Velocity<3>>(v);
    s.set<filtering::Bias<2>>(bias);
    return s;
}

//! Build a diagonal covariance from position, velocity and bias-angle standard deviations (not variances).
inline Matrix8 diagCovariance(float posStd, float velStd, float biasStd = kBiasStd) {
    Vector8 d;
    d << posStd * posStd, posStd * posStd, posStd * posStd, velStd * velStd, velStd * velStd, velStd * velStd,
        biasStd * biasStd, biasStd * biasStd;
    return d.asDiagonal();
}

inline Matrix8 smallProcessNoise() { return Matrix8::Identity() * 1E-12; }

inline Eigen::Vector3f headingOf(TestState const& s) {
    Eigen::Vector3f const r = s.get<filtering::Position<3>>();
    return r / r.stableNorm();
}

// A representative flyby state (km, km/s).
inline TestState nominalTruth() { return makeState({3000.0, 1000.0, 500.0}, {1.0, -2.0, 0.5}); }

inline FlybyFilterConfig baseConfig(TestState const& initial, Matrix8 const& P) {
    return FlybyFilterConfig::create(
        kAlpha, kBeta, kMu, smallProcessNoise(), initial, P, kNoiseScale, kMaxBias, kMaxBias);
}

inline FlybyFilterConfig configWithProcessNoise(TestState const& initial,
                                                Matrix8 const& P,
                                                Matrix8 const& processNoise) {
    return FlybyFilterConfig::create(kAlpha, kBeta, kMu, processNoise, initial, P, kNoiseScale, kMaxBias, kMaxBias);
}

inline FlybyFilterConfig configWithNoiseScale(TestState const& initial, Matrix8 const& P, float noiseScale) {
    return FlybyFilterConfig::create(
        kAlpha, kBeta, kMu, smallProcessNoise(), initial, P, noiseScale, kMaxBias, kMaxBias);
}

//! A fresh heading reading carrying its own noise covariance and sun direction, as the adapter builds it from the
//! messages. The zero sun direction default models the reading without the heading bias.
inline HeadingData makeHeadingData(double timeTag,
                                   Eigen::Vector3f const& rhat,
                                   Eigen::Matrix3f const& covar_N = headingCovariance(),
                                   Eigen::Vector3f const& shat_N = Eigen::Vector3f::Zero()) {
    HeadingData heading;
    heading.timeTag = timeTag;
    heading.rhat_BN_N = rhat;
    heading.covar_N = covar_N;
    heading.shat_N = shat_N;
    return heading;
}

//! Pack a heading observation with an isotropic noise covariance of standard deviation `headingStd`, and a unit
//! sun direction (zero: the reading is modeled without the heading bias).
inline HeadingMeasurement makeHeadingMeasurement(double timeTag,
                                                 Eigen::Vector3f const& rhat,
                                                 float headingStd,
                                                 Eigen::Vector3f const& shat_N = Eigen::Vector3f::Zero()) {
    HeadingMeasurement m;
    m.timeTag = timeTag;
    m.rhat_BN_N = rhat;
    m.shat_N = shat_N;
    m.covar = (headingStd * headingStd) * Eigen::Matrix3f::Identity();
    m.valid = true;
    return m;
}

// A complete set of valid Config inputs; individual tests override one field.
struct ConfigInputs {
    float alpha = kAlpha;
    float beta = kBeta;
    float mu = kMu;
    Matrix8 processNoise = smallProcessNoise();
    TestState initialState = nominalTruth();
    Matrix8 initialCovariance = diagCovariance(100.0, 0.1);
    float noiseScale = kNoiseScale;
    float maxAcrossBias = kMaxBias;
    float maxAlongBias = kMaxBias;
    float measurementDelay = 0.0;
};

inline FlybyFilterConfig buildConfig(ConfigInputs const& in) {
    return FlybyFilterConfig::create(in.alpha,
                                     in.beta,
                                     in.mu,
                                     in.processNoise,
                                     in.initialState,
                                     in.initialCovariance,
                                     in.noiseScale,
                                     in.maxAcrossBias,
                                     in.maxAlongBias,
                                     in.measurementDelay);
}

//! Turn arbitrary fuzz inputs into a guaranteed-valid config, or nullopt when they cannot make one.
//! Keeps |r| away from zero so the two-body dynamics and the heading model stay well conditioned.
inline std::optional<FlybyFilterConfig> tryFuzzConfig(Eigen::Vector3f const& rOffset,
                                                      Eigen::Vector3f const& vOffset,
                                                      float mu,
                                                      Vector8 const& covDiagRaw,
                                                      float q,
                                                      float noiseScale) {
    if (!rOffset.allFinite() || !vOffset.allFinite() || !covDiagRaw.allFinite() || !std::isfinite(mu) ||
        !std::isfinite(q) || !std::isfinite(noiseScale) || mu <= 0.0 || q < 0.0 || noiseScale <= 0.0) {
        return std::nullopt;
    }
    TestState const initial = makeState(nominalTruth().get<filtering::Position<3>>() + rOffset,
                                        nominalTruth().get<filtering::Velocity<3>>() + vOffset);
    if (initial.get<filtering::Position<3>>().stableNorm() < 1.0) {
        return std::nullopt;
    }
    // A strictly positive diagonal is symmetric positive definite, so the covariance always validates. The
    // bias-angle variances are capped at the square of their maximum: a prior wider than the bound is not physical.
    Vector8 covDiag = covDiagRaw.cwiseAbs() + Vector8::Constant(1E-6);
    covDiag.tail<2>() = covDiag.tail<2>().cwiseMin(kMaxBias * kMaxBias);
    return FlybyFilterConfig::create(
        kAlpha, kBeta, mu, Matrix8::Identity() * q, initial, covDiag.asDiagonal(), noiseScale, kMaxBias, kMaxBias);
}

inline bool finiteSymmetricPsd(Matrix8 const& P) {
    return P.allFinite() && P.isApprox(P.transpose(), 1E-5) && isPositiveSemiDefinite<TestState::size>(P);
}

// ---- Property helpers: one filter invariant each, shared by the fixed-input and fuzzed suites. ----

//! For any valid config and finite, bounded inputs, one update() leaves the estimate physically
//! meaningful: state and covariance finite, covariance symmetric and positive semi-definite.
inline void propertyUpdateKeepsStateValidAndBounded(Eigen::Vector3f const& rOffset,
                                                    Eigen::Vector3f const& vOffset,
                                                    float mu,
                                                    Vector8 const& covDiagRaw,
                                                    float q,
                                                    Eigen::Vector3f const& rhatRaw,
                                                    float dt,
                                                    Eigen::Vector3f const& shatRaw = Eigen::Vector3f::Zero()) {
    std::optional<FlybyFilterConfig> const cfg = tryFuzzConfig(rOffset, vOffset, mu, covDiagRaw, q, kNoiseScale);
    if (!cfg || !rhatRaw.allFinite() || rhatRaw.stableNorm() < 1E-3 || !shatRaw.allFinite() || !std::isfinite(dt) ||
        dt < 0.0) {
        return;
    }
    FlybyFilterAlgorithm algo(*cfg);

    HeadingData heading;
    heading.timeTag = dt;
    heading.rhat_BN_N = rhatRaw.stableNormalized();
    heading.covar_N = headingCovariance();
    heading.shat_N = shatRaw;
    FlybyFilterOutput const out = algo.update(dt, heading);

    EXPECT_TRUE(algo.getState().raw().allFinite());
    EXPECT_TRUE(finiteSymmetricPsd(algo.getCovariance()));
    EXPECT_TRUE(out.filterState.state.isApprox(algo.getState().raw()));
    EXPECT_TRUE(out.filterState.covariance.isApprox(algo.getCovariance()));
}

//! A bad measurement can never corrupt the estimate: with a non-finite heading the SRuKF skips the
//! update (its finite-input guard), so the state stays finite and the covariance stays PSD.
inline void propertyArbitraryMeasurementsPreserveState(Eigen::Vector3f const& rOffset,
                                                       Eigen::Vector3f const& vOffset,
                                                       float mu,
                                                       Vector8 const& covDiagRaw,
                                                       float q,
                                                       Eigen::Vector3f const& rhat,
                                                       float dt,
                                                       Eigen::Vector3f const& shat = Eigen::Vector3f::Zero()) {
    std::optional<FlybyFilterConfig> const cfg = tryFuzzConfig(rOffset, vOffset, mu, covDiagRaw, q, kNoiseScale);
    if (!cfg || !std::isfinite(dt) || dt < 0.0) {
        return;
    }
    FlybyFilterAlgorithm algo(*cfg);

    HeadingData heading;
    heading.timeTag = dt;
    heading.rhat_BN_N = rhat;
    heading.covar_N = headingCovariance();
    heading.shat_N = shat;
    algo.update(dt, heading);

    EXPECT_TRUE(algo.getState().raw().allFinite());
    EXPECT_TRUE(finiteSymmetricPsd(algo.getCovariance()));
}

//! Folding in a measurement never increases the total uncertainty: with no time propagation between
//! them (timeUpdate(0) adds no process noise), the covariance trace after a heading measurementUpdate
//! is no larger than before. This is the defining information-gain property of a Kalman-family update.
inline void propertyMeasurementDoesNotIncreaseCovariance(Eigen::Vector3f const& rOffset,
                                                         Eigen::Vector3f const& vOffset,
                                                         float mu,
                                                         Vector8 const& covDiagRaw,
                                                         Eigen::Vector3f const& rhatRaw,
                                                         Eigen::Vector3f const& shatRaw = Eigen::Vector3f::Zero()) {
    std::optional<FlybyFilterConfig> const cfg = tryFuzzConfig(rOffset, vOffset, mu, covDiagRaw, 0.0, kNoiseScale);
    if (!cfg || !rhatRaw.allFinite() || rhatRaw.stableNorm() < 1E-3 || !shatRaw.allFinite()) {
        return;
    }
    Eigen::Vector3f const shat = shatRaw.stableNorm() > 0.0F ? shatRaw.stableNormalized() : Eigen::Vector3f::Zero();
    FlybyFilterAlgorithm algo(*cfg);

    float const traceBefore = algo.getCovariance().trace();
    if (!algo.timeUpdate(0.0)) {
        return;
    }
    if (!algo.measurementUpdate(makeHeadingMeasurement(0.0, rhatRaw.stableNormalized(), kHeadingStd, shat))) {
        return;
    }
    float const traceAfter = algo.getCovariance().trace();
    EXPECT_LE(traceAfter, traceBefore + 1E-5 * (1.0 + traceBefore));
}

}  // namespace filtering::flybyFilter

#endif  // F32XMERA_FLYBYFILTER_TEST_HELPERS_H
