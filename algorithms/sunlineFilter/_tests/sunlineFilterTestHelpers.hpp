#ifndef F32XMERA_SUNLINEFILTER_TEST_HELPERS_H
#define F32XMERA_SUNLINEFILTER_TEST_HELPERS_H

// Shared builders, a fuzzable config factory, and property-check helpers for the
// SunlineFilter tests. The property helpers each assert one filter invariant that
// must hold for any valid configuration and bounded inputs; every helper guards
// unusable inputs with an early return so the fuzz harness drops them silently.
// Both test_sunlineFilter.cpp (fixed inputs) and test_sunlineFilter_fuzz.cpp
// (fuzzed inputs) call the same helpers.

#include "sunlineFilterAlgorithm.h"
#include "sunlineFilterSpecs.h"

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <optional>

namespace filtering::sunlineFilter {

// Bound for comparing two float computations of the same quantity: a few operations of roundoff
// with the 3-10x margin of docs/PRECISION_GUIDELINES.md (10 * epsilon ~ 1.2e-6).
inline constexpr float kRoundoffTol = 10.0F * std::numeric_limits<float>::epsilon();

using TestState = SunlineFilterAlgorithm::State;
using Vector7 = Eigen::Matrix<float, 7, 1>;
using Matrix7 = Eigen::Matrix<float, 7, 7>;

inline constexpr float kAlpha = 0.02;
inline constexpr float kBeta = 2.0;
inline constexpr float kBiasLowerBound = 0.5;
inline constexpr float kBiasUpperBound = 1.5;

inline TestState makeState(Eigen::Vector3f const& sHat, Eigen::Vector3f const& omega, float bias) {
    TestState s;
    s.set<filtering::Position<3>>(sHat);
    s.set<filtering::Velocity<3>>(omega);
    Eigen::Vector<float, 1> b;
    b(0) = bias;
    s.set<filtering::Bias<1>>(b);
    return s;
}

inline Matrix7 diagCovariance(float posStd, float velStd, float biasStd) {
    Vector7 d;
    d << posStd * posStd, posStd * posStd, posStd * posStd, velStd * velStd, velStd * velStd, velStd * velStd,
        biasStd * biasStd;
    return d.asDiagonal();
}

inline Matrix7 smallProcessNoise() {
    float const q = 1E-5;
    Vector7 d;
    d << q, q, q, q, q, q, q;
    return d.asDiagonal();
}

// Three-CSS geometry (unit boresights in rows 0..2; remaining rows unused/zero).
inline Eigen::Matrix<float, MaxCss, 3> threeCssNHat() {
    Eigen::Matrix<float, MaxCss, 3> nHat = Eigen::Matrix<float, MaxCss, 3>::Zero();
    nHat.row(0) = Eigen::RowVector3f(0.707, -0.5, 0.5);
    nHat.row(1) = Eigen::RowVector3f(0.707, 0.5, 0.5);
    nHat.row(2) = Eigen::RowVector3f(-0.707, 0.0, 0.707);
    return nHat;
}

// Single-boresight CSS geometry for rate-only tests. numberOfCss must be >= 1, but
// these tests never feed a CSS measurement, so the lone boresight is inert.
inline Eigen::Matrix<float, MaxCss, 3> oneCssNHat() {
    Eigen::Matrix<float, MaxCss, 3> nHat = Eigen::Matrix<float, MaxCss, 3>::Zero();
    nHat.row(0) = Eigen::RowVector3f(0.0, 0.0, 1.0);
    return nHat;
}

// Validated config with a single (unused) CSS sensor; for dynamics / timeUpdate / rate-only tests.
inline SunlineFilterConfig rateOnlyConfig(TestState const& initial, Matrix7 const& P) {
    return SunlineFilterConfig::create(kAlpha,
                                       kBeta,
                                       smallProcessNoise(),
                                       initial,
                                       P,
                                       kBiasLowerBound,
                                       kBiasUpperBound,
                                       oneCssNHat(),
                                       Eigen::Vector<float, MaxCss>::Ones(),
                                       1,
                                       0.0,
                                       1E-2,
                                       1E-3);
}

// rateOnlyConfig with an explicit process noise (for exercising process-noise-driven covariance growth).
inline SunlineFilterConfig rateOnlyConfigWithProcessNoise(TestState const& initial,
                                                          Matrix7 const& P,
                                                          Matrix7 const& processNoise) {
    return SunlineFilterConfig::create(kAlpha,
                                       kBeta,
                                       processNoise,
                                       initial,
                                       P,
                                       kBiasLowerBound,
                                       kBiasUpperBound,
                                       oneCssNHat(),
                                       Eigen::Vector<float, MaxCss>::Ones(),
                                       1,
                                       0.0,
                                       1E-2,
                                       1E-3);
}

// Validated config with the three-CSS geometry, scale factor = 1, and the given sensor threshold.
inline SunlineFilterConfig threeCssConfig(TestState const& initial, Matrix7 const& P, float sensorThreshold) {
    return SunlineFilterConfig::create(kAlpha,
                                       kBeta,
                                       smallProcessNoise(),
                                       initial,
                                       P,
                                       kBiasLowerBound,
                                       kBiasUpperBound,
                                       threeCssNHat(),
                                       Eigen::Vector<float, MaxCss>::Ones(),
                                       3,
                                       sensorThreshold,
                                       1E-2,
                                       1E-3);
}

// A complete set of valid Config inputs; individual validation tests override one field to
// drive a specific validation branch.
struct ConfigInputs {
    float alpha = kAlpha;
    float beta = kBeta;
    Matrix7 processNoise = smallProcessNoise();
    TestState initialState = makeState(Eigen::Vector3f(0.0, 0.0, 1.0), Eigen::Vector3f::Zero(), 1.0);
    Matrix7 initialCovariance = diagCovariance(1E-2, 1E-3, 1E-2);
    float biasLowerBound = kBiasLowerBound;
    float biasUpperBound = kBiasUpperBound;
    Eigen::Matrix<float, MaxCss, 3> cssNHat = threeCssNHat();
    Eigen::Vector<float, MaxCss> cssScaleFactor = Eigen::Vector<float, MaxCss>::Ones();
    uint32_t numberOfCss = 3;
    float sensorThreshold = 0.0;
    float cssMeasStd = 1E-2;
    float gyroStd = 1E-3;
};

inline SunlineFilterConfig buildConfig(ConfigInputs const& in) {
    return SunlineFilterConfig::create(in.alpha,
                                       in.beta,
                                       in.processNoise,
                                       in.initialState,
                                       in.initialCovariance,
                                       in.biasLowerBound,
                                       in.biasUpperBound,
                                       in.cssNHat,
                                       in.cssScaleFactor,
                                       in.numberOfCss,
                                       in.sensorThreshold,
                                       in.cssMeasStd,
                                       in.gyroStd);
}

// ---------------------------------------------------------------------------
// Fuzzable config factory. Builds a valid three-CSS config from raw
// initial-condition inputs: it normalizes the heading (dropping degenerate
// near-zero directions), makes the covariance strictly positive definite, and
// keeps the remaining parameters at fixed valid constants. Returns nullopt for
// unusable inputs so the fuzz harness skips them.
// ---------------------------------------------------------------------------
inline std::optional<SunlineFilterConfig> tryFuzzConfig(Eigen::Vector3f const& sHatRaw,
                                                        Eigen::Vector3f const& omega,
                                                        float bias,
                                                        Vector7 const& covDiagRaw) {
    if (!sHatRaw.allFinite() || sHatRaw.stableNorm() < 1E-3 || !omega.allFinite() || !std::isfinite(bias) ||
        !covDiagRaw.allFinite()) {
        return std::nullopt;
    }
    // A strictly positive diagonal is symmetric positive definite, so the covariance always validates.
    Vector7 const covDiag = covDiagRaw.cwiseAbs() + Vector7::Constant(1E-6);
    return threeCssConfig(makeState(sHatRaw.stableNormalized(), omega, bias), covDiag.asDiagonal(), 0.0);
}

// Drives one update() with the given (possibly non-finite) CSS cos-values and rate at a shared
// measurement time, propagating dt to the output time.
inline void driveUpdate(SunlineFilterAlgorithm& algo,
                        Eigen::Vector<float, MaxCss> const& cssCos,
                        Eigen::Vector3f const& rate,
                        float dt) {
    CssData css;
    css.timeTag = 1.0;
    css.cosValues = cssCos;
    RateData r;
    r.timeTag = 1.0;
    r.rate = rate;
    algo.update(1.0 + dt, css, r);
}

// ---------------------------------------------------------------------------
// Property helpers.
// ---------------------------------------------------------------------------

// For any valid config and finite, bounded inputs, one update() leaves the estimate physically
// meaningful: the state and covariance are finite, the heading is a unit vector (regularize
// renormalizes it), the CSS bias is inside its configured bounds (regularize clamps it), and the
// covariance stays symmetric.
inline void propertyUpdateKeepsStateValidAndBounded(Eigen::Vector3f sHatRaw,
                                                    Eigen::Vector3f omega,
                                                    float bias,
                                                    Vector7 covDiagRaw,
                                                    Eigen::Vector<float, MaxCss> cssCos,
                                                    Eigen::Vector3f rate,
                                                    float dt) {
    std::optional<SunlineFilterConfig> const cfg = tryFuzzConfig(sHatRaw, omega, bias, covDiagRaw);
    if (!cfg || !cssCos.allFinite() || !rate.allFinite() || !std::isfinite(dt) || dt < 0.0) {
        return;
    }
    SunlineFilterAlgorithm algo(*cfg);
    driveUpdate(algo, cssCos, rate, dt);

    TestState const s = algo.getState();
    Matrix7 const P = algo.getCovariance();
    EXPECT_TRUE(s.raw().allFinite());
    EXPECT_TRUE(P.allFinite());
    EXPECT_NEAR(s.get<filtering::Position<3>>().stableNorm(), 1.0, 1E-6);
    float const biasOut = s.get<filtering::Bias<1>>()(0);
    EXPECT_GE(biasOut, kBiasLowerBound - 1E-6);
    EXPECT_LE(biasOut, kBiasUpperBound + 1E-6);
    EXPECT_LE((P - P.transpose()).norm(), 1E-5 * (1.0 + P.norm()));
}

// A bad measurement can never corrupt the estimate: even with non-finite CSS cos-values or rate,
// the SRUKF skips the update (its finite-input guard) so the state stays finite and the heading
// stays a unit vector. The config inputs are finite; only the measurement data is arbitrary.
inline void propertyArbitraryMeasurementsPreserveState(Eigen::Vector3f sHatRaw,
                                                       Eigen::Vector3f omega,
                                                       float bias,
                                                       Vector7 covDiagRaw,
                                                       Eigen::Vector<float, MaxCss> cssCos,
                                                       Eigen::Vector3f rate,
                                                       float dt) {
    std::optional<SunlineFilterConfig> const cfg = tryFuzzConfig(sHatRaw, omega, bias, covDiagRaw);
    if (!cfg || !std::isfinite(dt) || dt < 0.0) {
        return;
    }
    SunlineFilterAlgorithm algo(*cfg);
    driveUpdate(algo, cssCos, rate, dt);

    TestState const s = algo.getState();
    EXPECT_TRUE(s.raw().allFinite());
    EXPECT_TRUE(algo.getCovariance().allFinite());
    EXPECT_NEAR(s.get<filtering::Position<3>>().stableNorm(), 1.0, 1E-6);
}

// Folding in a measurement never increases the total uncertainty: with no time propagation between
// them (timeUpdate(0) adds no process noise), the covariance trace after a rate measurementUpdate is
// no larger than before. This is the defining information-gain property of a Kalman-family update.
inline void propertyRateMeasurementDoesNotIncreaseCovariance(Eigen::Vector3f sHatRaw,
                                                             Eigen::Vector3f omega,
                                                             float bias,
                                                             Vector7 covDiagRaw,
                                                             Eigen::Vector3f rate) {
    std::optional<SunlineFilterConfig> const cfg = tryFuzzConfig(sHatRaw, omega, bias, covDiagRaw);
    if (!cfg || !rate.allFinite()) {
        return;
    }
    SunlineFilterAlgorithm algo(*cfg);
    float const traceBefore = algo.getCovariance().trace();

    algo.timeUpdate(0.0);
    RateMeasurement r;
    r.timeTag = 0.0;
    r.omega_BN_B = rate;
    r.covar = (1E-3 * 1E-3) * Eigen::Matrix3f::Identity();
    r.valid = true;
    algo.measurementUpdate(r);

    float const traceAfter = algo.getCovariance().trace();
    EXPECT_LE(traceAfter, traceBefore + 1E-5 * (1.0 + traceBefore));
}

}  // namespace filtering::sunlineFilter

#endif  // F32XMERA_SUNLINEFILTER_TEST_HELPERS_H
