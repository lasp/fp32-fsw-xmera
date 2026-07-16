// Unit tests for SunlineFilterAlgorithm and the SRUKF numerical helpers.
//
// Organized like the other modules:
// - Setup: construction seeds the filter from the validated config.
// - Property: invariants that hold for any valid config and bounded inputs
//   (shared with the fuzz harness via sunlineFilterTestHelpers.hpp).
// - Edge cases: dynamics/timeUpdate/measurementUpdate boundary behaviors.
// - Regression: a fixed end-to-end scenario the filter must reproduce.
// - Config: factory validation, static validators, and round-trip.
// - SrukfDetail: the SRUKF static numerical helpers.

#include "sunlineFilterTestHelpers.hpp"

#include <filteringCore/srukf.hpp>

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <cmath>

namespace filtering::sunlineFilter {
namespace {

using State = SunlineFilterAlgorithm::State;

// Alias for invoking the numerical helpers (static methods on the SRuKF class
// template). The State/Dynamics arguments don't affect the helpers' behavior;
// any valid instantiation works.
using SRuKF = ::filtering::SRuKF<SunlineState, SunlineDynamics>;

}  // namespace

// ============================================================================
// Setup: construction runs the two-phase init — the filter state and covariance
// are seeded from the validated config, and the config round-trips.
// ============================================================================

TEST(SunlineFilterAlgorithmSetup, ConstructionSeedsStateAndCovarianceFromConfig) {
    State const initial = makeState(Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(0.01, -0.02, 0.0), 1.1);
    Matrix7 const P0 = diagCovariance(1E-2, 1E-3, 1E-2);
    SunlineFilterConfig const cfg = threeCssConfig(initial, P0, 0.0);

    SunlineFilterAlgorithm const algo(cfg);

    EXPECT_TRUE(algo.getState().raw().isApprox(initial.raw()));
    EXPECT_TRUE(algo.getCovariance().isApprox(P0));
    EXPECT_FALSE(algo.getLastCssResiduals().valid);
    EXPECT_FALSE(algo.getLastRateResiduals().valid);
}

// ============================================================================
// Property tests: each drives a shared helper (see sunlineFilterTestHelpers.hpp)
// with fixed inputs. The fuzz harness re-runs the same helpers over random
// inputs.
// ============================================================================

TEST(SunlineFilterAlgorithmProperty, UpdateKeepsStateValidAndBounded) {
    propertyUpdateKeepsStateValidAndBounded(
        Eigen::Vector3d(0.2, -0.3, 0.9),
        Eigen::Vector3d(0.01, 0.0, -0.02),
        1.2,
        (Vector7() << 1E-2, 1E-2, 1E-2, 1E-3, 1E-3, 1E-3, 1E-2).finished(),
        (Eigen::Vector<double, MaxCss>() << 0.6, 0.5, 0.7, 0, 0, 0, 0, 0).finished(),
        Eigen::Vector3d(0.01, 0.0, -0.02),
        0.5);
}

TEST(SunlineFilterAlgorithmProperty, ArbitraryMeasurementsPreserveState) {
    double const nan = std::numeric_limits<double>::quiet_NaN();
    double const inf = std::numeric_limits<double>::infinity();
    // Non-finite CSS cos-values and rate must be skipped, not folded in.
    propertyArbitraryMeasurementsPreserveState(
        Eigen::Vector3d(0.0, 0.0, 1.0),
        Eigen::Vector3d(0.01, 0.0, 0.0),
        1.0,
        (Vector7() << 1E-2, 1E-2, 1E-2, 1E-3, 1E-3, 1E-3, 1E-2).finished(),
        (Eigen::Vector<double, MaxCss>() << nan, 0.5, 0.7, 0, 0, 0, 0, 0).finished(),
        Eigen::Vector3d(inf, 0.0, 0.0),
        0.5);
}

TEST(SunlineFilterAlgorithmProperty, RateMeasurementDoesNotIncreaseCovariance) {
    propertyRateMeasurementDoesNotIncreaseCovariance(Eigen::Vector3d(0.0, 0.0, 1.0),
                                                     Eigen::Vector3d(0.01, 0.01, 0.01),
                                                     1.0,
                                                     (Vector7() << 1E-2, 1E-2, 1E-2, 1E-1, 1E-1, 1E-1, 1E-1).finished(),
                                                     Eigen::Vector3d(0.012, 0.008, 0.011));
}

// ============================================================================
// Edge cases.
// ============================================================================

// Dynamics: |s_hat| is a conserved quantity under ds/dt = s × omega.
//
// Pure analytical check: the derivative the dynamics functor returns must be
// orthogonal to s (so d/dt(s·s) = 2 s·(ds/dt) = 0), and omega/bias derivatives
// must be zero. No integrator involved — this isolates the dynamics from any
// numerical concerns in propagate().
TEST(SunlineFilterAlgorithmDynamics, DerivativeIsOrthogonalToHeading) {
    Eigen::Vector3d const sHat = Eigen::Vector3d(0.4, -0.7, 0.6).normalized();
    Eigen::Vector3d const omega = Eigen::Vector3d(0.02, -0.005, 0.01);
    State s = makeState(sHat, omega, 0.6);

    State const dot = SunlineDynamics{}(0.0, s);

    Eigen::Vector3d const dsdt = dot.get<filtering::Position<3>>();
    EXPECT_NEAR(sHat.dot(dsdt), 0.0, 1E-12);

    EXPECT_TRUE(dot.get<filtering::Velocity<3>>().isApprox(Eigen::Vector3d::Zero(), 1E-12));
    EXPECT_DOUBLE_EQ(dot.get<filtering::Bias<1>>()(0), 0.0);
}

// Integrated check: a short timeUpdate must preserve |s| to RK4 precision.
// `srukf::timeUpdate` calls `propagate(dynamics, state, {0, dt}, dt)`, which
// does a SINGLE RK4 step of size dt — accurate only for small dt. Anything
// long-horizon would trip on integrator error, not the dynamics' invariant.
TEST(SunlineFilterAlgorithmDynamics, HeadingMagnitudePreservedOverSmallTimeUpdate) {
    Eigen::Vector3d const sHat0 = Eigen::Vector3d(0.0, 0.0, 1.0);
    Eigen::Vector3d const omega0 = Eigen::Vector3d(0.02, -0.005, 0.01);

    SunlineFilterAlgorithm algo(rateOnlyConfig(makeState(sHat0, omega0, 0.6), diagCovariance(1E-2, 1E-3, 1E-2)));

    algo.timeUpdate(1.0);

    Eigen::Vector3d const sHat = algo.getState().get<filtering::Position<3>>();
    EXPECT_NEAR(sHat.norm(), sHat0.norm(), 1E-8);
}

// With omega = 0 the heading must not move.
TEST(SunlineFilterAlgorithmTimeUpdate, ZeroRateLeavesHeadingFixed) {
    Eigen::Vector3d const sHat0 = Eigen::Vector3d(0.0, 1.0, 0.0).normalized();
    SunlineFilterAlgorithm algo(
        rateOnlyConfig(makeState(sHat0, Eigen::Vector3d::Zero(), 1.0), diagCovariance(1E-2, 1E-3, 1E-2)));

    algo.timeUpdate(10.0);
    Eigen::Vector3d const sHat = algo.getState().get<filtering::Position<3>>();
    EXPECT_TRUE(sHat.isApprox(sHat0, 1E-9));
}

// dt == 0 short-circuit: the state must equal the anchor on return, with no
// dynamics evolution applied.
TEST(SunlineFilterAlgorithmTimeUpdate, ZeroDtCollapsesToAnchor) {
    Eigen::Vector3d const sHat0 = Eigen::Vector3d(0.0, 0.0, 1.0);
    Eigen::Vector3d const omega0 = Eigen::Vector3d(0.1, 0.0, 0.0);

    SunlineFilterAlgorithm algo(rateOnlyConfig(makeState(sHat0, omega0, 1.0), diagCovariance(1E-2, 1E-3, 1E-2)));

    algo.timeUpdate(0.0);
    State const s = algo.getState();
    EXPECT_TRUE(s.get<filtering::Position<3>>().isApprox(sHat0, 1E-12));
    EXPECT_TRUE(s.get<filtering::Velocity<3>>().isApprox(omega0, 1E-12));
    EXPECT_DOUBLE_EQ(s.get<filtering::Bias<1>>()(0), 1.0);
}

TEST(SunlineFilterAlgorithmTimeUpdate, NaNDtReturnsTrueAndLeavesStateUnchanged) {
    Eigen::Vector3d const sHat0 = Eigen::Vector3d(0.0, 0.0, 1.0);
    Eigen::Vector3d const omega0 = Eigen::Vector3d(0.01, 0.0, -0.02);
    SunlineFilterAlgorithm algo(rateOnlyConfig(makeState(sHat0, omega0, 1.0), diagCovariance(1E-2, 1E-3, 1E-2)));
    State const before = algo.getState();

    EXPECT_TRUE(algo.timeUpdate(std::numeric_limits<double>::quiet_NaN()))
        << "a NaN dt is a no-op (zero sub-steps), so timeUpdate stays valid";
    EXPECT_TRUE(algo.getState().raw().isApprox(before.raw(), 1E-12)) << "a NaN dt should leave the state unchanged";
}

TEST(SunlineFilterAlgorithmTimeUpdate, ZeroDtLeavesCovarianceUnchanged) {
    Matrix7 const P0 = diagCovariance(1E-2, 1E-3, 1E-2);
    SunlineFilterAlgorithm algo(rateOnlyConfigWithProcessNoise(
        makeState(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d::Zero(), 1.0), P0, Matrix7::Identity() * 1E-2));

    EXPECT_TRUE(algo.timeUpdate(0.0));
    EXPECT_TRUE(algo.getCovariance().isApprox(P0, 1E-10));
}

TEST(SunlineFilterAlgorithmTimeUpdate, GrowsCovarianceWithProcessNoise) {
    Matrix7 const P0 = diagCovariance(1E-2, 1E-3, 1E-2);
    SunlineFilterAlgorithm algo(rateOnlyConfigWithProcessNoise(
        makeState(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d::Zero(), 1.0), P0, Matrix7::Identity() * 1E-2));

    EXPECT_TRUE(algo.timeUpdate(1.0));
    Matrix7 const P = algo.getCovariance();
    EXPECT_GT(P.trace(), P0.trace()) << "process noise should grow covariance";
    EXPECT_TRUE(P.isApprox(P.transpose(), 1E-10)) << "covariance not symmetric";
    EXPECT_TRUE(SunlineFilterConfig::isValidInitialCovariance(P)) << "covariance not PSD";
}

// A single rate measurement shrinks the rate-block of P. Drives the
// SequentialFilter pair directly (timeUpdate + measurementUpdate) rather than
// going through the queue-driven update() — keeps the test focused on the SRUKF
// math, independent of the pack/queue plumbing.
TEST(SunlineFilterAlgorithmMeasurementUpdate, RateMeasurementShrinksRateCovariance) {
    Eigen::Vector3d const sHat0 = Eigen::Vector3d(0.0, 0.0, 1.0);
    Eigen::Vector3d const omega0 = Eigen::Vector3d(0.01, 0.01, 0.01);

    SunlineFilterAlgorithm algo(rateOnlyConfig(makeState(sHat0, omega0, 1.0), diagCovariance(1E-2, 1E-1, 1E-1)));

    Matrix7 const covar0 = algo.getCovariance();

    // timeUpdate(0) rewinds to the anchor and populates sigma points around
    // it — required precondition for measurementUpdate to have fresh sigma
    // points to compute residuals against.
    algo.timeUpdate(0.0);

    RateMeasurement r;
    r.timeTag = 0.0;
    r.omega_BN_B = Eigen::Vector3d(0.012, 0.008, 0.011);
    r.covar = (1E-3 * 1E-3) * Eigen::Matrix3d::Identity();
    r.valid = true;
    algo.measurementUpdate(r);

    Matrix7 const covarN = algo.getCovariance();
    for (int i = 3; i < 6; ++i) {
        EXPECT_LT(covarN(i, i), covar0(i, i)) << "rate cov diag index " << i;
    }
    EXPECT_TRUE(algo.getLastRateResiduals().valid);
}

// A non-finite rate measurement is skipped by the SRUKF: the estimate is left
// untouched and no residual is reported valid (exercises the std::optional
// no-value path in applyMeasurement).
TEST(SunlineFilterAlgorithmMeasurementUpdate, NonFiniteRateMeasurementIsSkipped) {
    Eigen::Vector3d const sHat0 = Eigen::Vector3d(0.0, 0.0, 1.0);
    SunlineFilterAlgorithm algo(
        rateOnlyConfig(makeState(sHat0, Eigen::Vector3d::Zero(), 1.0), diagCovariance(1E-2, 1E-1, 1E-1)));
    algo.timeUpdate(0.0);
    Matrix7 const covarBefore = algo.getCovariance();
    Vector7 const stateBefore = algo.getState().raw();

    RateMeasurement r;
    r.timeTag = 0.0;
    r.omega_BN_B = Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
    r.covar = (1E-3 * 1E-3) * Eigen::Matrix3d::Identity();
    r.valid = true;
    EXPECT_FALSE(algo.measurementUpdate(r)) << "a non-finite measurement update must report false";

    EXPECT_TRUE(algo.getState().raw().isApprox(stateBefore));
    EXPECT_TRUE(algo.getCovariance().isApprox(covarBefore));
    EXPECT_FALSE(algo.getLastRateResiduals().valid);
}

// With the measurement covariance R >> the state covariance P, the rate update is nearly
// uninformative: the state barely moves and the post-fit residual approximates the pre-fit.
TEST(SunlineFilterAlgorithmMeasurementUpdate, HighMeasurementNoiseLeavesStateNearlyUnchanged) {
    SunlineFilterAlgorithm algo(rateOnlyConfig(makeState(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d::Zero(), 1.0),
                                               diagCovariance(1E-2, 1E-2, 1E-2)));
    EXPECT_TRUE(algo.timeUpdate(0.0));
    State const before = algo.getState();

    RateMeasurement r;
    r.timeTag = 0.0;
    r.omega_BN_B = Eigen::Vector3d(0.5, 0.0, 0.0);  // far from the prior
    r.covar = 1E8 * Eigen::Matrix3d::Identity();    // R >> P
    r.valid = true;
    EXPECT_TRUE(algo.measurementUpdate(r));

    EXPECT_LT((algo.getState().raw() - before.raw()).norm(), 1E-5) << "state moved despite R >> P";
    RateResidualsOutput const res = algo.getLastRateResiduals();
    EXPECT_TRUE(res.postFit.isApprox(res.preFit, 1E-4)) << "postFit should approx preFit when R >> P";
}

// ============================================================================
// Measurement packing + application (through the public update() path) and
// residual recording.
// ============================================================================

// An informative rate update pulls the estimate toward the measurement, so the post-fit
// residual is smaller than the pre-fit.
TEST(SunlineFilterAlgorithmMeasurements, InformativeMeasurementReducesResidual) {
    SunlineFilterAlgorithm algo(rateOnlyConfig(makeState(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d::Zero(), 1.0),
                                               diagCovariance(1E-2, 1E-1, 1E-2)));
    EXPECT_TRUE(algo.timeUpdate(0.0));

    RateMeasurement r;
    r.timeTag = 0.0;
    r.omega_BN_B = Eigen::Vector3d(0.05, -0.05, 0.05);
    r.covar = (1E-3 * 1E-3) * Eigen::Matrix3d::Identity();
    r.valid = true;
    EXPECT_TRUE(algo.measurementUpdate(r));

    RateResidualsOutput const res = algo.getLastRateResiduals();
    EXPECT_TRUE(res.valid);
    EXPECT_LT(res.postFit.norm(), res.preFit.norm());
}

// A rate reading fires a residual only when fresh (timeTag > 0); a stale/empty reading is
// dropped and the recorded observation echoes the input rate.
TEST(SunlineFilterAlgorithmMeasurements, RateDataFiresResidualOnlyWhenFresh) {
    SunlineFilterAlgorithm algo(rateOnlyConfig(makeState(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d::Zero(), 1.0),
                                               diagCovariance(1E-2, 1E-1, 1E-2)));

    SunlineFilterOutput const stale = algo.update(1.0, CssData{}, RateData{});
    EXPECT_FALSE(stale.rateResiduals.valid);

    RateData rate;
    rate.timeTag = 2.0;
    rate.rate = Eigen::Vector3d(0.02, -0.01, 0.015);
    SunlineFilterOutput const fresh = algo.update(2.0, CssData{}, rate);
    EXPECT_TRUE(fresh.rateResiduals.valid);
    EXPECT_TRUE(fresh.rateResiduals.observation.isApprox(rate.rate, 1E-12));
}

// A CSS reading fires a residual only when fresh (timeTag > 0); a stale/empty reading is dropped.
TEST(SunlineFilterAlgorithmMeasurements, CssDataFiresResidualOnlyWhenFresh) {
    SunlineFilterAlgorithm algo(threeCssConfig(
        makeState(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d::Zero(), 1.0), diagCovariance(1E-2, 1E-2, 1E-1), 0.0));

    SunlineFilterOutput const stale = algo.update(1.0, CssData{}, RateData{});
    EXPECT_FALSE(stale.cssResiduals.valid);

    CssData css;
    css.timeTag = 2.0;
    css.cosValues(0) = 0.5;
    css.cosValues(1) = 0.5;
    css.cosValues(2) = 0.707;
    SunlineFilterOutput const fresh = algo.update(2.0, css, RateData{});
    EXPECT_TRUE(fresh.cssResiduals.valid);
    EXPECT_EQ(fresh.cssResiduals.numberOfActiveCss, 3);
}

// Packing threads the configured gyro-noise std into the measurement covariance: a larger std
// makes the rate update less informative, so the rate block of the covariance shrinks less.
TEST(SunlineFilterAlgorithmMeasurements, LargerMeasurementNoiseStdShrinksCovarianceLess) {
    auto const rateBlockTrace = [](Matrix7 const& P) { return P(3, 3) + P(4, 4) + P(5, 5); };

    ConfigInputs base;
    base.initialState = makeState(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d::Zero(), 1.0);
    base.initialCovariance = diagCovariance(1E-2, 1E-1, 1E-2);

    RateData rate;
    rate.timeTag = 1.0;
    rate.rate = Eigen::Vector3d(0.01, 0.0, 0.0);

    ConfigInputs sharpIn = base;
    sharpIn.gyroStd = 1E-3;
    SunlineFilterAlgorithm sharp(buildConfig(sharpIn));
    sharp.update(1.0, CssData{}, rate);

    ConfigInputs looseIn = base;
    looseIn.gyroStd = 1E-1;
    SunlineFilterAlgorithm loose(buildConfig(looseIn));
    loose.update(1.0, CssData{}, rate);

    EXPECT_LT(rateBlockTrace(sharp.getCovariance()), rateBlockTrace(loose.getCovariance()));
    EXPECT_LT(rateBlockTrace(loose.getCovariance()), rateBlockTrace(base.initialCovariance));
}

// Queue-driven path: both per-kind residual slots in the returned
// SunlineFilterOutput end valid when both kinds carry data — the regression case
// that motivated splitting Residuals into per-kind storage.
TEST(SunlineFilterAlgorithmUpdate, UpdateWithRateAndCssExposesBothResiduals) {
    Eigen::Vector3d const sHat0 = Eigen::Vector3d(0.0, 0.0, 1.0);
    SunlineFilterAlgorithm algo(
        threeCssConfig(makeState(sHat0, Eigen::Vector3d(0.01, 0.0, 0.0), 1.0), diagCovariance(1E-2, 1E-2, 1E-1), 0.0));

    CssData css;
    css.timeTag = 1.0;
    css.cosValues(0) = 0.5;
    css.cosValues(1) = 0.5;
    css.cosValues(2) = 0.707;

    RateData rate;
    rate.timeTag = 1.0;
    rate.rate = Eigen::Vector3d(0.01, 0.0, 0.0);

    SunlineFilterOutput const out = algo.update(2.0, css, rate);

    EXPECT_EQ(out.cssResiduals.numberOfActiveCss, 3);
}

// CSS readings all below the configured threshold must produce a `valid = false`
// CSS residual (the pack method's threshold gate), while the rate channel —
// which has no threshold — still fires.
TEST(SunlineFilterAlgorithmUpdate, CssBelowThresholdNotProcessed) {
    SunlineFilterAlgorithm algo(threeCssConfig(
        makeState(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0.01, 0, 0), 1.0), diagCovariance(1E-2, 1E-2, 1E-1), 0.5));

    CssData css;
    css.timeTag = 1.0;
    css.cosValues(0) = 0.1;  // below threshold
    css.cosValues(1) = 0.2;
    css.cosValues(2) = 0.3;

    RateData rate;
    rate.timeTag = 1.0;
    rate.rate = Eigen::Vector3d(0.01, 0.0, 0.0);

    SunlineFilterOutput const out = algo.update(2.0, css, rate);

    EXPECT_EQ(out.cssResiduals.numberOfActiveCss, 0);
}

// Through the queue-driven update(), a bad CSS reading is rejected by applySequentialRobust
// (which calls clear()) and the filter recovers: the residual is invalid, the state stays finite,
// and a normal update right after is processed cleanly.
TEST(SunlineFilterAlgorithmUpdate, BadMeasurementIsRejectedAndFilterRecovers) {
    SunlineFilterAlgorithm algo(threeCssConfig(
        makeState(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0.01, 0, 0), 1.0), diagCovariance(1E-1, 1E-2, 1E-1), 0.0));

    // A good update first, to establish a finite anchor.
    CssData good;
    good.timeTag = 1.0;
    good.cosValues(0) = 0.5;
    good.cosValues(1) = 0.5;
    good.cosValues(2) = 0.707;
    algo.update(1.0, good, RateData{});

    // A NaN CSS reading must not corrupt the filter.
    CssData bad;
    bad.timeTag = 2.0;
    bad.cosValues(0) = std::numeric_limits<double>::quiet_NaN();
    bad.cosValues(1) = 0.5;
    bad.cosValues(2) = 0.707;
    SunlineFilterOutput const out = algo.update(2.0, bad, RateData{});

    EXPECT_FALSE(out.cssResiduals.valid) << "bad measurement must not produce a valid residual";
    EXPECT_TRUE(algo.getState().raw().allFinite()) << "filter state must stay finite after a bad update";
    EXPECT_TRUE(algo.getCovariance().allFinite()) << "filter covariance must stay finite after a bad update";

    // A normal update right after the bad one must be processed cleanly -- confirming clear() left
    // no NaNs behind: the residual is valid and the state/residuals stay finite.
    CssData recover;
    recover.timeTag = 3.0;
    recover.cosValues(0) = 0.5;
    recover.cosValues(1) = 0.5;
    recover.cosValues(2) = 0.707;
    SunlineFilterOutput const recovered = algo.update(3.0, recover, RateData{});

    EXPECT_TRUE(recovered.cssResiduals.valid) << "the post-recovery measurement must be applied";
    EXPECT_TRUE(recovered.cssResiduals.postFit.allFinite()) << "residuals must be finite after recovery";
    EXPECT_TRUE(algo.getState().raw().allFinite()) << "state must be finite after recovery";
}

// Each update() with empty measurements re-propagates from the unchanged anchor to
// currentSeconds, so with process noise the covariance trace grows with currentSeconds.
TEST(SunlineFilterAlgorithmUpdate, WithoutMeasurementsGrowsCovarianceMonotonically) {
    Matrix7 const P0 = diagCovariance(1E-2, 1E-3, 1E-2);
    SunlineFilterAlgorithm algo(rateOnlyConfigWithProcessNoise(
        makeState(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d::Zero(), 1.0), P0, Matrix7::Identity() * 1E-3));

    algo.update(1.0, CssData{}, RateData{});
    double const trace1 = algo.getCovariance().trace();
    algo.update(4.0, CssData{}, RateData{});
    double const trace4 = algo.getCovariance().trace();
    algo.update(9.0, CssData{}, RateData{});
    double const trace9 = algo.getCovariance().trace();

    EXPECT_GT(trace1, P0.trace());
    EXPECT_GT(trace4, trace1);
    EXPECT_GT(trace9, trace4);
}

TEST(SunlineFilterAlgorithmReInit, ReInitializePreservesEstimateReInitializeAllResetsIt) {
    SunlineFilterAlgorithm algo(threeCssConfig(
        makeState(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0.01, 0, 0), 1.0), diagCovariance(1E-2, 1E-2, 1E-1), 0.0));

    State const initialState = algo.getState();
    Matrix7 const initialCovariance = algo.getCovariance();

    CssData css;
    css.timeTag = 1.0;
    css.cosValues(0) = 0.5;
    css.cosValues(1) = 0.5;
    css.cosValues(2) = 0.707;
    RateData rate;
    rate.timeTag = 1.0;
    rate.rate = Eigen::Vector3d(0.01, 0.0, 0.0);
    algo.update(2.0, css, rate);

    State const movedState = algo.getState();
    Matrix7 const movedCovariance = algo.getCovariance();
    ASSERT_FALSE(movedCovariance.isApprox(initialCovariance));
    EXPECT_TRUE(algo.getLastCssResiduals().valid);

    algo.reInitializeExceptPersistentStates();
    EXPECT_TRUE(algo.getState().raw().isApprox(movedState.raw()));
    EXPECT_TRUE(algo.getCovariance().isApprox(movedCovariance));
    EXPECT_FALSE(algo.getLastCssResiduals().valid);

    algo.reInitialize();
    EXPECT_TRUE(algo.getState().raw().isApprox(initialState.raw()));
    EXPECT_TRUE(algo.getCovariance().isApprox(initialCovariance));
}

// A filter reconfigured to a larger process noise must behave like one constructed with it.
TEST(SunlineFilterAlgorithmReInit, SetConfigReDerivesProcessNoise) {
    State const initial = makeState(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d::Zero(), 1.0);
    Matrix7 const P0 = diagCovariance(1E-2, 1E-2, 1E-1);
    Matrix7 const smallProcessNoise = Matrix7::Identity() * 1E-12;
    Matrix7 const largeProcessNoise = Matrix7::Identity() * 1E-2;
    constexpr double dt = 1.0;

    SunlineFilterAlgorithm reference(rateOnlyConfigWithProcessNoise(initial, P0, largeProcessNoise));
    reference.timeUpdate(dt);

    SunlineFilterAlgorithm smallOnly(rateOnlyConfigWithProcessNoise(initial, P0, smallProcessNoise));
    smallOnly.timeUpdate(dt);
    ASSERT_FALSE(smallOnly.getCovariance().isApprox(reference.getCovariance()));

    SunlineFilterAlgorithm algo(rateOnlyConfigWithProcessNoise(initial, P0, smallProcessNoise));
    algo.setConfig(rateOnlyConfigWithProcessNoise(initial, P0, largeProcessNoise));
    algo.timeUpdate(dt);
    EXPECT_TRUE(algo.getCovariance().isApprox(reference.getCovariance()));
}

// ============================================================================
// Regression: a fixed, deterministic scenario. With three CSS boresights
// observing a constant sun direction (scale factor 1, so cos_i = nHat_i · sTrue)
// and a zero body rate, repeated updates must drive the heading estimate from a
// wrong initial guess to the true sun direction.
// ============================================================================

TEST(SunlineFilterAlgorithmRegression, ConvergesToConstantSunDirection) {
    Eigen::Matrix<double, MaxCss, 3> const nHat = threeCssNHat();
    Eigen::Vector3d const sTrue = Eigen::Vector3d(0.2, -0.1, 1.0).normalized();

    // Start the estimate away from the truth.
    Eigen::Vector3d const sGuess = Eigen::Vector3d(-0.3, 0.4, 0.85).normalized();
    SunlineFilterAlgorithm algo(
        threeCssConfig(makeState(sGuess, Eigen::Vector3d::Zero(), 1.0), diagCovariance(3E-1, 1E-3, 1E-2), 0.0));

    double const angleBefore = std::acos(std::clamp(sGuess.dot(sTrue), -1.0, 1.0));

    // Noise-free CSS cos-values consistent with the true sun direction.
    CssData css;
    css.timeTag = 0.0;
    for (int i = 0; i < 3; ++i) {
        css.cosValues(i) = nHat.row(i).dot(sTrue.transpose());
    }
    RateData const noRate;  // timeTag 0 -> not enqueued; pure CSS updates

    double t = 0.0;
    for (int step = 0; step < 40; ++step) {
        t += 1.0;
        css.timeTag = t;
        algo.update(t, css, noRate);
    }

    Eigen::Vector3d const sEst = algo.getState().get<filtering::Position<3>>();
    double const angleAfter = std::acos(std::clamp(sEst.dot(sTrue), -1.0, 1.0));

    EXPECT_LT(angleAfter, angleBefore);
    EXPECT_LT(angleAfter, 1E-2) << "estimate did not converge to the true sun direction";
    EXPECT_NEAR(sEst.norm(), 1.0, 1E-6);
}

// ============================================================================
// SRuKF static helpers: numerical helpers.
// ============================================================================

TEST(SrukfDetail, ForwardSubstitutionSolvesLowerTriangular) {
    Eigen::Matrix3d L;
    L << 2.0, 0.0, 0.0, 1.0, 3.0, 0.0, 0.5, 1.0, 4.0;
    Eigen::Vector3d const xTruth(1.0, 2.0, 3.0);
    Eigen::Matrix<double, 3, 1> const b = L * xTruth;

    Eigen::Matrix<double, 3, 1> const x = SRuKF::forwardSubstitution<3, 1>(L, b);
    EXPECT_TRUE(x.col(0).isApprox(xTruth, 1E-12));
}

TEST(SrukfDetail, BackSubstitutionSolvesUpperTriangular) {
    Eigen::Matrix3d U;
    U << 2.0, 1.0, 0.5, 0.0, 3.0, 1.0, 0.0, 0.0, 4.0;
    Eigen::Vector3d const xTruth(1.0, 2.0, 3.0);
    Eigen::Matrix<double, 3, 1> const b = U * xTruth;

    Eigen::Matrix<double, 3, 1> const x = SRuKF::backSubstitution<3, 1>(U, b);
    EXPECT_TRUE(x.col(0).isApprox(xTruth, 1E-12));
}

TEST(SrukfDetail, CholeskyDecompositionReconstructsP) {
    Eigen::Matrix3d P;
    P << 4.0, 2.0, 0.5, 2.0, 5.0, 1.0, 0.5, 1.0, 6.0;
    Eigen::Matrix3d const L = SRuKF::choleskyDecomposition<3>(P);
    EXPECT_TRUE((L * L.transpose()).isApprox(P, 1E-10));

    // Returned factor should be lower-triangular.
    for (int i = 0; i < L.rows(); ++i) {
        for (int j = i + 1; j < L.cols(); ++j) {
            EXPECT_NEAR(L(i, j), 0.0, 1E-12) << "(" << i << "," << j << ")";
        }
    }
}

// SRUKF feeds qrDecompositionJustR a wider-than-tall A and expects back a
// square N×N factor such that R * R^T == A * A^T. The function transposes its
// internal R, so the returned matrix is lower-triangular.
TEST(SrukfDetail, QrDecompositionJustRPreservesNormalEquations) {
    Eigen::Matrix<double, 3, 9> A;
    A << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 2.0, -1.0, 0.0, 1.0,
        2.0, 3.0, 4.0, 5.0, 6.0;

    Eigen::Matrix3d const R = SRuKF::qrDecompositionJustR<3, 9>(A);

    for (int i = 0; i < R.rows(); ++i) {
        for (int j = i + 1; j < R.cols(); ++j) {
            EXPECT_NEAR(R(i, j), 0.0, 1E-10) << "(" << i << "," << j << ")";
        }
    }
    EXPECT_TRUE((R * R.transpose()).isApprox(A * A.transpose(), 1E-9));
}

TEST(SrukfDetail, CholeskyUpDownDateMatchesExplicitUpdate) {
    Eigen::Matrix3d P0;
    P0 << 4.0, 1.0, 0.0, 1.0, 3.0, 0.5, 0.0, 0.5, 2.0;
    Eigen::Matrix3d const S0 = SRuKF::choleskyDecomposition<3>(P0);

    Eigen::Vector3d const v(0.1, -0.2, 0.3);

    // Up-date with +coef and down-date with -coef must reconstruct P0 ± coef v vᵀ.
    {
        double const coef = 0.5;
        Eigen::Matrix3d const S1 = SRuKF::choleskyUpDownDate<3>(S0, v, coef);
        Eigen::Matrix3d const P1 = P0 + coef * v * v.transpose();
        EXPECT_TRUE((S1 * S1.transpose()).isApprox(P1, 1E-9));
    }
    {
        double const coef = -0.5;
        Eigen::Matrix3d const S1 = SRuKF::choleskyUpDownDate<3>(S0, v, coef);
        Eigen::Matrix3d const P1 = P0 - 0.5 * v * v.transpose();
        EXPECT_TRUE((S1 * S1.transpose()).isApprox(P1, 1E-9));
    }
}

// ============================================================================
// SunlineFilterConfig: factory validation, static validators, and round-trip.
// ============================================================================

TEST(SunlineFilterConfig, ValidInputsDoNotThrow) { EXPECT_NO_THROW(buildConfig({})); }

TEST(SunlineFilterConfig, RejectsAlphaOutsideOpenUnitInterval) {
    for (double bad : {0.0, 1.0, -0.1, 1.5}) {  // (0, 1) open interval: endpoints excluded
        ConfigInputs in;
        in.alpha = bad;
        EXPECT_THROW(buildConfig(in), fsw::invalid_argument) << "alpha=" << bad;
    }
}

TEST(SunlineFilterConfig, RejectsBetaOutsideRange) {
    for (double bad : {-0.1, 2.5}) {  // [0, 2] closed interval
        ConfigInputs in;
        in.beta = bad;
        EXPECT_THROW(buildConfig(in), fsw::invalid_argument) << "beta=" << bad;
    }
}

TEST(SunlineFilterConfig, RejectsNonPositiveSemiDefiniteProcessNoise) {
    ConfigInputs in;
    in.processNoise = -Matrix7::Identity();  // negative definite
    EXPECT_THROW(buildConfig(in), fsw::invalid_argument);
}

TEST(SunlineFilterConfig, RejectsNonPositiveSemiDefiniteCovariance) {
    ConfigInputs in;
    in.initialCovariance = -Matrix7::Identity();  // negative definite
    EXPECT_THROW(buildConfig(in), fsw::invalid_argument);
}

TEST(SunlineFilterConfig, RejectsNonPositiveBiasBounds) {
    ConfigInputs lower;
    lower.biasLowerBound = 0.0;
    EXPECT_THROW(buildConfig(lower), fsw::invalid_argument);

    ConfigInputs upper;
    upper.biasUpperBound = -1.0;
    EXPECT_THROW(buildConfig(upper), fsw::invalid_argument);
}

TEST(SunlineFilterConfig, RejectsNonUnitCssNHat) {
    ConfigInputs in;
    in.cssNHat.row(0) = Eigen::RowVector3d(0.5, 0.0, 0.0);  // norm 0.5, outside 1e-3 of unit
    EXPECT_THROW(buildConfig(in), fsw::invalid_argument);
}

TEST(SunlineFilterConfig, RejectsNegativeCssScaleFactor) {
    ConfigInputs in;
    in.cssScaleFactor(1) = -0.1;
    EXPECT_THROW(buildConfig(in), fsw::invalid_argument);
}

TEST(SunlineFilterConfig, RejectsNumberOfCssOutOfRange) {
    ConfigInputs zero;
    zero.numberOfCss = 0;
    EXPECT_THROW(buildConfig(zero), fsw::invalid_argument);

    ConfigInputs tooMany;
    tooMany.numberOfCss = MaxCss + 1;
    EXPECT_THROW(buildConfig(tooMany), fsw::invalid_argument);
}

TEST(SunlineFilterConfig, RejectsNegativeThresholdAndNoiseStds) {
    ConfigInputs threshold;
    threshold.sensorThreshold = -1E-3;
    EXPECT_THROW(buildConfig(threshold), fsw::invalid_argument);

    ConfigInputs cssStd;
    cssStd.cssMeasStd = -1E-3;
    EXPECT_THROW(buildConfig(cssStd), fsw::invalid_argument);

    ConfigInputs gyroStd;
    gyroStd.gyroStd = -1E-3;
    EXPECT_THROW(buildConfig(gyroStd), fsw::invalid_argument);
}

TEST(SunlineFilterConfig, RejectsBiasLowerBoundNotLessThanUpper) {
    ConfigInputs equal;
    equal.biasLowerBound = 1.0;
    equal.biasUpperBound = 1.0;
    EXPECT_THROW(buildConfig(equal), fsw::invalid_argument);

    ConfigInputs inverted;
    inverted.biasLowerBound = 1.5;
    inverted.biasUpperBound = 0.5;
    EXPECT_THROW(buildConfig(inverted), fsw::invalid_argument);
}

TEST(SunlineFilterConfig, RejectsMissingLeadingCssNHat) {
    ConfigInputs in;
    in.cssNHat = Eigen::Matrix<double, MaxCss, 3>::Zero();
    in.cssNHat.row(0) = Eigen::RowVector3d(1.0, 0.0, 0.0);
    in.cssNHat.row(1) = Eigen::RowVector3d(0.0, 1.0, 0.0);
    in.numberOfCss = 3;
    EXPECT_THROW(buildConfig(in), fsw::invalid_argument);
}

TEST(SunlineFilterConfig, IgnoresCssNHatRowsBeyondNumberOfCss) {
    ConfigInputs in;
    in.numberOfCss = 2;
    EXPECT_NO_THROW(buildConfig(in));
}

TEST(SunlineFilterConfig, StaticValidatorsCheckBoundaries) {
    EXPECT_TRUE(SunlineFilterConfig::isValidBiasBounds(0.5, 1.5));
    EXPECT_FALSE(SunlineFilterConfig::isValidBiasBounds(1.5, 0.5));
    EXPECT_FALSE(SunlineFilterConfig::isValidBiasBounds(1.0, 1.0));
    EXPECT_TRUE(SunlineFilterConfig::isValidCssNHat(threeCssNHat(), 3));
    EXPECT_FALSE(SunlineFilterConfig::isValidCssNHat(threeCssNHat(), 4));  // row 3 is zero
    EXPECT_TRUE(SunlineFilterConfig::isValidBiasLowerBound(1E-9));
    EXPECT_FALSE(SunlineFilterConfig::isValidBiasLowerBound(0.0));
    EXPECT_TRUE(SunlineFilterConfig::isValidSensorThreshold(0.0));
    EXPECT_FALSE(SunlineFilterConfig::isValidSensorThreshold(-1E-9));
    EXPECT_TRUE(SunlineFilterConfig::isValidCssMeasurementNoiseStd(0.0));
    EXPECT_FALSE(SunlineFilterConfig::isValidGyroMeasurementNoiseStd(-1E-9));
    EXPECT_FALSE(SunlineFilterConfig::isValidNumberOfCss(0));
    EXPECT_TRUE(SunlineFilterConfig::isValidNumberOfCss(1));
    EXPECT_TRUE(SunlineFilterConfig::isValidNumberOfCss(MaxCss));
    EXPECT_FALSE(SunlineFilterConfig::isValidNumberOfCss(MaxCss + 1));
    EXPECT_TRUE(SunlineFilterConfig::isValidProcessNoise(Matrix7::Identity()));
    EXPECT_FALSE(SunlineFilterConfig::isValidProcessNoise(-Matrix7::Identity()));
    EXPECT_TRUE(SunlineFilterConfig::isValidInitialCovariance(Matrix7::Identity()));
    EXPECT_FALSE(SunlineFilterConfig::isValidInitialCovariance(-Matrix7::Identity()));
}

TEST(SunlineFilterConfig, GettersRoundTripAndNormalizeNHat) {
    SunlineFilterConfig const cfg = buildConfig({});
    EXPECT_DOUBLE_EQ(cfg.getAlpha(), kAlpha);
    EXPECT_DOUBLE_EQ(cfg.getBeta(), kBeta);
    EXPECT_DOUBLE_EQ(cfg.getBiasLowerBound(), kBiasLowerBound);
    EXPECT_DOUBLE_EQ(cfg.getBiasUpperBound(), kBiasUpperBound);
    EXPECT_EQ(cfg.getNumberOfCss(), 3);
    EXPECT_DOUBLE_EQ(cfg.getCssMeasurementNoiseStd(), 1E-2);
    EXPECT_DOUBLE_EQ(cfg.getGyroMeasurementNoiseStd(), 1E-3);
    // Active boresights are stored normalized; unused rows stay zero.
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(cfg.getCssNHat().row(i).norm(), 1.0, 1E-12);
    }
    EXPECT_TRUE(cfg.getCssNHat().row(3).isZero());
}

TEST(SunlineFilterConfig, SetConfigSwapsConfiguration) {
    SunlineFilterAlgorithm algo(buildConfig({}));
    ConfigInputs other;
    other.numberOfCss = 2;
    EXPECT_NO_THROW(algo.setConfig(buildConfig(other)));
}

}  // namespace filtering::sunlineFilter
