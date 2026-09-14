// Unit tests for FlybyFilterAlgorithm (angles-only two-body flyby SRuKF on filteringCore).
//
// Sections (grouped simplest-first):
//   * Config: factory validation, static validators, getter round-trips.
//   * Lifecycle: construction seeds state/covariance; reInitializeExceptPersistentStates /
//     reInitialize / setConfig / clear().
//   * Output: getFilterOutput() and the snapshot returned by update() agree with the accessors.
//   * Scheduler: freshness gating, stale drops, delayed-but-newer measurements, rollback after a
//     bad update, and the BatchSize == 1 queue limit.
//   * Dynamics: two-body point-mass gravity r_dot = v, v_dot = -mu/|r|^3 r.
//   * timeUpdate(): zero-dt no-op; propagation matches filtering::propagate; covariance growth under Q.
//   * measurementUpdate(): heading update shrinks covariance (symmetric + PSD); high-noise limit;
//     bad-measurement rejection (handled inside the SRuKF).
//   * Measurements through update(): residual freshness and contents, measurement-noise sensitivity,
//     monotone covariance growth without measurements.
//   * Degenerate geometry: both the dynamics and the heading model divide by |r|.
//   * SRuKF static helpers: forward/back substitution, Cholesky, QR-just-R, Cholesky up/down-date.
//   * Convergence: angles-only heading measurements along a propagated two-body arc.
//
// All quantities are in the filter's internal units (km, km/s); the adapter handles SI<->internal.

#include "flybyFilterTestHelpers.hpp"

#include <filteringCore/dynamicsModel.hpp>
#include <filteringCore/srukf.hpp>

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <cmath>
#include <limits>
#include <memory>
#include <random>

namespace filtering::flybyFilter {
namespace {

using State = FlybyFilterAlgorithm::State;

// Alias for invoking the numerical helpers (static methods on the SRuKF class template). The
// State/Dynamics arguments don't affect the helpers' behavior; any valid instantiation works.
using SRuKF = ::filtering::SRuKF<FlybyState, FlybyDynamics>;

}  // namespace

// ============================================================================
// Config: factory validation, static validators, getter round-trips.
// ============================================================================

TEST(FlybyFilterConfig, ValidInputsDoNotThrow) { EXPECT_NO_THROW(buildConfig({})); }

TEST(FlybyFilterConfig, RejectsNonFiniteInitialState) {
    ConfigInputs in;
    in.initialState =
        makeState(Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0), Eigen::Vector3d::Zero());
    EXPECT_THROW(buildConfig(in), fsw::invalid_argument);
}

TEST(FlybyFilterConfig, RejectsAlphaOutsideHalfOpenUnitInterval) {
    for (double bad : {0.0, -0.1, 1.5}) {  // (0, 1]: zero and anything above one are rejected
        ConfigInputs in;
        in.alpha = bad;
        EXPECT_THROW(buildConfig(in), fsw::invalid_argument) << "alpha=" << bad;
    }

    // alpha = 1 is the inclusive upper bound: the sigma points sit one standard deviation out.
    ConfigInputs unitAlpha;
    unitAlpha.alpha = 1.0;
    EXPECT_NO_THROW(buildConfig(unitAlpha));
}

TEST(FlybyFilterConfig, RejectsBetaOutsideRange) {
    for (double bad : {-0.1, 2.5}) {  // [0, 2] closed interval
        ConfigInputs in;
        in.beta = bad;
        EXPECT_THROW(buildConfig(in), fsw::invalid_argument) << "beta=" << bad;
    }
}

TEST(FlybyFilterConfig, RejectsNonPositiveMu) {
    for (double bad : {0.0, -1.0}) {
        ConfigInputs in;
        in.mu = bad;
        EXPECT_THROW(buildConfig(in), fsw::invalid_argument) << "mu=" << bad;
    }
}

TEST(FlybyFilterConfig, RejectsNonPositiveSemiDefiniteProcessNoise) {
    ConfigInputs in;
    in.processNoise = -Matrix6::Identity();
    EXPECT_THROW(buildConfig(in), fsw::invalid_argument);
}

TEST(FlybyFilterConfig, RejectsNonPositiveSemiDefiniteCovariance) {
    ConfigInputs in;
    in.initialCovariance = -Matrix6::Identity();
    EXPECT_THROW(buildConfig(in), fsw::invalid_argument);
}

TEST(FlybyFilterConfig, RejectsNegativeHeadingNoiseStd) {
    ConfigInputs in;
    in.headingStd = -1E-3;
    EXPECT_THROW(buildConfig(in), fsw::invalid_argument);
}

TEST(FlybyFilterConfig, AcceptsZeroHeadingNoiseStd) {
    ConfigInputs in;
    in.headingStd = 0.0;
    EXPECT_NO_THROW(buildConfig(in));
}

TEST(FlybyFilterConfig, StaticValidatorsCheckBoundaries) {
    EXPECT_TRUE(FlybyFilterConfig::isValidMu(1.0));
    EXPECT_FALSE(FlybyFilterConfig::isValidMu(0.0));
    EXPECT_TRUE(FlybyFilterConfig::isValidHeadingMeasurementNoiseStd(0.0));
    EXPECT_FALSE(FlybyFilterConfig::isValidHeadingMeasurementNoiseStd(-1E-9));
    EXPECT_TRUE(FlybyFilterConfig::isValidProcessNoise(Matrix6::Identity()));
    EXPECT_FALSE(FlybyFilterConfig::isValidProcessNoise(-Matrix6::Identity()));
    EXPECT_TRUE(FlybyFilterConfig::isValidInitialCovariance(Matrix6::Identity()));
    EXPECT_FALSE(FlybyFilterConfig::isValidInitialCovariance(-Matrix6::Identity()));
}

TEST(FlybyFilterConfig, GettersRoundTrip) {
    ConfigInputs in;
    in.processNoise = Matrix6::Identity() * 3E-4;
    in.initialCovariance = diagCovariance(200.0, 0.2);
    in.initialState = makeState({3100.0, 900.0, 450.0}, {1.1, -1.9, 0.4});
    FlybyFilterConfig const cfg = buildConfig(in);

    EXPECT_DOUBLE_EQ(cfg.getAlpha(), kAlpha);
    EXPECT_DOUBLE_EQ(cfg.getBeta(), kBeta);
    EXPECT_DOUBLE_EQ(cfg.getMu(), kMu);
    EXPECT_DOUBLE_EQ(cfg.getHeadingMeasurementNoiseStd(), kHeadingStd);
    EXPECT_TRUE(cfg.getProcessNoise().isApprox(in.processNoise, 1E-12));
    EXPECT_TRUE(cfg.getInitialCovariance().isApprox(in.initialCovariance, 1E-12));
    EXPECT_TRUE(cfg.getInitialState().raw().isApprox(in.initialState.raw(), 1E-12));
}

// ============================================================================
// Lifecycle: construction seeds the filter; reInitializeExceptPersistentStates / reInitialize / setConfig.
// ============================================================================

TEST(FlybyFilterAlgorithmLifecycle, ConstructorSeedsStateAndCovarianceFromConfig) {
    State const initial = nominalTruth();
    Matrix6 const P0 = diagCovariance(100.0, 0.1);
    FlybyFilterAlgorithm algo(baseConfig(initial, P0));

    EXPECT_TRUE(algo.getState().raw().isApprox(initial.raw(), 1E-9));
    EXPECT_TRUE(algo.getCovariance().isApprox(P0, 1E-9));
}

TEST(FlybyFilterAlgorithmLifecycle, ReInitializeExceptPersistentStatesPreservesEstimateReInitializeResetsIt) {
    State const initial = nominalTruth();
    Matrix6 const P0 = diagCovariance(100.0, 0.1);
    FlybyFilterAlgorithm algo(baseConfig(initial, P0));

    HeadingData heading;
    heading.timeTag = 10.0;
    heading.rhat_BN_N = headingOf(initial);
    algo.update(20.0, heading);

    State const movedState = algo.getState();
    Matrix6 const movedCovariance = algo.getCovariance();
    ASSERT_FALSE(movedCovariance.isApprox(P0));
    EXPECT_TRUE(algo.getLastHeadingResiduals().valid);

    algo.reInitializeExceptPersistentStates();
    EXPECT_TRUE(algo.getState().raw().isApprox(movedState.raw()));
    EXPECT_TRUE(algo.getCovariance().isApprox(movedCovariance));
    EXPECT_FALSE(algo.getLastHeadingResiduals().valid);

    algo.reInitialize();
    EXPECT_TRUE(algo.getState().raw().isApprox(initial.raw(), 1E-9));
    EXPECT_TRUE(algo.getCovariance().isApprox(P0, 1E-9));
}

TEST(FlybyFilterAlgorithmLifecycle, SetConfigReDerivesMu) {
    State const initial = nominalTruth();
    Matrix6 const P0 = diagCovariance(100.0, 0.1);
    constexpr double dt = 30.0;

    // A reference filter using a larger mu propagates to a different state.
    FlybyFilterAlgorithm reference(
        FlybyFilterConfig::create(kAlpha, kBeta, 2.0 * kMu, smallProcessNoise(), initial, P0, kHeadingStd));
    EXPECT_TRUE(reference.timeUpdate(dt));

    FlybyFilterAlgorithm algo(baseConfig(initial, P0));
    algo.setConfig(FlybyFilterConfig::create(kAlpha, kBeta, 2.0 * kMu, smallProcessNoise(), initial, P0, kHeadingStd));
    EXPECT_TRUE(algo.timeUpdate(dt));
    EXPECT_TRUE(algo.getState().raw().isApprox(reference.getState().raw(), 1E-9));
}

TEST(FlybyFilterAlgorithmLifecycle, SetConfigReDerivesProcessNoise) {
    TestState const initial = nominalTruth();
    Matrix6 const P0 = diagCovariance(100.0, 0.1);
    Matrix6 const smallQ = Matrix6::Identity() * 1E-12;
    Matrix6 const largeQ = Matrix6::Identity() * 1E-4;
    constexpr double dt = 10.0;

    FlybyFilterAlgorithm reference(configWithProcessNoise(initial, P0, largeQ));
    EXPECT_TRUE(reference.timeUpdate(dt));

    FlybyFilterAlgorithm smallOnly(configWithProcessNoise(initial, P0, smallQ));
    EXPECT_TRUE(smallOnly.timeUpdate(dt));
    ASSERT_FALSE(smallOnly.getCovariance().isApprox(reference.getCovariance()));

    FlybyFilterAlgorithm algo(configWithProcessNoise(initial, P0, smallQ));
    algo.setConfig(configWithProcessNoise(initial, P0, largeQ));
    EXPECT_TRUE(algo.timeUpdate(dt));
    EXPECT_TRUE(algo.getCovariance().isApprox(reference.getCovariance()));
}

TEST(FlybyFilterAlgorithmLifecycle, SetConfigPreservesTheCurrentEstimate) {
    TestState const initial = nominalTruth();
    Matrix6 const P0 = diagCovariance(100.0, 0.1);
    FlybyFilterAlgorithm algo(baseConfig(initial, P0));

    // Move the estimate off its seed before swapping the configuration.
    ASSERT_TRUE(algo.timeUpdate(50.0));
    TestState const moved = algo.getState();
    Matrix6 const movedCovariance = algo.getCovariance();
    ASSERT_FALSE(moved.raw().isApprox(initial.raw()));

    // A config carrying a different seed must re-derive the filter parameters without disturbing
    // the running estimate -- only reInitialize() re-seeds.
    TestState const otherSeed = makeState({4000.0, -500.0, 250.0}, {0.5, -1.0, 0.25});
    algo.setConfig(baseConfig(otherSeed, diagCovariance(10.0, 0.01)));

    EXPECT_TRUE(algo.getState().raw().isApprox(moved.raw(), 1E-12)) << "setConfig must not re-seed the state";
    EXPECT_TRUE(algo.getCovariance().isApprox(movedCovariance, 1E-12)) << "setConfig must not re-seed the covariance";

    // ...and the new seed is what reInitialize() restores.
    algo.reInitialize();
    EXPECT_TRUE(algo.getState().raw().isApprox(otherSeed.raw(), 1E-9));
}

TEST(FlybyFilterAlgorithmLifecycle, ClearRevertsToTheLastMeasurementAnchor) {
    TestState const initial = nominalTruth();
    Matrix6 const P0 = diagCovariance(100.0, 0.1);
    FlybyFilterAlgorithm algo(baseConfig(initial, P0));

    // A measurement stamped at the call time leaves the filter sitting exactly on its anchor.
    HeadingData heading;
    heading.timeTag = 10.0;
    heading.rhat_BN_N = headingOf(initial);
    algo.update(10.0, heading);
    ASSERT_TRUE(algo.getLastHeadingResiduals().valid);

    TestState const anchor = algo.getState();
    Matrix6 const anchorCovariance = algo.getCovariance();

    ASSERT_TRUE(algo.timeUpdate(500.0));
    ASSERT_FALSE(algo.getState().raw().isApprox(anchor.raw())) << "propagation should move off the anchor";

    algo.clear();
    EXPECT_TRUE(algo.getState().raw().isApprox(anchor.raw(), 1E-12)) << "clear() must restore the last-good state";
    EXPECT_TRUE(algo.getCovariance().isApprox(anchorCovariance, 1E-12))
        << "clear() must restore the last-good covariance";
    EXPECT_FALSE(algo.getLastHeadingResiduals().valid) << "clear() must invalidate the residual snapshot";
}

// ============================================================================
// Output accessors: getFilterOutput() and the snapshot returned by update().
// ============================================================================

TEST(FlybyFilterAlgorithmOutput, GetFilterOutputMatchesTheStateAccessors) {
    TestState const initial = nominalTruth();
    FlybyFilterAlgorithm algo(baseConfig(initial, diagCovariance(100.0, 0.1)));

    FilterStateOutput const seeded = algo.getFilterOutput();
    EXPECT_TRUE(seeded.state.isApprox(algo.getState().raw(), 1E-12));
    EXPECT_TRUE(seeded.covariance.isApprox(algo.getCovariance(), 1E-12));

    ASSERT_TRUE(algo.timeUpdate(30.0));
    FilterStateOutput const propagated = algo.getFilterOutput();
    EXPECT_TRUE(propagated.state.isApprox(algo.getState().raw(), 1E-12));
    EXPECT_TRUE(propagated.covariance.isApprox(algo.getCovariance(), 1E-12));
}

TEST(FlybyFilterAlgorithmOutput, UpdateSnapshotMatchesTheAccessors) {
    TestState const initial = nominalTruth();
    FlybyFilterAlgorithm algo(baseConfig(initial, diagCovariance(100.0, 0.1)));

    HeadingData heading;
    heading.timeTag = 10.0;
    heading.rhat_BN_N = headingOf(initial);
    FlybyFilterOutput const out = algo.update(20.0, heading);

    EXPECT_TRUE(out.filterState.state.isApprox(algo.getState().raw(), 1E-12));
    EXPECT_TRUE(out.filterState.covariance.isApprox(algo.getCovariance(), 1E-12));

    HeadingResidualsOutput const& latest = algo.getLastHeadingResiduals();
    EXPECT_EQ(out.headingResiduals.valid, latest.valid);
    EXPECT_TRUE(out.headingResiduals.observation.isApprox(latest.observation, 1E-12));
    EXPECT_TRUE(out.headingResiduals.preFit.isApprox(latest.preFit, 1E-12));
    EXPECT_TRUE(out.headingResiduals.postFit.isApprox(latest.postFit, 1E-12));
}

// ============================================================================
// Scheduler: how update() drives applySequentialRobust -- freshness gating, stale drops,
// delayed-but-newer measurements, and rollback after a bad update.
// ============================================================================

//! Drive one measurement-free cycle and return the resulting state.
namespace {
TestState propagateOnly(TestState const& initial, Matrix6 const& P, double callTime) {
    FlybyFilterAlgorithm algo(baseConfig(initial, P));
    algo.update(callTime, HeadingData{});
    return algo.getState();
}
}  // namespace

TEST(FlybyFilterAlgorithmScheduler, NoMeasurementCyclePropagatesToTheCallTime) {
    TestState const initial = nominalTruth();
    Matrix6 const P0 = diagCovariance(100.0, 0.1);
    FlybyFilterAlgorithm algo(baseConfig(initial, P0));

    constexpr double callTime = 40.0;
    FlybyFilterOutput const out = algo.update(callTime, HeadingData{});

    TestState const expected = filtering::propagate(FlybyDynamics{kMu}, initial, {0.0, callTime});
    EXPECT_TRUE(algo.getState().raw().isApprox(expected.raw(), 1E-9))
        << "an empty cycle must still propagate to the call time";
    EXPECT_FALSE(out.headingResiduals.valid) << "no measurement fired, so no residual is reported";
}

TEST(FlybyFilterAlgorithmScheduler, NonPositiveTimeTagIsNotEnqueued) {
    TestState const initial = nominalTruth();
    Matrix6 const P0 = diagCovariance(100.0, 0.1);
    constexpr double callTime = 40.0;

    // timeTag <= 0 is the adapter's "no fresh reading this cycle" sentinel; both 0 and a negative
    // tag must behave exactly like an empty HeadingData.
    TestState const reference = propagateOnly(initial, P0, callTime);
    for (double staleTag : {0.0, -5.0}) {
        FlybyFilterAlgorithm algo(baseConfig(initial, P0));
        HeadingData heading;
        heading.timeTag = staleTag;
        heading.rhat_BN_N = Eigen::Vector3d(0.0, 0.0, 1.0);  // deliberately far from the prior
        FlybyFilterOutput const out = algo.update(callTime, heading);

        EXPECT_FALSE(out.headingResiduals.valid) << "timeTag=" << staleTag;
        EXPECT_TRUE(algo.getState().raw().isApprox(reference.raw(), 1E-12)) << "timeTag=" << staleTag;
    }
}

TEST(FlybyFilterAlgorithmScheduler, MeasurementOlderThanTheAnchorIsDropped) {
    TestState const initial = nominalTruth();
    Matrix6 const P0 = diagCovariance(100.0, 0.1);
    FlybyFilterAlgorithm algo(baseConfig(initial, P0));

    // Anchor the filter at t = 100 with a good measurement.
    HeadingData fresh;
    fresh.timeTag = 100.0;
    fresh.rhat_BN_N = headingOf(initial);
    ASSERT_TRUE(algo.update(100.0, fresh).headingResiduals.valid);
    TestState const anchor = algo.getState();

    // A measurement stamped before the anchor is discarded by the scheduler, leaving the cycle
    // equivalent to a pure propagation.
    HeadingData stale;
    stale.timeTag = 50.0;
    stale.rhat_BN_N = Eigen::Vector3d(0.0, 0.0, 1.0);
    FlybyFilterOutput const out = algo.update(110.0, stale);

    EXPECT_FALSE(out.headingResiduals.valid) << "a measurement older than the anchor must be dropped";
    TestState const expected = filtering::propagate(FlybyDynamics{kMu}, anchor, {0.0, 10.0});
    EXPECT_TRUE(algo.getState().raw().isApprox(expected.raw(), 1E-9));
}

TEST(FlybyFilterAlgorithmScheduler, DelayedButNewerMeasurementIsAppliedAtItsOwnTimeTag) {
    TestState const initial = nominalTruth();
    Matrix6 const P0 = diagCovariance(100.0, 0.1);

    // Deliver the same measurement, stamped at t = 105, either on time or late (at t = 120). Because
    // the filter anchors to the time tag rather than the delivery time, both runs must agree.
    auto run = [&](double deliveryTime) {
        FlybyFilterAlgorithm algo(baseConfig(initial, P0));
        HeadingData first;
        first.timeTag = 100.0;
        first.rhat_BN_N = headingOf(initial);
        algo.update(100.0, first);

        HeadingData delayed;
        delayed.timeTag = 105.0;
        delayed.rhat_BN_N = headingOf(initial);
        FlybyFilterOutput const out = algo.update(deliveryTime, delayed);
        EXPECT_TRUE(out.headingResiduals.valid) << "delivered at " << deliveryTime;

        // Bring both runs to the same final time before comparing.
        algo.update(120.0, HeadingData{});
        return algo.getState();
    };

    TestState const onTime = run(105.0);
    TestState const late = run(120.0);
    EXPECT_TRUE(late.raw().isApprox(onTime.raw(), 1E-9))
        << "a late measurement must land at its time tag, not its delivery time";
}

TEST(FlybyFilterAlgorithmScheduler, BadMeasurementIsRolledBackAndTheCycleStillPropagates) {
    TestState const initial = nominalTruth();
    Matrix6 const P0 = diagCovariance(100.0, 0.1);

    auto anchored = [&]() {
        auto algo = std::make_unique<FlybyFilterAlgorithm>(baseConfig(initial, P0));
        HeadingData good;
        good.timeTag = 100.0;
        good.rhat_BN_N = headingOf(initial);
        algo->update(100.0, good);
        return algo;
    };

    // A NaN heading fails inside the SRuKF; applySequentialRobust rolls the filter back to the
    // anchor and then propagates the remainder of the cycle, so the result is indistinguishable
    // from a cycle in which the measurement never arrived.
    auto poisoned = anchored();
    HeadingData bad;
    bad.timeTag = 110.0;
    bad.rhat_BN_N = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    FlybyFilterOutput const out = poisoned->update(110.0, bad);

    auto untouched = anchored();
    untouched->update(110.0, HeadingData{});

    EXPECT_FALSE(out.headingResiduals.valid) << "a rejected measurement must not report a residual";
    EXPECT_TRUE(poisoned->getState().raw().allFinite()) << "rollback must leave the state finite";
    EXPECT_TRUE(poisoned->getState().raw().isApprox(untouched->getState().raw(), 1E-9))
        << "a rejected measurement must leave the same estimate as no measurement at all";
    EXPECT_TRUE(poisoned->getCovariance().isApprox(untouched->getCovariance(), 1E-9));

    // The filter recovers: the next good measurement fires normally.
    HeadingData recovery;
    recovery.timeTag = 120.0;
    recovery.rhat_BN_N = headingOf(poisoned->getState());
    EXPECT_TRUE(poisoned->update(120.0, recovery).headingResiduals.valid);
}

TEST(FlybyFilterAlgorithmScheduler, QueueAdmitsOneMeasurementPerCycle) {
    // BatchSize == 1: the flyby filter takes at most one heading per cycle, and a second enqueue in
    // the same cycle is refused rather than silently overwriting the first.
    static_assert(BatchSize == 1, "flybyFilter processes a single heading measurement per cycle");

    filtering::measurement_queue<Measurement, BatchSize> queue;
    EXPECT_TRUE(queue.enqueue(1.0, makeHeadingMeasurement(1.0, Eigen::Vector3d::UnitX(), kHeadingStd)));
    EXPECT_FALSE(queue.enqueue(2.0, makeHeadingMeasurement(2.0, Eigen::Vector3d::UnitY(), kHeadingStd)))
        << "the queue is full at BatchSize measurements";
}

// ============================================================================
// Dynamics: two-body point-mass gravity.
// ============================================================================

TEST(FlybyFilterAlgorithmDynamics, DerivativeMatchesTwoBody) {
    Eigen::Vector3d const r(3000.0, 1000.0, 500.0);
    Eigen::Vector3d const v(1.0, -2.0, 0.5);
    State const s = makeState(r, v);

    State const dot = FlybyDynamics{kMu}(0.0, s);

    Eigen::Vector3d const expectedVDot = -kMu / std::pow(r.norm(), 3) * r;
    EXPECT_TRUE(dot.get<filtering::Position<3>>().isApprox(v, 1E-12));
    EXPECT_TRUE(dot.get<filtering::Velocity<3>>().isApprox(expectedVDot, 1E-12));
}

// ============================================================================
// timeUpdate(): propagation and covariance growth.
// ============================================================================

TEST(FlybyFilterAlgorithmTimeUpdate, ZeroDtLeavesStateAndCovarianceUnchanged) {
    State const initial = nominalTruth();
    Matrix6 const P0 = diagCovariance(100.0, 0.1);
    FlybyFilterAlgorithm algo(configWithProcessNoise(initial, P0, Matrix6::Identity() * 1E-6));

    EXPECT_TRUE(algo.timeUpdate(0.0));
    EXPECT_TRUE(algo.getState().raw().isApprox(initial.raw(), 1E-12));
    EXPECT_TRUE(algo.getCovariance().isApprox(P0, 1E-10));
}

TEST(FlybyFilterAlgorithmTimeUpdate, PropagatesAlongTwoBodyOrbit) {
    State const initial = nominalTruth();
    Matrix6 const P0 = diagCovariance(100.0, 0.1);
    FlybyFilterAlgorithm algo(baseConfig(initial, P0));

    constexpr double dt = 30.0;
    EXPECT_TRUE(algo.timeUpdate(dt));

    // The central sigma point (== the reported state) follows the two-body flow exactly.
    State const predicted = filtering::propagate(FlybyDynamics{kMu}, initial, {0.0, dt});
    EXPECT_TRUE(algo.getState().raw().isApprox(predicted.raw(), 1E-9)) << "state must follow the two-body propagation";
}

TEST(FlybyFilterAlgorithmTimeUpdate, GrowsCovarianceWithProcessNoise) {
    State const initial = nominalTruth();
    Matrix6 const P0 = diagCovariance(100.0, 0.1);
    FlybyFilterAlgorithm algo(configWithProcessNoise(initial, P0, Matrix6::Identity() * 1E-4));

    double const tracePrior = algo.getCovariance().trace();
    ASSERT_TRUE(algo.timeUpdate(10.0));

    // Compare against a noise-free propagation: process noise can only add uncertainty.
    FlybyFilterAlgorithm noiseFree(configWithProcessNoise(initial, P0, Matrix6::Zero()));
    ASSERT_TRUE(noiseFree.timeUpdate(10.0));

    Matrix6 const P = algo.getCovariance();
    EXPECT_GE(P.trace(), noiseFree.getCovariance().trace() - 1E-9) << "process noise should not shrink covariance";
    EXPECT_GT(P.trace(), 0.0);
    EXPECT_GT(P.trace(), tracePrior) << "process noise should grow the covariance over a finite dt";
    EXPECT_TRUE(P.isApprox(P.transpose(), 1E-8)) << "covariance not symmetric";
    EXPECT_TRUE(isPositiveSemiDefinite<6>(P)) << "covariance not PSD";
}

// ============================================================================
// measurementUpdate(): heading update shrinks covariance; high-noise limit; bad-update rejection.
// ============================================================================

TEST(FlybyFilterAlgorithmMeasurementUpdate, HeadingMeasurementShrinksCovariance) {
    State const initial = nominalTruth();
    Matrix6 const P0 = diagCovariance(100.0, 0.1);
    FlybyFilterAlgorithm algo(baseConfig(initial, P0));

    Matrix6 const covar0 = algo.getCovariance();
    EXPECT_TRUE(algo.timeUpdate(0.0));

    HeadingMeasurement m;
    m.timeTag = 0.0;
    m.rhat_BN_N = headingOf(initial);  // consistent heading
    m.covar = (kHeadingStd * kHeadingStd) * Eigen::Matrix3d::Identity();
    m.valid = true;
    EXPECT_TRUE(algo.measurementUpdate(m));

    Matrix6 const covarN = algo.getCovariance();
    EXPECT_LT(covarN.trace(), covar0.trace()) << "a heading update should reduce total uncertainty";
    EXPECT_TRUE(covarN.isApprox(covarN.transpose(), 1E-8)) << "covariance not symmetric";
    EXPECT_TRUE(isPositiveSemiDefinite<6>(covarN)) << "covariance not PSD";
    EXPECT_TRUE(algo.getLastHeadingResiduals().valid);
}

TEST(FlybyFilterAlgorithmMeasurementUpdate, HighMeasurementNoiseLeavesStateNearlyUnchanged) {
    State const initial = nominalTruth();
    FlybyFilterAlgorithm algo(baseConfig(initial, diagCovariance(100.0, 0.1)));
    EXPECT_TRUE(algo.timeUpdate(0.0));
    State const before = algo.getState();

    HeadingMeasurement m;
    m.timeTag = 0.0;
    m.rhat_BN_N = Eigen::Vector3d(0.0, 0.0, 1.0);  // far from the prior heading
    m.covar = 1E8 * Eigen::Matrix3d::Identity();   // R >> P
    m.valid = true;
    EXPECT_TRUE(algo.measurementUpdate(m));

    EXPECT_LT((algo.getState().raw() - before.raw()).norm(), 1E-3) << "state moved despite R >> P";
    HeadingResidualsOutput const res = algo.getLastHeadingResiduals();
    EXPECT_TRUE(res.postFit.isApprox(res.preFit, 1E-3)) << "postFit should approx preFit when R >> P";
}

TEST(FlybyFilterAlgorithmMeasurementUpdate, BadMeasurementReturnsFalseAndLeavesResidualInvalid) {
    State const initial = nominalTruth();
    FlybyFilterAlgorithm algo(baseConfig(initial, diagCovariance(100.0, 0.1)));
    EXPECT_TRUE(algo.timeUpdate(0.0));

    HeadingMeasurement m;
    m.timeTag = 0.0;
    m.rhat_BN_N = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    m.covar = (kHeadingStd * kHeadingStd) * Eigen::Matrix3d::Identity();
    m.valid = true;

    EXPECT_FALSE(algo.measurementUpdate(m)) << "a non-finite measurement update must report false";
    EXPECT_FALSE(algo.getLastHeadingResiduals().valid) << "no residual is recorded for a bad update";
    EXPECT_TRUE(algo.getState().raw().allFinite()) << "state must stay finite after a rejected update";
}

TEST(FlybyFilterAlgorithmMeasurements, HeadingDataFiresResidualOnlyWhenFresh) {
    TestState const initial = nominalTruth();
    FlybyFilterAlgorithm algo(baseConfig(initial, diagCovariance(100.0, 0.1)));

    // A stale time tag is the "no reading this cycle" sentinel: no residual is reported.
    algo.update(10.0, HeadingData{});
    EXPECT_FALSE(algo.getLastHeadingResiduals().valid);

    HeadingData fresh;
    fresh.timeTag = 20.0;
    fresh.rhat_BN_N = headingOf(initial);
    algo.update(20.0, fresh);

    HeadingResidualsOutput const& res = algo.getLastHeadingResiduals();
    EXPECT_TRUE(res.valid);
    EXPECT_TRUE(res.observation.isApprox(fresh.rhat_BN_N, 1E-12)) << "the residual records the supplied heading";
}

TEST(FlybyFilterAlgorithmMeasurements, InformativeMeasurementReducesResidual) {
    // Seed with a heading error so the measurement has something to correct.
    TestState const truth = nominalTruth();
    TestState const initial = makeState(truth.get<filtering::Position<3>>() + Eigen::Vector3d(200.0, -150.0, 90.0),
                                        truth.get<filtering::Velocity<3>>());
    FlybyFilterAlgorithm algo(baseConfig(initial, diagCovariance(300.0, 0.1)));

    HeadingData heading;
    heading.timeTag = 10.0;
    heading.rhat_BN_N = headingOf(truth);
    algo.update(10.0, heading);

    HeadingResidualsOutput const& res = algo.getLastHeadingResiduals();
    ASSERT_TRUE(res.valid);
    EXPECT_LT(res.postFit.norm(), res.preFit.norm()) << "an informative update must shrink the residual";
}

TEST(FlybyFilterAlgorithmMeasurements, LargerMeasurementNoiseStdShrinksCovarianceLess) {
    TestState const initial = nominalTruth();
    Matrix6 const P0 = diagCovariance(100.0, 0.1);

    // The noise covariance is built inside packHeadingMeasurement() from the configured std, so a
    // larger configured std must yield a smaller correction for the same reading.
    auto traceAfterUpdate = [&](double headingStd) {
        FlybyFilterAlgorithm algo(configWithHeadingStd(initial, P0, headingStd));
        HeadingData heading;
        heading.timeTag = 10.0;
        heading.rhat_BN_N = headingOf(initial);
        algo.update(10.0, heading);
        return algo.getCovariance().trace();
    };

    double const tight = traceAfterUpdate(1E-5);
    double const loose = traceAfterUpdate(1E-1);
    EXPECT_LT(tight, loose) << "a tighter measurement std should shrink the covariance more";
    EXPECT_LT(loose, P0.trace()) << "even a loose measurement should not grow the covariance";
}

TEST(FlybyFilterAlgorithmMeasurements, WithoutMeasurementsGrowsCovarianceMonotonically) {
    TestState const initial = nominalTruth();
    Matrix6 const P0 = diagCovariance(100.0, 0.1);
    FlybyFilterAlgorithm algo(configWithProcessNoise(initial, P0, Matrix6::Identity() * 1E-6));

    // Three measurement-free cycles: total uncertainty must be non-decreasing at each step.
    algo.update(10.0, HeadingData{});
    double const trace1 = algo.getCovariance().trace();
    algo.update(40.0, HeadingData{});
    double const trace2 = algo.getCovariance().trace();
    algo.update(90.0, HeadingData{});
    double const trace3 = algo.getCovariance().trace();

    EXPECT_GT(trace2, trace1);
    EXPECT_GT(trace3, trace2);
    EXPECT_TRUE(finiteSymmetricPsd(algo.getCovariance()));
}

// ============================================================================
// Degenerate geometry: both the dynamics and the heading model divide by |r|.
// ============================================================================

TEST(FlybyFilterAlgorithmDegenerate, ZeroPositionMakesTheDynamicsNonFinite) {
    // Documents the model's precondition: two-body gravity is singular at the central body, so a
    // sigma point that reaches r = 0 produces a non-finite derivative.
    TestState const atOrigin = makeState(Eigen::Vector3d::Zero(), Eigen::Vector3d(1.0, -2.0, 0.5));
    TestState const dot = FlybyDynamics{kMu}(0.0, atOrigin);

    EXPECT_TRUE(dot.get<filtering::Position<3>>().allFinite()) << "r_dot = v stays finite";
    EXPECT_FALSE(dot.get<filtering::Velocity<3>>().allFinite()) << "v_dot = -mu/|r|^3 r is singular at r = 0";
}

TEST(FlybyFilterAlgorithmDegenerate, ZeroPositionSeedMakesTimeUpdateReportFailure) {
    // A filter seeded at the central body cannot propagate; timeUpdate must report the failure
    // rather than silently publishing a non-finite estimate.
    TestState const atOrigin = makeState(Eigen::Vector3d::Zero(), Eigen::Vector3d(1.0, -2.0, 0.5));
    FlybyFilterAlgorithm algo(baseConfig(atOrigin, diagCovariance(1.0, 1E-3)));

    EXPECT_FALSE(algo.timeUpdate(10.0)) << "propagation from r = 0 must be reported as invalid";
}

// ============================================================================
// SRuKF static helpers: numerical helpers (filter-agnostic).
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
// Convergence: angles-only heading measurements along a propagated two-body arc.
// ============================================================================

TEST(FlybyFilterAlgorithmConvergence, TracksHeadingAndShrinksCovarianceOverAnArc) {
    State const truth0 = nominalTruth();

    // Seed the filter with a position/velocity error from truth.
    State const initial = makeState(truth0.get<filtering::Position<3>>() + Eigen::Vector3d(60.0, -40.0, 30.0),
                                    truth0.get<filtering::Velocity<3>>() + Eigen::Vector3d(0.02, 0.03, -0.01));
    Matrix6 const P0 = diagCovariance(150.0, 0.1);
    FlybyFilterAlgorithm algo(configWithProcessNoise(initial, P0, Matrix6::Identity() * 1E-10));

    double const initialPosErr =
        (algo.getState().get<filtering::Position<3>>() - truth0.get<filtering::Position<3>>()).norm();
    double const initialTrace = algo.getCovariance().trace();

    constexpr double dt = 10.0;
    State truth = truth0;
    for (int i = 1; i <= 80; ++i) {
        truth = filtering::propagate(FlybyDynamics{kMu}, truth, {0.0, dt});
        HeadingData heading;
        heading.timeTag = i * dt;
        heading.rhat_BN_N = headingOf(truth);  // exact heading measurement
        algo.update(i * dt, heading);
    }

    // The observed heading is tracked tightly, total uncertainty shrinks, and the position estimate
    // has moved toward truth (angles-only range is only weakly observable, so we assert improvement
    // rather than a tight absolute bound).
    double const headingErr = (headingOf(algo.getState()) - headingOf(truth)).norm();
    double const finalPosErr =
        (algo.getState().get<filtering::Position<3>>() - truth.get<filtering::Position<3>>()).norm();
    EXPECT_LT(headingErr, 1E-2) << "heading error " << headingErr;
    EXPECT_LT(algo.getCovariance().trace(), initialTrace) << "covariance should shrink over the arc";
    EXPECT_LT(finalPosErr, initialPosErr)
        << "position estimate should improve (" << finalPosErr << " vs " << initialPosErr << ")";
}

TEST(FlybyFilterAlgorithmConvergence, TracksHeadingUnderNoisyMeasurements) {
    State const truth0 = nominalTruth();
    State const initial = makeState(truth0.get<filtering::Position<3>>() + Eigen::Vector3d(60.0, -40.0, 30.0),
                                    truth0.get<filtering::Velocity<3>>() + Eigen::Vector3d(0.02, 0.03, -0.01));
    Matrix6 const P0 = diagCovariance(150.0, 0.1);
    FlybyFilterAlgorithm algo(baseConfig(initial, P0));

    std::mt19937 gen(7);
    std::normal_distribution<double> noise(0.0, 1.0);

    double const initialTrace = algo.getCovariance().trace();

    constexpr double dt = 10.0;
    State truth = truth0;
    for (int i = 1; i <= 80; ++i) {
        truth = filtering::propagate(FlybyDynamics{kMu}, truth, {0.0, dt});
        Eigen::Vector3d rhat = headingOf(truth) + kHeadingStd * Eigen::Vector3d(noise(gen), noise(gen), noise(gen));
        rhat.normalize();
        HeadingData heading;
        heading.timeTag = i * dt;
        heading.rhat_BN_N = rhat;
        algo.update(i * dt, heading);
    }

    double const headingErr = (headingOf(algo.getState()) - headingOf(truth)).norm();
    EXPECT_LT(headingErr, 5E-2) << "heading error " << headingErr;  // generous, seeded
    EXPECT_LT(algo.getCovariance().trace(), initialTrace);
    EXPECT_TRUE(algo.getState().raw().allFinite());
}

}  // namespace filtering::flybyFilter
