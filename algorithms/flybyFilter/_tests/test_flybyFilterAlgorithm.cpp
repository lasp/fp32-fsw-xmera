// Unit tests for FlybyFilterAlgorithm (angles-only two-body flyby SRuKF on filteringCore).
//
// Sections (grouped simplest-first):
//   * Config: factory validation, static validators, getter round-trips.
//   * Lifecycle: construction seeds state/covariance; reInitializeExceptPersistentStates / reInitialize / setConfig.
//   * Dynamics: two-body point-mass gravity r_dot = v, v_dot = -mu/|r|^3 r.
//   * timeUpdate(): zero-dt no-op; propagation matches filtering::propagate; covariance growth under Q.
//   * measurementUpdate(): heading update shrinks covariance (symmetric + PSD); high-noise limit;
//     bad-measurement rejection (handled inside the SRuKF).
//   * Convergence: angles-only heading measurements along a propagated two-body arc.
//
// All quantities are in the filter's internal units (km, km/s); the adapter handles SI<->internal.

#include "flybyFilterAlgorithm.h"
#include "flybyFilterSpecs.h"

#include "utilities/fsw/validPSDCheck.h"

#include <filteringCore/dynamicsModel.hpp>

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <cmath>
#include <limits>
#include <random>

namespace filtering::flybyFilter {
namespace {

using State = FlybyFilterAlgorithm::State;
using Vector6 = Eigen::Matrix<double, 6, 1>;
using Matrix6 = Eigen::Matrix<double, 6, 6>;

// Mars gravitational parameter in internal units (km^3/s^2).
constexpr double kMu = 42828.314;
constexpr double kAlpha = 0.02;
constexpr double kBeta = 2.0;
constexpr double kHeadingStd = 1E-4;

State makeState(Eigen::Vector3d const& r, Eigen::Vector3d const& v) {
    State s;
    s.set<filtering::Position<3>>(r);
    s.set<filtering::Velocity<3>>(v);
    return s;
}

Matrix6 diagCovariance(double posStd, double velStd) {
    Vector6 d;
    d << posStd * posStd, posStd * posStd, posStd * posStd, velStd * velStd, velStd * velStd, velStd * velStd;
    return d.asDiagonal();
}

Matrix6 smallProcessNoise() { return Matrix6::Identity() * 1E-12; }

Eigen::Vector3d headingOf(State const& s) {
    Eigen::Vector3d const r = s.get<filtering::Position<3>>();
    return r / r.norm();
}

// A representative flyby state (km, km/s).
State nominalTruth() { return makeState({3000.0, 1000.0, 500.0}, {1.0, -2.0, 0.5}); }

FlybyFilterConfig baseConfig(State const& initial, Matrix6 const& P) {
    return FlybyFilterConfig::create(kAlpha, kBeta, kMu, smallProcessNoise(), initial, P, kHeadingStd);
}

FlybyFilterConfig configWithProcessNoise(State const& initial, Matrix6 const& P, Matrix6 const& processNoise) {
    return FlybyFilterConfig::create(kAlpha, kBeta, kMu, processNoise, initial, P, kHeadingStd);
}

// A complete set of valid Config inputs; individual tests override one field.
struct ConfigInputs {
    double alpha = kAlpha;
    double beta = kBeta;
    double mu = kMu;
    Matrix6 processNoise = smallProcessNoise();
    State initialState = nominalTruth();
    Matrix6 initialCovariance = diagCovariance(100.0, 0.1);
    double headingStd = kHeadingStd;
};

FlybyFilterConfig buildConfig(ConfigInputs const& in) {
    return FlybyFilterConfig::create(
        in.alpha, in.beta, in.mu, in.processNoise, in.initialState, in.initialCovariance, in.headingStd);
}

}  // namespace

// ============================================================================
// Config: factory validation, static validators, getter round-trips.
// ============================================================================

TEST(FlybyFilterConfig, ValidInputsDoNotThrow) { EXPECT_NO_THROW(buildConfig({})); }

TEST(FlybyFilterConfig, RejectsAlphaOutsideOpenUnitInterval) {
    for (double bad : {0.0, 1.0, -0.1, 1.5}) {  // (0, 1) open interval
        ConfigInputs in;
        in.alpha = bad;
        EXPECT_THROW(buildConfig(in), fsw::invalid_argument) << "alpha=" << bad;
    }
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

TEST(FlybyFilterAlgorithmLifecycle, ReInitializePreservesEstimateReInitializeAllResetsIt) {
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
    EXPECT_TRUE(P.isApprox(P.transpose(), 1E-8)) << "covariance not symmetric";
    EXPECT_TRUE(isPositiveSemiDefinite<6>(P)) << "covariance not PSD";
    (void)tracePrior;
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
