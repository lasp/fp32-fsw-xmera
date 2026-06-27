// Unit tests for InertialSRuKFAlgorithm and the SRUKF numerical helpers.
//
// Sections (each exercises one feature of the algorithm, mirroring the patterns
// in algorithms/filteringCore/_tests/test_srukf.cpp and the sunline algorithm test):
//   * Dynamics: sigma_dot = 1/4 B(sigma) omega; omega_dot = 0.
//   * timeUpdate(): zero-rate / zero-dt invariants; covariance growth under Q.
//   * Config: factory validation, static validators, getter round-trips.
//   * Lifecycle: construction seeds state/covariance; reInitialize / reInitializeAll.
//   * Measurement packing + application (through update()) and residual recording.
//   * Regularization: MRP shadow-set switch on over-unity attitude.
//   * measurementUpdate(): covariance shrink + symmetry + PSD; high-noise limit.
//   * Covariance growth without measurements.
//   * State convergence (exact and seeded-noisy measurements).
//   * SRuKF static helpers: forward/back substitution, Cholesky, QR-just-R,
//     and Cholesky up/down-date.

#include "inertialSRuKFAlgorithm.h"
#include "inertialSRuKFSpecs.h"

#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/validPSDCheck.h"

#include <filteringCore/srukf.hpp>

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <cmath>
#include <limits>
#include <random>

namespace filtering::inertialSRuKF {
namespace {

using State = InertialSRuKFAlgorithm::State;
using Vector6 = Eigen::Matrix<double, 6, 1>;
using Matrix6 = Eigen::Matrix<double, 6, 6>;
using SRuKF = ::filtering::SRuKF<InertialState, InertialDynamics>;

constexpr double kAlpha = 0.02;
constexpr double kBeta = 2.0;
constexpr double kStMeasStd = 1E-3;
constexpr double kGyroMeasStd = 1E-3;

State makeState(Eigen::Vector3d const& sigma, Eigen::Vector3d const& omega) {
    State s;
    s.set<filtering::MrpAttitude<3>>(sigma);
    s.set<filtering::AngularRate<3>>(omega);
    return s;
}

Matrix6 diagCovariance(double attStd, double rateStd) {
    Vector6 d;
    d << attStd * attStd, attStd * attStd, attStd * attStd, rateStd * rateStd, rateStd * rateStd, rateStd * rateStd;
    return d.asDiagonal();
}

Matrix6 smallProcessNoise() {
    double const q = 1E-10;
    return Matrix6::Identity() * q;
}

double attitudeTrace(Matrix6 const& P) { return P(0, 0) + P(1, 1) + P(2, 2); }
double rateTrace(Matrix6 const& P) { return P(3, 3) + P(4, 4) + P(5, 5); }

// Validated config for the dynamics / timeUpdate / measurement tests.
InertialSRuKFConfig baseConfig(State const& initial, Matrix6 const& P) {
    return InertialSRuKFConfig::create(kAlpha, kBeta, smallProcessNoise(), initial, P, kStMeasStd, kGyroMeasStd);
}

// baseConfig with an explicit process noise (for exercising process-noise-driven covariance growth).
InertialSRuKFConfig configWithProcessNoise(State const& initial, Matrix6 const& P, Matrix6 const& processNoise) {
    return InertialSRuKFConfig::create(kAlpha, kBeta, processNoise, initial, P, kStMeasStd, kGyroMeasStd);
}

// baseConfig with explicit measurement-noise standard deviations.
InertialSRuKFConfig configWithNoiseStds(State const& initial, Matrix6 const& P, double stStd, double gyroStd) {
    return InertialSRuKFConfig::create(kAlpha, kBeta, smallProcessNoise(), initial, P, stStd, gyroStd);
}

}  // namespace

// ============================================================================
// Dynamics: MRP kinematics sigma_dot = 1/4 B(sigma) omega; omega_dot = 0.
// ============================================================================

TEST(InertialSRuKFAlgorithmDynamics, DerivativeMatchesMrpKinematics) {
    Eigen::Vector3d const sigma(0.1, -0.2, 0.05);
    Eigen::Vector3d const omega(0.02, -0.005, 0.01);
    State s = makeState(sigma, omega);

    State const dot = InertialDynamics{}(0.0, s);

    Eigen::Vector3d const expectedSigmaDot = 0.25 * bmatMrp(sigma) * omega;
    EXPECT_TRUE(dot.get<filtering::MrpAttitude<3>>().isApprox(expectedSigmaDot, 1E-12));
    EXPECT_TRUE(dot.get<filtering::AngularRate<3>>().isApprox(Eigen::Vector3d::Zero(), 1E-12));
}

// With omega = 0 the attitude must not move.
TEST(InertialSRuKFAlgorithmTimeUpdate, ZeroRateLeavesAttitudeFixed) {
    Eigen::Vector3d const sigma0(0.1, -0.2, 0.05);
    InertialSRuKFAlgorithm algo(baseConfig(makeState(sigma0, Eigen::Vector3d::Zero()), diagCovariance(1E-2, 1E-3)));

    EXPECT_TRUE(algo.timeUpdate(10.0));
    Eigen::Vector3d const sigma = algo.getState().get<filtering::MrpAttitude<3>>();
    EXPECT_TRUE(sigma.isApprox(sigma0, 1E-9));
}

// dt == 0 short-circuit: the state must equal the anchor on return.
TEST(InertialSRuKFAlgorithmTimeUpdate, ZeroDtCollapsesToAnchor) {
    Eigen::Vector3d const sigma0(0.0, 0.0, 0.1);
    Eigen::Vector3d const omega0(0.1, 0.0, 0.0);

    InertialSRuKFAlgorithm algo(baseConfig(makeState(sigma0, omega0), diagCovariance(1E-2, 1E-3)));

    EXPECT_TRUE(algo.timeUpdate(0.0));
    State const s = algo.getState();
    EXPECT_TRUE(s.get<filtering::MrpAttitude<3>>().isApprox(sigma0, 1E-12));
    EXPECT_TRUE(s.get<filtering::AngularRate<3>>().isApprox(omega0, 1E-12));
}

// A NaN time step returns true: propagate() runs zero RK4 sub-steps because the loop
// bound `i < ceil(NaN / step)` is always false, so the NaN never enters the state and the
// finiteness check passes. The step is effectively a no-op (the state is unchanged), which
// is why timeUpdate cannot flag a NaN dt on its own.
TEST(InertialSRuKFAlgorithmTimeUpdate, NaNDtReturnsTrueAndLeavesStateUnchanged) {
    Eigen::Vector3d const sigma0(0.1, -0.2, 0.05);
    Eigen::Vector3d const omega0(0.01, 0.0, -0.02);
    InertialSRuKFAlgorithm algo(baseConfig(makeState(sigma0, omega0), diagCovariance(1E-2, 1E-3)));
    State const before = algo.getState();

    EXPECT_TRUE(algo.timeUpdate(std::numeric_limits<double>::quiet_NaN()))
        << "a NaN dt is a no-op (zero sub-steps), so timeUpdate stays valid";
    EXPECT_TRUE(algo.getState().raw().isApprox(before.raw(), 1E-12)) << "a NaN dt should leave the state unchanged";
}

// ============================================================================
// Lifecycle: construction seeds the filter; reInitialize / reInitializeAll.
// ============================================================================

TEST(InertialSRuKFAlgorithmLifecycle, ConstructorSeedsStateAndCovarianceFromConfig) {
    State const initial = makeState(Eigen::Vector3d(0.1, -0.2, 0.15), Eigen::Vector3d(0.01, 0.0, -0.02));
    Matrix6 const P0 = diagCovariance(1E-2, 1E-3);
    InertialSRuKFAlgorithm algo(baseConfig(initial, P0));

    EXPECT_TRUE(algo.getState().raw().isApprox(initial.raw(), 1E-12));
    EXPECT_TRUE(algo.getCovariance().isApprox(P0, 1E-12));
}

TEST(InertialSRuKFAlgorithmLifecycle, ReInitializePreservesEstimateReInitializeAllResetsIt) {
    InertialSRuKFAlgorithm algo(
        baseConfig(makeState(Eigen::Vector3d(0, 0, 0.05), Eigen::Vector3d(0.01, 0, 0)), diagCovariance(1E-2, 1E-2)));

    State const initialState = algo.getState();
    Matrix6 const initialCovariance = algo.getCovariance();

    StAttData stAtt;
    stAtt.timeTag = 1.0;
    stAtt.sigma_BN = Eigen::Vector3d(0.001, 0.0, 0.05);
    RateData rate;
    rate.timeTag = 1.0;
    rate.rate = Eigen::Vector3d(0.01, 0.0, 0.0);
    algo.update(2.0, stAtt, rate);

    State const movedState = algo.getState();
    Matrix6 const movedCovariance = algo.getCovariance();
    ASSERT_FALSE(movedCovariance.isApprox(initialCovariance));
    EXPECT_TRUE(algo.getLastStAttResiduals().valid);

    // reInitialize() keeps the current estimate but clears the residual snapshots.
    algo.reInitialize();
    EXPECT_TRUE(algo.getState().raw().isApprox(movedState.raw()));
    EXPECT_TRUE(algo.getCovariance().isApprox(movedCovariance));
    EXPECT_FALSE(algo.getLastStAttResiduals().valid);
    EXPECT_FALSE(algo.getLastRateResiduals().valid);

    // reInitializeAll() re-seeds state and covariance from the configuration.
    algo.reInitializeAll();
    EXPECT_TRUE(algo.getState().raw().isApprox(initialState.raw()));
    EXPECT_TRUE(algo.getCovariance().isApprox(initialCovariance));
}

TEST(InertialSRuKFAlgorithmLifecycle, ReInitializeAllRestoresSeedAfterConvergence) {
    State const initial = makeState(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    Matrix6 const P0 = diagCovariance(1E-1, 1E-2);
    InertialSRuKFAlgorithm algo(baseConfig(initial, P0));

    Eigen::Vector3d const truth(0.1, -0.2, 0.15);
    for (int i = 1; i <= 50; ++i) {
        StAttData st;
        st.timeTag = i * 0.5;
        st.sigma_BN = truth;
        algo.update(i * 0.5, st, RateData{});
    }
    ASSERT_FALSE(algo.getState().raw().isApprox(initial.raw(), 1E-3));

    algo.reInitializeAll();
    EXPECT_TRUE(algo.getState().raw().isApprox(initial.raw(), 1E-12));
    EXPECT_TRUE(algo.getCovariance().isApprox(P0, 1E-12));
}

// A filter reconfigured to a larger process noise must behave like one constructed with it.
TEST(InertialSRuKFAlgorithmLifecycle, SetConfigReDerivesProcessNoise) {
    State const initial = makeState(Eigen::Vector3d(0, 0, 0.05), Eigen::Vector3d::Zero());
    Matrix6 const P0 = diagCovariance(1E-2, 1E-2);
    Matrix6 const smallQ = Matrix6::Identity() * 1E-12;
    Matrix6 const largeQ = Matrix6::Identity() * 1E-2;
    constexpr double dt = 1.0;

    InertialSRuKFAlgorithm reference(configWithProcessNoise(initial, P0, largeQ));
    EXPECT_TRUE(reference.timeUpdate(dt));

    InertialSRuKFAlgorithm smallOnly(configWithProcessNoise(initial, P0, smallQ));
    EXPECT_TRUE(smallOnly.timeUpdate(dt));
    ASSERT_FALSE(smallOnly.getCovariance().isApprox(reference.getCovariance()));

    InertialSRuKFAlgorithm algo(configWithProcessNoise(initial, P0, smallQ));
    algo.setConfig(configWithProcessNoise(initial, P0, largeQ));
    EXPECT_TRUE(algo.timeUpdate(dt));
    EXPECT_TRUE(algo.getCovariance().isApprox(reference.getCovariance()));
}

// ============================================================================
// Measurement packing + application (through the public update() path) and
// residual recording.
// ============================================================================

TEST(InertialSRuKFAlgorithmMeasurements, StAttDataFiresResidualOnlyWhenFresh) {
    InertialSRuKFAlgorithm algo(
        baseConfig(makeState(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()), diagCovariance(1E-1, 1E-2)));

    // timeTag <= 0 is a stale/empty reading: no measurement fires.
    InertialSRuKFOutput const stale = algo.update(1.0, StAttData{}, RateData{});
    EXPECT_FALSE(stale.stAttResiduals.valid);
    EXPECT_FALSE(stale.rateResiduals.valid);

    // timeTag > 0 fires; the recorded observation echoes the input MRP.
    StAttData st;
    st.timeTag = 2.0;
    st.sigma_BN = Eigen::Vector3d(0.01, -0.02, 0.03);
    InertialSRuKFOutput const fresh = algo.update(2.0, st, RateData{});
    EXPECT_TRUE(fresh.stAttResiduals.valid);
    EXPECT_FALSE(fresh.rateResiduals.valid);
    EXPECT_TRUE(fresh.stAttResiduals.observation.isApprox(st.sigma_BN, 1E-12));
}

TEST(InertialSRuKFAlgorithmMeasurements, RateDataFiresResidualOnlyWhenFresh) {
    InertialSRuKFAlgorithm algo(
        baseConfig(makeState(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()), diagCovariance(1E-2, 1E-1)));

    InertialSRuKFOutput const stale = algo.update(1.0, StAttData{}, RateData{});
    EXPECT_FALSE(stale.rateResiduals.valid);

    RateData rate;
    rate.timeTag = 2.0;
    rate.rate = Eigen::Vector3d(0.02, -0.01, 0.015);
    InertialSRuKFOutput const fresh = algo.update(2.0, StAttData{}, rate);
    EXPECT_TRUE(fresh.rateResiduals.valid);
    EXPECT_FALSE(fresh.stAttResiduals.valid);
    EXPECT_TRUE(fresh.rateResiduals.observation.isApprox(rate.rate, 1E-12));
}

// Packing threads the configured measurement-noise std into the measurement covariance:
// a larger std makes the star-tracker update less informative, so the attitude block
// shrinks less.
TEST(InertialSRuKFAlgorithmMeasurements, LargerMeasurementNoiseStdShrinksCovarianceLess) {
    State const initial = makeState(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    Matrix6 const P0 = diagCovariance(1E-1, 1E-2);

    StAttData st;
    st.timeTag = 1.0;
    st.sigma_BN = Eigen::Vector3d(0.0, 0.0, 0.05);

    InertialSRuKFAlgorithm sharp(configWithNoiseStds(initial, P0, 1E-3, kGyroMeasStd));
    sharp.update(1.0, st, RateData{});

    InertialSRuKFAlgorithm loose(configWithNoiseStds(initial, P0, 1E-1, kGyroMeasStd));
    loose.update(1.0, st, RateData{});

    EXPECT_LT(attitudeTrace(sharp.getCovariance()), attitudeTrace(loose.getCovariance()));
    EXPECT_LT(attitudeTrace(loose.getCovariance()), attitudeTrace(P0));
}

TEST(InertialSRuKFAlgorithmMeasurements, InformativeMeasurementReducesResidual) {
    InertialSRuKFAlgorithm algo(
        baseConfig(makeState(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()), diagCovariance(1E-1, 1E-2)));
    EXPECT_TRUE(algo.timeUpdate(0.0));

    StAttMeasurement m;
    m.timeTag = 0.0;
    m.sigma_BN = Eigen::Vector3d(0.05, -0.05, 0.05);
    m.covar = (kStMeasStd * kStMeasStd) * Eigen::Matrix3d::Identity();
    m.valid = true;
    EXPECT_TRUE(algo.measurementUpdate(m));

    StAttResidualsOutput const res = algo.getLastStAttResiduals();
    EXPECT_TRUE(res.valid);
    EXPECT_LT(res.postFit.norm(), res.preFit.norm());
}

TEST(InertialSRuKFAlgorithmMeasurements, SequentialStAttThenRateBothApplied) {
    InertialSRuKFAlgorithm algo(
        baseConfig(makeState(Eigen::Vector3d(0, 0, 0.05), Eigen::Vector3d(0.01, 0, 0)), diagCovariance(1E-1, 1E-1)));
    Matrix6 const covar0 = algo.getCovariance();

    StAttData st;
    st.timeTag = 1.0;  // earlier
    st.sigma_BN = Eigen::Vector3d(0.001, 0.0, 0.05);
    RateData rate;
    rate.timeTag = 1.5;  // later
    rate.rate = Eigen::Vector3d(0.01, 0.0, 0.0);

    InertialSRuKFOutput const out = algo.update(2.0, st, rate);

    EXPECT_TRUE(out.stAttResiduals.valid);
    EXPECT_TRUE(out.rateResiduals.valid);
    EXPECT_LT(attitudeTrace(algo.getCovariance()), attitudeTrace(covar0));
    EXPECT_LT(rateTrace(algo.getCovariance()), rateTrace(covar0));
}

// ============================================================================
// Regularization: the MRP attitude is mapped to its inner shadow set when
// |sigma| > 1; the rate block is untouched. Exercised through update().
// ============================================================================

TEST(InertialSRuKFAlgorithmRegularize, SwitchesOverUnityMrpToInnerSet) {
    Eigen::Vector3d const overUnity(0.9, 0.9, 0.9);  // norm ~1.56
    ASSERT_GT(overUnity.norm(), 1.0);
    InertialSRuKFAlgorithm algo(baseConfig(makeState(overUnity, Eigen::Vector3d::Zero()), diagCovariance(1E-2, 1E-3)));

    // currentSeconds = 0: no propagation, no measurements -- only regularization runs.
    algo.update(0.0, StAttData{}, RateData{});

    Eigen::Vector3d const sigma = algo.getState().get<filtering::MrpAttitude<3>>();
    EXPECT_LE(sigma.norm(), 1.0);
    // Shadow switching preserves the physical attitude (same DCM).
    EXPECT_TRUE(mrpToDcm(sigma).isApprox(mrpToDcm(overUnity), 1E-9));
}

TEST(InertialSRuKFAlgorithmRegularize, LeavesRateUnchanged) {
    Eigen::Vector3d const overUnity(0.9, 0.9, 0.9);
    Eigen::Vector3d const omega(0.01, -0.02, 0.03);
    InertialSRuKFAlgorithm algo(baseConfig(makeState(overUnity, omega), diagCovariance(1E-2, 1E-3)));

    algo.update(0.0, StAttData{}, RateData{});

    EXPECT_TRUE(algo.getState().get<filtering::AngularRate<3>>().isApprox(omega, 1E-12));
}

// ============================================================================
// measurementUpdate: covariance shrinks, stays symmetric + PSD; high
// measurement noise leaves the state nearly unchanged.
// ============================================================================

TEST(InertialSRuKFAlgorithmMeasurementUpdate, StAttMeasurementShrinksAttitudeCovariance) {
    Eigen::Vector3d const sigma0(0.0, 0.0, 0.05);
    InertialSRuKFAlgorithm algo(
        baseConfig(makeState(sigma0, Eigen::Vector3d(0.01, 0.01, 0.01)), diagCovariance(1E-1, 1E-2)));

    Matrix6 const covar0 = algo.getCovariance();
    EXPECT_TRUE(algo.timeUpdate(0.0));

    StAttMeasurement m;
    m.timeTag = 0.0;
    m.sigma_BN = Eigen::Vector3d(0.001, -0.001, 0.051);
    m.covar = (kStMeasStd * kStMeasStd) * Eigen::Matrix3d::Identity();
    m.valid = true;
    EXPECT_TRUE(algo.measurementUpdate(m));

    Matrix6 const covarN = algo.getCovariance();
    for (int i = 0; i < 3; ++i) {
        EXPECT_LT(covarN(i, i), covar0(i, i)) << "attitude cov diag index " << i;
    }
    EXPECT_TRUE(covarN.isApprox(covarN.transpose(), 1E-10)) << "covariance not symmetric";
    EXPECT_TRUE(isPositiveSemiDefinite<6>(covarN)) << "covariance not PSD";
    EXPECT_TRUE(algo.getLastStAttResiduals().valid);
}

TEST(InertialSRuKFAlgorithmMeasurementUpdate, RateMeasurementShrinksRateCovariance) {
    Eigen::Vector3d const omega0(0.01, 0.01, 0.01);
    InertialSRuKFAlgorithm algo(baseConfig(makeState(Eigen::Vector3d::Zero(), omega0), diagCovariance(1E-2, 1E-1)));

    Matrix6 const covar0 = algo.getCovariance();
    EXPECT_TRUE(algo.timeUpdate(0.0));

    RateMeasurement r;
    r.timeTag = 0.0;
    r.omega_BN_B = Eigen::Vector3d(0.012, 0.008, 0.011);
    r.covar = (kGyroMeasStd * kGyroMeasStd) * Eigen::Matrix3d::Identity();
    r.valid = true;
    EXPECT_TRUE(algo.measurementUpdate(r));

    Matrix6 const covarN = algo.getCovariance();
    for (int i = 3; i < 6; ++i) {
        EXPECT_LT(covarN(i, i), covar0(i, i)) << "rate cov diag index " << i;
    }
    EXPECT_TRUE(covarN.isApprox(covarN.transpose(), 1E-10)) << "covariance not symmetric";
    EXPECT_TRUE(isPositiveSemiDefinite<6>(covarN)) << "covariance not PSD";
    EXPECT_TRUE(algo.getLastRateResiduals().valid);
}

TEST(InertialSRuKFAlgorithmMeasurementUpdate, HighMeasurementNoiseLeavesStateNearlyUnchanged) {
    InertialSRuKFAlgorithm algo(
        baseConfig(makeState(Eigen::Vector3d(0.0, 0.0, 0.05), Eigen::Vector3d::Zero()), diagCovariance(1E-2, 1E-2)));
    EXPECT_TRUE(algo.timeUpdate(0.0));
    State const before = algo.getState();

    StAttMeasurement m;
    m.timeTag = 0.0;
    m.sigma_BN = Eigen::Vector3d(0.5, 0.0, 0.0);  // far from the prior
    m.covar = 1E8 * Eigen::Matrix3d::Identity();  // R >> P
    m.valid = true;
    EXPECT_TRUE(algo.measurementUpdate(m));

    EXPECT_LT((algo.getState().raw() - before.raw()).norm(), 1E-5) << "state moved despite R >> P";
    StAttResidualsOutput const res = algo.getLastStAttResiduals();
    EXPECT_TRUE(res.postFit.isApprox(res.preFit, 1E-4)) << "postFit should approx preFit when R >> P";
}

// A measurement that drives the state non-finite is rejected: measurementUpdate returns
// false and the residual snapshot is left invalid (the SRuKF returns no value).
TEST(InertialSRuKFAlgorithmMeasurementUpdate, BadMeasurementReturnsFalseAndLeavesResidualInvalid) {
    InertialSRuKFAlgorithm algo(
        baseConfig(makeState(Eigen::Vector3d(0.0, 0.0, 0.05), Eigen::Vector3d::Zero()), diagCovariance(1E-2, 1E-2)));
    EXPECT_TRUE(algo.timeUpdate(0.0));

    StAttMeasurement m;
    m.timeTag = 0.0;
    m.sigma_BN = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    m.covar = (kStMeasStd * kStMeasStd) * Eigen::Matrix3d::Identity();
    m.valid = true;

    EXPECT_FALSE(algo.measurementUpdate(m)) << "a non-finite measurement update must report false";
    EXPECT_FALSE(algo.getLastStAttResiduals().valid) << "no residual is recorded for a bad update";
}

// Through the queue-driven update(), a bad star-tracker reading is rejected by
// applySequential (which calls clear()) and the filter recovers: the residual is invalid
// and the state stays finite.
TEST(InertialSRuKFAlgorithmUpdate, BadMeasurementIsRejectedAndFilterRecovers) {
    InertialSRuKFAlgorithm algo(
        baseConfig(makeState(Eigen::Vector3d(0, 0, 0.05), Eigen::Vector3d::Zero()), diagCovariance(1E-1, 1E-2)));

    // A good update first, to establish a finite anchor.
    StAttData good;
    good.timeTag = 1.0;
    good.sigma_BN = Eigen::Vector3d(0.0, 0.0, 0.05);
    algo.update(1.0, good, RateData{});

    // A NaN star-tracker reading must not corrupt the filter.
    StAttData bad;
    bad.timeTag = 2.0;
    bad.sigma_BN = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    InertialSRuKFOutput const out = algo.update(2.0, bad, RateData{});

    EXPECT_FALSE(out.stAttResiduals.valid) << "bad measurement must not produce a valid residual";
    EXPECT_TRUE(algo.getState().raw().allFinite()) << "filter state must stay finite after a bad update";
    EXPECT_TRUE(algo.getCovariance().allFinite()) << "filter covariance must stay finite after a bad update";

    // A normal update right after the bad one must be processed cleanly -- confirming the
    // clear() left no NaNs behind: the residual is valid, the state stays finite and tracks
    // the measurement, and the covariance is finite.
    StAttData recover;
    recover.timeTag = 3.0;
    recover.sigma_BN = Eigen::Vector3d(0.0, 0.0, 0.05);
    InertialSRuKFOutput const recovered = algo.update(3.0, recover, RateData{});

    EXPECT_TRUE(recovered.stAttResiduals.valid) << "the post-recovery measurement must be applied";
    EXPECT_TRUE(recovered.stAttResiduals.postFit.allFinite()) << "residuals must be finite after recovery";
    EXPECT_TRUE(algo.getState().raw().allFinite()) << "state must be finite after recovery";
    EXPECT_TRUE(algo.getState().get<filtering::MrpAttitude<3>>().isApprox(recover.sigma_BN, 1E-2))
        << "the recovered filter must track the new measurement";
}

// ============================================================================
// Covariance growth: no measurements / process noise.
// ============================================================================

TEST(InertialSRuKFAlgorithmTimeUpdate, ZeroDtLeavesCovarianceUnchanged) {
    Matrix6 const P0 = diagCovariance(1E-2, 1E-3);
    InertialSRuKFAlgorithm algo(configWithProcessNoise(
        makeState(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()), P0, Matrix6::Identity() * 1E-2));

    EXPECT_TRUE(algo.timeUpdate(0.0));
    EXPECT_TRUE(algo.getCovariance().isApprox(P0, 1E-10));
}

TEST(InertialSRuKFAlgorithmTimeUpdate, GrowsCovarianceWithProcessNoise) {
    Matrix6 const P0 = diagCovariance(1E-2, 1E-3);
    InertialSRuKFAlgorithm algo(configWithProcessNoise(
        makeState(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()), P0, Matrix6::Identity() * 1E-2));

    EXPECT_TRUE(algo.timeUpdate(1.0));
    Matrix6 const P = algo.getCovariance();
    EXPECT_GT(P.trace(), P0.trace()) << "process noise should grow covariance";
    EXPECT_TRUE(P.isApprox(P.transpose(), 1E-10)) << "covariance not symmetric";
    EXPECT_TRUE(isPositiveSemiDefinite<6>(P)) << "covariance not PSD";
}

TEST(InertialSRuKFAlgorithmUpdate, WithoutMeasurementsGrowsCovarianceMonotonically) {
    Matrix6 const P0 = diagCovariance(1E-2, 1E-3);
    InertialSRuKFAlgorithm algo(configWithProcessNoise(
        makeState(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()), P0, Matrix6::Identity() * 1E-3));

    // Each update() with empty measurements re-propagates from the unchanged anchor to
    // currentSeconds, so the covariance trace grows with currentSeconds.
    algo.update(1.0, StAttData{}, RateData{});
    double const trace1 = algo.getCovariance().trace();
    algo.update(4.0, StAttData{}, RateData{});
    double const trace4 = algo.getCovariance().trace();
    algo.update(9.0, StAttData{}, RateData{});
    double const trace9 = algo.getCovariance().trace();

    EXPECT_GT(trace1, P0.trace());
    EXPECT_GT(trace4, trace1);
    EXPECT_GT(trace9, trace4);
}

// ============================================================================
// State convergence.
// ============================================================================

TEST(InertialSRuKFAlgorithmConvergence, ConvergesToConstantAttitudeWithExactMeasurements) {
    Eigen::Vector3d const truthSigma(0.1, -0.2, 0.15);  // moderate, inside the unit sphere
    Eigen::Vector3d const truthOmega(0.0, 0.0, 0.0);

    State const initial = makeState(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.05, 0.05, 0.05));
    Matrix6 const P0 = diagCovariance(1E-1, 1E-2);
    InertialSRuKFAlgorithm algo(configWithProcessNoise(initial, P0, Matrix6::Identity() * 1E-8));

    double const dt = 0.5;
    for (int i = 1; i <= 200; ++i) {
        StAttData st;
        st.timeTag = i * dt;
        st.sigma_BN = truthSigma;
        RateData rate;
        rate.timeTag = i * dt;
        rate.rate = truthOmega;
        algo.update(i * dt, st, rate);
    }

    double const attErr = (algo.getState().get<filtering::MrpAttitude<3>>() - truthSigma).norm();
    double const rateErr = (algo.getState().get<filtering::AngularRate<3>>() - truthOmega).norm();
    EXPECT_LT(attErr, 1E-3) << "attitude error " << attErr;
    EXPECT_LT(rateErr, 5E-3) << "rate error " << rateErr;
    EXPECT_LT(algo.getCovariance().trace(), P0.trace());
}

TEST(InertialSRuKFAlgorithmConvergence, ConvergesUnderNoisyMeasurements) {
    Eigen::Vector3d const truthSigma(0.1, -0.2, 0.15);
    Eigen::Vector3d const truthOmega(0.0, 0.0, 0.0);
    double const stStd = 1E-3;
    double const gyroStd = 1E-3;

    State const initial = makeState(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.05, 0.05, 0.05));
    Matrix6 const P0 = diagCovariance(1E-1, 1E-2);
    InertialSRuKFAlgorithm algo(configWithNoiseStds(initial, P0, stStd, gyroStd));

    std::mt19937 gen(42);
    std::normal_distribution<double> noise(0.0, 1.0);
    auto sample = [&](double std) { return Eigen::Vector3d(std * noise(gen), std * noise(gen), std * noise(gen)); };

    double const dt = 0.5;
    for (int i = 1; i <= 400; ++i) {
        StAttData st;
        st.timeTag = i * dt;
        st.sigma_BN = truthSigma + sample(stStd);
        RateData rate;
        rate.timeTag = i * dt;
        rate.rate = truthOmega + sample(gyroStd);
        algo.update(i * dt, st, rate);
    }

    // Generous bound (~10 sigma) keeps the seeded test non-flaky while still
    // demonstrating convergence under noise.
    double const attErr = (algo.getState().get<filtering::MrpAttitude<3>>() - truthSigma).norm();
    double const rateErr = (algo.getState().get<filtering::AngularRate<3>>() - truthOmega).norm();
    EXPECT_LT(attErr, 10 * stStd) << "attitude error " << attErr;
    EXPECT_LT(rateErr, 10 * gyroStd) << "rate error " << rateErr;
    EXPECT_LT(algo.getCovariance().trace(), P0.trace());
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
// InertialSRuKFConfig: factory validation, static validators, getter round-trips.
// ============================================================================

namespace {

// A complete set of valid Config inputs; individual tests override one field to
// drive a specific validation branch.
struct ConfigInputs {
    double alpha = kAlpha;
    double beta = kBeta;
    Matrix6 processNoise = smallProcessNoise();
    State initialState = makeState(Eigen::Vector3d(0.0, 0.0, 0.05), Eigen::Vector3d::Zero());
    Matrix6 initialCovariance = diagCovariance(1E-2, 1E-3);
    double stMeasStd = kStMeasStd;
    double gyroStd = kGyroMeasStd;
};

InertialSRuKFConfig buildConfig(ConfigInputs const& in) {
    return InertialSRuKFConfig::create(
        in.alpha, in.beta, in.processNoise, in.initialState, in.initialCovariance, in.stMeasStd, in.gyroStd);
}

}  // namespace

TEST(InertialSRuKFConfig, ValidInputsDoNotThrow) { EXPECT_NO_THROW(buildConfig({})); }

TEST(InertialSRuKFConfig, RejectsNonPositiveSemiDefiniteProcessNoise) {
    ConfigInputs in;
    in.processNoise = -Matrix6::Identity();  // negative definite
    EXPECT_THROW(buildConfig(in), fsw::invalid_argument);
}

TEST(InertialSRuKFConfig, RejectsNonPositiveSemiDefiniteCovariance) {
    ConfigInputs in;
    in.initialCovariance = -Matrix6::Identity();  // negative definite
    EXPECT_THROW(buildConfig(in), fsw::invalid_argument);
}

TEST(InertialSRuKFConfig, RejectsNegativeNoiseStds) {
    ConfigInputs stStd;
    stStd.stMeasStd = -1E-3;
    EXPECT_THROW(buildConfig(stStd), fsw::invalid_argument);

    ConfigInputs gyroStd;
    gyroStd.gyroStd = -1E-3;
    EXPECT_THROW(buildConfig(gyroStd), fsw::invalid_argument);
}

TEST(InertialSRuKFConfig, AcceptsZeroNoiseStds) {
    ConfigInputs in;
    in.stMeasStd = 0.0;
    in.gyroStd = 0.0;
    EXPECT_NO_THROW(buildConfig(in));
}

TEST(InertialSRuKFConfig, AcceptsPositiveSemiDefiniteSingularMatrices) {
    // Rank-deficient but PSD (one zero eigenvalue) is allowed.
    Matrix6 singular = Matrix6::Identity();
    singular(5, 5) = 0.0;
    ConfigInputs in;
    in.processNoise = singular;
    in.initialCovariance = singular;
    EXPECT_NO_THROW(buildConfig(in));
}

TEST(InertialSRuKFConfig, StaticValidatorsCheckBoundaries) {
    EXPECT_TRUE(InertialSRuKFConfig::isValidStMeasurementNoiseStd(0.0));
    EXPECT_FALSE(InertialSRuKFConfig::isValidStMeasurementNoiseStd(-1E-9));
    EXPECT_TRUE(InertialSRuKFConfig::isValidGyroMeasurementNoiseStd(0.0));
    EXPECT_FALSE(InertialSRuKFConfig::isValidGyroMeasurementNoiseStd(-1E-9));
    EXPECT_TRUE(InertialSRuKFConfig::isValidProcessNoise(Matrix6::Identity()));
    EXPECT_FALSE(InertialSRuKFConfig::isValidProcessNoise(-Matrix6::Identity()));
    EXPECT_TRUE(InertialSRuKFConfig::isValidInitialCovariance(Matrix6::Identity()));
    EXPECT_FALSE(InertialSRuKFConfig::isValidInitialCovariance(-Matrix6::Identity()));
}

TEST(InertialSRuKFConfig, GettersRoundTrip) {
    ConfigInputs in;
    in.processNoise = Matrix6::Identity() * 3E-4;
    in.initialCovariance = diagCovariance(2E-2, 4E-3);
    in.initialState = makeState(Eigen::Vector3d(0.1, -0.2, 0.15), Eigen::Vector3d(0.01, -0.02, 0.03));
    InertialSRuKFConfig const cfg = buildConfig(in);

    EXPECT_DOUBLE_EQ(cfg.getAlpha(), kAlpha);
    EXPECT_DOUBLE_EQ(cfg.getBeta(), kBeta);
    EXPECT_DOUBLE_EQ(cfg.getStMeasurementNoiseStd(), kStMeasStd);
    EXPECT_DOUBLE_EQ(cfg.getGyroMeasurementNoiseStd(), kGyroMeasStd);
    EXPECT_TRUE(cfg.getProcessNoise().isApprox(in.processNoise, 1E-12));
    EXPECT_TRUE(cfg.getInitialCovariance().isApprox(in.initialCovariance, 1E-12));
    EXPECT_TRUE(cfg.getInitialState().raw().isApprox(in.initialState.raw(), 1E-12));
}

}  // namespace filtering::inertialSRuKF
