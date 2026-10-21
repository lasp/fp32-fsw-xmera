#include "flybyPointTestHelpers.hpp"
#include <gtest/gtest.h>
#include <array>
#include <limits>
#include <numbers>
#include <utility>
#include <vector>

namespace {
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

/*! Covariance diagonal with the same one-sigma value on every axis. */
Eigen::Vector3f variance(double sigma) { return Eigen::Vector3f::Constant(static_cast<float>(sigma * sigma)); }

/*! p with a one-sigma state check that only position can trip: positionSigma per axis, 1 m/s velocity sigma. */
ConfigParams withPositionSigma(double positionSigma, ConfigParams p = {}) {
    p.numberOfSigmas = 1.0F;
    p.groundPositionCovarianceDiag_N = variance(positionSigma);
    p.groundVelocityCovarianceDiag_N = variance(1.0);
    return p;
}

// Samples that can't be used: non-finite, (near) zero r or v, |r||v| overflows, (|v|/|r|)^2 overflows.
const std::array<std::pair<Eigen::Vector3f, Eigen::Vector3f>, 6> kUnusableSamples{{
    {Eigen::Vector3f::Constant(kNaN), kV},
    {kR0, Eigen::Vector3f{kInf, 0.0F, 0.0F}},
    {Eigen::Vector3f{1e-4F, 0.0F, 0.0F}, kV},
    {kR0, Eigen::Vector3f{0.0F, 1e-4F, 0.0F}},
    {Eigen::Vector3f{1e20F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 1e20F, 0.0F}},
    {Eigen::Vector3f{1.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 1e20F, 0.0F}},
}};
}  // namespace

// ---------------------------------------------------------------------------
// Correctness against the exact straight-line flyby
// ---------------------------------------------------------------------------

// The output follows the exact straight-line flyby from the ground state, for either orbit-normal sign.
TEST(FlybyPointTest, GroundStateSeedsProfile) {
    for (const int sign : {1, -1}) {
        SCOPED_TRACE("sign " + std::to_string(sign));
        const ConfigParams params{
            .controlPeriod = 300.0, .filterReadPeriods = 100U, .signOfOrbitNormalFrameVector = sign};
        FlybyPointAlgorithm alg(makeConfig(params));
        for (int k = 0; k <= 20; ++k) {
            const double t = k * params.controlPeriod;
            const AttGuideOutput out = alg.updateState(truthAt(t) + kShift, kV);
            expectFlags(out);
            expectReference(out, straightLineReference(kR0, kV, t, sign));
        }
    }
}

// The window average of noisy samples recovers the true state at the window end.
TEST(FlybyPointTest, AverageRecoversStraightLineState) {
    FlybyPointAlgorithm alg(makeConfig({.filterReadPeriods = 4U}));

    AttGuideOutput out{};
    Eigen::Vector3f lastSample;
    for (int k = 0; k <= 3; ++k) {
        lastSample = truthAt(k) + ((k % 2) == 1 ? kShift : Eigen::Vector3f(-kShift));
        out = alg.updateState(lastSample, kV);
        expectFlags(out);
        expectReference(out, straightLineReference(truthAt(k), kV));
    }
    const Eigen::Matrix3d RN = mrpToDcm(Eigen::Vector3d(out.sigma_RN.cast<double>()));
    EXPECT_GT((RN - straightLineReference(lastSample, kV).RN).cwiseAbs().maxCoeff(), 1e-4);
}

// ---------------------------------------------------------------------------
// Ground state
// ---------------------------------------------------------------------------

// create() rejects an unusable or collinear ground state; collinearity follows toleranceForCollinearity.
TEST(FlybyPointTest, ConfigRejectsUnusableOrCollinearGroundState) {
    for (size_t i = 0; i < kUnusableSamples.size(); ++i) {
        const auto& [r, v] = kUnusableSamples[i];
        EXPECT_ANY_THROW((void)makeConfig({.groundPosition_BN_N = r, .groundVelocity_BN_N = v})) << "unusable " << i;
    }

    const Eigen::Vector3d v = kV.cast<double>();
    const Eigen::Vector3d radial = kR0.cast<double>().stableNormalized();
    const Eigen::Vector3d transverse = (v - v.dot(radial) * radial).stableNormalized();
    const auto tilted = [&](double angle) {
        return Eigen::Vector3f(
            (v.stableNorm() * (std::cos(angle) * radial + std::sin(angle) * transverse)).cast<float>());
    };
    const std::array<std::pair<Eigen::Vector3f, float>, 4> collinear{{
        {tilted(0.0), 1e-3F},
        {tilted(std::numbers::pi), 1e-3F},
        {tilted(0.1), 1e-2F},
        {tilted(1e-7), 1e-30F},
    }};
    for (size_t i = 0; i < collinear.size(); ++i) {
        EXPECT_ANY_THROW((void)makeConfig(
            {.toleranceForCollinearity = collinear[i].second, .groundVelocity_BN_N = collinear[i].first}))
            << "collinear " << i;
    }

    FlybyPointAlgorithm alg(makeConfig({.toleranceForCollinearity = 1e-3F, .groundVelocity_BN_N = tilted(0.1)}));
    const AttGuideOutput seeded = alg.updateState(kR0, tilted(0.1));
    expectFlags(seeded);
    expectReference(seeded, straightLineReference(kR0, tilted(0.1)));
}

// setConfig() re-seeds the profile from the new ground state and discards the partial window.
TEST(FlybyPointTest, SetConfigReseedsFromGroundState) {
    FlybyPointAlgorithm alg(makeConfig(withPositionSigma(1e5, {.filterReadPeriods = 5U})));
    for (int k = 0; k <= 1; ++k) {
        expectFlags(alg.updateState(truthAt(k) + kPositionOffset, kV));
    }
    const Eigen::Vector3f newGround = truthAt(2.0) + kShift;
    alg.setConfig(makeConfig(withPositionSigma(1e5, {.filterReadPeriods = 2U, .groundPosition_BN_N = newGround})));

    const AttGuideOutput out0 = alg.updateState(newGround + kShift, kV);
    expectFlags(out0);
    expectReference(out0, straightLineReference(newGround, kV));

    const AttGuideOutput out1 = alg.updateState(newGround + kV + kShift, kV);
    expectFlags(out1);
    expectReference(out1, straightLineReference(newGround + kV + kShift, kV));
}

// reset() re-seeds the profile from the ground state and restarts the window.
TEST(FlybyPointTest, ResetReseedsFromGroundState) {
    FlybyPointAlgorithm alg(makeConfig(withPositionSigma(1e5, {.filterReadPeriods = 3U})));
    for (int k = 0; k <= 2; ++k) {
        alg.updateState(truthAt(k) + kShift, kV);
    }
    alg.updateState(truthAt(3.0) + kPositionOffset, kV);
    alg.reset();

    for (int k = 0; k <= 2; ++k) {
        SCOPED_TRACE("period " + std::to_string(k));
        const AttGuideOutput out = alg.updateState(truthAt(k) + kPositionOffset, kV);
        expectFlags(out, {.stateKnowledge = k == 2});
        expectReference(out, straightLineReference(kR0, kV, k));
    }
}

// ---------------------------------------------------------------------------
// Validity checks on the window average
// ---------------------------------------------------------------------------

// Each check accepts a candidate 1% under its threshold and rejects it 1% over, raising only its own flag.
TEST(FlybyPointTest, ThresholdsAcceptBelowAndRejectAbove) {
    const Eigen::Vector3f nearCandidate = truthAt(1.0) + kShift;
    const double speedOverDistance = kV.stableNorm() / (nearCandidate.cross(kV).stableNorm() / kV.stableNorm());
    const double peakRate = speedOverDistance * 180.0 / std::numbers::pi;
    const double peakAcceleration = 3.0 * std::numbers::sqrt3 / 8.0 * speedOverDistance * peakRate;
    const double positionError = kPositionOffset.stableNorm();
    const Eigen::Vector3f velocityOffset{0.0F, 10.0F, 0.0F};

    for (const double margin : {1.01, 0.99}) {
        const bool reject = margin < 1.0;
        struct Case {
            ConfigParams params;
            Eigen::Vector3f candidatePosition;
            Eigen::Vector3f candidateVelocity;
            ExpectedFlags flags;
        };
        const std::array<Case, 4> cases{{
            {{.maximumRateThreshold = static_cast<float>(margin * peakRate)}, nearCandidate, kV, {.maxRate = reject}},
            {{.maximumAccelerationThreshold = static_cast<float>(margin * peakAcceleration)},
             nearCandidate,
             kV,
             {.maxAcceleration = reject}},
            {withPositionSigma(margin * positionError), truthAt(1.0) + kPositionOffset, kV, {.stateKnowledge = reject}},
            {{.numberOfSigmas = 1.0F,
              .groundPositionCovarianceDiag_N = variance(1e9),
              .groundVelocityCovarianceDiag_N = variance(margin * velocityOffset.stableNorm())},
             truthAt(1.0),
             kV + velocityOffset,
             {.stateKnowledge = reject}},
        }};
        for (size_t i = 0; i < cases.size(); ++i) {
            SCOPED_TRACE("case " + std::to_string(i) + ", margin " + std::to_string(margin));
            FlybyPointAlgorithm alg(makeConfig(cases[i].params));
            alg.updateState(kR0, kV);
            const AttGuideOutput out = alg.updateState(cases[i].candidatePosition, cases[i].candidateVelocity);
            expectFlags(out, cases[i].flags);
            expectReference(out,
                            reject ? straightLineReference(kR0, kV, 1.0)
                                   : straightLineReference(cases[i].candidatePosition, cases[i].candidateVelocity));
        }
    }
}

// A candidate with r parallel to v is rejected as collinear and exceeds the rate and acceleration limits.
TEST(FlybyPointTest, CollinearCandidateIsRejected) {
    for (const float direction : {1.0F, -1.0F}) {
        FlybyPointAlgorithm alg(makeConfig());
        alg.updateState(kR0, kV);
        const AttGuideOutput out = alg.updateState(Eigen::Vector3f{-5e7F, 0.0F, 0.0F}, direction * kV);
        expectFlags(out, {.collinearity = true, .maxRate = true, .maxAcceleration = true});
        expectReference(out, straightLineReference(kR0, kV, 1.0));
    }
}

// ---------------------------------------------------------------------------
// State knowledge check
// ---------------------------------------------------------------------------

// The state check compares a re-read with the ground trajectory, not with the last accepted read.
TEST(FlybyPointTest, StateCheckIsAgainstGroundTrajectory) {
    FlybyPointAlgorithm alg(makeConfig(withPositionSigma(1e4)));
    alg.updateState(kR0, kV);

    const Eigen::Vector3f step{0.0F, 0.0F, 9e3F};
    const AttGuideOutput out1 = alg.updateState(truthAt(1.0) + step, kV);
    expectFlags(out1);
    expectReference(out1, straightLineReference(truthAt(1.0) + step, kV));

    const AttGuideOutput out2 = alg.updateState(truthAt(2.0) + 2.0F * step, kV);
    expectFlags(out2, {.stateKnowledge = true});
    expectReference(out2, straightLineReference(truthAt(1.0) + step, kV, 1.0));
}

// The position bound grows as sqrt(sigmaR0^2 + t^2 sigmaV0^2); only reset() and setConfig() restart t.
TEST(FlybyPointTest, StateCheckBoundGrowsFromGroundState) {
    const FlybyPointConfig cfg = makeConfig({.numberOfSigmas = 1.0F,
                                             .groundPositionCovarianceDiag_N = variance(1e3),
                                             .groundVelocityCovarianceDiag_N = variance(1e3)});
    FlybyPointAlgorithm alg(cfg);
    const Eigen::Vector3f offset{0.0F, 0.0F, 5e3F};
    const auto expectBoundGrows = [&]() {
        for (int k = 0; k <= 6; ++k) {
            SCOPED_TRACE("period " + std::to_string(k));
            const AttGuideOutput out = alg.updateState(truthAt(k) + offset, kV);
            expectFlags(out, {.stateKnowledge = k < 5});
            expectReference(out,
                            k < 5 ? straightLineReference(kR0, kV, k) : straightLineReference(truthAt(k) + offset, kV));
        }
    };

    expectBoundGrows();
    alg.reset();
    {
        SCOPED_TRACE("after reset()");
        expectBoundGrows();
    }
    alg.setConfig(cfg);
    {
        SCOPED_TRACE("after setConfig()");
        expectBoundGrows();
    }
}

// The state check is per axis, not on the error norm, and scaled by numberOfSigmas.
TEST(FlybyPointTest, StateCheckIsPerAxisAndScaledByNumberOfSigmas) {
    const ConfigParams params{.numberOfSigmas = 3.0F,
                              .groundPositionCovarianceDiag_N = Eigen::Vector3f{1e8F, 1e8F, 1e6F},
                              .groundVelocityCovarianceDiag_N = variance(1.0)};
    const std::array<std::pair<Eigen::Vector3f, bool>, 4> cases{{
        {Eigen::Vector3f{2.9e4F, 2.9e4F, 0.0F}, false},
        {Eigen::Vector3f{0.0F, 0.0F, 2.9e3F}, false},
        {Eigen::Vector3f{0.0F, 0.0F, 3.1e3F}, true},
        {Eigen::Vector3f{3.1e4F, 0.0F, 0.0F}, true},
    }};
    for (size_t i = 0; i < cases.size(); ++i) {
        SCOPED_TRACE("case " + std::to_string(i));
        const auto& [offset, reject] = cases[i];
        FlybyPointAlgorithm alg(makeConfig(params));
        alg.updateState(kR0, kV);
        const AttGuideOutput out = alg.updateState(truthAt(1.0) + offset, kV);
        expectFlags(out, {.stateKnowledge = reject});
        expectReference(
            out, reject ? straightLineReference(kR0, kV, 1.0) : straightLineReference(truthAt(1.0) + offset, kV));
    }
}

// ---------------------------------------------------------------------------
// Averaging window and unusable inputs
// ---------------------------------------------------------------------------

// Unusable samples are flagged, left out of the average, and counted at the window end.
TEST(FlybyPointTest, UnusableSamplesAreFlaggedAndLeftOut) {
    FlybyPointAlgorithm alg(makeConfig({.filterReadPeriods = 6U}));

    // A window of unusable samples only
    for (int k = 0; k <= 5; ++k) {
        const auto& [r, v] = kUnusableSamples[static_cast<size_t>(k)];
        const AttGuideOutput out = alg.updateState(r, v);
        expectFlags(out, {.inputSampleRejected = true, .rejectedSamplesInWindow = k == 5 ? 6U : 0U});
        expectReference(out, straightLineReference(kR0, kV, k));
    }

    // A window of shifted samples with one unusable sample: the average of the others is read
    for (int k = 6; k <= 11; ++k) {
        const bool unusable = k == 8;
        const AttGuideOutput out =
            unusable ? alg.updateState(kUnusableSamples[0].first, kV) : alg.updateState(truthAt(k) + kShift, kV);
        expectFlags(out, {.inputSampleRejected = unusable, .rejectedSamplesInWindow = k == 11 ? 1U : 0U});
        expectReference(out,
                        k == 11 ? straightLineReference(truthAt(k) + kShift, kV) : straightLineReference(kR0, kV, k));
    }
}

// An average whose velocities cancel is unusable, so no re-read is attempted.
TEST(FlybyPointTest, CancellingVelocitiesMakeNoAttempt) {
    FlybyPointAlgorithm alg(makeConfig({.filterReadPeriods = 2U}));
    expectFlags(alg.updateState(kR0, kV));
    const AttGuideOutput out = alg.updateState(truthAt(1.0), -kV);
    expectFlags(out);
    expectReference(out, straightLineReference(kR0, kV, 1.0));
}

// A re-read is attempted every filterReadPeriods periods, also for an inexact control period.
TEST(FlybyPointTest, ReReadsOnEveryFilterReadPeriod) {
    FlybyPointAlgorithm alg(makeConfig(withPositionSigma(1e3, {.controlPeriod = 0.1F, .filterReadPeriods = 3U})));
    for (int k = 0; k < 30; ++k) {
        SCOPED_TRACE("period " + std::to_string(k));
        const AttGuideOutput out = alg.updateState(kR0 + kPositionOffset, kV);
        expectFlags(out, {.stateKnowledge = (k % 3) == 2});
        expectReference(out, straightLineReference(kR0, kV, k * 0.1));
    }
}

// A rejected average starts a new window; the next window's good average is accepted.
TEST(FlybyPointTest, RejectedAverageRestartsWindow) {
    FlybyPointAlgorithm alg(makeConfig(withPositionSigma(1e3, {.filterReadPeriods = 3U})));
    for (int k = 0; k <= 5; ++k) {
        SCOPED_TRACE("period " + std::to_string(k));
        const Eigen::Vector3f sample = k <= 2 ? Eigen::Vector3f(truthAt(k) + kPositionOffset) : truthAt(k);
        const AttGuideOutput out = alg.updateState(sample, kV);
        expectFlags(out, {.stateKnowledge = k == 2});
        expectReference(out, straightLineReference(kR0, kV, k));
    }
}

// ---------------------------------------------------------------------------
// Zero reference fallback
// ---------------------------------------------------------------------------

// A reference that overflows float is output as zero, and the output recovers at the next accepted re-read.
TEST(FlybyPointTest, UnrepresentableReferenceFallsBackToZero) {
    // Ground state with f0 = |v| / |r| = 1.5e19 1/s: f0^2 fits in float, so it is usable, but 2 f0^2 in the
    // acceleration overflows. Its wide velocity covariance lets the re-read of the test trajectory pass the state
    // check.
    const Eigen::Vector3f r{1.0F, 0.0F, 0.0F};
    const Eigen::Vector3f v = 1.5e19F * Eigen::Vector3f{-1.0F, 1.0F, 0.0F}.stableNormalized();
    for (const int sign : {1, -1}) {
        SCOPED_TRACE("sign " + std::to_string(sign));
        FlybyPointAlgorithm alg(makeConfig({.filterReadPeriods = 3U,
                                            .signOfOrbitNormalFrameVector = sign,
                                            .groundPosition_BN_N = r,
                                            .groundVelocity_BN_N = v,
                                            .groundVelocityCovarianceDiag_N = Eigen::Vector3f::Constant(5e37F)}));
        for (int k = 0; k <= 2; ++k) {
            SCOPED_TRACE("period " + std::to_string(k));
            const AttGuideOutput out = alg.updateState(truthAt(k), kV);
            expectFlags(out);
            if (k < 2) {
                expectZeroGuidance(out);
            } else {
                expectReference(out, straightLineReference(kR0, kV, 2.0, sign));
            }
        }
    }
}

// At a near-head-on closest approach (sine of the r-v angle 4.5e-5) the rate denominator doesn't cancel in float: the
// output is the finite peak rate |v| / d, not the zero fallback.
TEST(FlybyPointTest, NearHeadOnClosestApproachIsFinite) {
    const Eigen::Vector3f r0{-1e8F, 4.5e3F, 0.0F};  // [m] closest approach distance d = 4.5 km
    const Eigen::Vector3f v{1e5F, 0.0F, 0.0F};      // [m/s] closest approach at t = 1000 s
    FlybyPointAlgorithm alg(makeConfig({.filterReadPeriods = 2000U,
                                        .toleranceForCollinearity = 1e-12F,
                                        .groundPosition_BN_N = r0,
                                        .groundVelocity_BN_N = v}));
    AttGuideOutput out{};
    for (int k = 0; k <= 1000; ++k) {
        out = alg.updateState((r0.cast<double>() + static_cast<double>(k) * v.cast<double>()).cast<float>(), v);
    }

    EXPECT_FALSE(out.sigma_RN.isZero(0.0F) && out.omega_RN_N.isZero(0.0F));
    const double peakRate = 1e5 / 4.5e3;  // [rad/s]
    EXPECT_NEAR(out.omega_RN_N.cast<double>().stableNorm(), peakRate, 1e-2 * peakRate);
    const Eigen::Matrix3d RN = mrpToDcm(Eigen::Vector3d(out.sigma_RN.cast<double>()));
    EXPECT_LT((RN - straightLineReference(r0, v, 1000.0).RN).cwiseAbs().maxCoeff(), 1e-2);
}

// ---------------------------------------------------------------------------
// Configuration and regression
// ---------------------------------------------------------------------------

// The configuration rejects every out-of-range value.
TEST(FlybyPointTest, ConfigRejectsInvalidValues) {
    EXPECT_NO_THROW((void)FlybyPointAlgorithm(makeConfig()));
    EXPECT_NO_THROW((void)makeConfig({.signOfOrbitNormalFrameVector = -1}));
    EXPECT_NO_THROW((void)makeConfig({.groundPositionCovarianceDiag_N = Eigen::Vector3f{1e8F, 1e-6F, 4e8F},
                                      .groundVelocityCovarianceDiag_N = Eigen::Vector3f{1.0F, 1e-6F, 4.0F}}));

    const std::array<ConfigParams, 28> invalid{{
        {.controlPeriod = 0.0F},
        {.controlPeriod = -1.0F},
        {.controlPeriod = kNaN},
        {.controlPeriod = kInf},
        {.filterReadPeriods = 0U},
        {.toleranceForCollinearity = 0.0F},
        {.toleranceForCollinearity = -1e-3F},
        {.signOfOrbitNormalFrameVector = 0},
        {.signOfOrbitNormalFrameVector = 2},
        {.maximumRateThreshold = 0.0F},
        {.maximumRateThreshold = -1.0F},
        {.maximumAccelerationThreshold = 0.0F},
        {.maximumAccelerationThreshold = -1.0F},
        {.numberOfSigmas = 0.0F},
        {.numberOfSigmas = -1.0F},
        {.numberOfSigmas = kNaN},
        {.groundPosition_BN_N = Eigen::Vector3f::Zero()},
        {.groundPosition_BN_N = Eigen::Vector3f{0.0F, kInf, 0.0F}},
        {.groundVelocity_BN_N = Eigen::Vector3f{kNaN, 0.0F, 0.0F}},
        {.groundVelocity_BN_N = kR0},
        {.groundPositionCovarianceDiag_N = Eigen::Vector3f{1.0F, 0.0F, 1.0F}},
        {.groundPositionCovarianceDiag_N = Eigen::Vector3f{1.0F, -1.0F, 1.0F}},
        {.groundPositionCovarianceDiag_N = Eigen::Vector3f{kNaN, 1.0F, 1.0F}},
        {.groundPositionCovarianceDiag_N = Eigen::Vector3f{1.0F, 1.0F, kInf}},
        {.groundVelocityCovarianceDiag_N = Eigen::Vector3f{0.0F, 1.0F, 1.0F}},
        {.groundVelocityCovarianceDiag_N = Eigen::Vector3f{-1.0F, 1.0F, 1.0F}},
        {.groundVelocityCovarianceDiag_N = Eigen::Vector3f{1.0F, kNaN, 1.0F}},
        {.groundVelocityCovarianceDiag_N = Eigen::Vector3f{1.0F, 1.0F, kInf}},
    }};
    for (size_t i = 0; i < invalid.size(); ++i) {
        EXPECT_ANY_THROW((void)makeConfig(invalid[i])) << "case " << i;
    }
}

// create() stores the state knowledge settings and the ground state as given.
TEST(FlybyPointTest, ConfigStoresValues) {
    const ConfigParams params{.numberOfSigmas = 2.5F,
                              .groundPosition_BN_N = kR0,
                              .groundVelocity_BN_N = kV,
                              .groundPositionCovarianceDiag_N = Eigen::Vector3f{1e8F, 2e8F, 3e8F},
                              .groundVelocityCovarianceDiag_N = Eigen::Vector3f{1.0F, 2.0F, 3.0F}};
    const FlybyPointConfig cfg = makeConfig(params);
    EXPECT_EQ(cfg.getNumberOfSigmas(), params.numberOfSigmas);
    EXPECT_EQ(cfg.getGroundPosition_BN_N(), params.groundPosition_BN_N);
    EXPECT_EQ(cfg.getGroundVelocity_BN_N(), params.groundVelocity_BN_N);
    EXPECT_EQ(cfg.getGroundPositionCovarianceDiag_N(), params.groundPositionCovarianceDiag_N);
    EXPECT_EQ(cfg.getGroundVelocityCovarianceDiag_N(), params.groundVelocityCovarianceDiag_N);
}

// The algorithm matches the single-precision reference model through accepted, rejected and unusable windows.
TEST(FlybyPointTest, MatchesReferenceModel) {
    const ConfigParams params = withPositionSigma(1e5, {.controlPeriod = 0.3F, .filterReadPeriods = 2U});
    std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> samples;
    for (int k = 0; k <= 8; ++k) {
        const Eigen::Vector3f r = truthAt(k * params.controlPeriod);
        if (k == 2 || k == 3) {
            samples.emplace_back(r + kPositionOffset, kV);
        } else if (k == 5) {
            samples.push_back(kUnusableSamples[0]);
        } else {
            samples.emplace_back(r + kShift, kV);
        }
    }
    expectMatchesReferenceModel(makeConfig(params), samples);
}
