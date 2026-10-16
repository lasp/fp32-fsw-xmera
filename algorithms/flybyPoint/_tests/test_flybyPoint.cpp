#include "flybyPointTestHelpers.hpp"
#include <gtest/gtest.h>
#include <array>
#include <limits>
#include <numbers>
#include <utility>
#include <vector>

namespace {
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr float kNaNF = std::numeric_limits<float>::quiet_NaN();

/*! Covariance diagonal with the same one-sigma value on every axis. */
Eigen::Vector3d variance(double sigma) { return Eigen::Vector3d::Constant(sigma * sigma); }

/*! p with a one-sigma state check that only position can trip: positionSigma per axis, 1 m/s velocity sigma. */
ConfigParams withPositionSigma(double positionSigma, ConfigParams p = {}) {
    p.numberOfSigmas = 1.0F;
    p.groundPositionCovarianceDiag_N = variance(positionSigma);
    p.groundVelocityCovarianceDiag_N = variance(1.0);
    return p;
}

// Samples that can't be used: non-finite, (near) zero r or v, |r||v| overflows, (|v|/|r|)^2 overflows.
const std::array<std::pair<Eigen::Vector3d, Eigen::Vector3d>, 6> kUnusableSamples{{
    {Eigen::Vector3d::Constant(kNaN), kV},
    {kR0, Eigen::Vector3d{kInf, 0, 0}},
    {Eigen::Vector3d{1e-4, 0, 0}, kV},
    {kR0, Eigen::Vector3d{0, 1e-4, 0}},
    {Eigen::Vector3d{1e200, 0, 0}, Eigen::Vector3d{0, 1e200, 0}},
    {Eigen::Vector3d{1, 0, 0}, Eigen::Vector3d{0, 1e200, 0}},
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
    Eigen::Vector3d lastSample;
    for (int k = 0; k <= 3; ++k) {
        lastSample = truthAt(k) + ((k % 2) == 1 ? kShift : Eigen::Vector3d(-kShift));
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

    const Eigen::Vector3d radial = kR0.stableNormalized();
    const Eigen::Vector3d transverse = (kV - kV.dot(radial) * radial).stableNormalized();
    const auto tilted = [&](double angle) {
        return kV.stableNorm() * (std::cos(angle) * radial + std::sin(angle) * transverse);
    };
    const std::array<std::pair<Eigen::Vector3d, float>, 4> collinear{{
        {tilted(0.0), 1e-3F},
        {tilted(std::numbers::pi), 1e-3F},
        {tilted(0.1), 1e-2F},
        {tilted(1e-14), 1e-30F},
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
    const Eigen::Vector3d newGround = truthAt(2.0) + kShift;
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
    const Eigen::Vector3d nearCandidate = truthAt(1.0) + kShift;
    const double speedOverDistance = kV.stableNorm() / (nearCandidate.cross(kV).stableNorm() / kV.stableNorm());
    const double peakRate = speedOverDistance * 180.0 / std::numbers::pi;
    const double peakAcceleration = 3.0 * std::numbers::sqrt3 / 8.0 * speedOverDistance * peakRate;
    const double positionError = kPositionOffset.stableNorm();
    const Eigen::Vector3d velocityOffset{0, 10, 0};

    for (const double margin : {1.01, 0.99}) {
        const bool reject = margin < 1.0;
        struct Case {
            ConfigParams params;
            Eigen::Vector3d candidatePosition;
            Eigen::Vector3d candidateVelocity;
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
    for (const double direction : {1.0, -1.0}) {
        FlybyPointAlgorithm alg(makeConfig());
        alg.updateState(kR0, kV);
        const AttGuideOutput out = alg.updateState(Eigen::Vector3d{-5e7, 0, 0}, direction * kV);
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

    const Eigen::Vector3d step{0, 0, 9e3};
    const AttGuideOutput out1 = alg.updateState(truthAt(1.0) + step, kV);
    expectFlags(out1);
    expectReference(out1, straightLineReference(truthAt(1.0) + step, kV));

    const AttGuideOutput out2 = alg.updateState(truthAt(2.0) + 2.0 * step, kV);
    expectFlags(out2, {.stateKnowledge = true});
    expectReference(out2, straightLineReference(truthAt(1.0) + step, kV, 1.0));
}

// The position bound grows as sqrt(sigmaR0^2 + t^2 sigmaV0^2); only reset() and setConfig() restart t.
TEST(FlybyPointTest, StateCheckBoundGrowsFromGroundState) {
    const FlybyPointConfig cfg = makeConfig({.numberOfSigmas = 1.0F,
                                             .groundPositionCovarianceDiag_N = variance(1.0),
                                             .groundVelocityCovarianceDiag_N = variance(1.0)});
    FlybyPointAlgorithm alg(cfg);
    const Eigen::Vector3d offset{0, 0, 5};
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
                              .groundPositionCovarianceDiag_N = Eigen::Vector3d{1e8, 1e8, 1e6},
                              .groundVelocityCovarianceDiag_N = variance(1.0)};
    const std::array<std::pair<Eigen::Vector3d, bool>, 4> cases{{
        {Eigen::Vector3d{2.9e4, 2.9e4, 0}, false},
        {Eigen::Vector3d{0, 0, 2.9e3}, false},
        {Eigen::Vector3d{0, 0, 3.1e3}, true},
        {Eigen::Vector3d{3.1e4, 0, 0}, true},
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
    FlybyPointAlgorithm alg(makeConfig(withPositionSigma(1.0, {.controlPeriod = 0.1, .filterReadPeriods = 3U})));
    for (int k = 0; k < 30; ++k) {
        SCOPED_TRACE("period " + std::to_string(k));
        const AttGuideOutput out = alg.updateState(kR0 + kPositionOffset, kV);
        expectFlags(out, {.stateKnowledge = (k % 3) == 2});
        expectReference(out, straightLineReference(kR0, kV, k * 0.1));
    }
}

// A rejected average starts a new window; the next window's good average is accepted.
TEST(FlybyPointTest, RejectedAverageRestartsWindow) {
    FlybyPointAlgorithm alg(makeConfig(withPositionSigma(1.0, {.filterReadPeriods = 3U})));
    for (int k = 0; k <= 5; ++k) {
        SCOPED_TRACE("period " + std::to_string(k));
        const Eigen::Vector3d sample = k <= 2 ? Eigen::Vector3d(truthAt(k) + kPositionOffset) : truthAt(k);
        const AttGuideOutput out = alg.updateState(sample, kV);
        expectFlags(out, {.stateKnowledge = k == 2});
        expectReference(out, straightLineReference(kR0, kV, k));
    }
}

// ---------------------------------------------------------------------------
// Zero reference fallback
// ---------------------------------------------------------------------------

// A reference that overflows float or double is output as zero, and recovers once finite.
TEST(FlybyPointTest, UnrepresentableReferenceFallsBackToZero) {
    const Eigen::Vector3d r{1, 0, 0};
    const Eigen::Vector3d direction = Eigen::Vector3d{-1, 1, 0}.stableNormalized();
    for (const int sign : {1, -1}) {
        SCOPED_TRACE("sign " + std::to_string(sign));
        const Eigen::Vector3d v = 1e20 * direction;
        FlybyPointAlgorithm recovering(makeConfig({.filterReadPeriods = 4U,
                                                   .signOfOrbitNormalFrameVector = sign,
                                                   .groundPosition_BN_N = r,
                                                   .groundVelocity_BN_N = v}));
        expectZeroGuidance(recovering.updateState(r, v));
        const AttGuideOutput next = recovering.updateState(r + v, v);
        expectFlags(next);
        expectReference(next, straightLineReference(r, v, 1.0, sign));

        const Eigen::Vector3d vHuge = 1e150 * direction;
        FlybyPointAlgorithm overflowing(makeConfig({.filterReadPeriods = 4U,
                                                    .signOfOrbitNormalFrameVector = sign,
                                                    .groundPosition_BN_N = r,
                                                    .groundVelocity_BN_N = vHuge}));
        for (int k = 0; k <= 2; ++k) {
            const AttGuideOutput out = overflowing.updateState(r, vHuge);
            expectZeroGuidance(out);
            expectFlags(out);
        }
    }
}

// ---------------------------------------------------------------------------
// Configuration and regression
// ---------------------------------------------------------------------------

// The configuration rejects every out-of-range value.
TEST(FlybyPointTest, ConfigRejectsInvalidValues) {
    EXPECT_NO_THROW((void)FlybyPointAlgorithm(makeConfig()));
    EXPECT_NO_THROW((void)makeConfig({.signOfOrbitNormalFrameVector = -1}));
    EXPECT_NO_THROW((void)makeConfig({.groundPositionCovarianceDiag_N = Eigen::Vector3d{1e8, 1e-6, 4e8},
                                      .groundVelocityCovarianceDiag_N = Eigen::Vector3d{1.0, 1e-6, 4.0}}));

    const std::array<ConfigParams, 28> invalid{{
        {.controlPeriod = 0.0},
        {.controlPeriod = -1.0},
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
        {.numberOfSigmas = kNaNF},
        {.groundPosition_BN_N = Eigen::Vector3d::Zero()},
        {.groundPosition_BN_N = Eigen::Vector3d{0, kInf, 0}},
        {.groundVelocity_BN_N = Eigen::Vector3d{kNaN, 0, 0}},
        {.groundVelocity_BN_N = kR0},
        {.groundPositionCovarianceDiag_N = Eigen::Vector3d{1.0, 0.0, 1.0}},
        {.groundPositionCovarianceDiag_N = Eigen::Vector3d{1.0, -1.0, 1.0}},
        {.groundPositionCovarianceDiag_N = Eigen::Vector3d{kNaN, 1.0, 1.0}},
        {.groundPositionCovarianceDiag_N = Eigen::Vector3d{1.0, 1.0, kInf}},
        {.groundVelocityCovarianceDiag_N = Eigen::Vector3d{0.0, 1.0, 1.0}},
        {.groundVelocityCovarianceDiag_N = Eigen::Vector3d{-1.0, 1.0, 1.0}},
        {.groundVelocityCovarianceDiag_N = Eigen::Vector3d{1.0, kNaN, 1.0}},
        {.groundVelocityCovarianceDiag_N = Eigen::Vector3d{1.0, 1.0, kInf}},
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
                              .groundPositionCovarianceDiag_N = Eigen::Vector3d{1e8, 2e8, 3e8},
                              .groundVelocityCovarianceDiag_N = Eigen::Vector3d{1.0, 2.0, 3.0}};
    const FlybyPointConfig cfg = makeConfig(params);
    EXPECT_EQ(cfg.getNumberOfSigmas(), params.numberOfSigmas);
    EXPECT_EQ(cfg.getGroundPosition_BN_N(), params.groundPosition_BN_N);
    EXPECT_EQ(cfg.getGroundVelocity_BN_N(), params.groundVelocity_BN_N);
    EXPECT_EQ(cfg.getGroundPositionCovarianceDiag_N(), params.groundPositionCovarianceDiag_N);
    EXPECT_EQ(cfg.getGroundVelocityCovarianceDiag_N(), params.groundVelocityCovarianceDiag_N);
}

// The algorithm matches the double-precision reference model through accepted, rejected and unusable windows.
TEST(FlybyPointTest, MatchesReferenceModel) {
    const ConfigParams params = withPositionSigma(1e5, {.controlPeriod = 0.3, .filterReadPeriods = 2U});
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> samples;
    for (int k = 0; k <= 8; ++k) {
        const Eigen::Vector3d r = truthAt(k * params.controlPeriod);
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
