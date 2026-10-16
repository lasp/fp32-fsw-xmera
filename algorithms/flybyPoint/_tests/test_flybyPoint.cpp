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

// The reference follows a straight-line flyby exactly, from the ground state through closest approach, for either sign
// of the orbit normal. The window is longer than the run, so every output is extrapolated from the ground state, and
// the samples, shifted off the line, have no effect.
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

// The window average recovers the true state at the window end: every sample is moved to the window end along its
// velocity, and noise that alternates in sign cancels. Until then the output follows the ground state, and a read of
// the last noisy sample alone would point visibly elsewhere.
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
// Validity checks on the window average
// ---------------------------------------------------------------------------

// Each check accepts a candidate 1% under its threshold and rejects it 1% over, raising only its own flag: the
// predicted peak rate |v|/d and peak acceleration (3 sqrt(3)/8)(|v|/d)^2 in degrees, with d the closest-approach
// distance, and the position and velocity errors on one axis against one propagated ground sigma. The first period's
// sample is the ground state itself; the candidate comes one period later. An accepted candidate moves the frame; after
// a rejection the frame keeps following the ground state.
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
            {{.numberOfSigmas = 1.0F,
              .groundPositionCovarianceDiag_N = variance(margin * positionError),
              .groundVelocityCovarianceDiag_N = variance(1.0)},
             truthAt(1.0) + kPositionOffset,
             kV,
             {.stateKnowledge = reject}},
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

// A candidate moving straight toward or away from the body (r x v exactly zero) is rejected as collinear, and its
// closest-approach distance is zero, so its predicted peak rate and acceleration exceed any limit.
TEST(FlybyPointTest, CollinearCandidateIsRejected) {
    for (const double direction : {1.0, -1.0}) {
        FlybyPointAlgorithm alg(makeConfig());
        alg.updateState(kR0, kV);
        const AttGuideOutput out = alg.updateState(Eigen::Vector3d{-5e7, 0, 0}, direction * kV);
        expectFlags(out, {.collinearity = true, .maxRate = true, .maxAcceleration = true});
        expectReference(out, straightLineReference(kR0, kV, 1.0));
    }
}

// The state check is against the ground-based trajectory, not the last accepted read. A read 9 km off the ground line
// is within the 10 km bound and is accepted; the next read, 18 km off the ground line, is rejected, though it is only
// 9 km from the accepted read's prediction, and the output keeps following the accepted read.
TEST(FlybyPointTest, StateCheckIsAgainstGroundTrajectory) {
    FlybyPointAlgorithm alg(makeConfig({.numberOfSigmas = 1.0F,
                                        .groundPositionCovarianceDiag_N = variance(1e4),
                                        .groundVelocityCovarianceDiag_N = variance(1.0)}));
    alg.updateState(kR0, kV);

    const Eigen::Vector3d step{0, 0, 9e3};
    const AttGuideOutput out1 = alg.updateState(truthAt(1.0) + step, kV);
    expectFlags(out1);
    expectReference(out1, straightLineReference(truthAt(1.0) + step, kV));

    const AttGuideOutput out2 = alg.updateState(truthAt(2.0) + 2.0 * step, kV);
    expectFlags(out2, {.stateKnowledge = true});
    expectReference(out2, straightLineReference(truthAt(1.0) + step, kV, 1.0));
}

// The ground uncertainty is propagated from the ground state: the position bound on each axis grows as
// sqrt(sigmaR0^2 + t^2 sigmaV0^2). With 1 m and 1 m/s, samples 5 m off the line are rejected until the bound
// sqrt(1 + t^2) passes 5 m at t = 5 s, and accepted then. reset() restarts the time, so the bound is 1 m again.
TEST(FlybyPointTest, StateKnowledgeBoundGrowsFromGroundState) {
    FlybyPointAlgorithm alg(makeConfig({.numberOfSigmas = 1.0F,
                                        .groundPositionCovarianceDiag_N = variance(1.0),
                                        .groundVelocityCovarianceDiag_N = variance(1.0)}));
    const Eigen::Vector3d offset{0, 0, 5};
    for (int k = 0; k <= 5; ++k) {
        SCOPED_TRACE("period " + std::to_string(k));
        const AttGuideOutput out = alg.updateState(truthAt(k) + offset, kV);
        expectFlags(out, {.stateKnowledge = k < 5});
        expectReference(out,
                        k < 5 ? straightLineReference(kR0, kV, k) : straightLineReference(truthAt(k) + offset, kV));
    }

    alg.reset();
    const AttGuideOutput out = alg.updateState(kR0 + offset, kV);
    expectFlags(out, {.stateKnowledge = true});
    expectReference(out, straightLineReference(kR0, kV));
}

// ---------------------------------------------------------------------------
// Ground state and unusable inputs
// ---------------------------------------------------------------------------

// A ground state that is unusable, or whose r and v are (nearly) collinear and so define no orbit plane, is rejected by
// the configuration. How close counts as collinear is set by toleranceForCollinearity, down to an orbit normal too
// short to normalize. A ground velocity tilted enough from r for the tolerance seeds the profile.
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

// An unusable sample is flagged and never used. It is left out of the average while the output keeps following the
// last accepted read (here the ground state); a window with no usable sample makes no attempt, and the window end
// reports how many samples were left out.
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

// Usable samples whose velocities cancel give an average with no velocity, which can't be read: no check is made and
// the output keeps following the last accepted read.
TEST(FlybyPointTest, CancellingVelocitiesMakeNoAttempt) {
    FlybyPointAlgorithm alg(makeConfig({.filterReadPeriods = 2U}));
    expectFlags(alg.updateState(kR0, kV));
    const AttGuideOutput out = alg.updateState(truthAt(1.0), -kV);
    expectFlags(out);
    expectReference(out, straightLineReference(kR0, kV, 1.0));
}

// A reference too large to represent falls back to zero, for either sign. With a ground state of f0 = |v|/|r| = 1e20
// 1/s only the first period overflows single precision and the next is output normally; with f0 = 1e150 1/s the
// acceleration overflows even double precision afterwards, and the output stays zero. The window is longer than the
// run, so the output follows the ground state.
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
// Re-read cadence and window bookkeeping
// ---------------------------------------------------------------------------

// A re-read is attempted on exactly every filterReadPeriods-th period, counting the first, even for a control period
// with no exact binary representation. Every sample is far off the prediction, so each attempt is rejected, which
// makes the attempts visible.
TEST(FlybyPointTest, ReReadsOnEveryFilterReadPeriod) {
    FlybyPointAlgorithm alg(makeConfig({.controlPeriod = 0.1,
                                        .filterReadPeriods = 3U,
                                        .numberOfSigmas = 1.0F,
                                        .groundPositionCovarianceDiag_N = variance(1.0),
                                        .groundVelocityCovarianceDiag_N = variance(1.0)}));
    for (int k = 0; k < 30; ++k) {
        SCOPED_TRACE("period " + std::to_string(k));
        const AttGuideOutput out = alg.updateState(kR0 + kPositionOffset, kV);
        expectFlags(out, {.stateKnowledge = (k % 3) == 2});
        expectReference(out, straightLineReference(kR0, kV, k * 0.1));
    }
}

// A rejected average starts a fresh window: no attempt is made until another full window has passed, and that window's
// good average is then accepted.
TEST(FlybyPointTest, RejectedAverageRestartsWindow) {
    FlybyPointAlgorithm alg(makeConfig({.filterReadPeriods = 3U,
                                        .numberOfSigmas = 1.0F,
                                        .groundPositionCovarianceDiag_N = variance(1.0),
                                        .groundVelocityCovarianceDiag_N = variance(1.0)}));
    for (int k = 0; k <= 5; ++k) {
        SCOPED_TRACE("period " + std::to_string(k));
        const Eigen::Vector3d sample = k <= 2 ? Eigen::Vector3d(truthAt(k) + kPositionOffset) : truthAt(k);
        const AttGuideOutput out = alg.updateState(sample, kV);
        expectFlags(out, {.stateKnowledge = k == 2});
        expectReference(out, straightLineReference(kR0, kV, k));
    }
}

// setConfig() re-seeds the profile from the new ground state, at zero elapsed time on the next period, and discards the
// partial window, also when the new window is shorter than the periods already elapsed. The two bad samples before the
// change are dropped, and the new 2-period window reads only the good samples after it.
TEST(FlybyPointTest, SetConfigReseedsFromGroundState) {
    FlybyPointAlgorithm alg(makeConfig({.filterReadPeriods = 5U,
                                        .numberOfSigmas = 1.0F,
                                        .groundPositionCovarianceDiag_N = variance(1e5),
                                        .groundVelocityCovarianceDiag_N = variance(1.0)}));
    for (int k = 0; k <= 1; ++k) {
        expectFlags(alg.updateState(truthAt(k) + kPositionOffset, kV));
    }
    const Eigen::Vector3d newGround = truthAt(2.0) + kShift;
    alg.setConfig(makeConfig({.filterReadPeriods = 2U,
                              .numberOfSigmas = 1.0F,
                              .groundPosition_BN_N = newGround,
                              .groundPositionCovarianceDiag_N = variance(1e5),
                              .groundVelocityCovarianceDiag_N = variance(1.0)}));

    const AttGuideOutput out0 = alg.updateState(newGround + kShift, kV);
    expectFlags(out0);
    expectReference(out0, straightLineReference(newGround, kV));

    const AttGuideOutput out1 = alg.updateState(newGround + kV + kShift, kV);
    expectFlags(out1);
    expectReference(out1, straightLineReference(newGround + kV + kShift, kV));
}

// reset() re-seeds the profile from the ground state, at zero elapsed time on the next period, and discards the partial
// window: after an accepted re-read moved the frame, the output follows the ground state again, and the next re-read
// comes a full window after the reset.
TEST(FlybyPointTest, ResetReseedsFromGroundState) {
    FlybyPointAlgorithm alg(makeConfig({.filterReadPeriods = 3U,
                                        .numberOfSigmas = 1.0F,
                                        .groundPositionCovarianceDiag_N = variance(1e5),
                                        .groundVelocityCovarianceDiag_N = variance(1.0)}));
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

// The configuration stores the ground-based state and covariance diagonals it was given.
TEST(FlybyPointTest, ConfigStoresGroundState) {
    const ConfigParams params{.groundPosition_BN_N = kR0,
                              .groundVelocity_BN_N = kV,
                              .groundPositionCovarianceDiag_N = Eigen::Vector3d{1e8, 2e8, 3e8},
                              .groundVelocityCovarianceDiag_N = Eigen::Vector3d{1.0, 2.0, 3.0}};
    const FlybyPointConfig cfg = makeConfig(params);
    EXPECT_EQ(cfg.getGroundPosition_BN_N(), params.groundPosition_BN_N);
    EXPECT_EQ(cfg.getGroundVelocity_BN_N(), params.groundVelocity_BN_N);
    EXPECT_EQ(cfg.getGroundPositionCovarianceDiag_N(), params.groundPositionCovarianceDiag_N);
    EXPECT_EQ(cfg.getGroundVelocityCovarianceDiag_N(), params.groundVelocityCovarianceDiag_N);
}

// The algorithm matches the test's double-precision copy of the same algorithm, period by period. The sequence holds
// an accepted window, a rejected window, an unusable sample and another accepted window, so this catches bookkeeping
// and single-precision errors; the formulas themselves are checked against the exact flyby above.
TEST(FlybyPointTest, MatchesReferenceModel) {
    const ConfigParams params{.controlPeriod = 0.3,
                              .filterReadPeriods = 2U,
                              .numberOfSigmas = 1.0F,
                              .groundPositionCovarianceDiag_N = variance(1e5),
                              .groundVelocityCovarianceDiag_N = variance(1.0)};
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
