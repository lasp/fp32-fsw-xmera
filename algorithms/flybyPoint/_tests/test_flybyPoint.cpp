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

// The reference follows a straight-line flyby exactly, from the first sample through closest approach, for either
// sign of the orbit normal. The window is longer than the run, so every output is extrapolated from the first sample.
TEST(FlybyPointTest, SeedFollowsStraightLineFlyby) {
    for (const int sign : {1, -1}) {
        SCOPED_TRACE("sign " + std::to_string(sign));
        const ConfigParams params{
            .controlPeriod = 300.0, .filterReadPeriods = 100U, .signOfOrbitNormalFrameVector = sign};
        FlybyPointAlgorithm alg(makeConfig(params));
        for (int k = 0; k <= 20; ++k) {
            const double t = k * params.controlPeriod;
            const AttGuideOutput out = alg.updateState(truthAt(t), kV);
            expectFlags(out);
            expectReference(out, straightLineReference(kR0, kV, t, sign));
        }
    }
}

// The window average recovers the true state at the window end: every sample is moved to the window end along its
// velocity, and noise that alternates in sign cancels. Until then the output keeps following the first sample, and a
// read of the last noisy sample alone would point visibly elsewhere.
TEST(FlybyPointTest, AverageRecoversStraightLineState) {
    FlybyPointAlgorithm alg(makeConfig({.filterReadPeriods = 4U}));
    alg.updateState(kR0, kV);

    AttGuideOutput out{};
    Eigen::Vector3d lastSample;
    for (int k = 1; k <= 4; ++k) {
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
// distance, and the distance from the predicted position. An accepted candidate moves the frame; after a rejection
// the frame keeps following the last accepted read.
TEST(FlybyPointTest, ThresholdsAcceptBelowAndRejectAbove) {
    const Eigen::Vector3d nearCandidate = truthAt(1.0) + kShift;
    const double speedOverDistance = kV.stableNorm() / (nearCandidate.cross(kV).stableNorm() / kV.stableNorm());
    const double peakRate = speedOverDistance * 180.0 / std::numbers::pi;
    const double peakAcceleration = 3.0 * std::numbers::sqrt3 / 8.0 * speedOverDistance * peakRate;
    const double positionError = kPositionOffset.stableNorm();

    for (const double margin : {1.01, 0.99}) {
        const bool reject = margin < 1.0;
        struct Case {
            ConfigParams params;
            Eigen::Vector3d candidate;
            ExpectedFlags flags;
        };
        const std::array<Case, 3> cases{{
            {{.maximumRateThreshold = static_cast<float>(margin * peakRate)}, nearCandidate, {.maxRate = reject}},
            {{.maximumAccelerationThreshold = static_cast<float>(margin * peakAcceleration)},
             nearCandidate,
             {.maxAcceleration = reject}},
            {{.positionKnowledgeSigma = static_cast<float>(margin * positionError)},
             truthAt(1.0) + kPositionOffset,
             {.positionKnowledge = reject}},
        }};
        for (size_t i = 0; i < cases.size(); ++i) {
            SCOPED_TRACE("case " + std::to_string(i) + ", margin " + std::to_string(margin));
            FlybyPointAlgorithm alg(makeConfig(cases[i].params));
            alg.updateState(kR0, kV);
            const AttGuideOutput out = alg.updateState(cases[i].candidate, kV);
            expectFlags(out, cases[i].flags);
            expectReference(
                out, reject ? straightLineReference(kR0, kV, 1.0) : straightLineReference(cases[i].candidate, kV));
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

// The position check predicts from the last accepted read, so after a small velocity change the next read is judged
// against the new trajectory. The second read is accepted, though it is 10 m off the line from the first sample.
TEST(FlybyPointTest, PositionCheckPredictsFromLastAcceptedRead) {
    FlybyPointAlgorithm alg(makeConfig({.positionKnowledgeSigma = 1.0F}));
    alg.updateState(kR0, kV);

    const Eigen::Vector3d v1 = kV + Eigen::Vector3d{0, 10, 0};
    const Eigen::Vector3d r1 = truthAt(1.0);
    const AttGuideOutput out1 = alg.updateState(r1, v1);
    expectFlags(out1);
    expectReference(out1, straightLineReference(r1, v1));

    const AttGuideOutput out2 = alg.updateState(r1 + v1, v1);
    expectFlags(out2);
    expectReference(out2, straightLineReference(r1 + v1, v1));
}

// ---------------------------------------------------------------------------
// First sample and unusable inputs
// ---------------------------------------------------------------------------

// A first sample whose r and v are (nearly) collinear defines no orbit plane and is refused: the output stays zero and
// the collinearity flag is raised, and the next good sample seeds. How close counts as collinear is set by
// toleranceForCollinearity, down to an orbit normal too short to normalize.
TEST(FlybyPointTest, CollinearSeedIsRefused) {
    const Eigen::Vector3d radial = kR0.stableNormalized();
    const Eigen::Vector3d transverse = (kV - kV.dot(radial) * radial).stableNormalized();
    const auto tilted = [&](double angle) {
        return kV.stableNorm() * (std::cos(angle) * radial + std::sin(angle) * transverse);
    };

    const std::array<std::pair<Eigen::Vector3d, float>, 4> refused{{
        {tilted(0.0), 1e-3F},
        {tilted(std::numbers::pi), 1e-3F},
        {tilted(0.1), 1e-2F},
        {tilted(1e-14), 1e-30F},
    }};
    for (size_t i = 0; i < refused.size(); ++i) {
        SCOPED_TRACE("case " + std::to_string(i));
        FlybyPointAlgorithm alg(makeConfig({.toleranceForCollinearity = refused[i].second}));
        const AttGuideOutput out = alg.updateState(kR0, refused[i].first);
        expectZeroGuidance(out);
        expectFlags(out, {.collinearity = true});

        const AttGuideOutput seeded = alg.updateState(kR0, kV);
        expectFlags(seeded);
        expectReference(seeded, straightLineReference(kR0, kV));
    }

    FlybyPointAlgorithm alg(makeConfig({.toleranceForCollinearity = 1e-3F}));
    const AttGuideOutput seeded = alg.updateState(kR0, tilted(0.1));
    expectFlags(seeded);
    expectReference(seeded, straightLineReference(kR0, tilted(0.1)));
}

// An unusable sample is flagged and never used. Before the first seed it gives a zero output and doesn't seed. In a
// window it is left out of the average while the output keeps following the last accepted read; a window with no
// usable sample makes no attempt, and the window end reports how many samples were left out.
TEST(FlybyPointTest, UnusableSamplesAreFlaggedAndLeftOut) {
    FlybyPointAlgorithm alg(makeConfig({.filterReadPeriods = 6U}));
    for (const auto& [r, v] : kUnusableSamples) {
        const AttGuideOutput out = alg.updateState(r, v);
        expectZeroGuidance(out);
        expectFlags(out, {.inputSampleRejected = true});
    }
    expectReference(alg.updateState(kR0, kV), straightLineReference(kR0, kV));

    // A window of unusable samples only
    for (int k = 1; k <= 6; ++k) {
        const auto& [r, v] = kUnusableSamples[static_cast<size_t>(k - 1)];
        const AttGuideOutput out = alg.updateState(r, v);
        expectFlags(out, {.inputSampleRejected = true, .rejectedSamplesInWindow = k == 6 ? 6U : 0U});
        expectReference(out, straightLineReference(kR0, kV, k));
    }

    // A window of shifted samples with one unusable sample: the average of the others is read
    for (int k = 7; k <= 12; ++k) {
        const bool unusable = k == 9;
        const AttGuideOutput out =
            unusable ? alg.updateState(kUnusableSamples[0].first, kV) : alg.updateState(truthAt(k) + kShift, kV);
        expectFlags(out, {.inputSampleRejected = unusable, .rejectedSamplesInWindow = k == 12 ? 1U : 0U});
        expectReference(out,
                        k == 12 ? straightLineReference(truthAt(k) + kShift, kV) : straightLineReference(kR0, kV, k));
    }
}

// Usable samples whose velocities cancel give an average with no velocity, which can't be read: no check is made and
// the output keeps following the last accepted read.
TEST(FlybyPointTest, CancellingVelocitiesMakeNoAttempt) {
    FlybyPointAlgorithm alg(makeConfig({.filterReadPeriods = 2U}));
    alg.updateState(kR0, kV);
    expectFlags(alg.updateState(truthAt(1.0), kV));
    const AttGuideOutput out = alg.updateState(truthAt(2.0), -kV);
    expectFlags(out);
    expectReference(out, straightLineReference(kR0, kV, 2.0));
}

// A reference too large to represent falls back to zero, for either sign. With f0 = |v|/|r| = 1e20 1/s only the first
// period overflows single precision and the next is output normally; with f0 = 1e150 1/s the acceleration overflows
// even double precision afterwards, and the output stays zero.
TEST(FlybyPointTest, UnrepresentableReferenceFallsBackToZero) {
    const Eigen::Vector3d r{1, 0, 0};
    const Eigen::Vector3d direction = Eigen::Vector3d{-1, 1, 0}.stableNormalized();
    for (const int sign : {1, -1}) {
        SCOPED_TRACE("sign " + std::to_string(sign));
        const FlybyPointConfig cfg = makeConfig({.filterReadPeriods = 3U, .signOfOrbitNormalFrameVector = sign});

        FlybyPointAlgorithm recovering(cfg);
        const Eigen::Vector3d v = 1e20 * direction;
        expectZeroGuidance(recovering.updateState(r, v));
        const AttGuideOutput next = recovering.updateState(r + v, v);
        expectFlags(next);
        expectReference(next, straightLineReference(r, v, 1.0, sign));

        FlybyPointAlgorithm overflowing(cfg);
        const Eigen::Vector3d vHuge = 1e150 * direction;
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

// A re-read is attempted on exactly every filterReadPeriods-th period, even for a control period with no exact binary
// representation. Every sample is far off the prediction, so each attempt is rejected, which makes the attempts
// visible.
TEST(FlybyPointTest, ReReadsOnEveryFilterReadPeriod) {
    FlybyPointAlgorithm alg(
        makeConfig({.controlPeriod = 0.1, .filterReadPeriods = 3U, .positionKnowledgeSigma = 1.0F}));
    alg.updateState(kR0, kV);
    for (int k = 1; k <= 30; ++k) {
        SCOPED_TRACE("period " + std::to_string(k));
        const AttGuideOutput out = alg.updateState(kR0 + kPositionOffset, kV);
        expectFlags(out, {.positionKnowledge = (k % 3) == 0});
        expectReference(out, straightLineReference(kR0, kV, k * 0.1));
    }
}

// A rejected average starts a fresh window: no attempt is made until another full window has passed, and that window's
// good average is then accepted.
TEST(FlybyPointTest, RejectedAverageRestartsWindow) {
    FlybyPointAlgorithm alg(makeConfig({.filterReadPeriods = 3U, .positionKnowledgeSigma = 1.0F}));
    alg.updateState(kR0, kV);
    for (int k = 1; k <= 6; ++k) {
        SCOPED_TRACE("period " + std::to_string(k));
        const Eigen::Vector3d sample = k <= 3 ? Eigen::Vector3d(truthAt(k) + kPositionOffset) : truthAt(k);
        const AttGuideOutput out = alg.updateState(sample, kV);
        expectFlags(out, {.positionKnowledge = k == 3});
        expectReference(out, straightLineReference(kR0, kV, k));
    }
}

// setConfig() discards the partial window, also when the new window is shorter than the periods already elapsed. The
// two bad samples before the change are dropped, and the new 2-period window reads only the good samples after it.
TEST(FlybyPointTest, SetConfigDiscardsPartialWindow) {
    FlybyPointAlgorithm alg(makeConfig({.filterReadPeriods = 5U, .positionKnowledgeSigma = 1e5F}));
    alg.updateState(kR0, kV);
    for (int k = 1; k <= 2; ++k) {
        expectFlags(alg.updateState(truthAt(k) + kPositionOffset, kV));
    }
    alg.setConfig(makeConfig({.filterReadPeriods = 2U, .positionKnowledgeSigma = 1e5F}));

    const AttGuideOutput out3 = alg.updateState(truthAt(3) + kShift, kV);
    expectFlags(out3);
    expectReference(out3, straightLineReference(kR0, kV, 3.0));

    const AttGuideOutput out4 = alg.updateState(truthAt(4) + kShift, kV);
    expectFlags(out4);
    expectReference(out4, straightLineReference(truthAt(4) + kShift, kV));
}

// reset() forgets the profile and the partial window: the output is zero until the next usable sample, which seeds
// from itself, and the next re-read comes a full window after that seed.
TEST(FlybyPointTest, ResetForgetsProfileAndWindow) {
    FlybyPointAlgorithm alg(makeConfig({.filterReadPeriods = 3U, .positionKnowledgeSigma = 1.0F}));
    alg.updateState(kR0, kV);
    alg.updateState(truthAt(1.0), kV);
    alg.reset();

    const AttGuideOutput unseeded = alg.updateState(kUnusableSamples[0].first, kV);
    expectZeroGuidance(unseeded);
    expectFlags(unseeded, {.inputSampleRejected = true});

    const Eigen::Vector3d newSeed = truthAt(2.0) + kShift;
    expectReference(alg.updateState(newSeed, kV), straightLineReference(newSeed, kV));
    for (int k = 1; k <= 3; ++k) {
        const AttGuideOutput out = alg.updateState(newSeed + static_cast<double>(k) * kV + kPositionOffset, kV);
        expectFlags(out, {.positionKnowledge = k == 3});
        expectReference(out, straightLineReference(newSeed, kV, k));
    }
}

// ---------------------------------------------------------------------------
// Configuration and regression
// ---------------------------------------------------------------------------

// The configuration rejects every out-of-range value.
TEST(FlybyPointTest, ConfigRejectsInvalidValues) {
    EXPECT_NO_THROW((void)FlybyPointAlgorithm(makeConfig()));
    EXPECT_NO_THROW((void)makeConfig({.signOfOrbitNormalFrameVector = -1}));

    const std::array<ConfigParams, 15> invalid{{
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
        {.positionKnowledgeSigma = 0.0F},
        {.positionKnowledgeSigma = -1e3F},
    }};
    for (size_t i = 0; i < invalid.size(); ++i) {
        EXPECT_ANY_THROW((void)makeConfig(invalid[i])) << "case " << i;
    }
}

// The algorithm matches the test's double-precision copy of the same algorithm, period by period. The sequence holds
// an accepted window, a rejected window, an unusable sample and another accepted window, so this catches bookkeeping
// and single-precision errors; the formulas themselves are checked against the exact flyby above.
TEST(FlybyPointTest, MatchesReferenceModel) {
    const ConfigParams params{.controlPeriod = 0.3, .filterReadPeriods = 2U, .positionKnowledgeSigma = 1e5F};
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> samples;
    for (int k = 0; k <= 8; ++k) {
        const Eigen::Vector3d r = truthAt(k * params.controlPeriod);
        if (k == 3 || k == 4) {
            samples.emplace_back(r + kPositionOffset, kV);
        } else if (k == 5) {
            samples.push_back(kUnusableSamples[0]);
        } else {
            samples.emplace_back(r + kShift, kV);
        }
    }
    expectMatchesReferenceModel(makeConfig(params), samples);
}
