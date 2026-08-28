#include "thrDesatDutyCycleTestHelpers.hpp"
#include "utilities/fsw/freestandingInvalidArgument.h"

#include <array>
#include <cstddef>
#include <vector>

namespace {

// A representative per-thruster desaturation force command: an eight-thruster RCS cluster where the mapping
// stage has left two thrusters idle. Deliberately not uniform, so a gate that scrambled the array would show.
const std::vector<float> kNominalForces = {1.2F, 0.2F, 0.0F, 1.6F, 1.2F, 0.2F, 1.6F, 0.0F};

// The nominal cadence: fire for one control period, then hold off for four so the wheels can re-settle.
constexpr uint32_t kNominalFiringPeriods = 1U;
constexpr uint32_t kNominalSettlingPeriods = 4U;

// Enough updates to cover several whole cycles of the nominal cadence.
constexpr uint32_t kManyUpdates = 20U;

ThrDesatDutyCycleConfig nominalConfig() {
    return ThrDesatDutyCycleConfig::create(kNominalFiringPeriods, kNominalSettlingPeriods);
}

// Assert that a cadence is accepted and round-trips through the getters.
void expectCadenceRoundTrips(uint32_t firingPeriods, uint32_t settlingPeriods) {
    const ThrDesatDutyCycleConfig cfg = ThrDesatDutyCycleConfig::create(firingPeriods, settlingPeriods);

    EXPECT_EQ(cfg.getFiringPeriods(), firingPeriods);
    EXPECT_EQ(cfg.getSettlingPeriods(), settlingPeriods);
    EXPECT_EQ(cfg.getCycleLength(), firingPeriods + settlingPeriods);
}

}  // namespace

TEST(ThrDesatDutyCycle, PassesForceThroughDuringTheFiringWindow) {
    const auto cfg = ThrDesatDutyCycleConfig::create(3U, 5U);
    ThrDesatDutyCycleAlgorithm alg{cfg};
    const auto thrusterForceCmd = makeForceCmd(kNominalForces);

    for (uint32_t update = 0U; update < cfg.getFiringPeriods(); ++update) {
        const auto gated = alg.update(thrusterForceCmd);
        for (std::size_t i = 0; i < kNominalForces.size(); ++i) {
            EXPECT_EQ(gated.at(i), kNominalForces[i]) << "update " << update << " thruster " << i;
        }
    }
}

TEST(ThrDesatDutyCycle, CommandsZeroForceDuringTheSettlingWindow) {
    const auto cfg = ThrDesatDutyCycleConfig::create(3U, 5U);
    ThrDesatDutyCycleAlgorithm alg{cfg};
    const auto thrusterForceCmd = makeForceCmd(kNominalForces);

    // Burn through the firing window first.
    for (uint32_t update = 0U; update < cfg.getFiringPeriods(); ++update) {
        (void)alg.update(thrusterForceCmd);
    }

    const std::array<float, kMaxThrusterCount> allZero{};
    for (uint32_t update = 0U; update < cfg.getSettlingPeriods(); ++update) {
        EXPECT_EQ(alg.update(thrusterForceCmd), allZero) << "settling update " << update;
    }
}

// With no settling periods the gate is fully open, which is how a caller disables the duty cycle.
TEST(ThrDesatDutyCycle, AlwaysFiresWhenThereAreNoSettlingPeriods) {
    ThrDesatDutyCycleAlgorithm alg{ThrDesatDutyCycleConfig::create(1U, 0U)};
    const auto thrusterForceCmd = makeForceCmd(kNominalForces);

    for (uint32_t update = 0U; update < kManyUpdates; ++update) {
        EXPECT_EQ(alg.update(thrusterForceCmd), thrusterForceCmd) << "update " << update;
    }
}

// The pulse train repeats with the cycle length; this pins the phase of the nominal 1-in-5 cadence explicitly
// rather than only through the reference implementation.
TEST(ThrDesatDutyCycle, CadenceRepeatsWithTheCycleLength) {
    ThrDesatDutyCycleAlgorithm alg{nominalConfig()};
    const auto thrusterForceCmd = makeForceCmd(kNominalForces);

    const std::vector<bool> expectedPattern = {true, false, false, false, false};
    for (uint32_t update = 0U; update < kManyUpdates; ++update) {
        const bool fired = alg.update(thrusterForceCmd).at(0) != 0.0F;
        EXPECT_EQ(fired, expectedPattern[update % expectedPattern.size()]) << "update " << update;
    }
}

TEST(ThrDesatDutyCycle, MatchesReferenceAcrossCases) {
    const auto thrusterForceCmd = makeForceCmd(kNominalForces);

    regressionTestThrDesatDutyCycle(thrusterForceCmd, ThrDesatDutyCycleConfig::create(1U, 0U), kManyUpdates);
    regressionTestThrDesatDutyCycle(thrusterForceCmd, ThrDesatDutyCycleConfig::create(1U, 1U), kManyUpdates);
    regressionTestThrDesatDutyCycle(thrusterForceCmd, nominalConfig(), kManyUpdates);
    regressionTestThrDesatDutyCycle(thrusterForceCmd, ThrDesatDutyCycleConfig::create(3U, 2U), kManyUpdates);
    regressionTestThrDesatDutyCycle(thrusterForceCmd, ThrDesatDutyCycleConfig::create(7U, 1U), kManyUpdates);
    // A settling window longer than the run: the gate fires once and then stays shut throughout.
    regressionTestThrDesatDutyCycle(thrusterForceCmd, ThrDesatDutyCycleConfig::create(1U, 100U), kManyUpdates);
}

TEST(ThrDesatDutyCycle, DeliversTheConfiguredDutyRatio) {
    const auto thrusterForceCmd = makeForceCmd(kNominalForces);

    testFiringCountMatchesDutyRatio(thrusterForceCmd, ThrDesatDutyCycleConfig::create(1U, 0U), 5U);
    testFiringCountMatchesDutyRatio(thrusterForceCmd, nominalConfig(), 5U);
    testFiringCountMatchesDutyRatio(thrusterForceCmd, ThrDesatDutyCycleConfig::create(3U, 2U), 4U);
}

TEST(ThrDesatDutyCycle, CadenceIsIndependentOfTheCommandedForce) {
    testCadenceIsIndependentOfCommand(makeForceCmd(kNominalForces), nominalConfig(), kManyUpdates);
    testCadenceIsIndependentOfCommand(
        makeForceCmd(kNominalForces), ThrDesatDutyCycleConfig::create(3U, 2U), kManyUpdates);
}

TEST(ThrDesatDutyCycle, OutputIsAlwaysTheInputOrZero) {
    testOutputIsInputOrZero(makeForceCmd(kNominalForces), nominalConfig(), kManyUpdates);
}

TEST(ThrDesatDutyCycle, GateActsOnTheWholeThrusterArray) {
    testGateActsOnTheWholeArray(makeForceCmd(kNominalForces), nominalConfig(), kManyUpdates);
}

TEST(ThrDesatDutyCycle, ReInitializeRestartsTheCadence) {
    testReInitializeRestartsCadence(makeForceCmd(kNominalForces), nominalConfig(), 10U, 3U);
    testReInitializeRestartsCadence(makeForceCmd(kNominalForces), ThrDesatDutyCycleConfig::create(3U, 2U), 10U, 7U);
}

// setConfig() installs a new cadence without restarting it, which is what separates reconfigure() from
// reInitialize() at the adapter level.
TEST(ThrDesatDutyCycle, SetConfigChangesTheCadenceWithoutRestartingIt) {
    ThrDesatDutyCycleAlgorithm alg{ThrDesatDutyCycleConfig::create(1U, 3U)};
    const auto thrusterForceCmd = makeForceCmd(kNominalForces);

    // Fire, then advance two updates into the settling window.
    EXPECT_NE(alg.update(thrusterForceCmd).at(0), 0.0F);
    EXPECT_EQ(alg.update(thrusterForceCmd).at(0), 0.0F);
    EXPECT_EQ(alg.update(thrusterForceCmd).at(0), 0.0F);

    // Widening the firing window to cover the whole cycle opens the gate from the next update onwards; the
    // counter keeps its phase, it is only reinterpreted against the new window.
    alg.setConfig(ThrDesatDutyCycleConfig::create(4U, 0U));
    for (uint32_t update = 0U; update < kManyUpdates; ++update) {
        EXPECT_NE(alg.update(thrusterForceCmd).at(0), 0.0F) << "update " << update;
    }
}

// A cadence shortened under a counter that has already run past the new cycle length must still produce a
// valid phase rather than reading out of range.
TEST(ThrDesatDutyCycle, HandlesACadenceShortenedBelowTheCurrentPhase) {
    ThrDesatDutyCycleAlgorithm alg{ThrDesatDutyCycleConfig::create(1U, 20U)};
    const auto thrusterForceCmd = makeForceCmd(kNominalForces);

    // Advance well past the cycle length the gate is about to be given.
    for (uint32_t update = 0U; update < 15U; ++update) {
        (void)alg.update(thrusterForceCmd);
    }

    alg.setConfig(ThrDesatDutyCycleConfig::create(1U, 1U));

    // The phase folds back into the new two-period cycle, so the gate must alternate from here on.
    bool previousFired = alg.update(thrusterForceCmd).at(0) != 0.0F;
    for (uint32_t update = 0U; update < kManyUpdates; ++update) {
        const bool fired = alg.update(thrusterForceCmd).at(0) != 0.0F;
        EXPECT_NE(fired, previousFired) << "update " << update;
        previousFired = fired;
    }
}

// A zero command stays zero whether the gate is open or shut, so the module never invents a firing.
TEST(ThrDesatDutyCycle, ZeroCommandStaysZero) {
    ThrDesatDutyCycleAlgorithm alg{nominalConfig()};
    const std::array<float, kMaxThrusterCount> allZero{};

    for (uint32_t update = 0U; update < kManyUpdates; ++update) {
        EXPECT_EQ(alg.update(allZero), allZero) << "update " << update;
    }
}

// ---------------------------------------------------------------------------
// Config tests.
// ---------------------------------------------------------------------------

TEST(ThrDesatDutyCycleConfigTest, AcceptsValidInputs) {
    EXPECT_NO_THROW((void)ThrDesatDutyCycleConfig::create(1U, 0U));  // gate held open, duty cycling disabled
    EXPECT_NO_THROW((void)ThrDesatDutyCycleConfig::create(1U, 4U));
    EXPECT_NO_THROW((void)ThrDesatDutyCycleConfig::create(3U, 2U));
    EXPECT_NO_THROW((void)ThrDesatDutyCycleConfig::create(100U, 10000U));
    // Both extremes of the representable cycle length: a single firing period followed by the longest
    // possible hold-off, and a firing window that fills the whole range with no hold-off at all.
    EXPECT_NO_THROW((void)ThrDesatDutyCycleConfig::create(1U, UINT32_MAX - 1U));
    EXPECT_NO_THROW((void)ThrDesatDutyCycleConfig::create(UINT32_MAX, 0U));
}

// Whatever cadence is configured must come back unchanged from the getters, and the derived cycle length must
// agree with it. The config stores the counts verbatim, so these are exact comparisons.
TEST(ThrDesatDutyCycleConfigTest, GettersRoundTrip) {
    expectCadenceRoundTrips(1U, 0U);
    expectCadenceRoundTrips(1U, 4U);
    expectCadenceRoundTrips(3U, 2U);
    expectCadenceRoundTrips(100U, 10000U);
    // The derived cycle length must stay exact at the top of the range, where a wrap would be silent.
    expectCadenceRoundTrips(1U, UINT32_MAX - 1U);
    expectCadenceRoundTrips(UINT32_MAX, 0U);
}

// A cycle that never fires would hold the thrusters off forever, silently disabling desaturation.
TEST(ThrDesatDutyCycleConfigTest, RejectsZeroFiringPeriods) {
    EXPECT_THROW((void)ThrDesatDutyCycleConfig::create(0U, 5U), fsw::invalid_argument);
    EXPECT_THROW((void)ThrDesatDutyCycleConfig::create(0U, 0U), fsw::invalid_argument);
}

// A cycle length that wrapped around would come out shorter than its own firing window.
TEST(ThrDesatDutyCycleConfigTest, RejectsCycleLengthOverflow) {
    EXPECT_THROW((void)ThrDesatDutyCycleConfig::create(2U, UINT32_MAX - 1U), fsw::invalid_argument);
    EXPECT_THROW((void)ThrDesatDutyCycleConfig::create(UINT32_MAX, 1U), fsw::invalid_argument);
}

// The public predicates must agree with create() exactly at the boundaries, since callers (the C shim's
// validateConfig, and Ada through it) use them to pre-check a cadence before constructing.
TEST(ThrDesatDutyCycleConfigTest, StaticValidatorsCheckBoundaries) {
    EXPECT_FALSE(ThrDesatDutyCycleConfig::isValidFiringPeriods(0U));
    EXPECT_TRUE(ThrDesatDutyCycleConfig::isValidFiringPeriods(1U));
    EXPECT_TRUE(ThrDesatDutyCycleConfig::isValidFiringPeriods(UINT32_MAX));

    EXPECT_TRUE(ThrDesatDutyCycleConfig::isValidSettlingPeriods(0U, 1U));
    EXPECT_TRUE(ThrDesatDutyCycleConfig::isValidSettlingPeriods(UINT32_MAX - 1U, 1U));
    EXPECT_FALSE(ThrDesatDutyCycleConfig::isValidSettlingPeriods(UINT32_MAX, 1U));
    EXPECT_TRUE(ThrDesatDutyCycleConfig::isValidSettlingPeriods(0U, UINT32_MAX));
    EXPECT_FALSE(ThrDesatDutyCycleConfig::isValidSettlingPeriods(1U, UINT32_MAX));
}
