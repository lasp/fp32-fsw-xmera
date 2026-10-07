#include "utilities/fsw/freestandingInvalidArgument.h"
#include "vectorDutyCycleTestHelpers.hpp"

#include <Eigen/Core>
#include <vector>

namespace {

// A representative input vector. Deliberately distinct per axis with mixed signs, so a gate
// that scrambled or rescaled the components would show.
const std::vector<float> kNominalComponents = {1.2e-2F, -3.5e-3F, 7.0e-4F};

// The nominal cadence: pass through for one control period, then hold off for four.
constexpr uint32_t kNominalOnPeriods = 1U;
constexpr uint32_t kNominalOffPeriods = 4U;

// Enough updates to cover several whole cycles of the nominal cadence.
constexpr uint32_t kManyUpdates = 20U;

VectorDutyCycleConfig nominalConfig() { return VectorDutyCycleConfig::create(kNominalOnPeriods, kNominalOffPeriods); }

// Assert that a cadence is accepted and round-trips through the getters.
void expectCadenceRoundTrips(uint32_t onPeriods, uint32_t offPeriods) {
    const VectorDutyCycleConfig cfg = VectorDutyCycleConfig::create(onPeriods, offPeriods);

    EXPECT_EQ(cfg.getOnPeriods(), onPeriods);
    EXPECT_EQ(cfg.getOffPeriods(), offPeriods);
}

}  // namespace

TEST(VectorDutyCycle, PassesVectorThroughDuringTheOnWindow) {
    const auto cfg = VectorDutyCycleConfig::create(3U, 5U);
    VectorDutyCycleAlgorithm alg{cfg};
    const Eigen::Vector3f inputVector = makeInputVector(kNominalComponents);

    for (uint32_t update = 0U; update < cfg.getOnPeriods(); ++update) {
        EXPECT_EQ(alg.update(inputVector), inputVector) << "update " << update;
    }
}

TEST(VectorDutyCycle, OutputsZeroDuringTheOffWindow) {
    const auto cfg = VectorDutyCycleConfig::create(3U, 5U);
    VectorDutyCycleAlgorithm alg{cfg};
    const Eigen::Vector3f inputVector = makeInputVector(kNominalComponents);

    // Burn through the on window first.
    for (uint32_t update = 0U; update < cfg.getOnPeriods(); ++update) {
        (void)alg.update(inputVector);
    }

    const Eigen::Vector3f allZero = Eigen::Vector3f::Zero();
    for (uint32_t update = 0U; update < cfg.getOffPeriods(); ++update) {
        EXPECT_EQ(alg.update(inputVector), allZero) << "off update " << update;
    }
}

// With no off periods the gate is fully open, which is how a caller disables the duty cycle.
TEST(VectorDutyCycle, AlwaysOnWhenThereAreNoOffPeriods) {
    VectorDutyCycleAlgorithm alg{VectorDutyCycleConfig::create(1U, 0U)};
    const Eigen::Vector3f inputVector = makeInputVector(kNominalComponents);

    for (uint32_t update = 0U; update < kManyUpdates; ++update) {
        EXPECT_EQ(alg.update(inputVector), inputVector) << "update " << update;
    }
}

// The pulse train repeats with the cycle length; this pins the phase of the nominal 1-in-5 cadence explicitly
// rather than only through the reference implementation.
TEST(VectorDutyCycle, CadenceRepeatsWithTheCycleLength) {
    VectorDutyCycleAlgorithm alg{nominalConfig()};
    const Eigen::Vector3f inputVector = makeInputVector(kNominalComponents);

    const std::vector<bool> expectedPattern = {true, false, false, false, false};
    for (uint32_t update = 0U; update < kManyUpdates; ++update) {
        const bool wasOn = alg.update(inputVector)(0) != 0.0F;
        EXPECT_EQ(wasOn, expectedPattern[update % expectedPattern.size()]) << "update " << update;
    }
}

TEST(VectorDutyCycle, MatchesReferenceAcrossCases) {
    const Eigen::Vector3f inputVector = makeInputVector(kNominalComponents);

    regressionTestVectorDutyCycle(inputVector, VectorDutyCycleConfig::create(1U, 0U), kManyUpdates);
    regressionTestVectorDutyCycle(inputVector, VectorDutyCycleConfig::create(1U, 1U), kManyUpdates);
    regressionTestVectorDutyCycle(inputVector, nominalConfig(), kManyUpdates);
    regressionTestVectorDutyCycle(inputVector, VectorDutyCycleConfig::create(3U, 2U), kManyUpdates);
    regressionTestVectorDutyCycle(inputVector, VectorDutyCycleConfig::create(7U, 1U), kManyUpdates);
    // An off window longer than the run: the gate passes through once and then stays shut throughout.
    regressionTestVectorDutyCycle(inputVector, VectorDutyCycleConfig::create(1U, 100U), kManyUpdates);
}

TEST(VectorDutyCycle, DeliversTheConfiguredDutyRatio) {
    const Eigen::Vector3f inputVector = makeInputVector(kNominalComponents);

    testOnCountMatchesDutyRatio(inputVector, VectorDutyCycleConfig::create(1U, 0U), 5U);
    testOnCountMatchesDutyRatio(inputVector, nominalConfig(), 5U);
    testOnCountMatchesDutyRatio(inputVector, VectorDutyCycleConfig::create(3U, 2U), 4U);
}

TEST(VectorDutyCycle, CadenceIsIndependentOfTheInput) {
    testCadenceIsIndependentOfInput(makeInputVector(kNominalComponents), nominalConfig(), kManyUpdates);
    testCadenceIsIndependentOfInput(
        makeInputVector(kNominalComponents), VectorDutyCycleConfig::create(3U, 2U), kManyUpdates);
}

TEST(VectorDutyCycle, OutputIsAlwaysTheInputOrZero) {
    testOutputIsInputOrZero(makeInputVector(kNominalComponents), nominalConfig(), kManyUpdates);
}

TEST(VectorDutyCycle, GateActsOnTheWholeVector) {
    testGateActsOnTheWholeVector(makeInputVector(kNominalComponents), nominalConfig(), kManyUpdates);
}

TEST(VectorDutyCycle, ReInitializeRestartsTheCadence) {
    testReInitializeRestartsCadence(makeInputVector(kNominalComponents), nominalConfig(), 10U, 3U);
    testReInitializeRestartsCadence(
        makeInputVector(kNominalComponents), VectorDutyCycleConfig::create(3U, 2U), 10U, 7U);
}

// setConfig() installs a new cadence without restarting it, which is what separates reconfigure() from
// reInitialize() at the adapter level.
TEST(VectorDutyCycle, SetConfigChangesTheCadenceWithoutRestartingIt) {
    VectorDutyCycleAlgorithm alg{VectorDutyCycleConfig::create(1U, 3U)};
    const Eigen::Vector3f inputVector = makeInputVector(kNominalComponents);

    // Pass through, then advance two updates into the off window.
    EXPECT_NE(alg.update(inputVector)(0), 0.0F);
    EXPECT_EQ(alg.update(inputVector)(0), 0.0F);
    EXPECT_EQ(alg.update(inputVector)(0), 0.0F);

    // Widening the on window to cover the whole cycle opens the gate from the next update onwards; the
    // counter keeps its phase, it is only reinterpreted against the new window.
    alg.setConfig(VectorDutyCycleConfig::create(4U, 0U));
    for (uint32_t update = 0U; update < kManyUpdates; ++update) {
        EXPECT_NE(alg.update(inputVector)(0), 0.0F) << "update " << update;
    }
}

// A cadence shortened under a counter that has already run past the new cycle length must still produce a
// valid phase rather than reading out of range.
TEST(VectorDutyCycle, HandlesACadenceShortenedBelowTheCurrentPhase) {
    VectorDutyCycleAlgorithm alg{VectorDutyCycleConfig::create(1U, 20U)};
    const Eigen::Vector3f inputVector = makeInputVector(kNominalComponents);

    // Advance well past the cycle length the gate is about to be given.
    for (uint32_t update = 0U; update < 15U; ++update) {
        (void)alg.update(inputVector);
    }

    alg.setConfig(VectorDutyCycleConfig::create(1U, 1U));

    // The phase folds back into the new two-period cycle, so the gate must alternate from here on.
    bool previousWasOn = alg.update(inputVector)(0) != 0.0F;
    for (uint32_t update = 0U; update < kManyUpdates; ++update) {
        const bool wasOn = alg.update(inputVector)(0) != 0.0F;
        EXPECT_NE(wasOn, previousWasOn) << "update " << update;
        previousWasOn = wasOn;
    }
}

// A zero input stays zero whether the gate is open or shut, so the module never invents a vector.
TEST(VectorDutyCycle, ZeroInputStaysZero) {
    VectorDutyCycleAlgorithm alg{nominalConfig()};
    const Eigen::Vector3f allZero = Eigen::Vector3f::Zero();

    for (uint32_t update = 0U; update < kManyUpdates; ++update) {
        EXPECT_EQ(alg.update(allZero), allZero) << "update " << update;
    }
}

// ---------------------------------------------------------------------------
// Config tests.
// ---------------------------------------------------------------------------

TEST(VectorDutyCycleConfigTest, AcceptsValidInputs) {
    EXPECT_NO_THROW((void)VectorDutyCycleConfig::create(1U, 0U));  // gate held open, duty cycling disabled
    EXPECT_NO_THROW((void)VectorDutyCycleConfig::create(1U, 4U));
    EXPECT_NO_THROW((void)VectorDutyCycleConfig::create(3U, 2U));
    EXPECT_NO_THROW((void)VectorDutyCycleConfig::create(100U, 10000U));
    // Both extremes of the representable cycle length: a single on period followed by the longest
    // possible hold-off, and an on window that fills the whole range with no hold-off at all.
    EXPECT_NO_THROW((void)VectorDutyCycleConfig::create(1U, UINT32_MAX - 1U));
    EXPECT_NO_THROW((void)VectorDutyCycleConfig::create(UINT32_MAX, 0U));
}

// Whatever cadence is configured must come back unchanged from the getters. The config stores the counts
// verbatim, so these are exact comparisons.
TEST(VectorDutyCycleConfigTest, GettersRoundTrip) {
    expectCadenceRoundTrips(1U, 0U);
    expectCadenceRoundTrips(1U, 4U);
    expectCadenceRoundTrips(3U, 2U);
    expectCadenceRoundTrips(100U, 10000U);
    // Both counts must stay exact at the top of the range, where a wrap would be silent.
    expectCadenceRoundTrips(1U, UINT32_MAX - 1U);
    expectCadenceRoundTrips(UINT32_MAX, 0U);
}

// A cycle that is never on would hold the vector at zero forever, silently disabling the input.
TEST(VectorDutyCycleConfigTest, RejectsZeroOnPeriods) {
    EXPECT_THROW((void)VectorDutyCycleConfig::create(0U, 5U), fsw::invalid_argument);
    EXPECT_THROW((void)VectorDutyCycleConfig::create(0U, 0U), fsw::invalid_argument);
}

// A cycle length that wrapped around would come out shorter than its own on window.
TEST(VectorDutyCycleConfigTest, RejectsCycleLengthOverflow) {
    EXPECT_THROW((void)VectorDutyCycleConfig::create(2U, UINT32_MAX - 1U), fsw::invalid_argument);
    EXPECT_THROW((void)VectorDutyCycleConfig::create(UINT32_MAX, 1U), fsw::invalid_argument);
}

// The public predicates must agree with create() exactly at the boundaries, since callers (the C shim's
// validateConfig, and Ada through it) use them to pre-check a cadence before constructing.
TEST(VectorDutyCycleConfigTest, StaticValidatorsCheckBoundaries) {
    EXPECT_FALSE(VectorDutyCycleConfig::isValidOnPeriods(0U));
    EXPECT_TRUE(VectorDutyCycleConfig::isValidOnPeriods(1U));
    EXPECT_TRUE(VectorDutyCycleConfig::isValidOnPeriods(UINT32_MAX));

    EXPECT_TRUE(VectorDutyCycleConfig::isValidOffPeriods(0U, 1U));
    EXPECT_TRUE(VectorDutyCycleConfig::isValidOffPeriods(UINT32_MAX - 1U, 1U));
    EXPECT_FALSE(VectorDutyCycleConfig::isValidOffPeriods(UINT32_MAX, 1U));
    EXPECT_TRUE(VectorDutyCycleConfig::isValidOffPeriods(0U, UINT32_MAX));
    EXPECT_FALSE(VectorDutyCycleConfig::isValidOffPeriods(1U, UINT32_MAX));
}
