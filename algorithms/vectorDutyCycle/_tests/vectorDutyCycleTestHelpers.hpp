#ifndef TEST_VECTOR_DUTY_CYCLE_H
#define TEST_VECTOR_DUTY_CYCLE_H

#include "vectorDutyCycleAlgorithm.h"
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cstddef>
#include <vector>

inline constexpr Eigen::Index kNumAxes = 3;

// Pack components into the input vector; missing components stay zero.
inline Eigen::Vector3f makeInputVector(const std::vector<float>& components) {
    Eigen::Vector3f inputVector = Eigen::Vector3f::Zero();
    for (Eigen::Index i = 0; i < kNumAxes && static_cast<std::size_t>(i) < components.size(); ++i) {
        inputVector(i) = components[static_cast<std::size_t>(i)];
    }
    return inputVector;
}

// Independent reference for the cadence, written from the module description rather than from the algorithm:
// a cycle is firingPeriods + settlingPeriods control periods long and fires during its leading slots, so the
// nth update since the last restart fires exactly when n modulo the cycle length is inside the firing window.
inline uint32_t referenceCycleLength(const VectorDutyCycleConfig& cfg) {
    return cfg.getFiringPeriods() + cfg.getSettlingPeriods();
}

inline bool referenceIsFiring(uint32_t updateIndex, const VectorDutyCycleConfig& cfg) {
    return (updateIndex % referenceCycleLength(cfg)) < cfg.getFiringPeriods();
}

// Index of the first non-zero component of a vector, or kNumAxes when the vector is all zero. A gated output
// is only distinguishable from a passed-through one where the input is non-zero, so the cadence-observing
// properties below watch this component.
inline Eigen::Index firstNonZeroAxis(const Eigen::Vector3f& inputVector) {
    for (Eigen::Index i = 0; i < kNumAxes; ++i) {
        if (inputVector(i) != 0.0F) {
            return i;
        }
    }
    return kNumAxes;
}

// ---------------------------------------------------------------------------
// Properties taking any admissible configuration: the unit tests drive these with fixed cadences, the
// property*/regressionFuzz* adapters below with generated ones.
// ---------------------------------------------------------------------------

// The gate only ever passes the input through or replaces it with zero -- it performs no arithmetic on the
// vector, so a passed-through component must be identical to the input, not merely close to it.
inline void testOutputIsInputOrZero(const Eigen::Vector3f& inputVector,
                                    const VectorDutyCycleConfig& cfg,
                                    uint32_t numUpdates) {
    VectorDutyCycleAlgorithm alg{cfg};

    for (uint32_t update = 0U; update < numUpdates; ++update) {
        const Eigen::Vector3f gated = alg.update(inputVector);

        for (Eigen::Index i = 0; i < kNumAxes; ++i) {
            const bool passedThrough = gated(i) == inputVector(i);
            const bool heldOff = gated(i) == 0.0F;
            EXPECT_TRUE(passedThrough || heldOff) << "update " << update << " axis " << i;
        }
    }
}

// The gate is all-or-nothing across the axes: within one update every component either passes through or is
// held off, so the output vector never points in a direction that the input did not.
inline void testGateActsOnTheWholeVector(const Eigen::Vector3f& inputVector,
                                         const VectorDutyCycleConfig& cfg,
                                         uint32_t numUpdates) {
    VectorDutyCycleAlgorithm alg{cfg};

    for (uint32_t update = 0U; update < numUpdates; ++update) {
        const Eigen::Vector3f gated = alg.update(inputVector);
        const bool firing = referenceIsFiring(update, cfg);

        for (Eigen::Index i = 0; i < kNumAxes; ++i) {
            EXPECT_EQ(gated(i), firing ? inputVector(i) : 0.0F) << "update " << update << " axis " << i;
        }
    }
}

// Over a whole number of cycles the gate fires on exactly firingPeriods updates per cycle, so the delivered
// duty ratio is exactly firingPeriods / (firingPeriods + settlingPeriods) with no drift or rounding.
inline void testFiringCountMatchesDutyRatio(const Eigen::Vector3f& inputVector,
                                            const VectorDutyCycleConfig& cfg,
                                            uint32_t numCycles) {
    const Eigen::Index watched = firstNonZeroAxis(inputVector);
    ASSERT_LT(watched, kNumAxes) << "an all-zero input cannot reveal the cadence";

    VectorDutyCycleAlgorithm alg{cfg};

    uint32_t firingUpdates = 0U;
    for (uint32_t update = 0U; update < numCycles * referenceCycleLength(cfg); ++update) {
        if (alg.update(inputVector)(watched) != 0.0F) {
            ++firingUpdates;
        }
    }

    EXPECT_EQ(firingUpdates, numCycles * cfg.getFiringPeriods());
}

// The cadence is free-running: it depends only on how many updates have run, never on the input. Two
// gates fed different input vectors must therefore fire on exactly the same updates.
inline void testCadenceIsIndependentOfInput(const Eigen::Vector3f& inputVector,
                                            const VectorDutyCycleConfig& cfg,
                                            uint32_t numUpdates) {
    const Eigen::Index watched = firstNonZeroAxis(inputVector);
    ASSERT_LT(watched, kNumAxes) << "an all-zero input cannot reveal the cadence";

    // A second input that is non-zero on the same component but differs everywhere it can.
    Eigen::Vector3f otherInputVector = Eigen::Vector3f::Zero();
    otherInputVector(watched) = -3.0F * inputVector(watched);

    VectorDutyCycleAlgorithm alg{cfg};
    VectorDutyCycleAlgorithm otherAlg{cfg};

    for (uint32_t update = 0U; update < numUpdates; ++update) {
        const bool fired = alg.update(inputVector)(watched) != 0.0F;
        const bool otherFired = otherAlg.update(otherInputVector)(watched) != 0.0F;
        EXPECT_EQ(fired, otherFired) << "update " << update;
    }
}

// reInitialize() restarts the cycle, so the updates following it repeat the pattern seen from construction.
inline void testReInitializeRestartsCadence(const Eigen::Vector3f& inputVector,
                                            const VectorDutyCycleConfig& cfg,
                                            uint32_t numUpdates,
                                            uint32_t updatesBeforeRestart) {
    const Eigen::Index watched = firstNonZeroAxis(inputVector);
    ASSERT_LT(watched, kNumAxes) << "an all-zero input cannot reveal the cadence";

    VectorDutyCycleAlgorithm alg{cfg};

    std::vector<bool> fromConstruction;
    for (uint32_t update = 0U; update < numUpdates; ++update) {
        fromConstruction.push_back(alg.update(inputVector)(watched) != 0.0F);
    }

    // Run an arbitrary number of further updates to move the counter off its starting phase, then restart.
    for (uint32_t update = 0U; update < updatesBeforeRestart; ++update) {
        (void)alg.update(inputVector);
    }
    alg.reInitialize();

    for (uint32_t update = 0U; update < numUpdates; ++update) {
        const bool fired = alg.update(inputVector)(watched) != 0.0F;
        EXPECT_EQ(fired, fromConstruction[update]) << "update " << update << " after reInitialize";
    }
}

// The algorithm's output over numUpdates updates must match the independent reference cadence exactly.
inline void regressionTestVectorDutyCycle(const Eigen::Vector3f& inputVector,
                                          const VectorDutyCycleConfig& cfg,
                                          uint32_t numUpdates) {
    VectorDutyCycleAlgorithm alg{cfg};

    for (uint32_t update = 0U; update < numUpdates; ++update) {
        const Eigen::Vector3f gated = alg.update(inputVector);
        const bool expectFiring = referenceIsFiring(update, cfg);

        for (Eigen::Index i = 0; i < kNumAxes; ++i) {
            // The gate does no arithmetic, so this is an exact comparison rather than a tolerance check.
            const float expected = expectFiring ? inputVector(i) : 0.0F;
            EXPECT_EQ(gated(i), expected) << "update " << update << " axis " << i;
        }
    }
}

// ---------------------------------------------------------------------------
// Fuzz adapters: build an input vector and a cadence from generated values, then delegate to a core property above.
// ---------------------------------------------------------------------------

namespace detail {

// Returns false when the generated values describe a cadence the config validation rejects, so the caller skips
// the case. The domains below stay inside the valid region, so this is a guard against the domains and the
// validators drifting apart rather than an expected outcome.
inline bool makeFuzzCase(const std::vector<float>& components,
                         float watchedComponent,
                         uint32_t firingPeriods,
                         uint32_t settlingPeriods,
                         Eigen::Vector3f& inputVector) {
    if (!VectorDutyCycleConfig::isValidFiringPeriods(firingPeriods) ||
        !VectorDutyCycleConfig::isValidSettlingPeriods(settlingPeriods, firingPeriods)) {
        return false;
    }

    inputVector = makeInputVector(components);
    // Component 0 is held non-zero so the gate's state is observable in every case: wherever the input is zero a
    // held-off output is indistinguishable from a passed-through one.
    inputVector(0) = watchedComponent;
    return true;
}

}  // namespace detail

inline void propertyOutputIsInputOrZero(const std::vector<float>& components,
                                        float watchedComponent,
                                        uint32_t firingPeriods,
                                        uint32_t settlingPeriods,
                                        uint32_t numUpdates) {
    Eigen::Vector3f inputVector = Eigen::Vector3f::Zero();
    if (!detail::makeFuzzCase(components, watchedComponent, firingPeriods, settlingPeriods, inputVector)) {
        return;
    }
    testOutputIsInputOrZero(inputVector, VectorDutyCycleConfig::create(firingPeriods, settlingPeriods), numUpdates);
}

inline void propertyGateActsOnTheWholeVector(const std::vector<float>& components,
                                             float watchedComponent,
                                             uint32_t firingPeriods,
                                             uint32_t settlingPeriods,
                                             uint32_t numUpdates) {
    Eigen::Vector3f inputVector = Eigen::Vector3f::Zero();
    if (!detail::makeFuzzCase(components, watchedComponent, firingPeriods, settlingPeriods, inputVector)) {
        return;
    }
    testGateActsOnTheWholeVector(
        inputVector, VectorDutyCycleConfig::create(firingPeriods, settlingPeriods), numUpdates);
}

inline void propertyFiringCountMatchesDutyRatio(const std::vector<float>& components,
                                                float watchedComponent,
                                                uint32_t firingPeriods,
                                                uint32_t settlingPeriods,
                                                uint32_t numCycles) {
    Eigen::Vector3f inputVector = Eigen::Vector3f::Zero();
    if (!detail::makeFuzzCase(components, watchedComponent, firingPeriods, settlingPeriods, inputVector)) {
        return;
    }
    testFiringCountMatchesDutyRatio(
        inputVector, VectorDutyCycleConfig::create(firingPeriods, settlingPeriods), numCycles);
}

inline void propertyCadenceIsIndependentOfInput(const std::vector<float>& components,
                                                float watchedComponent,
                                                uint32_t firingPeriods,
                                                uint32_t settlingPeriods,
                                                uint32_t numUpdates) {
    Eigen::Vector3f inputVector = Eigen::Vector3f::Zero();
    if (!detail::makeFuzzCase(components, watchedComponent, firingPeriods, settlingPeriods, inputVector)) {
        return;
    }
    testCadenceIsIndependentOfInput(
        inputVector, VectorDutyCycleConfig::create(firingPeriods, settlingPeriods), numUpdates);
}

inline void propertyReInitializeRestartsCadence(const std::vector<float>& components,
                                                float watchedComponent,
                                                uint32_t firingPeriods,
                                                uint32_t settlingPeriods,
                                                uint32_t numUpdates,
                                                uint32_t updatesBeforeRestart) {
    Eigen::Vector3f inputVector = Eigen::Vector3f::Zero();
    if (!detail::makeFuzzCase(components, watchedComponent, firingPeriods, settlingPeriods, inputVector)) {
        return;
    }
    testReInitializeRestartsCadence(
        inputVector, VectorDutyCycleConfig::create(firingPeriods, settlingPeriods), numUpdates, updatesBeforeRestart);
}

inline void regressionFuzzVectorDutyCycle(const std::vector<float>& components,
                                          float watchedComponent,
                                          uint32_t firingPeriods,
                                          uint32_t settlingPeriods,
                                          uint32_t numUpdates) {
    Eigen::Vector3f inputVector = Eigen::Vector3f::Zero();
    if (!detail::makeFuzzCase(components, watchedComponent, firingPeriods, settlingPeriods, inputVector)) {
        return;
    }
    // The gate performs no arithmetic, so the reference match is exact regardless of cadence or magnitude
    // and needs no error budget.
    regressionTestVectorDutyCycle(
        inputVector, VectorDutyCycleConfig::create(firingPeriods, settlingPeriods), numUpdates);
}

#endif
