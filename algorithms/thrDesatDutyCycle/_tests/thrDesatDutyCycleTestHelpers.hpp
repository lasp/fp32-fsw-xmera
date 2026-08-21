#ifndef TEST_THR_DESAT_DUTY_CYCLE_H
#define TEST_THR_DESAT_DUTY_CYCLE_H

#include "thrDesatDutyCycleAlgorithm.h"
#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <vector>

// Pack per-thruster forces into the algorithm's fixed-size command; unused entries stay zero.
inline std::array<float, kMaxThrusterCount> makeForceCmd(const std::vector<float>& forces) {
    std::array<float, kMaxThrusterCount> thrusterForceCmd{};
    for (std::size_t i = 0; i < forces.size() && i < kMaxThrusterCount; ++i) {
        thrusterForceCmd.at(i) = forces[i];
    }
    return thrusterForceCmd;
}

// Independent reference for the cadence, written from the module description rather than from the algorithm:
// a cycle is firingPeriods + settlingPeriods control periods long and fires during its leading slots, so the
// nth update since the last restart fires exactly when n modulo the cycle length is inside the firing window.
inline bool referenceIsFiring(uint32_t updateIndex, const ThrDesatDutyCycleConfig& cfg) {
    return (updateIndex % (cfg.getFiringPeriods() + cfg.getSettlingPeriods())) < cfg.getFiringPeriods();
}

// Index of the first non-zero entry of a command, or kMaxThrusterCount when the command is all zero. A gated
// output is only distinguishable from a passed-through one where the input is non-zero, so the cadence-observing
// properties below watch this entry.
inline std::size_t firstNonZeroThruster(const std::array<float, kMaxThrusterCount>& thrusterForceCmd) {
    for (std::size_t i = 0; i < kMaxThrusterCount; ++i) {
        if (thrusterForceCmd.at(i) != 0.0F) {
            return i;
        }
    }
    return kMaxThrusterCount;
}

// ---------------------------------------------------------------------------
// Properties taking any admissible configuration: the unit tests drive these with fixed cadences, the
// property*/regressionFuzz* adapters below with generated ones.
// ---------------------------------------------------------------------------

// The gate only ever passes the command through or replaces it with zero -- it performs no arithmetic on the
// force, so a passed-through entry must be bit-identical to the input, not merely close to it.
inline void testOutputIsInputOrZero(const std::array<float, kMaxThrusterCount>& thrusterForceCmd,
                                    const ThrDesatDutyCycleConfig& cfg,
                                    uint32_t numUpdates) {
    ThrDesatDutyCycleAlgorithm alg{cfg};

    for (uint32_t update = 0U; update < numUpdates; ++update) {
        const std::array<float, kMaxThrusterCount> gated = alg.update(thrusterForceCmd);

        for (std::size_t i = 0; i < kMaxThrusterCount; ++i) {
            const bool passedThrough = gated.at(i) == thrusterForceCmd.at(i);
            const bool heldOff = gated.at(i) == 0.0F;
            EXPECT_TRUE(passedThrough || heldOff) << "update " << update << " thruster " << i;
        }
    }
}

// The gate is all-or-nothing across the array: within one update every thruster either passes through or is
// held off, so a desaturation torque is never delivered by a partial subset of the cluster.
inline void testGateActsOnTheWholeArray(const std::array<float, kMaxThrusterCount>& thrusterForceCmd,
                                        const ThrDesatDutyCycleConfig& cfg,
                                        uint32_t numUpdates) {
    ThrDesatDutyCycleAlgorithm alg{cfg};

    for (uint32_t update = 0U; update < numUpdates; ++update) {
        const std::array<float, kMaxThrusterCount> gated = alg.update(thrusterForceCmd);
        const bool firing = referenceIsFiring(update, cfg);

        for (std::size_t i = 0; i < kMaxThrusterCount; ++i) {
            EXPECT_EQ(gated.at(i), firing ? thrusterForceCmd.at(i) : 0.0F) << "update " << update << " thruster " << i;
        }
    }
}

// Over a whole number of cycles the gate fires on exactly firingPeriods updates per cycle, so the delivered
// duty ratio is exactly firingPeriods / (firingPeriods + settlingPeriods) with no drift or rounding.
inline void testFiringCountMatchesDutyRatio(const std::array<float, kMaxThrusterCount>& thrusterForceCmd,
                                            const ThrDesatDutyCycleConfig& cfg,
                                            uint32_t numCycles) {
    const std::size_t watched = firstNonZeroThruster(thrusterForceCmd);
    ASSERT_LT(watched, kMaxThrusterCount) << "an all-zero command cannot reveal the cadence";

    ThrDesatDutyCycleAlgorithm alg{cfg};

    uint32_t firingUpdates = 0U;
    for (uint32_t update = 0U; update < numCycles * cfg.getCycleLength(); ++update) {
        if (alg.update(thrusterForceCmd).at(watched) != 0.0F) {
            ++firingUpdates;
        }
    }

    EXPECT_EQ(firingUpdates, numCycles * cfg.getFiringPeriods());
}

// The cadence is free-running: it depends only on how many updates have run, never on what was commanded. Two
// gates fed different force commands must therefore fire on exactly the same updates.
inline void testCadenceIsIndependentOfCommand(const std::array<float, kMaxThrusterCount>& thrusterForceCmd,
                                              const ThrDesatDutyCycleConfig& cfg,
                                              uint32_t numUpdates) {
    const std::size_t watched = firstNonZeroThruster(thrusterForceCmd);
    ASSERT_LT(watched, kMaxThrusterCount) << "an all-zero command cannot reveal the cadence";

    // A second command that is non-zero on the same entry but differs everywhere it can.
    std::array<float, kMaxThrusterCount> otherForceCmd{};
    otherForceCmd.at(watched) = -3.0F * thrusterForceCmd.at(watched);

    ThrDesatDutyCycleAlgorithm alg{cfg};
    ThrDesatDutyCycleAlgorithm otherAlg{cfg};

    for (uint32_t update = 0U; update < numUpdates; ++update) {
        const bool fired = alg.update(thrusterForceCmd).at(watched) != 0.0F;
        const bool otherFired = otherAlg.update(otherForceCmd).at(watched) != 0.0F;
        EXPECT_EQ(fired, otherFired) << "update " << update;
    }
}

// reInitialize() restarts the cycle, so the updates following it repeat the pattern seen from construction.
inline void testReInitializeRestartsCadence(const std::array<float, kMaxThrusterCount>& thrusterForceCmd,
                                            const ThrDesatDutyCycleConfig& cfg,
                                            uint32_t numUpdates,
                                            uint32_t updatesBeforeRestart) {
    const std::size_t watched = firstNonZeroThruster(thrusterForceCmd);
    ASSERT_LT(watched, kMaxThrusterCount) << "an all-zero command cannot reveal the cadence";

    ThrDesatDutyCycleAlgorithm alg{cfg};

    std::vector<bool> fromConstruction;
    for (uint32_t update = 0U; update < numUpdates; ++update) {
        fromConstruction.push_back(alg.update(thrusterForceCmd).at(watched) != 0.0F);
    }

    // Run an arbitrary number of further updates to move the counter off its starting phase, then restart.
    for (uint32_t update = 0U; update < updatesBeforeRestart; ++update) {
        (void)alg.update(thrusterForceCmd);
    }
    alg.reInitialize();

    for (uint32_t update = 0U; update < numUpdates; ++update) {
        const bool fired = alg.update(thrusterForceCmd).at(watched) != 0.0F;
        EXPECT_EQ(fired, fromConstruction[update]) << "update " << update << " after reInitialize";
    }
}

// The algorithm's output over numUpdates updates must match the independent reference cadence exactly.
inline void regressionTestThrDesatDutyCycle(const std::array<float, kMaxThrusterCount>& thrusterForceCmd,
                                            const ThrDesatDutyCycleConfig& cfg,
                                            uint32_t numUpdates) {
    ThrDesatDutyCycleAlgorithm alg{cfg};

    for (uint32_t update = 0U; update < numUpdates; ++update) {
        const std::array<float, kMaxThrusterCount> gated = alg.update(thrusterForceCmd);
        const bool expectFiring = referenceIsFiring(update, cfg);

        for (std::size_t i = 0; i < kMaxThrusterCount; ++i) {
            // The gate does no arithmetic, so this is a bit-exact comparison rather than a tolerance check.
            const float expected = expectFiring ? thrusterForceCmd.at(i) : 0.0F;
            EXPECT_EQ(gated.at(i), expected) << "update " << update << " thruster " << i;
        }
    }
}

#endif
