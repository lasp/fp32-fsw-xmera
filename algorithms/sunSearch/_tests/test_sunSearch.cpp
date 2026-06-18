#include "sunSearchAlgorithm.h"
#include "sunSearchTestHelpers.hpp"
#include "sunSearchTypes.h"
#include "utilities/fsw/timeConstants.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cstdint>

TEST(SunSearchTest, ReferenceTest) {
    testSunSearch(/* rotationTimes */ {5.0F, 10.0F, 7.0F, 3.0F},
                  /* rotationRates */ {0.1F, 0.2F, 0.15F, 0.05F},
                  /* rotationAxes */ {0, 1, 2, 0},
                  /* omega_BN_B  */ Eigen::Vector3f{0.01F, -0.02F, 0.03F},
                  /* dt           */ 0.1F,
                  /* numSteps     */ 300);
}

TEST(SunSearchTest, SetupTest) { testSunSearchSetup(); }

TEST(SunSearchTest, BodyRateErrorMatchesDefinition) {
    // omega_BR_B = omega_BN_B - omega_RN_B at every step, for non-trivial body rate.
    testSunSearch(
        {5.0F, 5.0F, 5.0F, 5.0F}, {0.1F, 0.2F, 0.3F, 0.4F}, {0, 1, 2, 0}, Eigen::Vector3f{0.5F, -0.4F, 0.3F}, 0.5F, 50);
}

TEST(SunSearchTest, HoldsLastOmegaAfterSequence) {
    // After the scripted sequence finishes (sum = 4s), omega_RN_B should hold the last slot's
    // commanded velocity indefinitely.
    const auto rotations = buildRotations({1.0F, 1.0F, 1.0F, 1.0F}, {0.1F, 0.2F, 0.3F, 0.4F}, {0, 1, 2, 0});
    const SunSearchConfig cfg = SunSearchConfig::create(rotations);
    SunSearchAlgorithm alg(cfg);

    const uint64_t startTime = 1000U;
    (void)alg.update(startTime, Eigen::Vector3f::Zero());  // latches sequence start

    const uint64_t lateTime = startTime + static_cast<uint64_t>(100.0F * kSec2NanoF);
    const SunSearchOutput out = alg.update(lateTime, Eigen::Vector3f::Zero());

    // Last slot: rate 0.4 about b1Hat_B (axis 0).
    EXPECT_NEAR(out.omega_RN_B[0], 0.4F, 1e-6);
    EXPECT_NEAR(out.omega_RN_B[1], 0.0F, 1e-6);
    EXPECT_NEAR(out.omega_RN_B[2], 0.0F, 1e-6);
}

TEST(SunSearchTest, SetConfigReArmsStartTime) {
    // After setConfig, the next update() must latch a fresh sequence start time so the new
    // sequence begins from elapsed = 0.
    const auto rotations1 = buildRotations({1.0F, 1.0F, 1.0F, 1.0F}, {0.1F, 0.2F, 0.3F, 0.4F}, {0, 1, 2, 0});
    SunSearchAlgorithm alg(SunSearchConfig::create(rotations1));

    (void)alg.update(1000U, Eigen::Vector3f::Zero());
    (void)alg.update(1000U + static_cast<uint64_t>(0.5F * kSec2NanoF), Eigen::Vector3f::Zero());

    const auto rotations2 = buildRotations({2.0F, 2.0F, 2.0F, 2.0F}, {1.0F, 1.0F, 1.0F, 1.0F}, {1, 1, 1, 1});
    alg.setConfig(SunSearchConfig::create(rotations2));

    // Far-future absolute time; if start was re-armed, elapsed = 0 and we are in slot 0 of cfg2.
    const uint64_t newCall = 1000U + static_cast<uint64_t>(100.0F * kSec2NanoF);
    const SunSearchOutput out = alg.update(newCall, Eigen::Vector3f::Zero());
    EXPECT_NEAR(out.omega_RN_B[1], 1.0F, 1e-6);  // slot 0 of cfg2: rate 1 about b2Hat_B
    EXPECT_NEAR(out.omega_RN_B[0], 0.0F, 1e-6);
    EXPECT_NEAR(out.omega_RN_B[2], 0.0F, 1e-6);
}

TEST(SunSearchTest, NegativeRateProducesSignedOmegaComponent) {
    // Signed rotationRate selects rotation direction along the chosen axis.
    const auto rotations = buildRotations({10.0F, 10.0F, 10.0F, 10.0F}, {-0.5F, 0.1F, 0.1F, 0.1F}, {2, 0, 0, 0});
    SunSearchAlgorithm alg(SunSearchConfig::create(rotations));

    const uint64_t startTime = 1000U;
    (void)alg.update(startTime, Eigen::Vector3f::Zero());

    const uint64_t midSlotZero = startTime + static_cast<uint64_t>(5.0F * kSec2NanoF);
    const SunSearchOutput out = alg.update(midSlotZero, Eigen::Vector3f::Zero());
    EXPECT_NEAR(out.omega_RN_B[2], -0.5F, 1e-6);  // b3Hat_B
    EXPECT_NEAR(out.omega_RN_B[0], 0.0F, 1e-6);
    EXPECT_NEAR(out.omega_RN_B[1], 0.0F, 1e-6);
}

TEST(SunSearchTest, ConfigPreservesAxisAndRateRoundTrip) {
    // The Config getter should return exactly what was supplied.
    const auto rotations = buildRotations({1.0F, 2.0F, 3.0F, 4.0F}, {0.1F, -0.2F, 0.3F, -0.4F}, {0, 1, 2, 0});
    const SunSearchConfig cfg = SunSearchConfig::create(rotations);
    for (uint32_t i = 0U; i < kNumRotations; ++i) {
        EXPECT_FLOAT_EQ(cfg.getRotations().at(i).rotationDuration, rotations[i].rotationDuration);
        EXPECT_FLOAT_EQ(cfg.getRotations().at(i).rotationRate, rotations[i].rotationRate);
        EXPECT_EQ(cfg.getRotations().at(i).rotationAxis, rotations[i].rotationAxis);
    }
}
