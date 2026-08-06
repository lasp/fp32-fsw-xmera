#include "thrMomentumManagementTestHelpers.hpp"

#include <Eigen/Core>

namespace {

constexpr float kAccuracy = 1e-5F;

// Nominal dumping threshold for the standard four-wheel pyramid with Js = 0.1 and speeds
// (10, -25, 50, 100) r/s, which puts the cluster momentum well above the threshold.
constexpr float kNominalHsMin = 100.0F / 6000.0F * 100.0F;

}  // namespace

// A threshold above the cluster momentum yields a zero request.
TEST(ThrMomentumManagement, NoDumpWhenBelowThreshold) {
    ThrMomentumManagementAlgorithm alg{
        ThrMomentumManagementConfig::create(1000.0F / 6000.0F * 100.0F, makeStandardRwArrayConfig())};

    const auto deltaH_B = alg.update(makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F}));

    ASSERT_TRUE(deltaH_B.has_value());
    EXPECT_TRUE(deltaH_B->isZero(kAccuracy));
}

TEST(ThrMomentumManagement, MatchesReferenceAcrossCases) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();

    regressionTestThrMomentumManagement(
        rwArrayConfig, makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F}), kNominalHsMin, kAccuracy);
    regressionTestThrMomentumManagement(
        rwArrayConfig, makeWheelSpeeds({0.0F, 0.0F, 0.0F, 0.0F}), kNominalHsMin, kAccuracy);
    regressionTestThrMomentumManagement(
        rwArrayConfig, makeWheelSpeeds({-500.0F, 400.0F, -300.0F, 200.0F}), 1.0F, kAccuracy);
    regressionTestThrMomentumManagement(rwArrayConfig, makeWheelSpeeds({1.0F, 1.0F, 1.0F, 1.0F}), 0.0F, kAccuracy);
}

// A single wheel spinning about a body axis dumps straight back along that axis.
TEST(ThrMomentumManagement, SingleWheelDumpsAlongItsSpinAxis) {
    const auto rwArrayConfig = makeRwArrayConfig({{0.0F, 0.0F, 1.0F}}, 0.2F);
    ThrMomentumManagementAlgorithm alg{ThrMomentumManagementConfig::create(1.0F, rwArrayConfig)};

    // hs = 0.2 * 50 = 10 Nms about +z; dumping to hsMin = 1 removes 9 Nms.
    const auto deltaH_B = alg.update(makeWheelSpeeds({50.0F}));

    ASSERT_TRUE(deltaH_B.has_value());
    EXPECT_NEAR((*deltaH_B)[0], 0.0F, kAccuracy);
    EXPECT_NEAR((*deltaH_B)[1], 0.0F, kAccuracy);
    EXPECT_NEAR((*deltaH_B)[2], -9.0F, kAccuracy);
}

// The requested change is anti-parallel to the momentum and leaves exactly hsMin behind.
TEST(ThrMomentumManagement, DumpLeavesExactlyHsMin) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();
    const auto wheelSpeeds = makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F});

    ThrMomentumManagementAlgorithm alg{ThrMomentumManagementConfig::create(kNominalHsMin, rwArrayConfig)};
    const auto deltaH_B = alg.update(wheelSpeeds);
    ASSERT_TRUE(deltaH_B.has_value());

    Eigen::Vector3f hs_B = Eigen::Vector3f::Zero();
    for (uint32_t i = 0U; i < rwArrayConfig.numRW; ++i) {
        hs_B += rwArrayConfig.JsList[i] * wheelSpeeds[i] * rwArrayConfig.GsMatrix_B.col(i);
    }

    // Post-dump momentum magnitude must equal the threshold, and the change must oppose the momentum.
    EXPECT_NEAR((hs_B + *deltaH_B).norm(), kNominalHsMin, kAccuracy);
    EXPECT_LT(deltaH_B->normalized().dot(hs_B.normalized()), -1.0F + kAccuracy);
}

// The one-shot latch: update() reports a request once, then disengages until reInitialize().
TEST(ThrMomentumManagement, LatchIsOneShotUntilReInitialize) {
    ThrMomentumManagementAlgorithm alg{ThrMomentumManagementConfig::create(kNominalHsMin, makeStandardRwArrayConfig())};
    const auto wheelSpeeds = makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F});

    EXPECT_TRUE(alg.update(wheelSpeeds).has_value());
    EXPECT_FALSE(alg.update(wheelSpeeds).has_value());
    EXPECT_FALSE(alg.update(wheelSpeeds).has_value());

    alg.reInitialize();
    EXPECT_TRUE(alg.update(wheelSpeeds).has_value());
    EXPECT_FALSE(alg.update(wheelSpeeds).has_value());
}

// setConfig replaces the configuration without re-arming the spent latch.
TEST(ThrMomentumManagement, SetConfigDoesNotReArmTheLatch) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();
    ThrMomentumManagementAlgorithm alg{ThrMomentumManagementConfig::create(kNominalHsMin, rwArrayConfig)};
    const auto wheelSpeeds = makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F});

    EXPECT_TRUE(alg.update(wheelSpeeds).has_value());

    alg.setConfig(ThrMomentumManagementConfig::create(1000.0F / 6000.0F * 100.0F, rwArrayConfig));
    EXPECT_FALSE(alg.update(wheelSpeeds).has_value());

    // Once re-armed, the new threshold is the one in force.
    alg.reInitialize();
    const auto deltaH_B = alg.update(wheelSpeeds);
    ASSERT_TRUE(deltaH_B.has_value());
    EXPECT_TRUE(deltaH_B->isZero(kAccuracy));
}

TEST(ThrMomentumManagement, ConfigRoundTrips) {
    testThrMomentumManagementSetup(kNominalHsMin, makeStandardRwArrayConfig(), kAccuracy);
    testThrMomentumManagementSetup(0.0F, makeRwArrayConfig({{0.0F, 0.0F, 1.0F}}, 0.05F), kAccuracy);
    // Non-unit spin axes are normalized on construction rather than rejected outright at this scale.
    testThrMomentumManagementSetup(2.5F, makeRwArrayConfig({{3.0F, 4.0F, 0.0F}}, 0.1F), kAccuracy);
}
