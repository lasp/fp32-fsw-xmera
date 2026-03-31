#include "thrFiringSchmittTestHelpers.hpp"
#include <gtest/gtest.h>

// ---------------------------------------------------------------------------
// Setup test
// ---------------------------------------------------------------------------

TEST(ThrFiringSchmittTest, SetupTest) {
    ThrFiringSchmittAlgorithm alg{};

    // --- Test expected exceptions ---

    // levelOn out of bounds
    EXPECT_THROW(alg.setLevelsOnOff(-0.1, 0.3), fsw::invalid_argument);
    EXPECT_THROW(alg.setLevelsOnOff(0.0, 0.3), fsw::invalid_argument);
    EXPECT_THROW(alg.setLevelsOnOff(1.1, 0.3), fsw::invalid_argument);
    // levelOff out of bounds
    EXPECT_THROW(alg.setLevelsOnOff(0.7, -0.1), fsw::invalid_argument);
    EXPECT_THROW(alg.setLevelsOnOff(0.7, 1.0), fsw::invalid_argument);
    EXPECT_THROW(alg.setLevelsOnOff(0.7, 1.1), fsw::invalid_argument);
    // levelOn less than levelOff
    EXPECT_THROW(alg.setLevelsOnOff(0.1, 0.2), fsw::invalid_argument);
    // Negative or zero thrMinFireTime
    EXPECT_THROW(alg.setThrMinFireTime(-0.1), fsw::invalid_argument);
    EXPECT_THROW(alg.setThrMinFireTime(0.0), fsw::invalid_argument);
    // Negative or zero controlPeriod
    EXPECT_THROW(alg.setControlPeriod(-0.1), fsw::invalid_argument);
    EXPECT_THROW(alg.setControlPeriod(0.0), fsw::invalid_argument);
    // onTimeSaturationFactor must be >= 1.0
    EXPECT_THROW(alg.setOnTimeSaturationFactor(0.5), fsw::invalid_argument);
    EXPECT_THROW(alg.setOnTimeSaturationFactor(0.99), fsw::invalid_argument);
    EXPECT_NO_THROW(alg.setOnTimeSaturationFactor(1.0));
    EXPECT_NO_THROW(alg.setOnTimeSaturationFactor(1.1));
    // Negative maxThrust
    ThrusterArrayConfig negThrustConfig{};
    negThrustConfig.numThrusters = 1;
    negThrustConfig.thrusters.at(0).maxThrust = -0.1F;
    EXPECT_THROW(alg.setupThrusters(negThrustConfig), fsw::invalid_argument);
    // Zero maxThrust is allowed
    ThrusterArrayConfig zeroThrustConfig{};
    zeroThrustConfig.numThrusters = 1;
    zeroThrustConfig.thrusters.at(0).maxThrust = 0.0F;
    EXPECT_NO_THROW(alg.setupThrusters(zeroThrustConfig));
}

// ---------------------------------------------------------------------------
// Regression test (reference comparison)
// ---------------------------------------------------------------------------

TEST(ThrFiringSchmittTest, RegressionTest) {
    testThrFiringSchmittRegression(0.7F,
                                   0.3F,
                                   0.01F,
                                   ThrustPulsingRegime::OFF_PULSING,
                                   4U,
                                   std::vector{3.1F, 4.2F, 5.3F, 6.4F},
                                   std::vector{2.1F, 1.2F, 0.3F, 10.4F},
                                   0.1F);
}

// ---------------------------------------------------------------------------
// Property tests
// ---------------------------------------------------------------------------

TEST(ThrFiringSchmittTest, OutputsAreWithinBounds) {
    propertyOutputsAreWithinBounds(0.75F,
                                   0.25F,
                                   0.02F,
                                   ThrustPulsingRegime::ON_PULSING,
                                   4U,
                                   std::vector{0.5F, 0.5F, 0.5F, 0.5F},
                                   std::vector{0.1F, 0.3F, 0.5F, 0.6F},
                                   0.5F);
}

TEST(ThrFiringSchmittTest, NonZeroOutputsExceedMinFireTime) {
    propertyNonZeroOutputsExceedMinFireTime(0.75F,
                                            0.25F,
                                            0.2F,
                                            ThrustPulsingRegime::ON_PULSING,
                                            4U,
                                            std::vector{0.5F, 0.5F, 0.5F, 0.5F},
                                            std::vector{0.1F, 0.3F, 0.5F, 0.05F},
                                            0.5F);
}

TEST(ThrFiringSchmittTest, OutputIsFinite) {
    propertyOutputIsFinite(0.75F,
                           0.25F,
                           0.02F,
                           ThrustPulsingRegime::OFF_PULSING,
                           4U,
                           std::vector{0.5F, 0.5F, 0.5F, 0.5F},
                           std::vector{-0.3F, -0.1F, 0.0F, -0.5F},
                           0.5F);
}

TEST(ThrFiringSchmittTest, ResetClearsState) {
    propertyResetClearsState(0.75F,
                             0.25F,
                             0.02F,
                             ThrustPulsingRegime::ON_PULSING,
                             4U,
                             std::vector{0.5F, 0.5F, 0.5F, 0.5F},
                             std::vector{0.1F, 0.3F, 0.5F, 0.05F},
                             0.5F);
}

TEST(ThrFiringSchmittTest, SaturatedInputProducesOversaturatedOutput) {
    propertySaturatedInputProducesOversaturatedOutput(0.02F, 4U, std::vector{0.5F, 1.0F, 2.0F, 0.3F}, 0.5F);
}

TEST(ThrFiringSchmittTest, ZeroForceProducesZeroOutput) { propertyZeroForceProducesZeroOutput(0.02F, 4U, 0.5F); }

// ---------------------------------------------------------------------------
// Edge-case tests
// ---------------------------------------------------------------------------

// Level exactly at levelOn threshold (strict >=) should turn thruster ON.
TEST(ThrFiringSchmittTest, ExactlyAtLevelOn) {
    ThrFiringSchmittAlgorithm alg{};
    alg.setLevelsOnOff(0.75F, 0.25F);
    alg.setThrMinFireTime(0.2F);
    alg.setControlPeriod(0.5F);
    alg.setThrustPulsingRegime(ThrustPulsingRegime::ON_PULSING);

    ThrusterArrayConfig config{};
    config.numThrusters = 1;
    config.thrusters.at(0).maxThrust = 1.0F;
    alg.setupThrusters(config);
    alg.reset();

    // onTime = F/Fmax * dt = F * 0.5; we want level = onTime/thrMinFireTime = 0.75
    // onTime = 0.75 * 0.2 = 0.15, so F = 0.15 / 0.5 = 0.3
    ThrusterForceCmd cmd{};
    cmd.thrForce.at(0) = 0.3F;

    auto out = alg.update(cmd);
    EXPECT_FLOAT_EQ(out.onTimeRequest.at(0), 0.2F);  // fires at thrMinFireTime
}

// Level exactly at levelOff threshold (strict <=) should turn thruster OFF.
TEST(ThrFiringSchmittTest, ExactlyAtLevelOff) {
    ThrFiringSchmittAlgorithm alg{};
    alg.setLevelsOnOff(0.75F, 0.25F);
    alg.setThrMinFireTime(0.2F);
    alg.setControlPeriod(0.5F);
    alg.setThrustPulsingRegime(ThrustPulsingRegime::ON_PULSING);

    ThrusterArrayConfig config{};
    config.numThrusters = 1;
    config.thrusters.at(0).maxThrust = 1.0F;
    alg.setupThrusters(config);
    alg.reset();

    // level = 0.25, onTime = 0.25 * 0.2 = 0.05, F = 0.05 / 0.5 = 0.1
    ThrusterForceCmd cmd{};
    cmd.thrForce.at(0) = 0.1F;

    auto out = alg.update(cmd);
    EXPECT_FLOAT_EQ(out.onTimeRequest.at(0), 0.0F);  // turned off
}

// Hysteresis: once ON via Schmitt, stays ON in the hysteresis band.
TEST(ThrFiringSchmittTest, HysteresisStaysOn) {
    ThrFiringSchmittAlgorithm alg{};
    alg.setLevelsOnOff(0.75F, 0.25F);
    alg.setThrMinFireTime(0.2F);
    alg.setControlPeriod(0.5F);
    alg.setThrustPulsingRegime(ThrustPulsingRegime::ON_PULSING);

    ThrusterArrayConfig config{};
    config.numThrusters = 1;
    config.thrusters.at(0).maxThrust = 1.0F;
    alg.setupThrusters(config);
    alg.reset();

    // First: level = 0.75 (at levelOn) -> turns ON
    ThrusterForceCmd cmdOn{};
    cmdOn.thrForce.at(0) = 0.3F;  // level = 0.75
    auto out1 = alg.update(cmdOn);
    EXPECT_FLOAT_EQ(out1.onTimeRequest.at(0), 0.2F);

    // Second: level = 0.5 (in hysteresis band) -> stays ON because prevState is ON
    ThrusterForceCmd cmdMid{};
    cmdMid.thrForce.at(0) = 0.2F;  // level = 0.5
    auto out2 = alg.update(cmdMid);
    EXPECT_FLOAT_EQ(out2.onTimeRequest.at(0), 0.2F);
}

// Hysteresis: once OFF via Schmitt, stays OFF in the hysteresis band.
TEST(ThrFiringSchmittTest, HysteresisStaysOff) {
    ThrFiringSchmittAlgorithm alg{};
    alg.setLevelsOnOff(0.75F, 0.25F);
    alg.setThrMinFireTime(0.2F);
    alg.setControlPeriod(0.5F);
    alg.setThrustPulsingRegime(ThrustPulsingRegime::ON_PULSING);

    ThrusterArrayConfig config{};
    config.numThrusters = 1;
    config.thrusters.at(0).maxThrust = 1.0F;
    alg.setupThrusters(config);
    alg.reset();

    // First: level = 0.25 (at levelOff) -> turns OFF
    ThrusterForceCmd cmdOff{};
    cmdOff.thrForce.at(0) = 0.1F;  // level = 0.25
    auto out1 = alg.update(cmdOff);
    EXPECT_FLOAT_EQ(out1.onTimeRequest.at(0), 0.0F);

    // Second: level = 0.5 (in hysteresis band) -> stays OFF because prevState is OFF
    ThrusterForceCmd cmdMid{};
    cmdMid.thrForce.at(0) = 0.2F;  // level = 0.5
    auto out2 = alg.update(cmdMid);
    EXPECT_FLOAT_EQ(out2.onTimeRequest.at(0), 0.0F);
}

// On-time exactly at controlPeriod triggers saturation (strict >=).
TEST(ThrFiringSchmittTest, ExactlyAtControlPeriod) {
    ThrFiringSchmittAlgorithm alg{};
    alg.setLevelsOnOff(0.75F, 0.25F);
    alg.setThrMinFireTime(0.02F);
    alg.setControlPeriod(0.5F);
    alg.setOnTimeSaturationFactor(1.1F);
    alg.setThrustPulsingRegime(ThrustPulsingRegime::ON_PULSING);

    ThrusterArrayConfig config{};
    config.numThrusters = 1;
    config.thrusters.at(0).maxThrust = 1.0F;
    alg.setupThrusters(config);
    alg.reset();

    // F = maxThrust -> onTime = 1.0/1.0 * 0.5 = 0.5 == controlPeriod -> saturates
    ThrusterForceCmd cmd{};
    cmd.thrForce.at(0) = 1.0F;

    auto out = alg.update(cmd);
    EXPECT_FLOAT_EQ(out.onTimeRequest.at(0), 1.1F * 0.5F);
}

// On-time exactly at thrMinFireTime is in normal range (not Schmitt trigger).
TEST(ThrFiringSchmittTest, ExactlyAtMinFireTime) {
    ThrFiringSchmittAlgorithm alg{};
    alg.setLevelsOnOff(0.75F, 0.25F);
    alg.setThrMinFireTime(0.2F);
    alg.setControlPeriod(0.5F);
    alg.setThrustPulsingRegime(ThrustPulsingRegime::ON_PULSING);

    ThrusterArrayConfig config{};
    config.numThrusters = 1;
    config.thrusters.at(0).maxThrust = 1.0F;
    alg.setupThrusters(config);
    alg.reset();

    // onTime = F/Fmax * dt = F * 0.5; want onTime = 0.2, so F = 0.4
    ThrusterForceCmd cmd{};
    cmd.thrForce.at(0) = 0.4F;

    auto out = alg.update(cmd);
    EXPECT_FLOAT_EQ(out.onTimeRequest.at(0), 0.2F);  // normal range, not Schmitt
}

// Zero thrusters produces empty (all-zero) output.
TEST(ThrFiringSchmittTest, ZeroThrusters) {
    ThrFiringSchmittAlgorithm alg{};
    alg.setLevelsOnOff(0.75F, 0.25F);
    alg.setThrMinFireTime(0.02F);
    alg.setControlPeriod(0.5F);
    alg.setThrustPulsingRegime(ThrustPulsingRegime::ON_PULSING);

    ThrusterArrayConfig config{};
    config.numThrusters = 0;
    alg.setupThrusters(config);
    alg.reset();

    ThrusterForceCmd cmd{};
    auto out = alg.update(cmd);

    for (uint32_t i = 0U; i < kMaxThrusterCount; ++i) {
        EXPECT_FLOAT_EQ(out.onTimeRequest.at(i), 0.0F);
    }
}

// Off-pulsing: force of -maxThrust produces zero output (effective force = 0 after clamp).
TEST(ThrFiringSchmittTest, OffPulsingMaxNegativeForce) {
    ThrFiringSchmittAlgorithm alg{};
    alg.setLevelsOnOff(0.75F, 0.25F);
    alg.setThrMinFireTime(0.02F);
    alg.setControlPeriod(0.5F);
    alg.setThrustPulsingRegime(ThrustPulsingRegime::OFF_PULSING);

    ThrusterArrayConfig config{};
    config.numThrusters = 1;
    config.thrusters.at(0).maxThrust = 0.5F;
    alg.setupThrusters(config);
    alg.reset();

    // F = -0.5, after adding maxThrust: 0.0, onTime = 0.0 -> zero
    ThrusterForceCmd cmd{};
    cmd.thrForce.at(0) = -0.5F;

    auto out = alg.update(cmd);
    EXPECT_FLOAT_EQ(out.onTimeRequest.at(0), 0.0F);
}
