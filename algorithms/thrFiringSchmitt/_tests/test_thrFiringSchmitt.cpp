#include "thrFiringSchmittTestHelpers.hpp"
#include <gtest/gtest.h>

// ---------------------------------------------------------------------------
// Config validation
// ---------------------------------------------------------------------------

TEST(ThrFiringSchmittConfigTest, RejectsInvalidParameters) {
    ThrFiringSchmittThrusterArray validArray{};
    validArray.numThrusters = 1;
    validArray.maxThrust.at(0) = 1.0F;
    const ThrFiringSchmittControlParameters validParams{
        0.75F, 0.25F, 0.2F, 0.5F, 1.0F, ThrustPulsingRegime::ON_PULSING};

    // levelOn out of bounds
    EXPECT_THROW(
        ThrFiringSchmittConfig::create(validArray, {-0.1F, 0.25F, 0.2F, 0.5F, 1.0F, ThrustPulsingRegime::ON_PULSING}),
        fsw::invalid_argument);
    EXPECT_THROW(
        ThrFiringSchmittConfig::create(validArray, {0.0F, 0.25F, 0.2F, 0.5F, 1.0F, ThrustPulsingRegime::ON_PULSING}),
        fsw::invalid_argument);
    EXPECT_THROW(
        ThrFiringSchmittConfig::create(validArray, {1.1F, 0.25F, 0.2F, 0.5F, 1.0F, ThrustPulsingRegime::ON_PULSING}),
        fsw::invalid_argument);
    // levelOff out of bounds
    EXPECT_THROW(
        ThrFiringSchmittConfig::create(validArray, {0.7F, -0.1F, 0.2F, 0.5F, 1.0F, ThrustPulsingRegime::ON_PULSING}),
        fsw::invalid_argument);
    EXPECT_THROW(
        ThrFiringSchmittConfig::create(validArray, {0.7F, 1.0F, 0.2F, 0.5F, 1.0F, ThrustPulsingRegime::ON_PULSING}),
        fsw::invalid_argument);
    // levelOn less than levelOff
    EXPECT_THROW(
        ThrFiringSchmittConfig::create(validArray, {0.1F, 0.2F, 0.2F, 0.5F, 1.0F, ThrustPulsingRegime::ON_PULSING}),
        fsw::invalid_argument);
    // Negative or zero thrMinFireTime
    EXPECT_THROW(
        ThrFiringSchmittConfig::create(validArray, {0.75F, 0.25F, -0.1F, 0.5F, 1.0F, ThrustPulsingRegime::ON_PULSING}),
        fsw::invalid_argument);
    EXPECT_THROW(
        ThrFiringSchmittConfig::create(validArray, {0.75F, 0.25F, 0.0F, 0.5F, 1.0F, ThrustPulsingRegime::ON_PULSING}),
        fsw::invalid_argument);
    // Negative or zero controlPeriod
    EXPECT_THROW(
        ThrFiringSchmittConfig::create(validArray, {0.75F, 0.25F, 0.2F, -0.1F, 1.0F, ThrustPulsingRegime::ON_PULSING}),
        fsw::invalid_argument);
    EXPECT_THROW(
        ThrFiringSchmittConfig::create(validArray, {0.75F, 0.25F, 0.2F, 0.0F, 1.0F, ThrustPulsingRegime::ON_PULSING}),
        fsw::invalid_argument);
    // onTimeSaturationFactor must be >= 1.0
    EXPECT_THROW(
        ThrFiringSchmittConfig::create(validArray, {0.75F, 0.25F, 0.2F, 0.5F, 0.99F, ThrustPulsingRegime::ON_PULSING}),
        fsw::invalid_argument);
    // numThrusters exceeding the compile-time maximum
    ThrFiringSchmittThrusterArray tooManyArray{};
    tooManyArray.numThrusters = kMaxThrusterCount + 1U;
    EXPECT_THROW(ThrFiringSchmittConfig::create(tooManyArray, validParams), fsw::invalid_argument);
    // Negative maxThrust
    ThrFiringSchmittThrusterArray negThrustArray{};
    negThrustArray.numThrusters = 1;
    negThrustArray.maxThrust.at(0) = -0.1F;
    EXPECT_THROW(ThrFiringSchmittConfig::create(negThrustArray, validParams), fsw::invalid_argument);

    // Valid configuration is accepted
    EXPECT_NO_THROW(ThrFiringSchmittConfig::create(validArray, validParams));
    // onTimeSaturationFactor == 1.0 is allowed
    EXPECT_NO_THROW(
        ThrFiringSchmittConfig::create(validArray, {0.75F, 0.25F, 0.2F, 0.5F, 1.0F, ThrustPulsingRegime::ON_PULSING}));
    // Zero maxThrust is allowed
    ThrFiringSchmittThrusterArray zeroThrustArray{};
    zeroThrustArray.numThrusters = 1;
    zeroThrustArray.maxThrust.at(0) = 0.0F;
    EXPECT_NO_THROW(ThrFiringSchmittConfig::create(zeroThrustArray, validParams));
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
    auto alg = makeSchmittAlgorithm(0.75F, 0.25F, 0.2F, ThrustPulsingRegime::ON_PULSING, 0.5F, 1.0F, 1U, {1.0F});

    // onTime = F/Fmax * dt = F * 0.5; we want level = onTime/thrMinFireTime = 0.75
    // onTime = 0.75 * 0.2 = 0.15, so F = 0.15 / 0.5 = 0.3
    ThrusterForceCmd cmd{};
    cmd.thrForce.at(0) = 0.3F;

    auto out = alg.update(cmd);
    EXPECT_FLOAT_EQ(out.onTimeRequest.at(0), 0.2F);  // fires at thrMinFireTime
}

// Level exactly at levelOff threshold (strict <=) should turn thruster OFF.
TEST(ThrFiringSchmittTest, ExactlyAtLevelOff) {
    auto alg = makeSchmittAlgorithm(0.75F, 0.25F, 0.2F, ThrustPulsingRegime::ON_PULSING, 0.5F, 1.0F, 1U, {1.0F});

    // level = 0.25, onTime = 0.25 * 0.2 = 0.05, F = 0.05 / 0.5 = 0.1
    ThrusterForceCmd cmd{};
    cmd.thrForce.at(0) = 0.1F;

    auto out = alg.update(cmd);
    EXPECT_FLOAT_EQ(out.onTimeRequest.at(0), 0.0F);  // turned off
}

// Hysteresis: once ON via Schmitt, stays ON in the hysteresis band.
TEST(ThrFiringSchmittTest, HysteresisStaysOn) {
    auto alg = makeSchmittAlgorithm(0.75F, 0.25F, 0.2F, ThrustPulsingRegime::ON_PULSING, 0.5F, 1.0F, 1U, {1.0F});

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
    auto alg = makeSchmittAlgorithm(0.75F, 0.25F, 0.2F, ThrustPulsingRegime::ON_PULSING, 0.5F, 1.0F, 1U, {1.0F});

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
    auto alg = makeSchmittAlgorithm(0.75F, 0.25F, 0.02F, ThrustPulsingRegime::ON_PULSING, 0.5F, 1.1F, 1U, {1.0F});

    // F = maxThrust -> onTime = 1.0/1.0 * 0.5 = 0.5 == controlPeriod -> saturates
    ThrusterForceCmd cmd{};
    cmd.thrForce.at(0) = 1.0F;

    auto out = alg.update(cmd);
    EXPECT_FLOAT_EQ(out.onTimeRequest.at(0), 1.1F * 0.5F);
}

// On-time exactly at thrMinFireTime is in normal range (not Schmitt trigger).
TEST(ThrFiringSchmittTest, ExactlyAtMinFireTime) {
    auto alg = makeSchmittAlgorithm(0.75F, 0.25F, 0.2F, ThrustPulsingRegime::ON_PULSING, 0.5F, 1.0F, 1U, {1.0F});

    // onTime = F/Fmax * dt = F * 0.5; want onTime = 0.2, so F = 0.4
    ThrusterForceCmd cmd{};
    cmd.thrForce.at(0) = 0.4F;

    auto out = alg.update(cmd);
    EXPECT_FLOAT_EQ(out.onTimeRequest.at(0), 0.2F);  // normal range, not Schmitt
}

// Zero thrusters produces empty (all-zero) output.
TEST(ThrFiringSchmittTest, ZeroThrusters) {
    auto alg = makeSchmittAlgorithm(0.75F, 0.25F, 0.02F, ThrustPulsingRegime::ON_PULSING, 0.5F, 1.0F, 0U, {});

    ThrusterForceCmd cmd{};
    auto out = alg.update(cmd);

    for (uint32_t i = 0U; i < kMaxThrusterCount; ++i) {
        EXPECT_FLOAT_EQ(out.onTimeRequest.at(i), 0.0F);
    }
}

// Off-pulsing: force of -maxThrust produces zero output (effective force = 0 after clamp).
TEST(ThrFiringSchmittTest, OffPulsingMaxNegativeForce) {
    auto alg = makeSchmittAlgorithm(0.75F, 0.25F, 0.02F, ThrustPulsingRegime::OFF_PULSING, 0.5F, 1.0F, 1U, {0.5F});

    // F = -0.5, after adding maxThrust: 0.0, onTime = 0.0 -> zero
    ThrusterForceCmd cmd{};
    cmd.thrForce.at(0) = -0.5F;

    auto out = alg.update(cmd);
    EXPECT_FLOAT_EQ(out.onTimeRequest.at(0), 0.0F);
}
