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
