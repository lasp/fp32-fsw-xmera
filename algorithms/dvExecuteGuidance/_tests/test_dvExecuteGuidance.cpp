#include "dvExecuteGuidanceTestHelpers.hpp"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

TEST(DvExecuteGuidanceTest, Setup) { testDvExecuteGuidanceSetup(); }

// ---------------------------------------------------------------------------
// Regression tests: drive the burn state machine through scripted scenarios and compare to the
// independent reference implementation at every step.
// ---------------------------------------------------------------------------

TEST(DvExecuteGuidanceTest, RegressionNominalBurn) {
    // Burn starts at t = 0.5 s; 2 m/s^2 along +z accumulates 5 m/s of delta-V over the run, meeting
    // the 5 m/s command. No min/max-time gates.
    regressionTestDvExecuteGuidance(/* minTime = */ 0.0F,
                                    /* maxTime = */ 0.0F,
                                    /* controlPeriod = */ 0.5F,
                                    /* dvInrtlCmd = */ Eigen::Vector3f{0.0F, 0.0F, 5.0F},
                                    /* acceleration = */ Eigen::Vector3f{0.0F, 0.0F, 2.0F},
                                    /* burnStartTime = */ 500000000U,
                                    /* numSteps = */ 10);
}

TEST(DvExecuteGuidanceTest, RegressionMinTimeGate) {
    // The delta-V target is reached quickly, but minTime = 4 s holds the burn open until the burn
    // time exceeds the minimum.
    regressionTestDvExecuteGuidance(/* minTime = */ 4.0F,
                                    /* maxTime = */ 0.0F,
                                    /* controlPeriod = */ 0.5F,
                                    /* dvInrtlCmd = */ Eigen::Vector3f{0.0F, 0.0F, 4.3F},
                                    /* acceleration = */ Eigen::Vector3f{0.0F, 0.0F, 2.0F},
                                    /* burnStartTime = */ 0U,
                                    /* numSteps = */ 12);
}

TEST(DvExecuteGuidanceTest, RegressionMaxTimeCutoff) {
    // The delta-V target is never reached, so the maxTime = 3 s cutoff forces completion.
    regressionTestDvExecuteGuidance(/* minTime = */ 0.0F,
                                    /* maxTime = */ 3.0F,
                                    /* controlPeriod = */ 0.5F,
                                    /* dvInrtlCmd = */ Eigen::Vector3f{0.0F, 0.0F, 100.0F},
                                    /* acceleration = */ Eigen::Vector3f{0.0F, 0.0F, 2.0F},
                                    /* burnStartTime = */ 0U,
                                    /* numSteps = */ 10);
}

TEST(DvExecuteGuidanceTest, RegressionDelayedStart) {
    // Burn commanded to start at t = 1 s; the thrusters stay commanded off until then.
    regressionTestDvExecuteGuidance(/* minTime = */ 0.0F,
                                    /* maxTime = */ 0.0F,
                                    /* controlPeriod = */ 0.5F,
                                    /* dvInrtlCmd = */ Eigen::Vector3f{0.0F, 0.0F, 10.0F},
                                    /* acceleration = */ Eigen::Vector3f{0.0F, 0.0F, 2.0F},
                                    /* burnStartTime = */ 1000000000U,
                                    /* numSteps = */ 10);
}

// ---------------------------------------------------------------------------
// Property tests.
// ---------------------------------------------------------------------------

TEST(DvExecuteGuidanceTest, PropertyFlagsWellFormed) {
    propertyOutputFlagsWellFormed({0.0F, 0.0F, 5.0F}, {0.0F, 0.0F, 2.0F});
    propertyOutputFlagsWellFormed({1.0F, -2.0F, 3.0F}, {-0.5F, 1.0F, 0.25F});
    propertyOutputFlagsWellFormed({0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 0.0F});
}

// ---------------------------------------------------------------------------
// Edge case tests.
// ---------------------------------------------------------------------------

TEST(DvExecuteGuidanceTest, EdgeThrustersHeldOffBeforeBurnStart) {
    // callTime is well before the commanded burn start: the burn never begins, so the module keeps
    // the thrusters commanded off with both flags clear.
    DvExecuteGuidanceAlgorithm alg{DvExecuteGuidanceConfig::create(0.0F, 0.0F, 0.5F)};
    const DvExecuteGuidanceOutput out = alg.update(/* callTime = */ 100000000U,
                                                   Eigen::Vector3f::Zero(),
                                                   Eigen::Vector3f{0.0F, 0.0F, 5.0F},
                                                   /* burnStartTime = */ 1000000000U);
    EXPECT_EQ(out.burnExecuting, 0U);
    EXPECT_EQ(out.burnComplete, 0U);
    EXPECT_TRUE(out.commandThrustersOff);
}

TEST(DvExecuteGuidanceTest, EdgeZeroCommandedDvCompletesImmediately) {
    // A zero commanded delta-V is satisfied on the first executing step (0 >= 0), so with no minimum
    // time the burn completes immediately and the thrusters are commanded off.
    DvExecuteGuidanceAlgorithm alg{DvExecuteGuidanceConfig::create(0.0F, 0.0F, 0.5F)};
    const DvExecuteGuidanceOutput out =
        alg.update(/* callTime = */ 0U, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), /* burnStartTime = */ 0U);
    EXPECT_EQ(out.burnComplete, 1U);
    EXPECT_EQ(out.burnExecuting, 0U);
    EXPECT_TRUE(out.commandThrustersOff);
}

TEST(DvExecuteGuidanceTest, EdgeMaxTimeZeroDisablesMaxCriterion) {
    // With maxTime = 0 the max-time cutoff is disabled: an unreachable delta-V target keeps the burn
    // executing indefinitely.
    DvExecuteGuidanceAlgorithm alg{DvExecuteGuidanceConfig::create(0.0F, 0.0F, 0.5F)};
    const uint64_t stepNs = 500000000U;
    DvExecuteGuidanceOutput out{};
    for (int k = 0; k < 50; ++k) {
        out = alg.update(static_cast<uint64_t>(k) * stepNs,
                         Eigen::Vector3f{0.0F, 0.0F, 0.1F},
                         Eigen::Vector3f{0.0F, 0.0F, 1.0e6F},
                         /* burnStartTime = */ 0U);
    }
    EXPECT_EQ(out.burnComplete, 0U);
    EXPECT_EQ(out.burnExecuting, 1U);
    EXPECT_FALSE(out.commandThrustersOff);
}

// ---------------------------------------------------------------------------
// Config validation tests.
// ---------------------------------------------------------------------------

TEST(DvExecuteGuidanceConfigTest, AcceptsValidConfigurations) {
    EXPECT_NO_THROW(DvExecuteGuidanceConfig::create(0.0F, 0.0F, 0.5F));
    EXPECT_NO_THROW(DvExecuteGuidanceConfig::create(2.0F, 10.0F, 0.1F));
}

TEST(DvExecuteGuidanceConfigTest, RejectsNegativeMinTime) {
    EXPECT_THROW(DvExecuteGuidanceConfig::create(-1.0F, 0.0F, 0.5F), fsw::invalid_argument);
}

TEST(DvExecuteGuidanceConfigTest, RejectsNegativeMaxTime) {
    EXPECT_THROW(DvExecuteGuidanceConfig::create(0.0F, -1.0F, 0.5F), fsw::invalid_argument);
}

TEST(DvExecuteGuidanceConfigTest, RejectsNonPositiveControlPeriod) {
    EXPECT_THROW(DvExecuteGuidanceConfig::create(0.0F, 0.0F, 0.0F), fsw::invalid_argument);
    EXPECT_THROW(DvExecuteGuidanceConfig::create(0.0F, 0.0F, -0.1F), fsw::invalid_argument);
}

TEST(DvExecuteGuidanceConfigTest, RejectsNonFiniteInputs) {
    const float nan = std::nanf("");
    const float inf = std::numeric_limits<float>::infinity();
    EXPECT_THROW(DvExecuteGuidanceConfig::create(nan, 0.0F, 0.5F), fsw::invalid_argument);
    EXPECT_THROW(DvExecuteGuidanceConfig::create(0.0F, inf, 0.5F), fsw::invalid_argument);
    EXPECT_THROW(DvExecuteGuidanceConfig::create(0.0F, 0.0F, nan), fsw::invalid_argument);
}

TEST(DvExecuteGuidanceConfigTest, GettersRoundTrip) {
    const auto cfg = DvExecuteGuidanceConfig::create(2.0F, 10.0F, 0.25F);
    EXPECT_FLOAT_EQ(cfg.getMinTime(), 2.0F);
    EXPECT_FLOAT_EQ(cfg.getMaxTime(), 10.0F);
    EXPECT_FLOAT_EQ(cfg.getControlPeriod(), 0.25F);
}
