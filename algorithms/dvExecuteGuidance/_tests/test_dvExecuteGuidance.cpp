#include "dvExecuteGuidanceTestHelpers.hpp"
#include <gtest/gtest.h>

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
