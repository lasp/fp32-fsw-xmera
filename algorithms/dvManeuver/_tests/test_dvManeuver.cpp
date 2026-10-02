#include "dvManeuverTestHelpers.hpp"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

namespace {
const Eigen::Vector3f kCmdForce_B{0.0F, 0.0F, 10.0F};
const Eigen::Vector3f kCmdDv_N{0.0F, 0.0F, 5.0F};
}  // namespace

TEST(DvManeuverTest, Setup) { testDvManeuverSetup(); }

// ---------------------------------------------------------------------------
// Regression tests: drive the burn state machine through scripted scenarios and compare to the
// independent reference implementation at every step.
// ---------------------------------------------------------------------------

TEST(DvManeuverTest, RegressionNominalBurn) {
    // Burn starts at t = 0.5 s; 2 m/s^2 along +z accumulates 5 m/s of delta-V over the run, meeting
    // the 5 m/s command. No minTime gate; maxTime is set well beyond the burn's completion time so it
    // never triggers.
    regressionTestDvManeuver(/* minTime = */ 0.0F,
                             /* maxTime = */ 3.0F,
                             /* controlPeriod = */ 0.5F,
                             /* cmdForce_B = */ kCmdForce_B,
                             /* cmdDv_N = */ Eigen::Vector3f{0.0F, 0.0F, 5.0F},
                             /* acceleration = */ Eigen::Vector3f{0.0F, 0.0F, 2.0F},
                             /* burnStartTime = */ 500000000U,
                             /* numSteps = */ 10);
}

TEST(DvManeuverTest, RegressionMinTimeGate) {
    // The delta-V target is reached quickly, but minTime = 4 s holds the burn open until the burn
    // time exceeds the minimum.
    regressionTestDvManeuver(/* minTime = */ 4.0F,
                             /* maxTime = */ 100.0F,
                             /* controlPeriod = */ 0.5F,
                             /* cmdForce_B = */ kCmdForce_B,
                             /* cmdDv_N = */ Eigen::Vector3f{0.0F, 0.0F, 4.3F},
                             /* acceleration = */ Eigen::Vector3f{0.0F, 0.0F, 2.0F},
                             /* burnStartTime = */ 0U,
                             /* numSteps = */ 12);
}

TEST(DvManeuverTest, RegressionMaxTimeCutoff) {
    // The delta-V target is never reached, so the maxTime = 3 s cutoff forces completion.
    regressionTestDvManeuver(/* minTime = */ 0.0F,
                             /* maxTime = */ 3.0F,
                             /* controlPeriod = */ 0.5F,
                             /* cmdForce_B = */ kCmdForce_B,
                             /* cmdDv_N = */ Eigen::Vector3f{0.0F, 0.0F, 100.0F},
                             /* acceleration = */ Eigen::Vector3f{0.0F, 0.0F, 2.0F},
                             /* burnStartTime = */ 0U,
                             /* numSteps = */ 10);
}

TEST(DvManeuverTest, RegressionDelayedStart) {
    // Burn commanded to start at t = 1 s; the force command stays zero until then.
    regressionTestDvManeuver(/* minTime = */ 0.0F,
                             /* maxTime = */ 100.0F,
                             /* controlPeriod = */ 0.5F,
                             /* cmdForce_B = */ kCmdForce_B,
                             /* cmdDv_N = */ Eigen::Vector3f{0.0F, 0.0F, 10.0F},
                             /* acceleration = */ Eigen::Vector3f{0.0F, 0.0F, 2.0F},
                             /* burnStartTime = */ 1000000000U,
                             /* numSteps = */ 10);
}

TEST(DvManeuverTest, ReconfigureCannotReExecuteCompletedBurn) {
    const Eigen::Vector3f dvCmd{0.0F, 0.0F, 1.0F};
    DvManeuverAlgorithm alg{DvManeuverConfig::create(0.0F, 10.0F, 0.5F, kCmdForce_B, dvCmd, /* burnStartTime = */ 0U)};

    // Start the burn.
    DvManeuverOutput out = alg.update(/* callTime = */ 0U, Eigen::Vector3f::Zero());

    EXPECT_EQ(out.state, DvManeuverBurnState::Executing);
    EXPECT_EQ(out.cmdForce_B, kCmdForce_B);

    // Reach the commanded delta-V and complete the burn.
    out = alg.update(/* callTime = */ 500000000U, Eigen::Vector3f{0.0F, 0.0F, 1.0F});

    EXPECT_EQ(out.state, DvManeuverBurnState::Complete);
    EXPECT_EQ(out.cmdForce_B, Eigen::Vector3f::Zero());

    // Change the configuration after completion.
    alg.setConfig(DvManeuverConfig::create(2.0F, 10.0F, 0.5F, kCmdForce_B, dvCmd, /* burnStartTime = */ 0U));

    // Completion must remain latched.
    out = alg.update(/* callTime = */ 1000000000U, Eigen::Vector3f{0.0F, 0.0F, 1.0F});

    EXPECT_EQ(out.state, DvManeuverBurnState::Complete);
    EXPECT_EQ(out.cmdForce_B, Eigen::Vector3f::Zero());

    // A second update verifies the old burn cannot restart.
    out = alg.update(/* callTime = */ 1500000000U, Eigen::Vector3f{0.0F, 0.0F, 1.0F});

    EXPECT_EQ(out.state, DvManeuverBurnState::Complete);
    EXPECT_EQ(out.cmdForce_B, Eigen::Vector3f::Zero());
}

// ---------------------------------------------------------------------------
// Property tests.
// ---------------------------------------------------------------------------

TEST(DvManeuverTest, PropertyOutputWellFormed) {
    propertyOutputWellFormed(kCmdForce_B, {0.0F, 0.0F, 5.0F}, {0.0F, 0.0F, 2.0F});
    propertyOutputWellFormed({1.0F, -2.0F, 3.0F}, {1.0F, -2.0F, 3.0F}, {-0.5F, 1.0F, 0.25F});
    propertyOutputWellFormed({0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 0.0F});
}

// ---------------------------------------------------------------------------
// Edge case tests.
// ---------------------------------------------------------------------------

TEST(DvManeuverTest, EdgeZeroForceBeforeBurnStart) {
    // callTime is well before the commanded burn start: the burn never begins, so the module commands
    // a zero force and stays in the pending state.
    DvManeuverAlgorithm alg{
        DvManeuverConfig::create(0.0F, 1.0F, 0.5F, kCmdForce_B, kCmdDv_N, /* burnStartTime = */ 1000000000U)};
    const DvManeuverOutput out = alg.update(/* callTime = */ 100000000U, Eigen::Vector3f::Zero());
    EXPECT_EQ(out.state, DvManeuverBurnState::Pending);
    EXPECT_EQ(out.cmdForce_B, Eigen::Vector3f::Zero());
}

TEST(DvManeuverTest, EdgeZeroCommandedDvCompletesImmediately) {
    // A zero commanded delta-V is satisfied on the first executing step (0 >= 0), so with no minimum
    // time the burn completes immediately and the force command is zero.
    DvManeuverAlgorithm alg{
        DvManeuverConfig::create(0.0F, 1.0F, 0.5F, kCmdForce_B, Eigen::Vector3f::Zero(), /* burnStartTime = */ 0U)};
    const DvManeuverOutput out = alg.update(/* callTime = */ 0U, Eigen::Vector3f::Zero());
    EXPECT_EQ(out.state, DvManeuverBurnState::Complete);
    EXPECT_EQ(out.cmdForce_B, Eigen::Vector3f::Zero());
}

TEST(DvManeuverTest, EdgeCountsOnlyDeltaVFromThisBurn) {
    // The spacecraft already carries 5 m/s of delta-V accumulated before this burn even starts.
    // dvInitial must latch onto that value at burn start, so only the delta-V accumulated during THIS
    // burn counts toward completion -- not the pre-existing total.
    const Eigen::Vector3f priorAccumDV{0.0F, 0.0F, 5.0F};
    const Eigen::Vector3f dvCmd{0.0F, 0.0F, 1.0F};
    DvManeuverAlgorithm alg{DvManeuverConfig::create(0.0F, 10.0F, 0.5F, kCmdForce_B, dvCmd, /* burnStartTime = */ 0U)};

    // Burn starts immediately; dvAccumulated already carries the prior 5 m/s offset.
    DvManeuverOutput out = alg.update(/* callTime = */ 0U, priorAccumDV);

    EXPECT_EQ(out.state, DvManeuverBurnState::Executing);

    // Halfway through this burn's own delta-V: 5.5 - 5.0 = 0.5 m/s.
    out = alg.update(/* callTime = */ 500000000U, priorAccumDV + Eigen::Vector3f{0.0F, 0.0F, 0.5F});

    EXPECT_EQ(out.state, DvManeuverBurnState::Executing);

    // This burn's own delta-V reaches the commanded 1.0 m/s: 6.0 - 5.0 = 1.0 m/s.
    out = alg.update(/* callTime = */ 1000000000U, priorAccumDV + Eigen::Vector3f{0.0F, 0.0F, 1.0F});

    EXPECT_EQ(out.state, DvManeuverBurnState::Complete);
}

TEST(DvManeuverTest, EdgeMinTimeBoundaryRequiresStrictlyGreater) {
    // Since the commanded delta-V is zero, the target is already met and only minTime can delay completion.
    DvManeuverAlgorithm alg{DvManeuverConfig::create(/* minTime = */ 1.0F,
                                                     /* maxTime = */ 100.0F,
                                                     /* controlPeriod = */ 0.5F,
                                                     /* cmdForce_B = */ kCmdForce_B,
                                                     /* cmdDv_N = */ Eigen::Vector3f::Zero(),
                                                     /* burnStartTime = */ 0U)};
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();

    // burnTime = 0.5 s, below minTime: not yet complete.
    DvManeuverOutput out = alg.update(/* callTime = */ 0U, zero);

    EXPECT_EQ(out.state, DvManeuverBurnState::Executing);

    // burnTime = 1.0 s, exactly equal to minTime: still not complete.
    // The gate requires burnTime > minTime, not >=.
    out = alg.update(/* callTime = */ 500000000U, zero);

    EXPECT_EQ(out.state, DvManeuverBurnState::Executing);

    // burnTime = 1.5 s, now strictly greater than minTime: complete.
    out = alg.update(/* callTime = */ 1000000000U, zero);

    EXPECT_EQ(out.state, DvManeuverBurnState::Complete);
}

TEST(DvManeuverTest, EdgeMaxTimeBoundaryRequiresStrictlyGreater) {
    // The delta-V target is intentionally set out of reach, so this test only checks the maxTime cutoff.
    DvManeuverAlgorithm alg{DvManeuverConfig::create(/* minTime = */ 0.0F,
                                                     /* maxTime = */ 1.0F,
                                                     /* controlPeriod = */ 0.5F,
                                                     /* cmdForce_B = */ kCmdForce_B,
                                                     /* cmdDv_N = */ Eigen::Vector3f{0.0F, 0.0F, 100.0F},
                                                     /* burnStartTime = */ 0U)};
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();

    // burnTime = 0.5 s, below maxTime: not yet complete.
    DvManeuverOutput out = alg.update(/* callTime = */ 0U, zero);

    EXPECT_EQ(out.state, DvManeuverBurnState::Executing);

    // burnTime = 1.0 s, exactly equal to maxTime: still not complete.
    // The cutoff requires burnTime > maxTime, not >=.
    out = alg.update(/* callTime = */ 500000000U, zero);

    EXPECT_EQ(out.state, DvManeuverBurnState::Executing);

    // burnTime = 1.5 s, now strictly greater than maxTime: complete.
    out = alg.update(/* callTime = */ 1000000000U, zero);

    EXPECT_EQ(out.state, DvManeuverBurnState::Complete);
}

// ---------------------------------------------------------------------------
// Config validation tests.
// ---------------------------------------------------------------------------

TEST(DvManeuverConfigTest, AcceptsValidConfigurations) {
    EXPECT_NO_THROW(DvManeuverConfig::create(0.0F, 10.0F, 0.5F, kCmdForce_B, kCmdDv_N, 0U));
    EXPECT_NO_THROW(DvManeuverConfig::create(2.0F, 10.0F, 0.1F, kCmdForce_B, kCmdDv_N, 0U));
    EXPECT_NO_THROW(DvManeuverConfig::create(0.0F, 10.0F, 0.5F, Eigen::Vector3f::Zero(), kCmdDv_N, 0U));
    EXPECT_NO_THROW(DvManeuverConfig::create(0.0F, 10.0F, 0.5F, kCmdForce_B, Eigen::Vector3f::Zero(), 0U));
}

TEST(DvManeuverConfigTest, RejectsNegativeMinTime) {
    EXPECT_THROW(DvManeuverConfig::create(-1.0F, 1.0F, 0.5F, kCmdForce_B, kCmdDv_N, 0U), fsw::invalid_argument);
}

TEST(DvManeuverConfigTest, RejectsNegativeMaxTime) {
    EXPECT_THROW(DvManeuverConfig::create(0.0F, -1.0F, 0.5F, kCmdForce_B, kCmdDv_N, 0U), fsw::invalid_argument);
}

TEST(DvManeuverConfigTest, RejectsMaxTimeZero) {
    EXPECT_THROW(DvManeuverConfig::create(0.0F, 0.0F, 0.5F, kCmdForce_B, kCmdDv_N, 0U), fsw::invalid_argument);
}

TEST(DvManeuverConfigTest, RejectsMaxTimeEqualToMinTime) {
    EXPECT_THROW(DvManeuverConfig::create(1.0F, 1.0F, 0.5F, kCmdForce_B, kCmdDv_N, 0U), fsw::invalid_argument);
}

TEST(DvManeuverConfigTest, RejectsNonPositiveControlPeriod) {
    EXPECT_THROW(DvManeuverConfig::create(0.0F, 1.0F, 0.0F, kCmdForce_B, kCmdDv_N, 0U), fsw::invalid_argument);
    EXPECT_THROW(DvManeuverConfig::create(0.0F, 1.0F, -0.1F, kCmdForce_B, kCmdDv_N, 0U), fsw::invalid_argument);
}

TEST(DvManeuverConfigTest, RejectsNonFiniteInputs) {
    const float nan = std::nanf("");
    const float inf = std::numeric_limits<float>::infinity();
    EXPECT_THROW(DvManeuverConfig::create(nan, 0.0F, 0.5F, kCmdForce_B, kCmdDv_N, 0U), fsw::invalid_argument);
    EXPECT_THROW(DvManeuverConfig::create(0.0F, inf, 0.5F, kCmdForce_B, kCmdDv_N, 0U), fsw::invalid_argument);
    EXPECT_THROW(DvManeuverConfig::create(0.0F, 1.0F, nan, kCmdForce_B, kCmdDv_N, 0U), fsw::invalid_argument);
    EXPECT_THROW(DvManeuverConfig::create(0.0F, 1.0F, 0.5F, Eigen::Vector3f{nan, 0.0F, 0.0F}, kCmdDv_N, 0U),
                 fsw::invalid_argument);
    EXPECT_THROW(DvManeuverConfig::create(0.0F, 1.0F, 0.5F, Eigen::Vector3f{0.0F, -inf, 0.0F}, kCmdDv_N, 0U),
                 fsw::invalid_argument);
    EXPECT_THROW(DvManeuverConfig::create(0.0F, 1.0F, 0.5F, kCmdForce_B, Eigen::Vector3f{0.0F, 0.0F, nan}, 0U),
                 fsw::invalid_argument);
    EXPECT_THROW(DvManeuverConfig::create(0.0F, 1.0F, 0.5F, kCmdForce_B, Eigen::Vector3f{inf, 0.0F, 0.0F}, 0U),
                 fsw::invalid_argument);
}

TEST(DvManeuverConfigTest, GettersRoundTrip) {
    const auto cfg = DvManeuverConfig::create(2.0F, 10.0F, 0.25F, kCmdForce_B, kCmdDv_N, 1500000000U);
    EXPECT_FLOAT_EQ(cfg.getMinTime(), 2.0F);
    EXPECT_FLOAT_EQ(cfg.getMaxTime(), 10.0F);
    EXPECT_FLOAT_EQ(cfg.getControlPeriod(), 0.25F);
    EXPECT_EQ(cfg.getCmdForce(), kCmdForce_B);
    EXPECT_EQ(cfg.getCmdDv(), kCmdDv_N);
    EXPECT_EQ(cfg.getBurnStartTime(), 1500000000U);
}
