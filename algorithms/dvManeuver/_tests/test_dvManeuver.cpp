#include "dvManeuverTestHelpers.hpp"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

namespace {
const Eigen::Vector3f kCmdForce_B{0.0F, 0.0F, 10.0F};
const Eigen::Vector3f kCmdDv_N{0.0F, 0.0F, 5.0F};
constexpr uint64_t kControlPeriod = 500000000U;  // [ns]
}  // namespace

TEST(DvManeuverTest, Setup) { testDvManeuverSetup(); }

// ---------------------------------------------------------------------------
// Regression tests: drive the burn state machine through scripted scenarios and compare to the
// independent reference implementation at every step.
// ---------------------------------------------------------------------------

TEST(DvManeuverTest, RegressionNominalBurn) {
    // The burn starts at the first update; 2 m/s^2 along +z accumulates 5 m/s of delta-V after 2.5 s, meeting
    // the 5 m/s command. No minTime gate; maxTime is set beyond the burn's completion time so it never triggers.
    regressionTestDvManeuver(/* minTime = */ 0U,
                             /* maxTime = */ 3000000000U,
                             /* stepNs = */ 500000000U,
                             kCmdForce_B,
                             /* cmdDv_N = */ Eigen::Vector3f{0.0F, 0.0F, 5.0F},
                             /* acceleration = */ Eigen::Vector3f{0.0F, 0.0F, 2.0F},
                             /* numSteps = */ 10);
}

TEST(DvManeuverTest, RegressionMinTimeGate) {
    // The delta-V target is reached quickly, but minTime = 4 s holds the burn open until the burn
    // time exceeds the minimum.
    regressionTestDvManeuver(/* minTime = */ 4000000000U,
                             /* maxTime = */ 100000000000U,
                             /* stepNs = */ 500000000U,
                             kCmdForce_B,
                             /* cmdDv_N = */ Eigen::Vector3f{0.0F, 0.0F, 4.3F},
                             /* acceleration = */ Eigen::Vector3f{0.0F, 0.0F, 2.0F},
                             /* numSteps = */ 12);
}

TEST(DvManeuverTest, RegressionMaxTimeCutoff) {
    // The delta-V target is never reached, so the maxTime = 3 s cutoff forces completion.
    regressionTestDvManeuver(/* minTime = */ 0U,
                             /* maxTime = */ 3000000000U,
                             /* stepNs = */ 500000000U,
                             kCmdForce_B,
                             /* cmdDv_N = */ Eigen::Vector3f{0.0F, 0.0F, 100.0F},
                             /* acceleration = */ Eigen::Vector3f{0.0F, 0.0F, 2.0F},
                             /* numSteps = */ 10);
}

TEST(DvManeuverTest, ReconfigureCannotReExecuteCompletedBurn) {
    const Eigen::Vector3f dvCmd{0.0F, 0.0F, 1.0F};
    DvManeuverAlgorithm alg{DvManeuverConfig::create(
        /* minTime = */ 0U,
        /* maxTime = */ 10000000000U,
        kControlPeriod,
        kCmdForce_B,
        dvCmd)};

    // Start the burn.
    DvManeuverOutput out = alg.update(/* dvAccumulated = */ Eigen::Vector3f::Zero());

    EXPECT_EQ(out.state, DvManeuverBurnState::Executing);
    EXPECT_EQ(out.cmdForce_B, kCmdForce_B);

    // Reach the commanded delta-V and complete the burn.
    out = alg.update(/* dvAccumulated = */ Eigen::Vector3f{0.0F, 0.0F, 1.0F});

    EXPECT_EQ(out.state, DvManeuverBurnState::Complete);
    EXPECT_EQ(out.cmdForce_B, Eigen::Vector3f::Zero());

    // Change the configuration after completion.
    alg.setConfig(DvManeuverConfig::create(
        /* minTime = */ 2000000000U,
        /* maxTime = */ 10000000000U,
        kControlPeriod,
        kCmdForce_B,
        dvCmd));

    // Completion must remain latched.
    out = alg.update(/* dvAccumulated = */ Eigen::Vector3f{0.0F, 0.0F, 1.0F});

    EXPECT_EQ(out.state, DvManeuverBurnState::Complete);
    EXPECT_EQ(out.cmdForce_B, Eigen::Vector3f::Zero());

    // A second update verifies the old burn cannot restart.
    out = alg.update(/* dvAccumulated = */ Eigen::Vector3f{0.0F, 0.0F, 1.0F});

    EXPECT_EQ(out.state, DvManeuverBurnState::Complete);
    EXPECT_EQ(out.cmdForce_B, Eigen::Vector3f::Zero());
}

TEST(DvManeuverTest, ReInitializeStartsNewBurn) {
    // minTime = 1 s makes the burn time observable: a burn time that did not restart would pass the gate at once.
    const Eigen::Vector3f dvCmd{0.0F, 0.0F, 1.0F};
    DvManeuverAlgorithm alg{DvManeuverConfig::create(
        /* minTime = */ 1000000000U,
        /* maxTime = */ 10000000000U,
        kControlPeriod,
        kCmdForce_B,
        dvCmd)};

    EXPECT_EQ(alg.update(/* dvAccumulated = */ Eigen::Vector3f::Zero()).state, DvManeuverBurnState::Executing);
    EXPECT_EQ(alg.update(dvCmd).state, DvManeuverBurnState::Executing);
    EXPECT_EQ(alg.update(dvCmd).state, DvManeuverBurnState::Complete);

    // Start the next burn. The flight software also resets the accumulated delta-V at the start of the new burn.
    alg.reInitialize();

    DvManeuverOutput out = alg.update(/* dvAccumulated = */ Eigen::Vector3f::Zero());
    EXPECT_EQ(out.state, DvManeuverBurnState::Executing);
    EXPECT_EQ(out.cmdForce_B, kCmdForce_B);

    // 1.5 m/s meets the command, but only 0.5 s of the new burn has elapsed.
    out = alg.update(/* dvAccumulated = */ Eigen::Vector3f{0.0F, 0.0F, 1.5F});
    EXPECT_EQ(out.state, DvManeuverBurnState::Executing);

    out = alg.update(/* dvAccumulated = */ Eigen::Vector3f{0.0F, 0.0F, 2.0F});
    EXPECT_EQ(out.state, DvManeuverBurnState::Complete);
}

TEST(DvManeuverTest, SetConfigDuringBurnAppliesNextUpdate) {
    const Eigen::Vector3f firstForce_B{0.0F, 0.0F, 10.0F};
    const Eigen::Vector3f secondForce_B{1.0F, 2.0F, 3.0F};
    DvManeuverAlgorithm alg{DvManeuverConfig::create(
        /* minTime = */ 0U,
        /* maxTime = */ 10000000000U,
        kControlPeriod,
        firstForce_B,
        /* cmdDv_N = */ Eigen::Vector3f{0.0F, 0.0F, 2.0F})};

    DvManeuverOutput out = alg.update(/* dvAccumulated = */ Eigen::Vector3f::Zero());
    EXPECT_EQ(out.state, DvManeuverBurnState::Executing);
    EXPECT_EQ(out.cmdForce_B, firstForce_B);

    // A new force applies on the next update.
    alg.setConfig(DvManeuverConfig::create(
        /* minTime = */ 0U,
        /* maxTime = */ 10000000000U,
        kControlPeriod,
        secondForce_B,
        /* cmdDv_N = */ Eigen::Vector3f{0.0F, 0.0F, 2.0F}));
    out = alg.update(/* dvAccumulated = */ Eigen::Vector3f{0.0F, 0.0F, 0.5F});
    EXPECT_EQ(out.state, DvManeuverBurnState::Executing);
    EXPECT_EQ(out.cmdForce_B, secondForce_B);

    // A smaller delta-V command applies on the next update.
    alg.setConfig(DvManeuverConfig::create(
        /* minTime = */ 0U,
        /* maxTime = */ 10000000000U,
        kControlPeriod,
        secondForce_B,
        /* cmdDv_N = */ Eigen::Vector3f{0.0F, 0.0F, 1.0F}));
    out = alg.update(/* dvAccumulated = */ Eigen::Vector3f{0.0F, 0.0F, 1.0F});
    EXPECT_EQ(out.state, DvManeuverBurnState::Complete);
    EXPECT_EQ(out.cmdForce_B, Eigen::Vector3f::Zero());
}

// ---------------------------------------------------------------------------
// Property tests.
// ---------------------------------------------------------------------------

TEST(DvManeuverTest, PropertyOutputWellFormed) {
    propertyOutputWellFormed(kCmdForce_B,
                             /* cmdDv_N = */ {0.0F, 0.0F, 5.0F},
                             /* acceleration = */ {0.0F, 0.0F, 2.0F},
                             /* minTime = */ 0U,
                             /* maxTimeAboveMinTime = */ 100000000000U,
                             /* stepNs = */ 500000000U);
    propertyOutputWellFormed(
        /* cmdForce_B = */ {1.0F, -2.0F, 3.0F},
        /* cmdDv_N = */ {1.0F, -2.0F, 3.0F},
        /* acceleration = */ {-0.5F, 1.0F, 0.25F},
        /* minTime = */ 0U,
        /* maxTimeAboveMinTime = */ 100000000000U,
        /* stepNs = */ 500000000U);
    propertyOutputWellFormed(
        /* cmdForce_B = */ {0.0F, 0.0F, 0.0F},
        /* cmdDv_N = */ {0.0F, 0.0F, 0.0F},
        /* acceleration = */ {0.0F, 0.0F, 0.0F},
        /* minTime = */ 0U,
        /* maxTimeAboveMinTime = */ 100000000000U,
        /* stepNs = */ 500000000U);
    // Both time gates active: the delta-V is reached early, so minTime decides; a large command leaves maxTime.
    propertyOutputWellFormed(kCmdForce_B,
                             /* cmdDv_N = */ {0.0F, 0.0F, 1.0F},
                             /* acceleration = */ {0.0F, 0.0F, 2.0F},
                             /* minTime = */ 2000000000U,
                             /* maxTimeAboveMinTime = */ 3000000000U,
                             /* stepNs = */ 100000000U);
    propertyOutputWellFormed(kCmdForce_B,
                             /* cmdDv_N = */ {0.0F, 0.0F, 100.0F},
                             /* acceleration = */ {0.0F, 0.0F, 2.0F},
                             /* minTime = */ 2000000000U,
                             /* maxTimeAboveMinTime = */ 3000000000U,
                             /* stepNs = */ 100000000U);
}

// ---------------------------------------------------------------------------
// Edge case tests.
// ---------------------------------------------------------------------------

TEST(DvManeuverTest, EdgeZeroCommandedDvCompletesImmediately) {
    // A zero commanded delta-V is satisfied on the first executing step (0 >= 0), so with no minimum
    // time the burn completes immediately and the force command is zero.
    DvManeuverAlgorithm alg{DvManeuverConfig::create(/* minTime = */ 0U,
                                                     /* maxTime = */ 1000000000U,
                                                     kControlPeriod,
                                                     kCmdForce_B,
                                                     /* cmdDv_N = */ Eigen::Vector3f::Zero())};
    const DvManeuverOutput out = alg.update(/* dvAccumulated = */ Eigen::Vector3f::Zero());
    EXPECT_EQ(out.state, DvManeuverBurnState::Complete);
    EXPECT_EQ(out.cmdForce_B, Eigen::Vector3f::Zero());
}

TEST(DvManeuverTest, EdgeExecutesFromFirstUpdate) {
    // The burn has no start time: it executes and commands the force from the first update.
    DvManeuverAlgorithm alg{DvManeuverConfig::create(
        /* minTime = */ 0U, /* maxTime = */ 10000000000U, kControlPeriod, kCmdForce_B, kCmdDv_N)};
    const DvManeuverOutput out = alg.update(/* dvAccumulated = */ Eigen::Vector3f::Zero());
    EXPECT_EQ(out.state, DvManeuverBurnState::Executing);
    EXPECT_EQ(out.cmdForce_B, kCmdForce_B);
}

TEST(DvManeuverTest, EdgeCompletionUsesDeltaVMagnitudeOnly) {
    // The command points along x while the delta-V accumulates along z. Only the magnitudes are compared, so the burn
    // completes once 1 m/s has accumulated in any direction.
    DvManeuverAlgorithm alg{DvManeuverConfig::create(
        /* minTime = */ 0U,
        /* maxTime = */ 10000000000U,
        kControlPeriod,
        kCmdForce_B,
        /* cmdDv_N = */ Eigen::Vector3f{1.0F, 0.0F, 0.0F})};

    EXPECT_EQ(alg.update(/* dvAccumulated = */ Eigen::Vector3f::Zero()).state, DvManeuverBurnState::Executing);
    EXPECT_EQ(alg.update(/* dvAccumulated = */ Eigen::Vector3f{0.0F, 0.0F, 0.5F}).state,
              DvManeuverBurnState::Executing);
    EXPECT_EQ(alg.update(/* dvAccumulated = */ Eigen::Vector3f{0.0F, 0.0F, 1.0F}).state, DvManeuverBurnState::Complete);
}

TEST(DvManeuverTest, EdgeAccumulatedDvCountsFromFirstUpdate) {
    // The module compares the accumulated delta-V with the command directly. Delta-V that is already present at
    // the first executing update counts, so a command it already meets completes the burn at once.
    DvManeuverAlgorithm alg{DvManeuverConfig::create(/* minTime = */ 0U,
                                                     /* maxTime = */ 10000000000U,
                                                     kControlPeriod,
                                                     kCmdForce_B,
                                                     /* cmdDv_N = */ Eigen::Vector3f{0.0F, 0.0F, 1.0F})};
    const DvManeuverOutput out = alg.update(/* dvAccumulated = */ Eigen::Vector3f{0.0F, 0.0F, 1.0F});
    EXPECT_EQ(out.state, DvManeuverBurnState::Complete);
    EXPECT_EQ(out.cmdForce_B, Eigen::Vector3f::Zero());
}

TEST(DvManeuverTest, EdgeMinTimeGatePassesAtLimit) {
    // Since the commanded delta-V is zero, the target is already met and only minTime can delay completion.
    // The burn time grows by controlPeriod per update, and the gate passes when it reaches minTime.
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();
    DvManeuverAlgorithm alg{DvManeuverConfig::create(/* minTime = */ 1000000000U,
                                                     /* maxTime = */ 100000000000U,
                                                     kControlPeriod,
                                                     kCmdForce_B,
                                                     /* cmdDv_N = */ Eigen::Vector3f::Zero())};

    // Burn times 0 s and 0.5 s: not yet complete.
    EXPECT_EQ(alg.update(zero).state, DvManeuverBurnState::Executing);
    EXPECT_EQ(alg.update(zero).state, DvManeuverBurnState::Executing);

    // Burn time 1 s, exactly minTime: complete.
    EXPECT_EQ(alg.update(zero).state, DvManeuverBurnState::Complete);

    // With a minTime 1 ns longer, the same burn time of 1 s does not complete the burn yet.
    DvManeuverAlgorithm longerAlg{DvManeuverConfig::create(/* minTime = */ 1000000001U,
                                                           /* maxTime = */ 100000000000U,
                                                           kControlPeriod,
                                                           kCmdForce_B,
                                                           /* cmdDv_N = */ Eigen::Vector3f::Zero())};
    EXPECT_EQ(longerAlg.update(zero).state, DvManeuverBurnState::Executing);
    EXPECT_EQ(longerAlg.update(zero).state, DvManeuverBurnState::Executing);
    EXPECT_EQ(longerAlg.update(zero).state, DvManeuverBurnState::Executing);
    EXPECT_EQ(longerAlg.update(zero).state, DvManeuverBurnState::Complete);
}

TEST(DvManeuverTest, EdgeMaxTimeGatePassesAtLimit) {
    // The delta-V target is intentionally set out of reach, so this test only checks the maxTime cutoff.
    // The burn time grows by controlPeriod per update, and the gate passes when it reaches maxTime.
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();
    DvManeuverAlgorithm alg{DvManeuverConfig::create(/* minTime = */ 0U,
                                                     /* maxTime = */ 1000000000U,
                                                     kControlPeriod,
                                                     kCmdForce_B,
                                                     /* cmdDv_N = */ Eigen::Vector3f{0.0F, 0.0F, 100.0F})};

    // Burn times 0 s and 0.5 s: not yet complete.
    EXPECT_EQ(alg.update(zero).state, DvManeuverBurnState::Executing);
    EXPECT_EQ(alg.update(zero).state, DvManeuverBurnState::Executing);

    // Burn time 1 s, exactly maxTime: complete.
    EXPECT_EQ(alg.update(zero).state, DvManeuverBurnState::Complete);

    // With a maxTime 1 ns longer, the same burn time of 1 s does not complete the burn yet.
    DvManeuverAlgorithm longerAlg{DvManeuverConfig::create(/* minTime = */ 0U,
                                                           /* maxTime = */ 1000000001U,
                                                           kControlPeriod,
                                                           kCmdForce_B,
                                                           /* cmdDv_N = */ Eigen::Vector3f{0.0F, 0.0F, 100.0F})};
    EXPECT_EQ(longerAlg.update(zero).state, DvManeuverBurnState::Executing);
    EXPECT_EQ(longerAlg.update(zero).state, DvManeuverBurnState::Executing);
    EXPECT_EQ(longerAlg.update(zero).state, DvManeuverBurnState::Executing);
    EXPECT_EQ(longerAlg.update(zero).state, DvManeuverBurnState::Complete);
}

TEST(DvManeuverTest, EdgeTimeGatesExactAtTenAndHundredHz) {
    // 0.1 s and 0.01 s cannot be stored exactly as a float, so a running float sum of the period drifts from the
    // true time. The burn time is an integer nanosecond count instead, so each gate passes exactly at its limit.
    constexpr uint64_t kTenHzStepNs = 100000000U;
    DvManeuverAlgorithm maxAlg{DvManeuverConfig::create(/* minTime = */ 0U,
                                                        /* maxTime = */ 10000000000U,
                                                        kTenHzStepNs,
                                                        kCmdForce_B,
                                                        /* cmdDv_N = */ Eigen::Vector3f{0.0F, 0.0F, 100.0F})};
    for (uint64_t k = 0U; k < 100U; ++k) {
        ASSERT_EQ(maxAlg.update(/* dvAccumulated = */ Eigen::Vector3f::Zero()).state, DvManeuverBurnState::Executing)
            << k;
    }
    EXPECT_EQ(maxAlg.update(/* dvAccumulated = */ Eigen::Vector3f::Zero()).state, DvManeuverBurnState::Complete);

    // A zero delta-V command leaves only the minTime gate, which must pass at exactly 1 s at 100 Hz.
    constexpr uint64_t kHundredHzStepNs = 10000000U;
    DvManeuverAlgorithm minAlg{DvManeuverConfig::create(/* minTime = */ 1000000000U,
                                                        /* maxTime = */ 100000000000U,
                                                        kHundredHzStepNs,
                                                        kCmdForce_B,
                                                        /* cmdDv_N = */ Eigen::Vector3f::Zero())};
    for (uint64_t k = 0U; k < 100U; ++k) {
        ASSERT_EQ(minAlg.update(/* dvAccumulated = */ Eigen::Vector3f::Zero()).state, DvManeuverBurnState::Executing)
            << k;
    }
    EXPECT_EQ(minAlg.update(/* dvAccumulated = */ Eigen::Vector3f::Zero()).state, DvManeuverBurnState::Complete);
}

// ---------------------------------------------------------------------------
// Config validation tests.
// ---------------------------------------------------------------------------

TEST(DvManeuverConfigTest, AcceptsValidConfigurations) {
    EXPECT_NO_THROW(DvManeuverConfig::create(
        /* minTime = */ 0U,
        /* maxTime = */ 10000000000U,
        kControlPeriod,
        kCmdForce_B,
        kCmdDv_N));
    EXPECT_NO_THROW(DvManeuverConfig::create(
        /* minTime = */ 2000000000U,
        /* maxTime = */ 10000000000U,
        kControlPeriod,
        kCmdForce_B,
        kCmdDv_N));
    EXPECT_NO_THROW(DvManeuverConfig::create(/* minTime = */ 0U,
                                             /* maxTime = */ 10000000000U,
                                             kControlPeriod,
                                             /* cmdForce_B = */ Eigen::Vector3f::Zero(),
                                             kCmdDv_N));
    EXPECT_NO_THROW(DvManeuverConfig::create(/* minTime = */ 0U,
                                             /* maxTime = */ 10000000000U,
                                             kControlPeriod,
                                             kCmdForce_B,
                                             /* cmdDv_N = */ Eigen::Vector3f::Zero()));
}

TEST(DvManeuverConfigTest, RejectsMaxTimeZero) {
    EXPECT_THROW(DvManeuverConfig::create(
                     /* minTime = */ 0U, /* maxTime = */ 0U, kControlPeriod, kCmdForce_B, kCmdDv_N),
                 fsw::invalid_argument);
}

TEST(DvManeuverConfigTest, RejectsMaxTimeEqualToMinTime) {
    EXPECT_THROW(DvManeuverConfig::create(
                     /* minTime = */ 1000000000U,
                     /* maxTime = */ 1000000000U,
                     kControlPeriod,
                     kCmdForce_B,
                     kCmdDv_N),
                 fsw::invalid_argument);
}

TEST(DvManeuverConfigTest, RejectsNonFiniteInputs) {
    const float nan = std::nanf("");
    const float inf = std::numeric_limits<float>::infinity();
    EXPECT_THROW(DvManeuverConfig::create(/* minTime = */ 0U,
                                          /* maxTime = */ 1000000000U,
                                          kControlPeriod,
                                          /* cmdForce_B = */ Eigen::Vector3f{nan, 0.0F, 0.0F},
                                          kCmdDv_N),
                 fsw::invalid_argument);
    EXPECT_THROW(DvManeuverConfig::create(/* minTime = */ 0U,
                                          /* maxTime = */ 1000000000U,
                                          kControlPeriod,
                                          /* cmdForce_B = */ Eigen::Vector3f{0.0F, -inf, 0.0F},
                                          kCmdDv_N),
                 fsw::invalid_argument);
    EXPECT_THROW(DvManeuverConfig::create(/* minTime = */ 0U,
                                          /* maxTime = */ 1000000000U,
                                          kControlPeriod,
                                          kCmdForce_B,
                                          /* cmdDv_N = */ Eigen::Vector3f{0.0F, 0.0F, nan}),
                 fsw::invalid_argument);
    EXPECT_THROW(DvManeuverConfig::create(/* minTime = */ 0U,
                                          /* maxTime = */ 1000000000U,
                                          kControlPeriod,
                                          kCmdForce_B,
                                          /* cmdDv_N = */ Eigen::Vector3f{inf, 0.0F, 0.0F}),
                 fsw::invalid_argument);
}

TEST(DvManeuverConfigTest, RejectsZeroControlPeriod) {
    EXPECT_THROW(DvManeuverConfig::create(/* minTime = */ 0U,
                                          /* maxTime = */ 1000000000U,
                                          /* controlPeriod = */ 0U,
                                          kCmdForce_B,
                                          kCmdDv_N),
                 fsw::invalid_argument);
}

TEST(DvManeuverConfigTest, GettersRoundTrip) {
    const auto cfg = DvManeuverConfig::create(/* minTime = */ 2000000000U,
                                              /* maxTime = */ 10000000000U,
                                              kControlPeriod,
                                              kCmdForce_B,
                                              kCmdDv_N);
    EXPECT_EQ(cfg.getMinTime(), 2000000000U);
    EXPECT_EQ(cfg.getMaxTime(), 10000000000U);
    EXPECT_EQ(cfg.getControlPeriod(), kControlPeriod);
    EXPECT_EQ(cfg.getCmdForce(), kCmdForce_B);
    EXPECT_EQ(cfg.getCmdDv(), kCmdDv_N);
}
