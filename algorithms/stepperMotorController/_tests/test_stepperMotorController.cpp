#include "stepperMotorControllerTestHelpers.hpp"
#include <numbers>

// ---------------------------------------------------------------------------
// Regression tests
// ---------------------------------------------------------------------------

TEST(StepperMotorControllerTest, RegressionNominalMove) {
    regressionTestMultiStep(std::numbers::pi_v<float> / 180.0F,  // stepAngle (1 deg/step = 360 steps/rev)
                            0.0F,                                // minAngle
                            2.0F * std::numbers::pi_v<float>,    // maxAngle (full circle)
                            10.0F * static_cast<float>(std::numbers::pi) / 180.0F,  // referenceAngle (10 deg)
                            0.0F,                                                   // initialAngle
                            10.0F,                                                  // controlFrequency
                            10.0F,                                                  // motorFrequency
                            2,                                                      // settleCountMax
                            1);                                                     // minStepCommand
}

TEST(StepperMotorControllerTest, RegressionNonZeroInit) {
    regressionTestMultiStep(std::numbers::pi_v<float> / 180.0F,  // stepAngle (1 deg/step = 360 steps/rev)
                            0.0F,                                // minAngle
                            2.0F * std::numbers::pi_v<float>,    // maxAngle (full circle)
                            10.0F * static_cast<float>(std::numbers::pi) / 180.0F,
                            -45.0F * static_cast<float>(std::numbers::pi) / 180.0F,
                            10.0F,
                            10.0F,
                            2,
                            1);
}

TEST(StepperMotorControllerTest, RegressionShortestPath) {
    regressionTestMultiStep(std::numbers::pi_v<float> / 180.0F,  // stepAngle (1 deg/step = 360 steps/rev)
                            0.0F,                                // minAngle
                            2.0F * std::numbers::pi_v<float>,    // maxAngle (full circle)
                            162.0F * static_cast<float>(std::numbers::pi) / 180.0F,
                            -162.0F * static_cast<float>(std::numbers::pi) / 180.0F,
                            10.0F,
                            10.0F,
                            0,
                            1);
}

TEST(StepperMotorControllerTest, RegressionFractionalFrequency) {
    regressionTestMultiStep(std::numbers::pi_v<float> / 180.0F,  // stepAngle (1 deg/step = 360 steps/rev)
                            0.0F,                                // minAngle
                            2.0F * std::numbers::pi_v<float>,    // maxAngle (full circle)
                            9.0F * static_cast<float>(std::numbers::pi) / 180.0F,
                            0.0F,
                            10.0F,
                            15.0F,  // 1.5 steps/tick
                            0,
                            1);
}

TEST(StepperMotorControllerTest, RegressionLargeMove) {
    regressionTestMultiStep(2.0F * std::numbers::pi_v<float> / 200.0F,  // stepAngle (1.8 deg/step = 200 steps/rev)
                            0.0F,                                       // minAngle
                            2.0F * std::numbers::pi_v<float>,           // maxAngle (full circle)
                            90.0F * static_cast<float>(std::numbers::pi) / 180.0F,
                            -90.0F * static_cast<float>(std::numbers::pi) / 180.0F,
                            10.0F,
                            50.0F,
                            5,
                            2);
}

// ---------------------------------------------------------------------------
// Setup tests (config validation + round-trip)
// ---------------------------------------------------------------------------

TEST(StepperMotorControllerTest, SetupTest) {
    constexpr float twoPi = 2.0F * std::numbers::pi_v<float>;
    const StepperMotorAngleRange fullRange{0.0F, twoPi};

    const auto makeConfig = [&](float stepAngle, const StepperMotorAngleRange& range, uint32_t minStepCommand) {
        return StepperMotorControllerConfig::create(stepAngle, range, 10U, minStepCommand);
    };

    // stepAngle must be in [2*pi/kMaxStepsPerRev, 2*pi].
    EXPECT_THROW(makeConfig(0.0F, fullRange, 1U), fsw::invalid_argument);
    EXPECT_THROW(makeConfig(-0.1F, fullRange, 1U), fsw::invalid_argument);
    EXPECT_THROW(makeConfig(twoPi + 0.1F, fullRange, 1U), fsw::invalid_argument);
    EXPECT_THROW(makeConfig(twoPi / 100001.0F, fullRange, 1U), fsw::invalid_argument);  // stepsPerRev > 100k
    EXPECT_NO_THROW(makeConfig(twoPi / 100000.0F, fullRange, 1U));                      // stepsPerRev = 100k (boundary)
    EXPECT_NO_THROW(makeConfig(twoPi / 200.0F, fullRange, 1U));

    // minStepCommand must be greater than zero (settleCountMax has no constraint).
    EXPECT_THROW(makeConfig(twoPi / 200.0F, fullRange, 0U), fsw::invalid_argument);
    EXPECT_NO_THROW(makeConfig(twoPi / 200.0F, fullRange, 1U));

    // Motor angle range: reject min < -2pi, max > 2pi, min >= max.
    EXPECT_THROW(makeConfig(twoPi / 200.0F, StepperMotorAngleRange{-twoPi - 0.1F, 1.0F}, 1U), fsw::invalid_argument);
    EXPECT_THROW(makeConfig(twoPi / 200.0F, StepperMotorAngleRange{1.0F, twoPi + 0.1F}, 1U), fsw::invalid_argument);
    EXPECT_THROW(makeConfig(twoPi / 200.0F, StepperMotorAngleRange{2.0F, 1.0F}, 1U), fsw::invalid_argument);
    EXPECT_THROW(makeConfig(twoPi / 200.0F, StepperMotorAngleRange{1.0F, 1.0F}, 1U), fsw::invalid_argument);
    EXPECT_NO_THROW(makeConfig(twoPi / 200.0F, StepperMotorAngleRange{-twoPi, twoPi}, 1U));
    EXPECT_NO_THROW(makeConfig(twoPi / 200.0F, StepperMotorAngleRange{-1.0F, 1.0F}, 1U));

    // Getter round-trips. The derived stepsPerRev / full-circle values are algorithm-internal and exercised
    // behaviorally by the WrapAtExactlyHalfRevolution and RangeUsesLinearDeltaForPartialRange tests.
    const auto cfg = StepperMotorControllerConfig::create(twoPi / 200.0F, StepperMotorAngleRange{0.5F, 1.5F}, 7U, 5U);
    EXPECT_FLOAT_EQ(cfg.getStepAngle(), twoPi / 200.0F);
    EXPECT_EQ(cfg.getSettleCountMax(), 7U);
    EXPECT_EQ(cfg.getMinStepCommand(), 5U);
    EXPECT_FLOAT_EQ(cfg.getAngleRange().minAngle, 0.5F);
    EXPECT_FLOAT_EQ(cfg.getAngleRange().maxAngle, 1.5F);
}

TEST(StepperMotorControllerTest, ResetBehavior) {
    constexpr float twoPi = 2.0F * std::numbers::pi_v<float>;
    auto alg = makeStepperMotorControllerAlgorithm(twoPi / 360.0F, 0.0F, twoPi, 0U, 1U);

    // Drive the algorithm through a full cycle at currentPosition=0 → 30
    constexpr float thirtyDeg = 30.0F * static_cast<float>(std::numbers::pi) / 180.0F;
    constexpr float tenDeg = 10.0F * static_cast<float>(std::numbers::pi) / 180.0F;
    StepperMotorSim sim{};
    sim.controlFrequency = 10.0F;
    sim.motorFrequency = 360.0F;  // all steps in 1 tick
    sim.reset(0);
    for (int i = 0; i < 20; ++i) {
        const auto out = alg.update(sim.currentPosition, thirtyDeg, sim.isMoving());
        sim.applyOutput(out);
        sim.advance();
    }
    // Motor is now at position 30

    // reInitialize should restore the algorithm state machine to IDLE regardless of past state
    alg.reInitialize();
    // Caller provides the fresh currentPosition (back to 0) on next update
    auto out = alg.update(0, tenDeg, false);

    // After reInitialize, IDLE sees currentPosition=0, desired=10 → MOVE(10)
    EXPECT_EQ(out.commandType, StepperMotorCommandType::MOVE);
    EXPECT_EQ(out.stepsToMove, 10);
}

// ---------------------------------------------------------------------------
// Property tests
// ---------------------------------------------------------------------------

TEST(StepperMotorControllerTest, OutputCommandTypeIsValid) {
    constexpr float twoPi = 2.0F * std::numbers::pi_v<float>;
    propertyOutputCommandTypeIsValid(std::numbers::pi_v<float> / 180.0F, 0.0F, twoPi, 0.5F, 0.0F, 10.0F, 10.0F, 2, 1);
}

TEST(StepperMotorControllerTest, OutputCommandTypeIsValidLargeAngle) {
    constexpr float twoPi = 2.0F * std::numbers::pi_v<float>;
    propertyOutputCommandTypeIsValid(
        2.0F * std::numbers::pi_v<float> / 200.0F, 0.0F, twoPi, 1.0F, -0.5F, 5.0F, 15.0F, 5, 2);
}

TEST(StepperMotorControllerTest, MoveStepsWithinHalfRevolution) {
    constexpr float twoPi = 2.0F * std::numbers::pi_v<float>;
    propertyMoveStepsWithinHalfRevolution(std::numbers::pi_v<float> / 180.0F, 0.0F, twoPi, 0.5F, 0.0F, 1);
}

TEST(StepperMotorControllerTest, MoveStepsWithinHalfRevolutionLargeAngle) {
    constexpr float twoPi = 2.0F * std::numbers::pi_v<float>;
    propertyMoveStepsWithinHalfRevolution(2.0F * std::numbers::pi_v<float> / 200.0F, 0.0F, twoPi, 1.0F, -0.5F, 2);
}

TEST(StepperMotorControllerTest, MotorReachesTarget) {
    constexpr float twoPi = 2.0F * std::numbers::pi_v<float>;
    propertyMotorReachesTarget(std::numbers::pi_v<float> / 180.0F, 0.0F, twoPi, 0.5F, 0.0F, 10.0F, 10.0F, 2, 1);
}

TEST(StepperMotorControllerTest, MotorReachesTargetFractionalFreq) {
    constexpr float twoPi = 2.0F * std::numbers::pi_v<float>;
    propertyMotorReachesTarget(std::numbers::pi_v<float> / 180.0F, 0.0F, twoPi, 0.5F, 0.0F, 10.0F, 15.0F, 0, 1);
}

// ---------------------------------------------------------------------------
// Edge-case tests
// ---------------------------------------------------------------------------

TEST(StepperMotorControllerTest, ZeroStepMove) {
    constexpr float twoPi = 2.0F * std::numbers::pi_v<float>;
    auto alg = makeStepperMotorControllerAlgorithm(twoPi / 360.0F, 0.0F, twoPi, 10U, 1U);

    auto out = alg.update(0, 0.0F, false);
    EXPECT_EQ(out.commandType, StepperMotorCommandType::NONE);
    EXPECT_EQ(out.stepsToMove, 0);
}

TEST(StepperMotorControllerTest, BelowMinStepCommand) {
    constexpr float twoPi = 2.0F * std::numbers::pi_v<float>;
    auto alg = makeStepperMotorControllerAlgorithm(twoPi / 360.0F, 0.0F, twoPi, 10U, 6U);

    // 5 deg = 5 steps, minStepCommand = 6, abs(5) is NOT >= 6 → no MOVE
    constexpr float fiveDeg = 5.0F * static_cast<float>(std::numbers::pi) / 180.0F;
    auto out = alg.update(0, fiveDeg, false);
    EXPECT_EQ(out.commandType, StepperMotorCommandType::NONE);
}

TEST(StepperMotorControllerTest, AtMinStepCommand) {
    constexpr float twoPi = 2.0F * std::numbers::pi_v<float>;
    auto alg = makeStepperMotorControllerAlgorithm(twoPi / 360.0F, 0.0F, twoPi, 10U, 6U);

    // 6 deg = 6 steps, minStepCommand = 6, abs(6) >= 6 → MOVE
    constexpr float sixDeg = 6.0F * static_cast<float>(std::numbers::pi) / 180.0F;
    auto out = alg.update(0, sixDeg, false);
    EXPECT_EQ(out.commandType, StepperMotorCommandType::MOVE);
    EXPECT_EQ(out.stepsToMove, 6);
}

TEST(StepperMotorControllerTest, WrapAtExactlyHalfRevolution) {
    constexpr float twoPi = 2.0F * std::numbers::pi_v<float>;
    auto alg = makeStepperMotorControllerAlgorithm(twoPi / 360.0F, 0.0F, twoPi, 10U, 1U);

    // 180 deg = 180 steps = exactly half revolution
    constexpr float pi = static_cast<float>(std::numbers::pi);
    auto out = alg.update(0, pi, false);
    EXPECT_EQ(out.commandType, StepperMotorCommandType::MOVE);
    // wrapDelta(180, 360): 180 % 360 = 180, 180 <= 180 (not >), so result = 180
    EXPECT_EQ(abs(out.stepsToMove), 180);
}

TEST(StepperMotorControllerTest, NegativeWrap) {
    constexpr float twoPi = 2.0F * std::numbers::pi_v<float>;
    auto alg = makeStepperMotorControllerAlgorithm(twoPi / 360.0F, 0.0F, twoPi, 10U, 1U);

    // 181 deg: naive steps = 181, wrapDelta(181, 360) = 181 > 180 → 181 - 360 = -179
    constexpr float deg181 = 181.0F * static_cast<float>(std::numbers::pi) / 180.0F;
    auto out = alg.update(0, deg181, false);
    EXPECT_EQ(out.commandType, StepperMotorCommandType::MOVE);
    EXPECT_LT(out.stepsToMove, 0);
    EXPECT_EQ(out.stepsToMove, -179);
}

TEST(StepperMotorControllerTest, DesiredChangeMidMove) {
    constexpr float twoPi = 2.0F * std::numbers::pi_v<float>;
    auto alg = makeStepperMotorControllerAlgorithm(twoPi / 360.0F, 0.0F, twoPi, 0U, 1U);

    StepperMotorSim sim{};
    sim.controlFrequency = 10.0F;
    sim.motorFrequency = 10.0F;  // 1 step/tick
    sim.reset(0);

    constexpr float pi = static_cast<float>(std::numbers::pi);
    constexpr float twentyDeg = 20.0F * pi / 180.0F;
    constexpr float tenDeg = 10.0F * pi / 180.0F;

    // Tick 1: IDLE -> MOVE(20)
    auto out = alg.update(sim.currentPosition, twentyDeg, sim.isMoving());
    EXPECT_EQ(out.commandType, StepperMotorCommandType::MOVE);
    EXPECT_EQ(out.stepsToMove, 20);
    sim.applyOutput(out);
    sim.advance();

    // Tick 2: MOVING, advance 1 step
    out = alg.update(sim.currentPosition, twentyDeg, sim.isMoving());
    EXPECT_EQ(out.commandType, StepperMotorCommandType::NONE);
    sim.applyOutput(out);
    sim.advance();

    // Tick 3: change desired to 10 deg while moving → STOP
    out = alg.update(sim.currentPosition, tenDeg, sim.isMoving());
    EXPECT_EQ(out.commandType, StepperMotorCommandType::STOP);
}

TEST(StepperMotorControllerTest, DesiredChangeBelowMinStepCommand) {
    constexpr float twoPi = 2.0F * std::numbers::pi_v<float>;
    // minStepCommand = 4: allow small changes (< 4 steps) without interrupting
    auto alg = makeStepperMotorControllerAlgorithm(twoPi / 360.0F, 0.0F, twoPi, 0U, 4U);

    StepperMotorSim sim{};
    sim.controlFrequency = 10.0F;
    sim.motorFrequency = 10.0F;
    sim.reset(0);

    constexpr float pi = static_cast<float>(std::numbers::pi);
    constexpr float twentyDeg = 20.0F * pi / 180.0F;
    constexpr float twentyTwoDeg = 22.0F * pi / 180.0F;

    // Tick 1: IDLE -> MOVE(20)
    auto out = alg.update(sim.currentPosition, twentyDeg, sim.isMoving());
    EXPECT_EQ(out.commandType, StepperMotorCommandType::MOVE);
    sim.applyOutput(out);
    sim.advance();

    // Tick 2: tiny change in reference (22 vs 20), |delta|=2 < minStepCommand=4 → keep going
    out = alg.update(sim.currentPosition, twentyTwoDeg, sim.isMoving());
    EXPECT_EQ(out.commandType, StepperMotorCommandType::NONE);
}

TEST(StepperMotorControllerTest, StepAccumulatorFractionalRatio) {
    constexpr float twoPi = 2.0F * std::numbers::pi_v<float>;
    auto alg = makeStepperMotorControllerAlgorithm(twoPi / 360.0F, 0.0F, twoPi, 0U, 1U);

    StepperMotorSim sim{};
    sim.controlFrequency = 10.0F;
    sim.motorFrequency = 15.0F;  // 1.5 steps/tick
    sim.reset(0);

    constexpr float sixDeg = 6.0F * static_cast<float>(std::numbers::pi) / 180.0F;

    // Tick 1: IDLE -> MOVE(6)
    auto out = alg.update(sim.currentPosition, sixDeg, sim.isMoving());
    EXPECT_EQ(out.commandType, StepperMotorCommandType::MOVE);
    EXPECT_EQ(out.stepsToMove, 6);
    sim.applyOutput(out);
    sim.advance();

    // Ticks 2-5: MOVING with pattern 1, 2, 1, 2 = 6 total → should reach target
    for (int i = 0; i < 4; ++i) {
        out = alg.update(sim.currentPosition, sixDeg, sim.isMoving());
        sim.applyOutput(out);
        sim.advance();
    }

    // By now should be in SETTLING (motor reached target, MOVING transitions directly to SETTLING
    // when isMotorMoving becomes false)
    out = alg.update(sim.currentPosition, sixDeg, sim.isMoving());
    sim.applyOutput(out);
    sim.advance();

    // Tick 7: IDLE, desired == current → NONE
    out = alg.update(sim.currentPosition, sixDeg, sim.isMoving());
    sim.applyOutput(out);
    sim.advance();
    EXPECT_EQ(out.commandType, StepperMotorCommandType::NONE);

    // Verify by commanding return to 0: should get exactly -6
    out = alg.update(sim.currentPosition, 0.0F, sim.isMoving());
    EXPECT_EQ(out.commandType, StepperMotorCommandType::MOVE);
    EXPECT_EQ(out.stepsToMove, -6);
}

TEST(StepperMotorControllerTest, SettleCountZero) {
    constexpr float twoPi = 2.0F * std::numbers::pi_v<float>;
    auto alg = makeStepperMotorControllerAlgorithm(twoPi / 360.0F, 0.0F, twoPi, 0U, 1U);

    StepperMotorSim sim{};
    sim.controlFrequency = 10.0F;
    sim.motorFrequency = 360.0F;  // all steps in 1 tick
    sim.reset(0);

    constexpr float tenDeg = 10.0F * static_cast<float>(std::numbers::pi) / 180.0F;

    // Tick 1: IDLE -> MOVE(10)
    auto out = alg.update(sim.currentPosition, tenDeg, sim.isMoving());
    EXPECT_EQ(out.commandType, StepperMotorCommandType::MOVE);
    sim.applyOutput(out);
    sim.advance();

    // Tick 2: MOVING, motor reached target (isMotorMoving=false) → SETTLING (skips STOPPING)
    out = alg.update(sim.currentPosition, tenDeg, sim.isMoving());
    sim.applyOutput(out);
    sim.advance();

    // Tick 3: SETTLING → IDLE (settleCount=0 >= settleCountMax=0)
    out = alg.update(sim.currentPosition, tenDeg, sim.isMoving());
    sim.applyOutput(out);
    sim.advance();

    // Tick 4: IDLE, desired == current → NONE
    out = alg.update(sim.currentPosition, tenDeg, sim.isMoving());
    EXPECT_EQ(out.commandType, StepperMotorCommandType::NONE);
}

// ---------------------------------------------------------------------------
// Motor angle range tests
// ---------------------------------------------------------------------------

// Out-of-range references (above max and below min) are rejected: no MOVE issued, motor stays put.
TEST(StepperMotorControllerTest, RangeRejectsOutOfBoundsReference) {
    constexpr float pi = static_cast<float>(std::numbers::pi);
    auto alg = makeStepperMotorControllerAlgorithm(
        std::numbers::pi_v<float> / 180.0F, pi / 4.0F, pi / 2.0F, 10U, 1U);  // [45 deg, 90 deg]

    // Reference below minAngle: 30 deg
    auto out = alg.update(50, 30.0F * pi / 180.0F, false);
    EXPECT_EQ(out.commandType, StepperMotorCommandType::NONE);
    EXPECT_EQ(out.stepsToMove, 0);

    // Reference above maxAngle: 120 deg
    out = alg.update(50, 120.0F * pi / 180.0F, false);
    EXPECT_EQ(out.commandType, StepperMotorCommandType::NONE);
    EXPECT_EQ(out.stepsToMove, 0);

    // In-range reference: 60 deg → MOVE
    out = alg.update(50, 60.0F * pi / 180.0F, false);
    EXPECT_EQ(out.commandType, StepperMotorCommandType::MOVE);
    EXPECT_EQ(out.stepsToMove, 10);  // 60 - 50
}

// While MOVING, an out-of-range new reference is ignored (desiredPosition stays unchanged); the
// motor continues toward the previously-commanded valid target.
TEST(StepperMotorControllerTest, RangeIgnoresOutOfBoundsRefDuringMove) {
    constexpr float pi = static_cast<float>(std::numbers::pi);
    auto alg = makeStepperMotorControllerAlgorithm(
        std::numbers::pi_v<float> / 180.0F, pi / 4.0F, pi / 2.0F, 0U, 1U);  // [45 deg, 90 deg]

    StepperMotorSim sim{};
    sim.controlFrequency = 10.0F;
    sim.motorFrequency = 10.0F;  // 1 step/tick
    sim.reset(50);               // start at 50 deg (in range)

    // Tick 1: in-range ref 70 deg -> MOVE(20)
    auto out = alg.update(sim.currentPosition, 70.0F * pi / 180.0F, sim.isMoving());
    EXPECT_EQ(out.commandType, StepperMotorCommandType::MOVE);
    EXPECT_EQ(out.stepsToMove, 20);
    sim.applyOutput(out);
    sim.advance();

    // Tick 2: out-of-range ref 120 deg → NONE; commanded position still 70
    out = alg.update(sim.currentPosition, 120.0F * pi / 180.0F, sim.isMoving());
    EXPECT_EQ(out.commandType, StepperMotorCommandType::NONE);
}

// Partial range spanning more than half a revolution must NOT use shortest-path wrap. The
// commanded delta equals desired - current even when |delta| > stepsPerRev/2.
TEST(StepperMotorControllerTest, RangeUsesLinearDeltaForPartialRange) {
    constexpr float pi = static_cast<float>(std::numbers::pi);
    auto alg = makeStepperMotorControllerAlgorithm(
        std::numbers::pi_v<float> / 180.0F, pi / 6.0F, 11.0F * pi / 6.0F, 10U, 1U);  // [30 deg, 330 deg], span 300 deg

    // Move from 40 deg to 320 deg. Linear delta = +280 (forward).
    // If wrap were used, delta would be -80 (backward through forbidden 30 deg seam) -- WRONG.
    const auto out = alg.update(40, 320.0F * pi / 180.0F, false);
    EXPECT_EQ(out.commandType, StepperMotorCommandType::MOVE);
    EXPECT_EQ(out.stepsToMove, 280);
}
