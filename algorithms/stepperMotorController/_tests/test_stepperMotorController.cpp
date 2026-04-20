#include "stepperMotorControllerTestHelpers.hpp"
#include <numbers>

// ---------------------------------------------------------------------------
// Regression tests
// ---------------------------------------------------------------------------

TEST(StepperMotorControllerTest, RegressionNominalMove) {
    regressionTestMultiStep(360,                                                    // stepsPerRevolution
                            10.0F * static_cast<float>(std::numbers::pi) / 180.0F,  // referenceAngle (10 deg)
                            0.0F,                                                   // initialAngle
                            10.0F,                                                  // controlFrequency
                            10.0F,                                                  // motorFrequency
                            2,                                                      // settleCountMax
                            0,                                                      // currentPositionTolerance
                            0);                                                     // desiredPositionTolerance
}

TEST(StepperMotorControllerTest, RegressionNonZeroInit) {
    regressionTestMultiStep(360,
                            10.0F * static_cast<float>(std::numbers::pi) / 180.0F,
                            -45.0F * static_cast<float>(std::numbers::pi) / 180.0F,
                            10.0F,
                            10.0F,
                            2,
                            0,
                            0);
}

TEST(StepperMotorControllerTest, RegressionShortestPath) {
    regressionTestMultiStep(360,
                            162.0F * static_cast<float>(std::numbers::pi) / 180.0F,
                            -162.0F * static_cast<float>(std::numbers::pi) / 180.0F,
                            10.0F,
                            10.0F,
                            0,
                            0,
                            0);
}

TEST(StepperMotorControllerTest, RegressionFractionalFrequency) {
    regressionTestMultiStep(360,
                            9.0F * static_cast<float>(std::numbers::pi) / 180.0F,
                            0.0F,
                            10.0F,
                            15.0F,  // 1.5 steps/tick
                            0,
                            0,
                            0);
}

TEST(StepperMotorControllerTest, RegressionLargeMove) {
    regressionTestMultiStep(200,
                            90.0F * static_cast<float>(std::numbers::pi) / 180.0F,
                            -90.0F * static_cast<float>(std::numbers::pi) / 180.0F,
                            10.0F,
                            50.0F,
                            5,
                            1,
                            0);
}

// ---------------------------------------------------------------------------
// Setup tests (setter validation + round-trip)
// ---------------------------------------------------------------------------

TEST(StepperMotorControllerTest, SetupTest) {
    StepperMotorControllerAlgorithm alg{};

    // stepsPerRevolution: reject <= 0
    EXPECT_THROW(alg.setStepsPerRevolution(0), fsw::invalid_argument);
    EXPECT_THROW(alg.setStepsPerRevolution(-1), fsw::invalid_argument);
    EXPECT_NO_THROW(alg.setStepsPerRevolution(200));
    EXPECT_EQ(alg.getStepsPerRevolution(), 200);

    // settleCountMax: reject < 0
    EXPECT_THROW(alg.setSettleCountMax(-1), fsw::invalid_argument);
    EXPECT_NO_THROW(alg.setSettleCountMax(0));
    EXPECT_EQ(alg.getSettleCountMax(), 0);
    EXPECT_NO_THROW(alg.setSettleCountMax(10));
    EXPECT_EQ(alg.getSettleCountMax(), 10);

    // currentPositionTolerance: reject < 0
    EXPECT_THROW(alg.setCurrentPositionTolerance(-1), fsw::invalid_argument);
    EXPECT_NO_THROW(alg.setCurrentPositionTolerance(0));
    EXPECT_EQ(alg.getCurrentPositionTolerance(), 0);
    EXPECT_NO_THROW(alg.setCurrentPositionTolerance(5));
    EXPECT_EQ(alg.getCurrentPositionTolerance(), 5);

    // desiredPositionTolerance: reject < 0
    EXPECT_THROW(alg.setDesiredPositionTolerance(-1), fsw::invalid_argument);
    EXPECT_NO_THROW(alg.setDesiredPositionTolerance(0));
    EXPECT_EQ(alg.getDesiredPositionTolerance(), 0);
    EXPECT_NO_THROW(alg.setDesiredPositionTolerance(3));
    EXPECT_EQ(alg.getDesiredPositionTolerance(), 3);
}

TEST(StepperMotorControllerTest, ResetBehavior) {
    StepperMotorControllerAlgorithm alg{};
    alg.setStepsPerRevolution(360);
    alg.setCurrentPositionTolerance(0);
    alg.setSettleCountMax(0);
    alg.reset();

    // Drive the algorithm through a full cycle at currentPosition=0 → 30
    constexpr float thirtyDeg = 30.0F * static_cast<float>(std::numbers::pi) / 180.0F;
    constexpr float tenDeg = 10.0F * static_cast<float>(std::numbers::pi) / 180.0F;
    StepperMotorSim sim{};
    sim.controlFrequency = 10.0F;
    sim.motorFrequency = 360.0F;  // all steps in 1 tick
    sim.reset(0);
    for (int i = 0; i < 20; ++i) {
        const auto out = alg.update(sim.currentPosition, thirtyDeg, false);
        sim.applyOutput(out);
        sim.advance();
    }
    // Motor is now at position 30

    // Reset should restore algorithm state machine to IDLE regardless of past state
    alg.reset();
    // Caller provides the fresh currentPosition (back to 0) on next update
    auto out = alg.update(0, tenDeg, false);

    // After reset, IDLE sees currentPosition=0, desired=10 → MOVE(10)
    EXPECT_EQ(out.commandType, StepperMotorCommandType::MOVE);
    EXPECT_EQ(out.stepsToMove, 10);
}

// ---------------------------------------------------------------------------
// Property tests
// ---------------------------------------------------------------------------

TEST(StepperMotorControllerTest, OutputCommandTypeIsValid) {
    propertyOutputCommandTypeIsValid(360, 0.5F, 0.0F, 10.0F, 10.0F, 2, 0, 0);
}

TEST(StepperMotorControllerTest, OutputCommandTypeIsValidLargeAngle) {
    propertyOutputCommandTypeIsValid(200, 1.0F, -0.5F, 5.0F, 15.0F, 5, 1, 0);
}

TEST(StepperMotorControllerTest, MoveStepsWithinHalfRevolution) {
    propertyMoveStepsWithinHalfRevolution(360, 0.5F, 0.0F, 0);
}

TEST(StepperMotorControllerTest, MoveStepsWithinHalfRevolutionLargeAngle) {
    propertyMoveStepsWithinHalfRevolution(200, 1.0F, -0.5F, 1);
}

TEST(StepperMotorControllerTest, MotorReachesTarget) {
    propertyMotorReachesTarget(360, 0.5F, 0.0F, 10.0F, 10.0F, 2, 0, 0);
}

TEST(StepperMotorControllerTest, MotorReachesTargetFractionalFreq) {
    propertyMotorReachesTarget(360, 0.5F, 0.0F, 10.0F, 15.0F, 0, 0, 0);
}

// ---------------------------------------------------------------------------
// Edge-case tests
// ---------------------------------------------------------------------------

TEST(StepperMotorControllerTest, ZeroStepMove) {
    StepperMotorControllerAlgorithm alg{};
    alg.setStepsPerRevolution(360);
    alg.setCurrentPositionTolerance(0);
    alg.reset();

    auto out = alg.update(0, 0.0F, false);
    EXPECT_EQ(out.commandType, StepperMotorCommandType::NONE);
    EXPECT_EQ(out.stepsToMove, 0);
}

TEST(StepperMotorControllerTest, ExactlyAtTolerance) {
    StepperMotorControllerAlgorithm alg{};
    alg.setStepsPerRevolution(360);
    alg.setCurrentPositionTolerance(5);
    alg.reset();

    // 5 deg = 5 steps, tolerance = 5, abs(5) is NOT > 5 → no MOVE
    constexpr float fiveDeg = 5.0F * static_cast<float>(std::numbers::pi) / 180.0F;
    auto out = alg.update(0, fiveDeg, false);
    EXPECT_EQ(out.commandType, StepperMotorCommandType::NONE);
}

TEST(StepperMotorControllerTest, OneStepOverTolerance) {
    StepperMotorControllerAlgorithm alg{};
    alg.setStepsPerRevolution(360);
    alg.setCurrentPositionTolerance(5);
    alg.reset();

    // 6 deg = 6 steps, tolerance = 5, abs(6) > 5 → MOVE
    constexpr float sixDeg = 6.0F * static_cast<float>(std::numbers::pi) / 180.0F;
    auto out = alg.update(0, sixDeg, false);
    EXPECT_EQ(out.commandType, StepperMotorCommandType::MOVE);
    EXPECT_EQ(out.stepsToMove, 6);
}

TEST(StepperMotorControllerTest, WrapAtExactlyHalfRevolution) {
    StepperMotorControllerAlgorithm alg{};
    alg.setStepsPerRevolution(360);
    alg.setCurrentPositionTolerance(0);
    alg.reset();

    // 180 deg = 180 steps = exactly half revolution
    constexpr float pi = static_cast<float>(std::numbers::pi);
    auto out = alg.update(0, pi, false);
    EXPECT_EQ(out.commandType, StepperMotorCommandType::MOVE);
    // wrapDelta(180, 360): 180 % 360 = 180, 180 <= 180 (not >), so result = 180
    EXPECT_EQ(abs(out.stepsToMove), 180);
}

TEST(StepperMotorControllerTest, NegativeWrap) {
    StepperMotorControllerAlgorithm alg{};
    alg.setStepsPerRevolution(360);
    alg.setCurrentPositionTolerance(0);
    alg.reset();

    // 181 deg: naive steps = 181, wrapDelta(181, 360) = 181 > 180 → 181 - 360 = -179
    constexpr float deg181 = 181.0F * static_cast<float>(std::numbers::pi) / 180.0F;
    auto out = alg.update(0, deg181, false);
    EXPECT_EQ(out.commandType, StepperMotorCommandType::MOVE);
    EXPECT_LT(out.stepsToMove, 0);
    EXPECT_EQ(out.stepsToMove, -179);
}

TEST(StepperMotorControllerTest, DesiredChangeMidMove) {
    StepperMotorControllerAlgorithm alg{};
    alg.setStepsPerRevolution(360);
    alg.setCurrentPositionTolerance(0);
    alg.setDesiredPositionTolerance(0);
    alg.setSettleCountMax(0);
    alg.reset();

    StepperMotorSim sim{};
    sim.controlFrequency = 10.0F;
    sim.motorFrequency = 10.0F;  // 1 step/tick
    sim.reset(0);

    constexpr float pi = static_cast<float>(std::numbers::pi);
    constexpr float twentyDeg = 20.0F * pi / 180.0F;
    constexpr float tenDeg = 10.0F * pi / 180.0F;

    // Tick 1: IDLE -> MOVE(20)
    auto out = alg.update(sim.currentPosition, twentyDeg, false);
    EXPECT_EQ(out.commandType, StepperMotorCommandType::MOVE);
    EXPECT_EQ(out.stepsToMove, 20);
    sim.applyOutput(out);
    sim.advance();

    // Tick 2: MOVING, advance 1 step
    out = alg.update(sim.currentPosition, twentyDeg, false);
    EXPECT_EQ(out.commandType, StepperMotorCommandType::NONE);
    sim.applyOutput(out);
    sim.advance();

    // Tick 3: change desired to 10 deg while moving → STOP
    out = alg.update(sim.currentPosition, tenDeg, false);
    EXPECT_EQ(out.commandType, StepperMotorCommandType::STOP);
}

TEST(StepperMotorControllerTest, DesiredChangeWithinDesiredTolerance) {
    StepperMotorControllerAlgorithm alg{};
    alg.setStepsPerRevolution(360);
    alg.setCurrentPositionTolerance(0);
    alg.setDesiredPositionTolerance(3);  // allow small changes without interrupting
    alg.setSettleCountMax(0);
    alg.reset();

    StepperMotorSim sim{};
    sim.controlFrequency = 10.0F;
    sim.motorFrequency = 10.0F;
    sim.reset(0);

    constexpr float pi = static_cast<float>(std::numbers::pi);
    constexpr float twentyDeg = 20.0F * pi / 180.0F;
    constexpr float twentyTwoDeg = 22.0F * pi / 180.0F;

    // Tick 1: IDLE -> MOVE(20)
    auto out = alg.update(sim.currentPosition, twentyDeg, false);
    EXPECT_EQ(out.commandType, StepperMotorCommandType::MOVE);
    sim.applyOutput(out);
    sim.advance();

    // Tick 2: tiny change in reference (22 vs 20), within desiredPositionTolerance → keep going
    out = alg.update(sim.currentPosition, twentyTwoDeg, false);
    EXPECT_EQ(out.commandType, StepperMotorCommandType::NONE);
}

TEST(StepperMotorControllerTest, StepAccumulatorFractionalRatio) {
    StepperMotorControllerAlgorithm alg{};
    alg.setStepsPerRevolution(360);
    alg.setCurrentPositionTolerance(0);
    alg.setSettleCountMax(0);
    alg.reset();

    StepperMotorSim sim{};
    sim.controlFrequency = 10.0F;
    sim.motorFrequency = 15.0F;  // 1.5 steps/tick
    sim.reset(0);

    constexpr float sixDeg = 6.0F * static_cast<float>(std::numbers::pi) / 180.0F;

    // Tick 1: IDLE -> MOVE(6)
    auto out = alg.update(sim.currentPosition, sixDeg, false);
    EXPECT_EQ(out.commandType, StepperMotorCommandType::MOVE);
    EXPECT_EQ(out.stepsToMove, 6);
    sim.applyOutput(out);
    sim.advance();

    // Ticks 2-5: MOVING with pattern 1, 2, 1, 2 = 6 total → should reach target
    for (int i = 0; i < 4; ++i) {
        out = alg.update(sim.currentPosition, sixDeg, false);
        sim.applyOutput(out);
        sim.advance();
    }

    // By now should be in STOPPING or SETTLING (motor not moving → immediate transition)
    out = alg.update(sim.currentPosition, sixDeg, false);
    sim.applyOutput(out);
    sim.advance();

    // Tick 7: IDLE, desired == current → NONE
    out = alg.update(sim.currentPosition, sixDeg, false);
    sim.applyOutput(out);
    sim.advance();
    EXPECT_EQ(out.commandType, StepperMotorCommandType::NONE);

    // Verify by commanding return to 0: should get exactly -6
    out = alg.update(sim.currentPosition, 0.0F, false);
    EXPECT_EQ(out.commandType, StepperMotorCommandType::MOVE);
    EXPECT_EQ(out.stepsToMove, -6);
}

TEST(StepperMotorControllerTest, SettleCountZero) {
    StepperMotorControllerAlgorithm alg{};
    alg.setStepsPerRevolution(360);
    alg.setCurrentPositionTolerance(0);
    alg.setSettleCountMax(0);
    alg.reset();

    StepperMotorSim sim{};
    sim.controlFrequency = 10.0F;
    sim.motorFrequency = 360.0F;  // all steps in 1 tick
    sim.reset(0);

    constexpr float tenDeg = 10.0F * static_cast<float>(std::numbers::pi) / 180.0F;

    // Tick 1: IDLE -> MOVE(10)
    auto out = alg.update(sim.currentPosition, tenDeg, false);
    EXPECT_EQ(out.commandType, StepperMotorCommandType::MOVE);
    sim.applyOutput(out);
    sim.advance();

    // Tick 2: MOVING, advance all steps → STOPPING
    out = alg.update(sim.currentPosition, tenDeg, false);
    sim.applyOutput(out);
    sim.advance();

    // Tick 3: STOPPING → SETTLING (isMotorMoving=false)
    out = alg.update(sim.currentPosition, tenDeg, false);
    sim.applyOutput(out);
    sim.advance();

    // Tick 4: SETTLING → IDLE (settleCount=0 >= 0)
    out = alg.update(sim.currentPosition, tenDeg, false);
    sim.applyOutput(out);
    sim.advance();

    // Tick 5: IDLE, desired == current → NONE
    out = alg.update(sim.currentPosition, tenDeg, false);
    EXPECT_EQ(out.commandType, StepperMotorCommandType::NONE);
}
