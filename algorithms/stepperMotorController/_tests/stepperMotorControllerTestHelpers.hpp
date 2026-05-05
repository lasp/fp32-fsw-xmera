#ifndef TEST_STEPPERMOTORCONTROLLER_H
#define TEST_STEPPERMOTORCONTROLLER_H

#include "stepperMotorControllerAlgorithm.h"
#include "stepperMotorControllerTypes.h"
#include "utilities/freestandingInvalidArgument.h"
#include <gtest/gtest.h>
#include <cmath>
#include <numbers>
#include <vector>

// ---------------------------------------------------------------------------
// Motor motion simulator (mirrors the adapter's position-tracking logic)
// ---------------------------------------------------------------------------

struct StepperMotorSim {
    int currentPosition{};
    int commandedPosition{};
    float stepAccumulator{};
    float controlFrequency{1.0F};
    float motorFrequency{1.0F};

    void reset(int initialStep) {
        currentPosition = initialStep;
        commandedPosition = initialStep;
        stepAccumulator = 0.0F;
    }

    void applyOutput(const StepperMotorControllerOutput& output) {
        if (output.commandType == StepperMotorCommandType::MOVE) {
            commandedPosition = currentPosition + output.stepsToMove;
            stepAccumulator = 0.0F;
        } else if (output.commandType == StepperMotorCommandType::STOP) {
            commandedPosition = currentPosition;
        }
    }

    void advance() {
        if (currentPosition == commandedPosition) {
            return;
        }
        stepAccumulator += motorFrequency / controlFrequency;
        const auto wholeSteps = static_cast<int>(stepAccumulator);
        stepAccumulator -= static_cast<float>(wholeSteps);
        const int remaining = abs(commandedPosition - currentPosition);
        const int stepsToAdvance = (wholeSteps < remaining) ? wholeSteps : remaining;
        if (currentPosition < commandedPosition) {
            currentPosition += stepsToAdvance;
        } else {
            currentPosition -= stepsToAdvance;
        }
    }
};

// ---------------------------------------------------------------------------
// Helper: convert an angle [rad] to a step position matching the algorithm's rounding
// ---------------------------------------------------------------------------

inline int angleToSteps(float angle, float stepAngle) { return static_cast<int>(round(angle / stepAngle)); }

// ---------------------------------------------------------------------------
// Independent reference implementation for regression cross-checking
// ---------------------------------------------------------------------------

struct StepperMotorControllerReference {
    int commandedPosition{};
    int desiredPosition{};
    uint32_t settleCount{};
    StepperMotorState state{StepperMotorState::IDLE};

    int stepsPerRevolution{360};
    float stepAngle{2.0F * std::numbers::pi_v<float> / 360.0F};
    uint32_t settleCountMax{10};
    uint32_t currentPositionTolerance{1};
    uint32_t desiredPositionTolerance{0};
    float minAngle{0.0F};
    float maxAngle{2.0F * std::numbers::pi_v<float>};
    bool isFullCircle{true};

    int wrapDelta(int delta) const {
        const int half = stepsPerRevolution / 2;
        delta = delta % stepsPerRevolution;
        if (delta > half) {
            delta -= stepsPerRevolution;
        }
        if (delta < -half) {
            delta += stepsPerRevolution;
        }
        return delta;
    }

    int stepDelta(int delta) const { return isFullCircle ? wrapDelta(delta) : delta; }

    void reset() {
        state = StepperMotorState::IDLE;
        commandedPosition = 0;
        desiredPosition = 0;
        settleCount = 0;
    }

    StepperMotorControllerOutput update(int currentPosition, float referenceAngle, bool isMotorMoving) {
        StepperMotorControllerOutput output{};
        const bool isReferenceInRange = isFullCircle || (referenceAngle >= minAngle && referenceAngle <= maxAngle);
        if (isReferenceInRange) {
            desiredPosition = angleToSteps(referenceAngle, stepAngle);
        }

        switch (state) {
            case StepperMotorState::OFF:
                break;

            case StepperMotorState::MOVING: {
                if (static_cast<uint32_t>(abs(stepDelta(commandedPosition - currentPosition))) <=
                    currentPositionTolerance) {
                    state = StepperMotorState::STOPPING;
                }
                if (static_cast<uint32_t>(abs(stepDelta(commandedPosition - desiredPosition))) >
                    desiredPositionTolerance) {
                    output.commandType = StepperMotorCommandType::STOP;
                    state = StepperMotorState::STOPPING;
                }
                break;
            }

            case StepperMotorState::STOPPING:
                if (!isMotorMoving) {
                    state = StepperMotorState::SETTLING;
                    settleCount = 0;
                }
                break;

            case StepperMotorState::SETTLING:
                if (settleCount >= settleCountMax) {
                    state = StepperMotorState::IDLE;
                } else {
                    settleCount++;
                }
                break;

            case StepperMotorState::IDLE: {
                if (!isReferenceInRange) {
                    break;
                }
                const int steps = stepDelta(desiredPosition - currentPosition);
                if (static_cast<uint32_t>(abs(steps)) > currentPositionTolerance) {
                    output.commandType = StepperMotorCommandType::MOVE;
                    output.stepsToMove = steps;
                    commandedPosition = desiredPosition;
                    state = StepperMotorState::MOVING;
                }
                break;
            }
        }
        return output;
    }
};

// ---------------------------------------------------------------------------
// Regression test helper: multi-step comparison against reference
// ---------------------------------------------------------------------------

inline void regressionTestMultiStep(float stepAngle,
                                    float minAngle,
                                    float maxAngle,
                                    float referenceAngle,
                                    float initialAngle,
                                    float controlFrequency,
                                    float motorFrequency,
                                    uint32_t settleCountMax,
                                    uint32_t currentPositionTolerance,
                                    uint32_t desiredPositionTolerance) {
    if (minAngle >= maxAngle) {
        return;  // skip invalid range (fuzzer may generate it)
    }
    const int stepsPerRevolution = static_cast<int>(round(2.0F * std::numbers::pi_v<float> / stepAngle));
    const int initialStep = angleToSteps(initialAngle, stepAngle);
    constexpr float kMinStepAngle = 2.0F * std::numbers::pi_v<float> / 100000.0F;
    const bool isFullCircle = ((maxAngle - minAngle) >= (2.0F * std::numbers::pi_v<float> - kMinStepAngle));

    // Setup algorithm
    StepperMotorControllerAlgorithm alg{};
    alg.setStepAngle(stepAngle);
    alg.setMotorAngleRange(minAngle, maxAngle);
    alg.setSettleCountMax(settleCountMax);
    alg.setCurrentPositionTolerance(currentPositionTolerance);
    alg.setDesiredPositionTolerance(desiredPositionTolerance);
    alg.reset();

    StepperMotorSim algSim{};
    algSim.controlFrequency = controlFrequency;
    algSim.motorFrequency = motorFrequency;
    algSim.reset(initialStep);

    // Setup reference
    StepperMotorControllerReference ref{};
    ref.stepsPerRevolution = stepsPerRevolution;
    ref.stepAngle = stepAngle;
    ref.settleCountMax = settleCountMax;
    ref.currentPositionTolerance = currentPositionTolerance;
    ref.desiredPositionTolerance = desiredPositionTolerance;
    ref.minAngle = minAngle;
    ref.maxAngle = maxAngle;
    ref.isFullCircle = isFullCircle;
    ref.reset();

    StepperMotorSim refSim{};
    refSim.controlFrequency = controlFrequency;
    refSim.motorFrequency = motorFrequency;
    refSim.reset(initialStep);

    // Run for enough ticks to complete a full cycle
    const int maxTicks = stepsPerRevolution + static_cast<int>(settleCountMax) + 20;
    for (int tick = 0; tick < maxTicks; ++tick) {
        const auto algOut = alg.update(algSim.currentPosition, referenceAngle, false);
        const auto refOut = ref.update(refSim.currentPosition, referenceAngle, false);

        EXPECT_EQ(algOut.commandType, refOut.commandType) << "Mismatch at tick " << tick;
        if (algOut.commandType == StepperMotorCommandType::MOVE) {
            EXPECT_EQ(algOut.stepsToMove, refOut.stepsToMove) << "Mismatch at tick " << tick;
        }

        algSim.applyOutput(algOut);
        refSim.applyOutput(refOut);
        algSim.advance();
        refSim.advance();
    }
}

// ---------------------------------------------------------------------------
// Property test helpers
// ---------------------------------------------------------------------------

inline void propertyOutputCommandTypeIsValid(float stepAngle,
                                             float minAngle,
                                             float maxAngle,
                                             float referenceAngle,
                                             float initialAngle,
                                             float controlFrequency,
                                             float motorFrequency,
                                             uint32_t settleCountMax,
                                             uint32_t currentPositionTolerance,
                                             uint32_t desiredPositionTolerance) {
    if (minAngle >= maxAngle) {
        return;
    }
    const int stepsPerRevolution = static_cast<int>(round(2.0F * std::numbers::pi_v<float> / stepAngle));
    StepperMotorControllerAlgorithm alg{};
    alg.setStepAngle(stepAngle);
    alg.setMotorAngleRange(minAngle, maxAngle);
    alg.setSettleCountMax(settleCountMax);
    alg.setCurrentPositionTolerance(currentPositionTolerance);
    alg.setDesiredPositionTolerance(desiredPositionTolerance);
    alg.reset();

    StepperMotorSim sim{};
    sim.controlFrequency = controlFrequency;
    sim.motorFrequency = motorFrequency;
    sim.reset(angleToSteps(initialAngle, stepAngle));

    const int maxTicks = stepsPerRevolution + static_cast<int>(settleCountMax) + 20;
    for (int tick = 0; tick < maxTicks; ++tick) {
        const auto out = alg.update(sim.currentPosition, referenceAngle, false);
        EXPECT_TRUE(out.commandType == StepperMotorCommandType::NONE ||
                    out.commandType == StepperMotorCommandType::STOP ||
                    out.commandType == StepperMotorCommandType::MOVE)
            << "Invalid command type at tick " << tick;
        sim.applyOutput(out);
        sim.advance();
    }
}

inline void propertyMoveStepsWithinHalfRevolution(float stepAngle,
                                                  float minAngle,
                                                  float maxAngle,
                                                  float referenceAngle,
                                                  float initialAngle,
                                                  uint32_t currentPositionTolerance) {
    if (minAngle >= maxAngle) {
        return;
    }
    const int stepsPerRevolution = static_cast<int>(round(2.0F * std::numbers::pi_v<float> / stepAngle));
    constexpr float kMinStepAngle = 2.0F * std::numbers::pi_v<float> / 100000.0F;
    const bool isFullCircle = ((maxAngle - minAngle) >= (2.0F * std::numbers::pi_v<float> - kMinStepAngle));
    // Half-revolution wrap-around shortest-path bound only holds for the full-circle configuration.
    if (!isFullCircle) {
        return;
    }
    StepperMotorControllerAlgorithm alg{};
    alg.setStepAngle(stepAngle);
    alg.setMotorAngleRange(minAngle, maxAngle);
    alg.setCurrentPositionTolerance(currentPositionTolerance);
    alg.reset();

    const int initialStep = angleToSteps(initialAngle, stepAngle);
    const auto out = alg.update(initialStep, referenceAngle, false);
    if (out.commandType == StepperMotorCommandType::MOVE) {
        EXPECT_LE(abs(out.stepsToMove), stepsPerRevolution / 2) << "MOVE steps exceed half revolution";
    }
}

inline void propertyMotorReachesTarget(float stepAngle,
                                       float minAngle,
                                       float maxAngle,
                                       float referenceAngle,
                                       float initialAngle,
                                       float controlFrequency,
                                       float motorFrequency,
                                       uint32_t settleCountMax,
                                       uint32_t currentPositionTolerance,
                                       uint32_t desiredPositionTolerance) {
    if (minAngle >= maxAngle) {
        return;
    }
    // Motor only reaches an in-range target; out-of-range references are rejected by design.
    if (referenceAngle < minAngle || referenceAngle > maxAngle) {
        return;
    }
    const int stepsPerRevolution = static_cast<int>(round(2.0F * std::numbers::pi_v<float> / stepAngle));
    StepperMotorControllerAlgorithm alg{};
    alg.setStepAngle(stepAngle);
    alg.setMotorAngleRange(minAngle, maxAngle);
    alg.setSettleCountMax(settleCountMax);
    alg.setCurrentPositionTolerance(currentPositionTolerance);
    alg.setDesiredPositionTolerance(desiredPositionTolerance);
    alg.reset();

    StepperMotorSim sim{};
    sim.controlFrequency = controlFrequency;
    sim.motorFrequency = motorFrequency;
    sim.reset(angleToSteps(initialAngle, stepAngle));

    // Run enough ticks for a full cycle to complete
    const int maxTicks = stepsPerRevolution + static_cast<int>(settleCountMax) + 20;
    bool sawMove = false;
    bool reachedIdle = false;
    for (int tick = 0; tick < maxTicks; ++tick) {
        const auto out = alg.update(sim.currentPosition, referenceAngle, false);
        if (out.commandType == StepperMotorCommandType::MOVE) {
            sawMove = true;
        }
        // After a MOVE was issued, the state should eventually return to IDLE with no further MOVE
        if (sawMove && out.commandType == StepperMotorCommandType::NONE) {
            reachedIdle = true;
        }
        sim.applyOutput(out);
        sim.advance();
    }

    // If a MOVE was issued, the motor should have eventually settled
    if (sawMove) {
        EXPECT_TRUE(reachedIdle) << "Motor never returned to IDLE after MOVE";
    }
}

#endif  // TEST_STEPPERMOTORCONTROLLER_H
