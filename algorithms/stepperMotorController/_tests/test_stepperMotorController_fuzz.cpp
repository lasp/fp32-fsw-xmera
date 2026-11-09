#include "stepperMotorControllerTestHelpers.hpp"
#include <fuzztest/fuzztest.h>
#include <numbers>

// stepsPerRevolution range: 1 ... 100000 steps (roughly 6.28 ... 6.28e-5 rad/step), the full range the config
// accepts. Upper bound kept at 100k so the fp32 round-trip stays within rounding tolerance
// (fp32 error grows as O(N^2 * eps); at N=100k the expected drift is ~0.19 steps, safely within
// the half-step round() margin).
constexpr uint32_t kStepsPerRevolutionMin = 1U;
constexpr uint32_t kStepsPerRevolutionMax = 100000U;

// Motor angle range domain: full [-2*pi, 2*pi]. The helpers early-skip when minAngle >= maxAngle,
// so the fuzzer naturally explores both valid full/partial ranges and skips invalid pairs.
constexpr float kAngleRangeMin = -2.0F * std::numbers::pi_v<float>;
constexpr float kAngleRangeMax = 2.0F * std::numbers::pi_v<float>;

// minStepCommand domain spans 1..10000 steps. The helpers early-skip combinations where
// minStepCommand * stepAngle (stepAngle = 2*pi / stepsPerRevolution) does not fit between stepAngle and the
// configured angular range (i.e. cases that could never produce a meaningful MOVE command).

// ---------------------------------------------------------------------------
// Regression fuzz: compare algorithm against reference for random parameters
// ---------------------------------------------------------------------------

FUZZ_TEST(StepperMotorControllerFuzz, regressionTestMultiStep)
    .WithDomains(fuzztest::InRange(kStepsPerRevolutionMin, kStepsPerRevolutionMax),  // stepsPerRevolution
                 fuzztest::InRange(kAngleRangeMin, kAngleRangeMax),                  // minAngle
                 fuzztest::InRange(kAngleRangeMin, kAngleRangeMax),                  // maxAngle
                 fuzztest::InRange(-100.0F, 100.0F),                                 // referenceAngle
                 fuzztest::InRange(-100.0F, 100.0F),                                 // initialAngle
                 fuzztest::InRange(1.0F, 100.0F),                                    // controlFrequency
                 fuzztest::InRange(1.0F, 100.0F),                                    // motorFrequency
                 fuzztest::InRange(0, 20),                                           // settleCountMax
                 fuzztest::InRange(1, 10000));                                       // minStepCommand

// ---------------------------------------------------------------------------
// Property fuzz tests
// ---------------------------------------------------------------------------

FUZZ_TEST(StepperMotorControllerPropertyFuzz, propertyOutputCommandTypeIsValid)
    .WithDomains(fuzztest::InRange(kStepsPerRevolutionMin, kStepsPerRevolutionMax),  // stepsPerRevolution
                 fuzztest::InRange(kAngleRangeMin, kAngleRangeMax),                  // minAngle
                 fuzztest::InRange(kAngleRangeMin, kAngleRangeMax),                  // maxAngle
                 fuzztest::InRange(-100.0F, 100.0F),                                 // referenceAngle
                 fuzztest::InRange(-100.0F, 100.0F),                                 // initialAngle
                 fuzztest::InRange(1.0F, 100.0F),                                    // controlFrequency
                 fuzztest::InRange(1.0F, 100.0F),                                    // motorFrequency
                 fuzztest::InRange(0, 20),                                           // settleCountMax
                 fuzztest::InRange(1, 10000));                                       // minStepCommand

FUZZ_TEST(StepperMotorControllerPropertyFuzz, propertyMoveStepsWithinHalfRevolution)
    .WithDomains(fuzztest::InRange(kStepsPerRevolutionMin, kStepsPerRevolutionMax),  // stepsPerRevolution
                 fuzztest::InRange(kAngleRangeMin, kAngleRangeMax),                  // minAngle
                 fuzztest::InRange(kAngleRangeMin, kAngleRangeMax),                  // maxAngle
                 fuzztest::InRange(-100.0F, 100.0F),                                 // referenceAngle
                 fuzztest::InRange(-100.0F, 100.0F),                                 // initialAngle
                 fuzztest::InRange(1, 10000));                                       // minStepCommand

FUZZ_TEST(StepperMotorControllerPropertyFuzz, propertyMotorReachesTarget)
    .WithDomains(fuzztest::InRange(kStepsPerRevolutionMin, kStepsPerRevolutionMax),  // stepsPerRevolution
                 fuzztest::InRange(kAngleRangeMin, kAngleRangeMax),                  // minAngle
                 fuzztest::InRange(kAngleRangeMin, kAngleRangeMax),                  // maxAngle
                 fuzztest::InRange(-100.0F, 100.0F),                                 // referenceAngle
                 fuzztest::InRange(-100.0F, 100.0F),                                 // initialAngle
                 fuzztest::InRange(1.0F, 100.0F),                                    // controlFrequency
                 fuzztest::InRange(1.0F, 100.0F),                                    // motorFrequency
                 fuzztest::InRange(0, 20),                                           // settleCountMax
                 fuzztest::InRange(1, 10000));                                       // minStepCommand
