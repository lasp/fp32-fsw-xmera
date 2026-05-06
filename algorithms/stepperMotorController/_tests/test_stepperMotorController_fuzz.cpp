#include "stepperMotorControllerTestHelpers.hpp"
#include <fuzztest/fuzztest.h>
#include <numbers>

// stepAngle range: (2*pi)/100000 ... (2*pi)/10  -> roughly 6.28e-5 ... 0.628 rad/step.
// Upper bound on stepsPerRev kept at 100k so the fp32 round-trip stays within rounding tolerance
// (fp32 error grows as O(N^2 * eps); at N=100k the expected drift is ~0.19 steps, safely within
// the half-step round() margin).
constexpr float kStepAngleMin = 2.0F * std::numbers::pi_v<float> / 100000.0F;
constexpr float kStepAngleMax = 2.0F * std::numbers::pi_v<float> / 10.0F;

// Motor angle range domain: full [-2*pi, 2*pi]. The helpers early-skip when minAngle >= maxAngle,
// so the fuzzer naturally explores both valid full/partial ranges and skips invalid pairs.
constexpr float kAngleRangeMin = -2.0F * std::numbers::pi_v<float>;
constexpr float kAngleRangeMax = 2.0F * std::numbers::pi_v<float>;

// ---------------------------------------------------------------------------
// Regression fuzz: compare algorithm against reference for random parameters
// ---------------------------------------------------------------------------

FUZZ_TEST(StepperMotorControllerFuzz, regressionTestMultiStep)
    .WithDomains(fuzztest::InRange(kStepAngleMin, kStepAngleMax),    // stepAngle
                 fuzztest::InRange(kAngleRangeMin, kAngleRangeMax),  // minAngle
                 fuzztest::InRange(kAngleRangeMin, kAngleRangeMax),  // maxAngle
                 fuzztest::InRange(-100.0F, 100.0F),                 // referenceAngle
                 fuzztest::InRange(-100.0F, 100.0F),                 // initialAngle
                 fuzztest::InRange(1.0F, 100.0F),                    // controlFrequency
                 fuzztest::InRange(1.0F, 100.0F),                    // motorFrequency
                 fuzztest::InRange(0, 20),                           // settleCountMax
                 fuzztest::InRange(0, 10),                           // currentPositionTolerance
                 fuzztest::InRange(0, 10));                          // desiredPositionTolerance

// ---------------------------------------------------------------------------
// Property fuzz tests
// ---------------------------------------------------------------------------

FUZZ_TEST(StepperMotorControllerPropertyFuzz, propertyOutputCommandTypeIsValid)
    .WithDomains(fuzztest::InRange(kStepAngleMin, kStepAngleMax),    // stepAngle
                 fuzztest::InRange(kAngleRangeMin, kAngleRangeMax),  // minAngle
                 fuzztest::InRange(kAngleRangeMin, kAngleRangeMax),  // maxAngle
                 fuzztest::InRange(-100.0F, 100.0F),                 // referenceAngle
                 fuzztest::InRange(-100.0F, 100.0F),                 // initialAngle
                 fuzztest::InRange(1.0F, 100.0F),                    // controlFrequency
                 fuzztest::InRange(1.0F, 100.0F),                    // motorFrequency
                 fuzztest::InRange(0, 20),                           // settleCountMax
                 fuzztest::InRange(0, 10),                           // currentPositionTolerance
                 fuzztest::InRange(0, 10));                          // desiredPositionTolerance

FUZZ_TEST(StepperMotorControllerPropertyFuzz, propertyMoveStepsWithinHalfRevolution)
    .WithDomains(fuzztest::InRange(kStepAngleMin, kStepAngleMax),    // stepAngle
                 fuzztest::InRange(kAngleRangeMin, kAngleRangeMax),  // minAngle
                 fuzztest::InRange(kAngleRangeMin, kAngleRangeMax),  // maxAngle
                 fuzztest::InRange(-100.0F, 100.0F),                 // referenceAngle
                 fuzztest::InRange(-100.0F, 100.0F),                 // initialAngle
                 fuzztest::InRange(0, 10));                          // currentPositionTolerance

FUZZ_TEST(StepperMotorControllerPropertyFuzz, propertyMotorReachesTarget)
    .WithDomains(fuzztest::InRange(kStepAngleMin, kStepAngleMax),    // stepAngle
                 fuzztest::InRange(kAngleRangeMin, kAngleRangeMax),  // minAngle
                 fuzztest::InRange(kAngleRangeMin, kAngleRangeMax),  // maxAngle
                 fuzztest::InRange(-100.0F, 100.0F),                 // referenceAngle
                 fuzztest::InRange(-100.0F, 100.0F),                 // initialAngle
                 fuzztest::InRange(1.0F, 100.0F),                    // controlFrequency
                 fuzztest::InRange(1.0F, 100.0F),                    // motorFrequency
                 fuzztest::InRange(0, 20),                           // settleCountMax
                 fuzztest::InRange(0, 10),                           // currentPositionTolerance
                 fuzztest::InRange(0, 10));                          // desiredPositionTolerance
