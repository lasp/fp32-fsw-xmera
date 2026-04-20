#include "stepperMotorControllerTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

// ---------------------------------------------------------------------------
// Regression fuzz: compare algorithm against reference for random parameters
// ---------------------------------------------------------------------------

FUZZ_TEST(StepperMotorControllerFuzz, regressionTestMultiStep)
    .WithDomains(fuzztest::InRange(10, 1000000),      // stepsPerRevolution
                 fuzztest::InRange(-100.0F, 100.0F),  // referenceAngle
                 fuzztest::InRange(-100.0F, 100.0F),  // initialAngle
                 fuzztest::InRange(1.0F, 100.0F),     // controlFrequency
                 fuzztest::InRange(1.0F, 100.0F),     // motorFrequency
                 fuzztest::InRange(0, 20),            // settleCountMax
                 fuzztest::InRange(0, 10),            // currentPositionTolerance
                 fuzztest::InRange(0, 10));           // desiredPositionTolerance

// ---------------------------------------------------------------------------
// Property fuzz tests
// ---------------------------------------------------------------------------

FUZZ_TEST(StepperMotorControllerPropertyFuzz, propertyOutputCommandTypeIsValid)
    .WithDomains(fuzztest::InRange(10, 1000000),      // stepsPerRevolution
                 fuzztest::InRange(-100.0F, 100.0F),  // referenceAngle
                 fuzztest::InRange(-100.0F, 100.0F),  // initialAngle
                 fuzztest::InRange(1.0F, 100.0F),     // controlFrequency
                 fuzztest::InRange(1.0F, 100.0F),     // motorFrequency
                 fuzztest::InRange(0, 20),            // settleCountMax
                 fuzztest::InRange(0, 10),            // currentPositionTolerance
                 fuzztest::InRange(0, 10));           // desiredPositionTolerance

FUZZ_TEST(StepperMotorControllerPropertyFuzz, propertyMoveStepsWithinHalfRevolution)
    .WithDomains(fuzztest::InRange(10, 1000000),      // stepsPerRevolution
                 fuzztest::InRange(-100.0F, 100.0F),  // referenceAngle
                 fuzztest::InRange(-100.0F, 100.0F),  // initialAngle
                 fuzztest::InRange(0, 10));           // currentPositionTolerance

FUZZ_TEST(StepperMotorControllerPropertyFuzz, propertyMotorReachesTarget)
    .WithDomains(fuzztest::InRange(10, 1000000),      // stepsPerRevolution
                 fuzztest::InRange(-100.0F, 100.0F),  // referenceAngle
                 fuzztest::InRange(-100.0F, 100.0F),  // initialAngle
                 fuzztest::InRange(1.0F, 100.0F),     // controlFrequency
                 fuzztest::InRange(1.0F, 100.0F),     // motorFrequency
                 fuzztest::InRange(0, 20),            // settleCountMax
                 fuzztest::InRange(0, 10),            // currentPositionTolerance
                 fuzztest::InRange(0, 10));           // desiredPositionTolerance
