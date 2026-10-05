#include "vectorDutyCycleTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

// Each property below is the same function the unit tests drive; only the inputs differ. The gate carries the
// vector without arithmetic, so the vector domain exists to prove the cadence is indifferent to magnitude rather
// than to probe numerical accuracy -- hence the wide, deliberately unphysical range including infinities is
// unnecessary, but both signs and a span of several decades are covered.
//
// Component 0 of every input is overwritten with the separate non-zero watchedComponent domain: wherever an input
// component is zero, a held-off output is indistinguishable from a passed-through one, so the cadence would not be
// observable. See detail::makeFuzzCase.

// Cadence domains are bounded so a case runs a few hundred updates at most; the cadence logic is integer and
// exact, so wider counts buy no additional coverage.
FUZZ_TEST(VectorDutyCyclePropertyFuzz, propertyOutputIsInputOrZero)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-100.0F, 100.0F)).WithSize(3),  // [-] input vector
                 fuzztest::InRange(0.01F, 100.0F),                                    // [-] watched component
                 fuzztest::InRange(1U, 20U),                                          // [-] firing periods
                 fuzztest::InRange(0U, 50U),                                          // [-] settling periods
                 fuzztest::InRange(1U, 30U));                                         // [-] update count

FUZZ_TEST(VectorDutyCyclePropertyFuzz, propertyGateActsOnTheWholeVector)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-100.0F, 100.0F)).WithSize(3),  // [-] input vector
                 fuzztest::InRange(0.01F, 100.0F),                                    // [-] watched component
                 fuzztest::InRange(1U, 20U),                                          // [-] firing periods
                 fuzztest::InRange(0U, 50U),                                          // [-] settling periods
                 fuzztest::InRange(1U, 30U));                                         // [-] update count

// The duty ratio must come out exact over whole cycles, which is the property the free-running counter exists
// to guarantee. Cycles are capped so the longest cadence still runs in bounded time.
FUZZ_TEST(VectorDutyCyclePropertyFuzz, propertyFiringCountMatchesDutyRatio)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-100.0F, 100.0F)).WithSize(3),  // [-] input vector
                 fuzztest::InRange(0.01F, 100.0F),                                    // [-] watched component
                 fuzztest::InRange(1U, 20U),                                          // [-] firing periods
                 fuzztest::InRange(0U, 50U),                                          // [-] settling periods
                 fuzztest::InRange(1U, 8U));                                          // [-] whole cycles

FUZZ_TEST(VectorDutyCyclePropertyFuzz, propertyCadenceIsIndependentOfInput)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-100.0F, 100.0F)).WithSize(3),  // [-] input vector
                 fuzztest::InRange(0.01F, 100.0F),                                    // [-] watched component
                 fuzztest::InRange(1U, 20U),                                          // [-] firing periods
                 fuzztest::InRange(0U, 50U),                                          // [-] settling periods
                 fuzztest::InRange(1U, 30U));                                         // [-] update count

// reInitialize() must restore the phase whatever phase the counter had reached, so updatesBeforeRestart ranges
// across and beyond a full cycle.
FUZZ_TEST(VectorDutyCyclePropertyFuzz, propertyReInitializeRestartsCadence)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-100.0F, 100.0F)).WithSize(3),  // [-] input vector
                 fuzztest::InRange(0.01F, 100.0F),                                    // [-] watched component
                 fuzztest::InRange(1U, 20U),                                          // [-] firing periods
                 fuzztest::InRange(0U, 50U),                                          // [-] settling periods
                 fuzztest::InRange(1U, 20U),                                          // [-] update count
                 fuzztest::InRange(0U, 80U));                                         // [-] updates before restart

FUZZ_TEST(VectorDutyCycleRegressionFuzz, regressionFuzzVectorDutyCycle)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-100.0F, 100.0F)).WithSize(3),  // [-] input vector
                 fuzztest::InRange(0.01F, 100.0F),                                    // [-] watched component
                 fuzztest::InRange(1U, 20U),                                          // [-] firing periods
                 fuzztest::InRange(0U, 50U),                                          // [-] settling periods
                 fuzztest::InRange(1U, 20U));                                         // [-] update count
