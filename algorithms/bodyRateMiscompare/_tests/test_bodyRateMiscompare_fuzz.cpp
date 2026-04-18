#include "bodyRateMiscompareTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

FUZZ_TEST(BodyRateMiscompareAlgorithmFuzz, runRegressionCase)
    .WithDomains(fuzztest::InRange(1e-6F, 1e6F),
                 fuzztest::InRange(1U, 10U),
                 fuzztest::ArrayOf<3>(fuzztest::InRange(-1e6F, 1e6F)),
                 fuzztest::ArrayOf<3>(fuzztest::InRange(-1e6F, 1e6F)),
                 fuzztest::Arbitrary<bool>());

// ---------------------------------------------------------------------------
// Property fuzz tests
// ---------------------------------------------------------------------------

FUZZ_TEST(BodyRateMiscomparePropertyFuzz, propertyOutputIsOneOfInputs)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3),
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3));

FUZZ_TEST(BodyRateMiscomparePropertyFuzz, propertyFaultFlagMatchesSource)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3),
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3));

FUZZ_TEST(BodyRateMiscomparePropertyFuzz, propertyIdenticalInputsNeverFault)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3));

FUZZ_TEST(BodyRateMiscomparePropertyFuzz, propertyFaultIsSticky)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3),
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3));

FUZZ_TEST(BodyRateMiscomparePropertyFuzz, propertyUseImuRatesAlwaysOutputsImu)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3),
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3));

FUZZ_TEST(BodyRateMiscomparePropertyFuzz, propertyOutputIsFinite)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3),
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3));
