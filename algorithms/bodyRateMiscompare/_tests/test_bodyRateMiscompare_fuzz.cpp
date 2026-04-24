#include "architecture/testUtilities/eigenFuzzDomains.hpp"
#include "bodyRateMiscompareTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

FUZZ_TEST(BodyRateMiscompareAlgorithmFuzz, runRegressionCase)
    .WithDomains(fuzztest::InRange(1e-6F, 1e6F),
                 fuzztest::InRange(1U, 10U),
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),
                 fuzztest::Arbitrary<bool>());

// ---------------------------------------------------------------------------
// Property fuzz tests
// ---------------------------------------------------------------------------

FUZZ_TEST(BodyRateMiscomparePropertyFuzz, propertyOutputIsOneOfInputs)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F), xmera::fuzz::Vector3fInRange(-1e6F, 1e6F));

FUZZ_TEST(BodyRateMiscomparePropertyFuzz, propertyFaultFlagMatchesSource)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F), xmera::fuzz::Vector3fInRange(-1e6F, 1e6F));

FUZZ_TEST(BodyRateMiscomparePropertyFuzz, propertyIdenticalInputsNeverFault)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F));

FUZZ_TEST(BodyRateMiscomparePropertyFuzz, propertyFaultIsSticky)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F), xmera::fuzz::Vector3fInRange(-1e6F, 1e6F));

FUZZ_TEST(BodyRateMiscomparePropertyFuzz, propertyUseImuRatesAlwaysOutputsImu)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F), xmera::fuzz::Vector3fInRange(-1e6F, 1e6F));

FUZZ_TEST(BodyRateMiscomparePropertyFuzz, propertyOutputIsFinite)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F), xmera::fuzz::Vector3fInRange(-1e6F, 1e6F));
