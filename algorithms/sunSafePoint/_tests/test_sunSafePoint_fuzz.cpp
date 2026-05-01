#include "sunSafePointTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

FUZZ_TEST(SunSafePointAlgorithmFuzz, regressionTestSunSafePoint)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-10.0F, 10.0F)).WithSize(3),  // sunVec
                 fuzztest::VectorOf(fuzztest::InRange(-5.0F, 5.0F)).WithSize(3),    // omega_BN_B
                 fuzztest::InRange(-5.0F, 5.0F),                                    // sunAxisSpinRate
                 fuzztest::VectorOf(fuzztest::InRange(-1.0F, 1.0F)).WithSize(3),    // sHatBdyCmd
                 fuzztest::VectorOf(fuzztest::InRange(-5.0F, 5.0F)).WithSize(3));   // omega_RN_B_cfg

// ---------------------------------------------------------------------------
// Property fuzz tests
// ---------------------------------------------------------------------------

FUZZ_TEST(SunSafePointPropertyFuzz, propertySigmaBrNormBounded)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-10.0F, 10.0F)).WithSize(3));

FUZZ_TEST(SunSafePointPropertyFuzz, propertyOmegaBrIdentity)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-10.0F, 10.0F)).WithSize(3),
                 fuzztest::VectorOf(fuzztest::InRange(-5.0F, 5.0F)).WithSize(3));

FUZZ_TEST(SunSafePointPropertyFuzz, propertyOutputIsFinite)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-10.0F, 10.0F)).WithSize(3));
