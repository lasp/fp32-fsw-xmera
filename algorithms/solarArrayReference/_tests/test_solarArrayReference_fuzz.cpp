#include "solarArrayReferenceTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

FUZZ_TEST(SolarArrayReferenceFuzz, regressionTestSolarArrayReference)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-10.0F, 10.0F)).WithSize(3),  // sigma_BN
                 fuzztest::VectorOf(fuzztest::InRange(-10.0F, 10.0F)).WithSize(3),  // sigma_RN
                 fuzztest::VectorOf(fuzztest::InRange(-10.0F, 10.0F)).WithSize(3),  // vehSunPntBdy
                 fuzztest::VectorOf(fuzztest::InRange(-1.0F, 1.0F)).WithSize(3),    // a1Hat_B
                 fuzztest::VectorOf(fuzztest::InRange(-1.0F, 1.0F)).WithSize(3),    // a2Hat_B
                 fuzztest::InRange(-100.0F, 100.0F));                               // theta
