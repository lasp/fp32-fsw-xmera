#include "solarArrayReferenceTestHelpers.hpp"
#include <fuzztest/fuzztest.h>
#include <numbers>

FUZZ_TEST(SolarArrayReferenceFuzz, regressionTestSolarArrayReference)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-10.0F, 10.0F)).WithSize(3),  // sigma_BN
                 fuzztest::VectorOf(fuzztest::InRange(-10.0F, 10.0F)).WithSize(3),  // sigma_RN
                 fuzztest::VectorOf(fuzztest::InRange(-10.0F, 10.0F)).WithSize(3),  // vehSunPntBdy
                 fuzztest::VectorOf(fuzztest::InRange(-1.0F, 1.0F)).WithSize(3),    // a1Hat_B
                 fuzztest::VectorOf(fuzztest::InRange(-1.0F, 1.0F)).WithSize(3),    // a2Hat_B
                 fuzztest::InRange(1e-3F, std::numbers::pi_v<float> / 2.0F),        // alignmentThreshold
                 fuzztest::InRange(-100.0F, 100.0F));                               // theta

// ---------------------------------------------------------------------------
// Property fuzz tests
// ---------------------------------------------------------------------------

FUZZ_TEST(SolarArrayReferencePropertyFuzz, propertyOutputIsFinite)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-1.0F, 1.0F)).WithSize(3),    // sigma_BN
                 fuzztest::VectorOf(fuzztest::InRange(-1.0F, 1.0F)).WithSize(3),    // sigma_RN
                 fuzztest::VectorOf(fuzztest::InRange(-10.0F, 10.0F)).WithSize(3),  // vehSunPntBdy
                 fuzztest::InRange(1e-3F, std::numbers::pi_v<float> / 2.0F),        // alignmentThreshold
                 fuzztest::InRange(-100.0F, 100.0F));                               // theta

FUZZ_TEST(SolarArrayReferencePropertyFuzz, propertySpecifiedAngleReturnsAngle)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-1.0F, 1.0F)).WithSize(3),    // sigma_BN
                 fuzztest::VectorOf(fuzztest::InRange(-1.0F, 1.0F)).WithSize(3),    // sigma_RN
                 fuzztest::VectorOf(fuzztest::InRange(-10.0F, 10.0F)).WithSize(3),  // vehSunPntBdy
                 fuzztest::InRange(-100.0F, 100.0F),                                // specifiedAngle
                 fuzztest::InRange(-100.0F, 100.0F));                               // theta

FUZZ_TEST(SolarArrayReferencePropertyFuzz, propertyAlignedSunReturnsCurrentTheta)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-1.0F, 1.0F)).WithSize(3),  // a1Hat_B
                 fuzztest::InRange(1e-3F, std::numbers::pi_v<float> / 2.0F),      // alignmentThreshold
                 fuzztest::InRange(-100.0F, 100.0F));                             // theta
