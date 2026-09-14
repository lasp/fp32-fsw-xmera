#include "thrustVectoringTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

// ---------------------------------------------------------------------------
// Regression fuzz tests
// ---------------------------------------------------------------------------

// The geometry ranges cover a thruster anywhere on a ten metre vehicle, and reach the values the configuration
// rejects. The request range reaches beyond thrust * |r_MC| over most of the geometry range, so the saturation
// branch is fuzzed alongside the requests the geometry can deliver in full.
FUZZ_TEST(ThrustVectoringFuzz, regressionCaseThrustVectoring)
    .WithDomains(xmera::fuzz::Vector3fInRange(-10.0F, 10.0F),     // r_MB_B
                 xmera::fuzz::Vector3fInRange(-10.0F, 10.0F),     // r_CB_B
                 fuzztest::InRange(0.0F, 100.0F),                 // thrust (config requires a positive thrust)
                 xmera::fuzz::Vector3fInRange(-2.0e3F, 2.0e3F));  // Lreq_B

// ---------------------------------------------------------------------------
// Property fuzz tests
// ---------------------------------------------------------------------------

// The ranges reach the values the configuration rejects -- a zero thrust, and a center of mass that lands on the
// joint when both position domains include the origin -- so the reject path is fuzzed alongside the solve.
FUZZ_TEST(ThrustVectoringPropertyFuzz, propertyOutputsFinite)
    .WithDomains(xmera::fuzz::Vector3fInRange(-10.0F, 10.0F),  // r_MB_B
                 xmera::fuzz::Vector3fInRange(-10.0F, 10.0F),  // r_CB_B
                 fuzztest::InRange(0.0F, 100.0F),              // thrust (config requires a positive thrust)
                 xmera::fuzz::Vector3fInRange(-5.0F, 5.0F));   // Lreq_B
