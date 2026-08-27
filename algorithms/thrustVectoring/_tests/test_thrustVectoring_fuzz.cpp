#include "thrustVectoringTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

// ---------------------------------------------------------------------------
// Property fuzz tests
// ---------------------------------------------------------------------------

// The ranges reach the values the configuration rejects -- a zero thrust, and a center of mass that lands on the
// joint when both position domains include the origin -- so the reject path is fuzzed alongside the solve.
FUZZ_TEST(ThrustVectoringPropertyFuzz, propertyOutputsFinite)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),    // sigma_MB (MRP)
                 xmera::fuzz::Vector3fInRange(-10.0F, 10.0F),  // r_MB_B
                 xmera::fuzz::Vector3fInRange(-10.0F, 10.0F),  // r_CB_B
                 fuzztest::InRange(0.0F, 10.0F),               // armLength
                 fuzztest::InRange(0.0F, 100.0F),              // thrust (config requires a positive thrust)
                 xmera::fuzz::Vector3fInRange(-5.0F, 5.0F));   // Lreq_B
