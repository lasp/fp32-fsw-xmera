#include "thrustVectoringTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

// ---------------------------------------------------------------------------
// Property fuzz tests
// ---------------------------------------------------------------------------

FUZZ_TEST(ThrustVectoringPropertyFuzz, propertyOutputsFinite)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),    // sigma_MB (MRP)
                 xmera::fuzz::Vector3fInRange(-10.0F, 10.0F),  // r_MB_B
                 xmera::fuzz::Vector3fInRange(-10.0F, 10.0F),  // r_FM_F
                 xmera::fuzz::Vector3fInRange(-10.0F, 10.0F),  // r_CB_B
                 xmera::fuzz::Vector3fInRange(-10.0F, 10.0F),  // rThrust_F
                 xmera::fuzz::Vector3fInRange(-10.0F, 10.0F),  // tHatThrust_F (normalized in helper)
                 fuzztest::InRange(0.0F, 100.0F),              // maxThrust (config requires a positive thrust)
                 xmera::fuzz::Vector3fInRange(-5.0F, 5.0F));   // Lreq_B
