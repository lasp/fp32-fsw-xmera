#include "architecture/testUtilities/eigenFuzzDomains.hpp"
#include "attTrackingErrorTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

FUZZ_TEST(attTrackingErrorAlgorithmFuzz, regressionTestAttTrackingError)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0f, 1.0f),     // sigma_BN
                 xmera::fuzz::Vector3fInRange(-10.0f, 10.0f),   // omega_BN_B
                 xmera::fuzz::Vector3fInRange(-1.0f, 1.0f),     // sigma_RN
                 xmera::fuzz::Vector3fInRange(-10.0f, 10.0f),   // omega_RN_N
                 xmera::fuzz::Vector3fInRange(-10.0f, 10.0f));  // domega_RN_N

// ------------------------------------------------------
// Property fuzz test
// ------------------------------------------------------

FUZZ_TEST(attTrackingErrorAlgorithmFuzz, zeroPropertyTest).WithDomains(xmera::fuzz::Vector3fInRange(-1.0f, 1.0f));

FUZZ_TEST(attTrackingErrorAlgorithmFuzz, identityPropertyTest)
    .WithDomains(xmera::fuzz::Vector3fInRange(-10.0f, 10.0f), xmera::fuzz::Vector3fInRange(-10.0f, 10.0f));

FUZZ_TEST(attTrackingErrorAlgorithmFuzz, finitenessPropertyTest)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0f, 1.0f),     // sigma_BN
                 xmera::fuzz::Vector3fInRange(-10.0f, 10.0f),   // omega_BN_B
                 xmera::fuzz::Vector3fInRange(-1.0f, 1.0f),     // sigma_RN
                 xmera::fuzz::Vector3fInRange(-10.0f, 10.0f),   // omega_RN_N
                 xmera::fuzz::Vector3fInRange(-10.0f, 10.0f));  // domega_RN_N
