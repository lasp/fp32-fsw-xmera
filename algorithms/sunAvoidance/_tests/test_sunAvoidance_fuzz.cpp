#include "sunAvoidanceTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

// ---------------------------------------------------------------------------
// Regression fuzz: exercise the shared regression helper on the maneuver path (Sun-avoidance enabled)
// with arbitrary Sun directions. For any attitudes and Sun directions away from a degeneracy or the
// discrete short/long-way decision boundary, the algorithm must agree with the independent reference and
// stay finite. (Near-boundary inputs are skipped in the helper -- see nearManeuverDecisionBoundary.)
// ---------------------------------------------------------------------------
FUZZ_TEST(SunAvoidanceFuzz, fuzzRegressionSunAvoidance)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e1F, 1e1F),   // sigma_BN
                 xmera::fuzz::Vector3fInRange(-1e1F, 1e1F),   // sigma_RN
                 xmera::fuzz::Vector3fInRange(-1e1F, 1e1F),   // omega_RN_N
                 xmera::fuzz::Vector3fInRange(-1e1F, 1e1F),   // domega_RN_N
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F));  // sHat_B
