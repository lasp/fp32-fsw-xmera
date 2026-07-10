#include "sunTrackErrorTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

// ---------------------------------------------------------------------------
// Regression fuzz: exercise the shared regression helper on the maneuver path (Sun-avoidance enabled)
// with realistic Sun geometry. For any attitudes and Sun positions away from a degeneracy or the
// discrete short/long-way decision boundary, the algorithm must agree with the independent reference and
// stay finite. (Near-boundary inputs are skipped in the helper -- see nearManeuverDecisionBoundary.)
// ---------------------------------------------------------------------------
FUZZ_TEST(SunTrackErrorFuzz, fuzzRegressionSunTrackError)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e1F, 1e1F),   // sigma_BN
                 xmera::fuzz::Vector3fInRange(-1e1F, 1e1F),   // sigma_RN
                 xmera::fuzz::Vector3fInRange(-1e1F, 1e1F),   // omega_RN_N
                 xmera::fuzz::Vector3fInRange(-1e1F, 1e1F),   // domega_RN_N
                 xmera::fuzz::Vector3dInRange(-2e11, 2e11),   // r_BN_N
                 xmera::fuzz::Vector3dInRange(-2e11, 2e11));  // r_SN_N
