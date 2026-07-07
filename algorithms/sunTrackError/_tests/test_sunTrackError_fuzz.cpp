#include "sunTrackErrorTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

// ---------------------------------------------------------------------------
// Regression fuzz: exercise the shared regression helper on the pass-through path (maneuver
// disabled). For any finite navigation attitude and reference frame the algorithm must agree with
// the independent reference. (The maneuver path is not fuzz-regressed -- see the note on
// fuzzRegressionSunTrackError in the test helpers.)
// ---------------------------------------------------------------------------
FUZZ_TEST(SunTrackErrorFuzz, fuzzRegressionSunTrackError)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e1F, 1e1F),   // sigma_BN (unused on the pass-through path)
                 xmera::fuzz::Vector3fInRange(-1e1F, 1e1F),   // sigma_RN
                 xmera::fuzz::Vector3fInRange(-1e1F, 1e1F),   // omega_RN_N
                 xmera::fuzz::Vector3fInRange(-1e1F, 1e1F));  // domega_RN_N
