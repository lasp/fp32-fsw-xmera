#include "dvManeuverTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

// ---------------------------------------------------------------------------
// Regression fuzz: for any finite configured force, commanded delta-V and acceleration, the burn
// state machine must agree with the independent reference implementation across every step.
// ---------------------------------------------------------------------------
FUZZ_TEST(DvManeuverFuzz, fuzzRegressionDvManeuver)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e3F, 1e3F),   // cmdForce_B [N]
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),   // cmdDv_N [m/s]
                 xmera::fuzz::Vector3fInRange(-1e3F, 1e3F));  // acceleration [m/s^2]

// ---------------------------------------------------------------------------
// Property fuzz: the burn state and force command are well-formed on every step for any finite inputs.
// ---------------------------------------------------------------------------
FUZZ_TEST(DvManeuverPropertyFuzz, propertyOutputWellFormed)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e3F, 1e3F),   // cmdForce_B [N]
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),   // cmdDv_N [m/s]
                 xmera::fuzz::Vector3fInRange(-1e3F, 1e3F));  // acceleration [m/s^2]
