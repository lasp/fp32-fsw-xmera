#include "dvExecuteGuidanceTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

// ---------------------------------------------------------------------------
// Regression fuzz: for any finite commanded delta-V and acceleration, the burn state machine must
// agree with the independent reference implementation across every step.
// ---------------------------------------------------------------------------
FUZZ_TEST(DvExecuteGuidanceFuzz, fuzzRegressionDvExecuteGuidance)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),   // dvInrtlCmd [m/s]
                 xmera::fuzz::Vector3fInRange(-1e3F, 1e3F));  // acceleration [m/s^2]

// ---------------------------------------------------------------------------
// Property fuzz: the output flags are well-formed on every step for any finite inputs.
// ---------------------------------------------------------------------------
FUZZ_TEST(DvExecuteGuidancePropertyFuzz, propertyOutputFlagsWellFormed)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),   // dvInrtlCmd [m/s]
                 xmera::fuzz::Vector3fInRange(-1e3F, 1e3F));  // acceleration [m/s^2]
