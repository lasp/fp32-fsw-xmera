#include "dvManeuverTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

// ---------------------------------------------------------------------------
// Regression fuzz: for any finite configured force, commanded delta-V, acceleration, time limits and update
// rate, the burn state machine must agree with the independent reference implementation across every step.
// ---------------------------------------------------------------------------
FUZZ_TEST(DvManeuverFuzz, fuzzRegressionDvManeuver)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e3F, 1e3F),             // cmdForce_B [N]
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),             // cmdDv_N [m/s]
                 xmera::fuzz::Vector3fInRange(-1e3F, 1e3F),             // acceleration [m/s^2]
                 fuzztest::InRange<uint64_t>(0U, 60000000000U),         // minTime [ns]
                 fuzztest::InRange<uint64_t>(1U, 120000000000U),        // maxTime - minTime [ns]
                 fuzztest::InRange<uint64_t>(10000000U, 1000000000U));  // step period [ns], 1 to 100 Hz

// ---------------------------------------------------------------------------
// Property fuzz: the burn state and force command are well-formed on every step for any finite inputs, time
// limits and update rate.
// ---------------------------------------------------------------------------
FUZZ_TEST(DvManeuverPropertyFuzz, propertyOutputWellFormed)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e3F, 1e3F),             // cmdForce_B [N]
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),             // cmdDv_N [m/s]
                 xmera::fuzz::Vector3fInRange(-1e3F, 1e3F),             // acceleration [m/s^2]
                 fuzztest::InRange<uint64_t>(0U, 60000000000U),         // minTime [ns]
                 fuzztest::InRange<uint64_t>(1U, 120000000000U),        // maxTime - minTime [ns]
                 fuzztest::InRange<uint64_t>(10000000U, 1000000000U));  // step period [ns], 1 to 100 Hz
