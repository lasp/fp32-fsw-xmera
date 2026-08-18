#include "thrMomentumManagementTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

// ---------------------------------------------------------------------------
// Property fuzz tests
// ---------------------------------------------------------------------------

FUZZ_TEST(ThrMomentumManagementPropertyFuzz, propertyTorqueOpposesExcessMomentum)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),      // spin axis 0 (normalized in helper)
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),      // spin axis 1 (normalized in helper)
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),      // spin axis 2 (normalized in helper)
                 xmera::fuzz::Vector3fInRange(-600.0F, 600.0F),  // [r/s] per-wheel speeds
                 fuzztest::InRange(1e-3F, 0.2F),                 // [kgm2] common spin-axis inertia
                 fuzztest::InRange(0.0F, 100.0F),                // [Nms] dumping threshold
                 fuzztest::InRange(1e-3F, 0.2F));                // [1/s] feedback gain

// ---------------------------------------------------------------------------
// Regression fuzz tests
// ---------------------------------------------------------------------------

FUZZ_TEST(ThrMomentumManagementRegressionFuzz, regressionFuzzThrMomentumManagement)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),      // spin axis 0 (normalized in helper)
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),      // spin axis 1 (normalized in helper)
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),      // spin axis 2 (normalized in helper)
                 xmera::fuzz::Vector3fInRange(-600.0F, 600.0F),  // [r/s] per-wheel speeds
                 fuzztest::InRange(1e-3F, 0.2F),                 // [kgm2] common spin-axis inertia
                 fuzztest::InRange(0.0F, 100.0F),                // [Nms] dumping threshold
                 fuzztest::InRange(1e-3F, 0.2F));                // [1/s] feedback gain
