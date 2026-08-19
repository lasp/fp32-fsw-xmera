#include "thrMomentumManagementTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

// Each property below is the same function the unit tests drive; only the inputs differ. Domains stay within
// realizable hardware, except propertyTorqueStaysFinite which is about robustness, not accuracy. The gain
// reaches 1e-9 because the config only requires K > 0, and that is what proves the tolerance is scale-free.

FUZZ_TEST(ThrMomentumManagementPropertyFuzz, propertyProportionalTorqueOpposesExcessMomentum)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),      // spin axis 0 (normalized in helper)
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),      // spin axis 1 (normalized in helper)
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),      // spin axis 2 (normalized in helper)
                 xmera::fuzz::Vector3fInRange(-600.0F, 600.0F),  // [r/s] per-wheel speeds
                 fuzztest::InRange(1e-3F, 0.2F),                 // [kgm2] common spin-axis inertia
                 fuzztest::InRange(0.0F, 100.0F),                // [Nms] dumping threshold
                 fuzztest::InRange(1e-9F, 0.2F));                // [1/s] proportional gain

FUZZ_TEST(ThrMomentumManagementPropertyFuzz, propertyTorqueIsOddInWheelSpeeds)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),      // spin axis 0 (normalized in helper)
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),      // spin axis 1 (normalized in helper)
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),      // spin axis 2 (normalized in helper)
                 xmera::fuzz::Vector3fInRange(-600.0F, 600.0F),  // [r/s] per-wheel speeds
                 fuzztest::InRange(1e-3F, 0.2F),                 // [kgm2] common spin-axis inertia
                 fuzztest::InRange(0.0F, 100.0F),                // [Nms] dumping threshold
                 fuzztest::InRange(1e-9F, 0.2F),                 // [1/s] proportional gain
                 fuzztest::InRange(0.0F, 0.05F),                 // [1/s2] integral gain
                 fuzztest::InRange(1e-3F, 1000.0F),              // [Nms2] anti-windup clamp
                 fuzztest::InRange(0.0F, 10.0F),                 // [s] control period
                 fuzztest::InRange(1U, 20U));                    // [-] update cycles

FUZZ_TEST(ThrMomentumManagementPropertyFuzz, propertyIntegralTermStaysBounded)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),      // spin axis 0 (normalized in helper)
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),      // spin axis 1 (normalized in helper)
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),      // spin axis 2 (normalized in helper)
                 xmera::fuzz::Vector3fInRange(-600.0F, 600.0F),  // [r/s] per-wheel speeds
                 fuzztest::InRange(1e-3F, 0.2F),                 // [kgm2] common spin-axis inertia
                 fuzztest::InRange(0.0F, 100.0F),                // [Nms] dumping threshold
                 fuzztest::InRange(1e-9F, 0.2F),                 // [1/s] proportional gain
                 fuzztest::InRange(0.0F, 0.05F),                 // [1/s2] integral gain
                 fuzztest::InRange(1e-3F, 1000.0F),              // [Nms2] anti-windup clamp
                 fuzztest::InRange(0.0F, 10.0F));                // [s] control period

// Deliberately unphysical: wheels and speeds orders of magnitude past anything real. The ceiling is not
// arbitrary -- hsExcess forms (hs - hsMin) * hs_B before dividing by hs, so the intermediate overflows once the
// cluster momentum nears sqrt(FLT_MAX) ~ 1.8e19 Nms. These domains cap it near 3e9.
FUZZ_TEST(ThrMomentumManagementPropertyFuzz, propertyTorqueStaysFinite)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),  // spin axis 0 (normalized in helper)
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),  // spin axis 1 (normalized in helper)
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),  // spin axis 2 (normalized in helper)
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // [r/s] per-wheel speeds
                 fuzztest::InRange(1e-3F, 1e3F),             // [kgm2] common spin-axis inertia
                 fuzztest::InRange(0.0F, 1e4F),              // [Nms] dumping threshold
                 fuzztest::InRange(1e-9F, 1e3F),             // [1/s] proportional gain
                 fuzztest::InRange(0.0F, 1e3F),              // [1/s2] integral gain
                 fuzztest::InRange(1e-3F, 1e6F),             // [Nms2] anti-windup clamp
                 fuzztest::InRange(0.0F, 100.0F),            // [s] control period
                 fuzztest::InRange(1U, 20U));                // [-] update cycles

FUZZ_TEST(ThrMomentumManagementRegressionFuzz, regressionFuzzThrMomentumManagement)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),      // spin axis 0 (normalized in helper)
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),      // spin axis 1 (normalized in helper)
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),      // spin axis 2 (normalized in helper)
                 xmera::fuzz::Vector3fInRange(-600.0F, 600.0F),  // [r/s] per-wheel speeds
                 fuzztest::InRange(1e-3F, 0.2F),                 // [kgm2] common spin-axis inertia
                 fuzztest::InRange(0.0F, 100.0F),                // [Nms] dumping threshold
                 fuzztest::InRange(1e-9F, 0.2F),                 // [1/s] proportional gain
                 fuzztest::InRange(0.0F, 0.05F),                 // [1/s2] integral gain
                 fuzztest::InRange(1e-3F, 1000.0F),              // [Nms2] anti-windup clamp
                 fuzztest::InRange(0.0F, 10.0F),                 // [s] control period
                 fuzztest::InRange(1U, 20U));                    // [-] update cycles
