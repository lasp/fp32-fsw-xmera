#include "thrMomentumManagementTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

// Domains are kept to physically realizable hardware: spin-axis inertias of a real wheel, speeds within its
// rated range, and gains that ask for torques an effector could actually deliver.

// ---------------------------------------------------------------------------
// Property fuzz tests
// ---------------------------------------------------------------------------

FUZZ_TEST(ThrMomentumManagementPropertyFuzz, propertyProportionalTorqueOpposesExcessMomentum)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),      // spin axis 0 (normalized in helper)
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),      // spin axis 1 (normalized in helper)
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),      // spin axis 2 (normalized in helper)
                 xmera::fuzz::Vector3fInRange(-600.0F, 600.0F),  // [r/s] per-wheel speeds
                 fuzztest::InRange(1e-3F, 0.2F),                 // [kgm2] common spin-axis inertia
                 fuzztest::InRange(0.0F, 100.0F),                // [Nms] dumping threshold
                 fuzztest::InRange(1e-9F, 0.2F),                 // [1/s] proportional gain
                 fuzztest::InRange(0.0F, 10.0F));                // [s] control period

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
                 fuzztest::InRange(1e-9F, 0.2F),                 // [1/s] proportional gain
                 fuzztest::InRange(0.0F, 0.05F),                 // [1/s2] integral gain
                 fuzztest::InRange(1e-3F, 1000.0F),              // [Nms2] anti-windup clamp
                 fuzztest::InRange(0.0F, 10.0F),                 // [s] control period
                 fuzztest::InRange(1U, 20U));                    // [-] number of update cycles
