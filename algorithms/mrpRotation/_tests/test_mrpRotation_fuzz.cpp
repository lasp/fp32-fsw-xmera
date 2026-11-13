#include "mrpRotationTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

// ---------------------------------------------------------------------------
// Regression fuzz: random configs and reference inputs must agree with the
// independent reference implementation across multiple steps.
// ---------------------------------------------------------------------------
FUZZ_TEST(MrpRotationAlgorithmFuzz, fuzzRegressionMrpRotation)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // sigma_RcR0
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // sigma_RpRc
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),  // spinAxis_Rp, normalized by the helper
                 fuzztest::InRange(-1e1F, 1e1F),             // spinRate [rad/s]
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // sigma_R0N
                 xmera::fuzz::Vector3fInRange(-1e1F, 1e1F),  // omega_R0N_N
                 fuzztest::InRange(1e-6F, 1e1F));            // updateTimeSec

// ---------------------------------------------------------------------------
// Property fuzz tests
// ---------------------------------------------------------------------------

FUZZ_TEST(MrpRotationPropertyFuzz, propertyOutputIsFinite)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // sigma_RcR0
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // sigma_RpRc
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),  // spinAxis_Rp, normalized by the helper
                 fuzztest::InRange(-1e1F, 1e1F));            // spinRate [rad/s]

FUZZ_TEST(MrpRotationPropertyFuzz, propertySigmaRNEqualsSigmaRRpWhenOffsetsAndInputRefAreIdentity)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),  // spinAxis_Rp, normalized by the helper
                 fuzztest::InRange(-1.0F, 1.0F));            // spinRate [rad/s], <= 0.1 rad per 0.1 s step

FUZZ_TEST(MrpRotationPropertyFuzz, propertySigmaRNNormLessOrEqualToOne)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // sigma_RcR0
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // sigma_RpRc
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),  // spinAxis_Rp, normalized by the helper
                 fuzztest::InRange(-1e1F, 1e1F));            // spinRate [rad/s]

FUZZ_TEST(MrpRotationPropertyFuzz, propertyOmegaRNDecomposesCorrectly)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),   // sigma_RcR0
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),   // sigma_RpRc
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),   // spinAxis_Rp, normalized by the helper
                 fuzztest::InRange(-1e-1F, 1e-1F),            // spinRate [rad/s], <= 0.025 rad per 0.25 s step
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),   // sigma_R0N
                 xmera::fuzz::Vector3fInRange(-1e1F, 1e1F));  // omega_R0N_N [rad/s]

FUZZ_TEST(MrpRotationPropertyFuzz, propertyOutputRefEqualsInputRefWhenRotationIsZero)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),   // sigma_R0N
                 xmera::fuzz::Vector3fInRange(-1e1F, 1e1F),   // omega_R0N_N [rad/s]
                 xmera::fuzz::Vector3fInRange(-1e1F, 1e1F));  // domega_R0N_N [rad/s^2]

FUZZ_TEST(MrpRotationPropertyFuzz, propertyOutputEqualsOffsetInputRefWhenRateIsZero)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),   // sigma_RcR0
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),   // sigma_RpRc
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F));  // sigma_R0N

FUZZ_TEST(MrpRotationPropertyFuzz, propertyRotationAxisFixedInInputRef)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // sigma_RcR0
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // sigma_RpRc
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),  // spinAxis_Rp, normalized by the helper
                 fuzztest::InRange(-1e-1F, 1e-1F));          // spinRate [rad/s], <= 0.05 rad per 0.5 s step
