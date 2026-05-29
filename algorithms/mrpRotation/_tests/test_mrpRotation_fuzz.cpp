#include "../architecture/testUtilities/eigenFuzzDomains.hpp"
#include "mrpRotationTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

// ---------------------------------------------------------------------------
// Regression fuzz: random configs and reference inputs must agree with the
// independent reference implementation across multiple steps.
// ---------------------------------------------------------------------------
FUZZ_TEST(MrpRotationAlgorithmFuzz, fuzzRegressionMrpRotation)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),  // initialSigmaRR0
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),  // omegaRR0R
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),  // sigma_R0N
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),  // omega_R0N_N
                 fuzztest::InRange(0.01F, 1.0F));            // updateTimeSec

// ---------------------------------------------------------------------------
// Property fuzz tests
// ---------------------------------------------------------------------------

FUZZ_TEST(MrpRotationPropertyFuzz, propertyOutputIsFinite)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),   // initialSigmaRR0
                 xmera::fuzz::Vector3fInRange(-5.0F, 5.0F));  // omegaRR0R

FUZZ_TEST(MrpRotationPropertyFuzz, propertySigmaRNEqualsSigmaRR0WhenInputRefIsIdentity)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),   // initialSigmaRR0 (MRP range)
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F));  // omegaRR0R [rad/s]

// Drive omegaRR0R large enough that the 100-step run crosses the |sigma| = 1 boundary several
// times, stressing mrpSwitch in conjunction with arbitrary starting sigma_RR0.
FUZZ_TEST(MrpRotationPropertyFuzz, propertySigmaRNNormLessOrEqualToOne)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),   // initialSigmaRR0 (MRP range)
                 xmera::fuzz::Vector3fInRange(-5.0F, 5.0F));  // omegaRR0R [rad/s]

FUZZ_TEST(MrpRotationPropertyFuzz, propertyOmegaRNDecomposesCorrectly)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),   // initialSigmaRR0 (MRP range)
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),   // omegaRR0R [rad/s]
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),   // sigma_R0N (MRP range)
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F));  // omega_R0N_N [rad/s]

// With zero internal rotation (sigma_RR0 = 0) and zero R-frame rate (omega_RR0_R = 0), the output
// reference frame R must coincide with the input reference frame R0 for any input reference.
FUZZ_TEST(MrpRotationPropertyFuzz, propertyOutputRefEqualsInputRefWhenRotationIsZero)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),   // sigma_R0N (MRP range)
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F),   // omega_R0N_N [rad/s]
                 xmera::fuzz::Vector3fInRange(-1.0F, 1.0F));  // domega_R0N_N [rad/s^2]
