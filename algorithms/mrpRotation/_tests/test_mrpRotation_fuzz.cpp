#include "mrpRotationTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

// ---------------------------------------------------------------------------
// Regression fuzz: random configs and reference inputs must agree with the
// independent reference implementation across multiple steps.
// ---------------------------------------------------------------------------
FUZZ_TEST(MrpRotationAlgorithmFuzz, fuzzRegressionMrpRotation)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // initialSigmaRR0
                 xmera::fuzz::Vector3fInRange(-1e1F, 1e1F),  // omegaRR0R
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),  // sigma_R0N
                 xmera::fuzz::Vector3fInRange(-1e1F, 1e1F),  // omega_R0N_N
                 fuzztest::InRange(1e-6F, 1e1F));            // updateTimeSec

// ---------------------------------------------------------------------------
// Property fuzz tests
// ---------------------------------------------------------------------------

FUZZ_TEST(MrpRotationPropertyFuzz, propertyOutputIsFinite)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),   // initialSigmaRR0
                 xmera::fuzz::Vector3fInRange(-1e1F, 1e1F));  // omegaRR0R

FUZZ_TEST(MrpRotationPropertyFuzz, propertySigmaRNEqualsSigmaRR0WhenInputRefIsIdentity)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),   // initialSigmaRR0 (MRP range)
                 xmera::fuzz::Vector3fInRange(-1e1F, 1e1F));  // omegaRR0R [rad/s]

FUZZ_TEST(MrpRotationPropertyFuzz, propertySigmaRNNormLessOrEqualToOne)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),   // initialSigmaRR0 (MRP range)
                 xmera::fuzz::Vector3fInRange(-1e1F, 1e1F));  // omegaRR0R [rad/s]

FUZZ_TEST(MrpRotationPropertyFuzz, propertyOmegaRNDecomposesCorrectly)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),   // initialSigmaRR0 (MRP range)
                 xmera::fuzz::Vector3fInRange(-1e1F, 1e1F),   // omegaRR0R [rad/s]
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),   // sigma_R0N (MRP range)
                 xmera::fuzz::Vector3fInRange(-1e1F, 1e1F));  // omega_R0N_N [rad/s]

FUZZ_TEST(MrpRotationPropertyFuzz, propertyOutputRefEqualsInputRefWhenRotationIsZero)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),   // sigma_R0N (MRP range)
                 xmera::fuzz::Vector3fInRange(-1e1F, 1e1F),   // omega_R0N_N [rad/s]
                 xmera::fuzz::Vector3fInRange(-1e1F, 1e1F));  // domega_R0N_N [rad/s^2]
