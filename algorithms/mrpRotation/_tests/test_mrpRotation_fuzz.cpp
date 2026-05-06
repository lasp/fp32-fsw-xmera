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
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F), xmera::fuzz::Vector3fInRange(-5.0F, 5.0F));

FUZZ_TEST(MrpRotationPropertyFuzz, propertyFirstStepNoIntegration)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1.0F, 1.0F), xmera::fuzz::Vector3fInRange(-1.0F, 1.0F));
