#include "mrpFeedbackTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

FUZZ_TEST(MrpFeedbackAlgorithmFuzz, testMrpFeedback)
    .WithDomains(xmera::fuzz::Vector3fInRange(-2e0F, 2e0F),                                     // sigma_BR_B
                 fuzztest::InRange(0.0F, 1e3F),                                                 // K
                 fuzztest::InRange(0.0F, 1e3F),                                                 // P
                 fuzztest::InRange(0.0F, 1e3F),                                                 // Ki
                 fuzztest::InRange(0.0F, 1e3F),                                                 // integralLimit
                 fuzztest::InRange(0, 1),                                                       // controlLawType
                 xmera::fuzz::Vector3fInRange(-1e1F, 1e1F),                                     // knownTorquePntB_B
                 xmera::fuzz::Vector3fInRange(-1e0F, 1e0F),                                     // omega_BR_B
                 xmera::fuzz::Vector3fInRange(-1e0F, 1e0F),                                     // omega_RN_B
                 xmera::fuzz::Vector3fInRange(-1e0F, 1e0F),                                     // domega_RN_B
                 fuzztest::VectorOf(fuzztest::InRange(-1e3F, 1e3F)).WithSize(kMaxNumRw),       // wheelSpeeds
                 fuzztest::VectorOf(fuzztest::Arbitrary<bool>()).WithSize(kMaxNumRw),          // wheelAvailabilityBool
                 fuzztest::InRange(0, kMaxNumRw),                                              // numRW
                 fuzztest::VectorOf(fuzztest::InRange(-1e0F, 1e0F)).WithSize(kMaxNumRw),       // JsList
                 fuzztest::VectorOf(fuzztest::InRange(-1e0F, 1e0F)).WithSize(kMaxNumRw * 3U),  // GsMatrix_B
                 fuzztest::VectorOf(fuzztest::InRange(-1e3F, 1e3F)).WithSize(9U),               // ISCPntB_B
                 fuzztest::Arbitrary<bool>(),                                                   // rwIsLinked
                 fuzztest::InRange(1e-6F, 1e1F)                                                 // controlPeriod
    );
