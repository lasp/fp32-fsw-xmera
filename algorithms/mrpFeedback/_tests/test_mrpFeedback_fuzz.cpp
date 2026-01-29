#include "mrpFeedbackTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

FUZZ_TEST(MrpFeedbackAlgorithmFuzz, testMrpFeedback)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-2e0F, 2e0F)).WithSize(3),                // sigma_BR_B
                 fuzztest::InRange(0.0F, 1e3F),                                                 // K
                 fuzztest::InRange(0.0F, 1e3F),                                                 // P
                 fuzztest::InRange(0.0F, 1e3F),                                                 // Ki
                 fuzztest::InRange(0.0F, 1e3F),                                                 // integralLimit
                 fuzztest::InRange(0, 1),                                                       // controlLawType
                 fuzztest::VectorOf(fuzztest::InRange(-1e1F, 1e1F)).WithSize(3U),               // knownTorquePntB_B
                 fuzztest::VectorOf(fuzztest::InRange(-1e0F, 1e0F)).WithSize(3U),               // omega_BR_B
                 fuzztest::VectorOf(fuzztest::InRange(-1e0F, 1e0F)).WithSize(3U),               // omega_RN_B
                 fuzztest::VectorOf(fuzztest::InRange(-1e0F, 1e0F)).WithSize(3U),               // domega_RN_B
                 fuzztest::VectorOf(fuzztest::InRange(-1e3F, 1e3F)).WithSize(RW_EFF_CNT),       // wheelSpeeds
                 fuzztest::VectorOf(fuzztest::Arbitrary<bool>()).WithSize(RW_EFF_CNT),          // wheelAvailabilityBool
                 fuzztest::InRange(0, RW_EFF_CNT),                                              // numRW
                 fuzztest::VectorOf(fuzztest::InRange(-1e1F, 1e1F)).WithSize(RW_EFF_CNT),       // uMax
                 fuzztest::VectorOf(fuzztest::InRange(-1e0F, 1e0F)).WithSize(RW_EFF_CNT),       // JsList
                 fuzztest::VectorOf(fuzztest::InRange(-1e0F, 1e0F)).WithSize(RW_EFF_CNT * 3U),  // GsMatrix_B
                 fuzztest::VectorOf(fuzztest::InRange(-1e3F, 1e3F)).WithSize(9U),               // ISCPntB_B
                 fuzztest::Arbitrary<bool>(),                                                   // rwIsLinked
                 fuzztest::InRange(1e-6F, 1e1F)                                                 // dt
    );
