#include "mrpSteeringTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

FUZZ_TEST(MrpSteeringAlgorithmFuzz, testMrpSteering)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3),                // sigma_BR_B
                 fuzztest::InRange(0.0F, 1e6F),                                                 // K1
                 fuzztest::InRange(0.0F, 1e6F),                                                 // K3
                 fuzztest::InRange(1e-6F, 1e6F),                                                // omegaMax
                 fuzztest::Arbitrary<bool>(),                                                   // ignoreFF
                 fuzztest::InRange(0.0F, 1e6F),                                                 // P
                 fuzztest::InRange(0.0F, 1e6F),                                                 // Ki
                 fuzztest::InRange(0.0F, 1e6F),                                                 // integralLimit
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3U),               // knownTorquePntB_B
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3U),               // omega_BR_B
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3U),               // omega_RN_B
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3U),               // domega_RN_B
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(RW_EFF_CNT),       // wheelSpeeds
                 fuzztest::VectorOf(fuzztest::Arbitrary<bool>()).WithSize(RW_EFF_CNT),          // wheelAvailabilityBool
                 fuzztest::InRange(0, RW_EFF_CNT),                                              // numRW
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(RW_EFF_CNT),       // uMax
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(RW_EFF_CNT),       // JsList
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(RW_EFF_CNT * 3U),  // GsMatrix_B
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(9U),               // ISCPntB_B
                 fuzztest::Arbitrary<bool>(),                                                   // rwIsLinked
                 fuzztest::InRange(1e-6F, 1e10F)                                                // dt
    );
