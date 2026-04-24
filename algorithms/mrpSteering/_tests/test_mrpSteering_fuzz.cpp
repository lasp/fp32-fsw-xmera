#include "architecture/testUtilities/eigenFuzzDomains.hpp"
#include "mrpSteeringTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

FUZZ_TEST(MrpSteeringAlgorithmFuzz, testMrpSteering)
    .WithDomains(xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),                                     // sigma_BR_B
                 fuzztest::InRange(0.0F, 1e6F),                                                 // K1
                 fuzztest::InRange(0.0F, 1e6F),                                                 // K3
                 fuzztest::InRange(1e-6F, 1e6F),                                                // omegaMax
                 fuzztest::Arbitrary<bool>(),                                                   // ignoreFF
                 fuzztest::InRange(0.0F, 1e6F),                                                 // P
                 fuzztest::InRange(0.0F, 1e6F),                                                 // Ki
                 fuzztest::InRange(0.0F, 1e6F),                                                 // integralLimit
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),                                     // knownTorquePntB_B
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),                                     // omega_BR_B
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),                                     // omega_RN_B
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),                                     // domega_RN_B
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(RW_EFF_CNT),       // wheelSpeeds
                 fuzztest::VectorOf(fuzztest::Arbitrary<bool>()).WithSize(RW_EFF_CNT),          // wheelAvailabilityBool
                 fuzztest::InRange(0, RW_EFF_CNT),                                              // numRW
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(RW_EFF_CNT),       // JsList
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(RW_EFF_CNT * 3U),  // GsMatrix_B
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(9U),               // ISCPntB_B
                 fuzztest::Arbitrary<bool>(),                                                   // rwIsLinked
                 fuzztest::InRange(1e-6F, 1e10F)                                                // dt
    );
