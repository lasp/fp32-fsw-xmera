#include "thrFiringSchmittTestHelpers.hpp"
#include <architecture/msgPayloadDef/definitions.h>
#include <fuzztest/fuzztest.h>

FUZZ_TEST(ThrFiringSchmittAlgorithmFuzz, testThrFiringSchmitt)
    .WithDomains(fuzztest::InRange(1e-6F, 1.0F),                                            // levelOn
                 fuzztest::InRange(0.0F, 0.999F),                                           // levelOff
                 fuzztest::InRange(1e-3F, 2.0F),                                            // thrMinFireTime
                 fuzztest::InRange(0U, 1U),                                                 // baseThrustState
                 fuzztest::InRange(0U, static_cast<uint32_t>(MAX_EFF_CNT)),                 // numThrusters
                 fuzztest::VectorOf(fuzztest::InRange(1e-3F, 1e4F)).WithSize(MAX_EFF_CNT),  // maxThrustVec
                 fuzztest::VectorOf(fuzztest::InRange(-1e4F, 1e4F)).WithSize(MAX_EFF_CNT),  // thrForceVec
                 fuzztest::InRange(1e-6F, 5.0F)                                             // dt
    );
