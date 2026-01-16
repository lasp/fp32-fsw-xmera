#include "rwMotorTorqueTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

FUZZ_TEST(RwMotorTorqueAlgorithmFuzz, testRwMotorTorque)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3),
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3),
                 fuzztest::VectorOf(fuzztest::Arbitrary<bool>()).WithSize(RW_EFF_CNT),
                 fuzztest::Arbitrary<bool>(),
                 fuzztest::Arbitrary<bool>(),
                 fuzztest::InRange(0, RW_EFF_CNT),
                 fuzztest::VectorOf(fuzztest::InRange(-1.0F, 1.0F)).WithSize(RW_EFF_CNT * 3U),
                 fuzztest::InRange(0, 3));
