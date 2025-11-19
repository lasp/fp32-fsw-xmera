#include "mrpSteeringTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

FUZZ_TEST(MrpSteeringAlgorithmFuzz, testMrpSteering)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3),
                 fuzztest::InRange(0.0F, 1e6F),
                 fuzztest::InRange(0.0F, 1e6F),
                 fuzztest::InRange(1e-6F, 1e6F),
                 fuzztest::Arbitrary<bool>());
