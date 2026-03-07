#include "sunlineEphemTestHelpers.hpp"

#include <fuzztest/fuzztest.h>

FUZZ_TEST(SunlineEphemFuzz, testSunlineEphem)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-1e12, 1e12)).WithSize(3),  // sunPosition
                 fuzztest::VectorOf(fuzztest::InRange(-1e12, 1e12)).WithSize(3),  // spacecraftPosition
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3)   // spacecraftAttitude
    );
