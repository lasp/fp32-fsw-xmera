#include "sunlineEphemTestHelpers.hpp"

#include <fuzztest/fuzztest.h>

FUZZ_TEST(SunlineEphemFuzz, testSunlineEphem)
    .WithDomains(
        // Position of spacecraft and sun [m], bounded by heliosphere (~120 AU, ~1.8e13 m)
        fuzztest::VectorOf(fuzztest::InRange(-2e13, 2e13)).WithSize(3),  // sunPosition
        fuzztest::VectorOf(fuzztest::InRange(-2e13, 2e13)).WithSize(3),  // spacecraftPosition
        // Within an order of magnitude of the overflow threshold (~4e9) of mrpToDcm
        fuzztest::VectorOf(fuzztest::InRange(-1e9F, 1e9F)).WithSize(3)  // spacecraftAttitude
    );
