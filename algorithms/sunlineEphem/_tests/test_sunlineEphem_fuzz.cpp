#include "sunlineEphemTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"

#include <fuzztest/fuzztest.h>

FUZZ_TEST(SunlineEphemFuzz, testSunlineEphem)
    .WithDomains(
        // Position of spacecraft and sun [m], bounded by heliosphere (~120 AU, ~1.8e13 m)
        xmera::fuzz::Vector3dInRange(-2e13, 2e13),  // sunPosition
        xmera::fuzz::Vector3dInRange(-2e13, 2e13),  // spacecraftPosition
        // Within an order of magnitude of the overflow threshold (~4e9) of mrpToDcm
        xmera::fuzz::Vector3fInRange(-1e9F, 1e9F)  // spacecraftAttitude
    );
