#include "sunSearchTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"

#include <fuzztest/fuzztest.h>

FUZZ_TEST(SunSearchAlgorithmFuzz, testSunSearch)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(1e-2F, 3.6e3F)).WithSize(kNumRotations),  // rotationDurations
                 fuzztest::VectorOf(fuzztest::InRange(-1e3F, 1e3F)).WithSize(kNumRotations),    // rotationRates
                 fuzztest::VectorOf(fuzztest::InRange(0, 2)).WithSize(kNumRotations),           // rotationAxes
                 xmera::fuzz::Vector3fInRange(-1e3F, 1e3F),                                     // omega_BN_B
                 fuzztest::InRange(1e-6F, 1e1F),                                                // dt
                 fuzztest::InRange(1, 100)                                                      // numSteps
    );
