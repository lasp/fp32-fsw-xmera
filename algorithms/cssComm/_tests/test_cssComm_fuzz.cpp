#include "cssCommTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

FUZZ_TEST(CssCommAlgorithmFuzz, regressionTestCssComm)
    .WithDomains(fuzztest::InRange<uint32_t>(1u, MAX_NUM_CSS_SENSORS),                              // numSensors
                 fuzztest::InRange(1e-5, 1e-2),                                                     // maxSensorValue
                 fuzztest::InRange<uint32_t>(1u, kMaxNumChebyPolys),                                // chebyCount
                 fuzztest::VectorOf(fuzztest::InRange(-1e1, 1e1)).WithSize(kMaxNumChebyPolys),      // chebyCoeffs
                 fuzztest::VectorOf(fuzztest::InRange(-1.5, 1.5)).WithSize(MAX_NUM_CSS_SENSORS));   // sensorInputRatios
