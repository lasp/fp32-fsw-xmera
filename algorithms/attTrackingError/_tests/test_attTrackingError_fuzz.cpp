#include "attTrackingErrorTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

FUZZ_TEST(attTrackingErrorAlgorithmFuzz, regressionTestAttTrackingError)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-1.0f, 1.0f)).WithSize(3),
                 fuzztest::VectorOf(fuzztest::InRange(-10.0f, 10.0f)).WithSize(3),
                 fuzztest::VectorOf(fuzztest::InRange(-1.0f, 1.0f)).WithSize(3),
                 fuzztest::VectorOf(fuzztest::InRange(-10.0f, 10.0f)).WithSize(3),
                 fuzztest::VectorOf(fuzztest::InRange(-10.0f, 10.0f)).WithSize(3));
