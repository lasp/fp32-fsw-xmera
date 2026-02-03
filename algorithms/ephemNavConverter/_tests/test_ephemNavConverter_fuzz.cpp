#include "ephemNavConverterTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

FUZZ_TEST(EphemNavConverterAlgorithmFuzz, testEphemNavConverter)
    .WithDomains(fuzztest::Finite<double>(),
                 fuzztest::VectorOf(fuzztest::Finite<double>()).WithSize(3),
                 fuzztest::VectorOf(fuzztest::Finite<double>()).WithSize(3));
