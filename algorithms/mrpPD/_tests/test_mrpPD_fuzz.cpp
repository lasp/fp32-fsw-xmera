#include "mrpPDTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

FUZZ_TEST(MrpPDAlgorithmFuzz, regressionTestMrpPD)
    .WithDomains(fuzztest::InRange(0.0F, 1e6F),
                 fuzztest::InRange(0.0F, 1e6F),
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3),
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3),
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3),
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3));
