#include "mimuMajorityVoteTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

FUZZ_TEST(MimuMajorityVoteAlgorithmFuzz, regressionTestMimuMajorityVote)
    .WithDomains(fuzztest::InRange(1e-6F, 1e3F),                                            // omegaThreshold
                 fuzztest::VectorOf(fuzztest::InRange(-1e3F, 1e3F)).WithSize(3),             // angVel1
                 fuzztest::VectorOf(fuzztest::InRange(-1e3F, 1e3F)).WithSize(3),             // angVel2
                 fuzztest::VectorOf(fuzztest::InRange(-1e3F, 1e3F)).WithSize(3));            // angVel3
