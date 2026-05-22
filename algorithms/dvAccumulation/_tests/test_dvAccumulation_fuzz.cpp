#include "../architecture/testUtilities/eigenFuzzDomains.hpp"
#include "dvAccumulationTestHelpers.hpp"

#include <fuzztest/fuzztest.h>

/*! Fuzz domain: variable-length parallel vectors of measTimes and accels (capped at
 *  MAX_ACC_BUF_PKT). Each accel component is bounded to ±100 m/s², measTimes are bounded
 *  to the [0, 1e10] ns range to keep dt finite. */
FUZZ_TEST(DvAccumulationFuzz, testDvAccumulationFuzz)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange<uint64_t>(0U, static_cast<uint64_t>(1e10)))
                     .WithMaxSize(static_cast<size_t>(MAX_ACC_BUF_PKT)),
                 fuzztest::VectorOf(xmera::fuzz::Vector3fInRange(-100.0F, 100.0F))
                     .WithMaxSize(static_cast<size_t>(MAX_ACC_BUF_PKT)));
