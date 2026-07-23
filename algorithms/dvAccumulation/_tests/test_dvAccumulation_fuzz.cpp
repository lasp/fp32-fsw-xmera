#include "dvAccumulationTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"

#include <fuzztest/fuzztest.h>

/*! Fuzz domain: parallel sequences of callTimes (ns) and body-frame accelerations, driving a
 *  sequence of update() calls. callTimes are unconstrained in order (so both the first-call
 *  time-reference set and the strictly-greater gate are exercised); accels are bounded to ±100 m/s² and callTimes to
 *  the [0, 1e10] ns range to keep dt finite. */
FUZZ_TEST(DvAccumulationFuzz, testDvAccumulationFuzz)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange<uint64_t>(0U, static_cast<uint64_t>(1e10))).WithMaxSize(128U),
                 fuzztest::VectorOf(xmera::fuzz::Vector3fInRange(-100.0F, 100.0F)).WithMaxSize(128U));
