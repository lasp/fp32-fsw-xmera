#include "dvAccumulationTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"

#include <fuzztest/fuzztest.h>

/*! Fuzz domain: a control period, a sequence of body-frame accelerations driving a sequence of
 *  update() calls, and the bias subtracted from each of them. The bias shares the acceleration
 *  bound: it is subtracted from the acceleration, so a wider bound would only re-cover the same
 *  arithmetic. */
FUZZ_TEST(DvAccumulationFuzz, testDvAccumulationFuzz)
    .WithDomains(fuzztest::InRange<float>(1e-3F, 10.0F),
                 fuzztest::VectorOf(xmera::fuzz::Vector3fInRange(-100.0F, 100.0F)).WithMaxSize(128U),
                 xmera::fuzz::Vector3fInRange(-100.0F, 100.0F));
