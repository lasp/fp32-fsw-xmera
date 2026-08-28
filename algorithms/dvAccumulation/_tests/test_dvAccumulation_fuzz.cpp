#include "dvAccumulationTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"

#include <fuzztest/fuzztest.h>

/*! Fuzz domain: a control period, a sequence of body-frame accelerations for a sequence of update()
 *  calls, and the bias that update() subtracts from each of them. The bias has the same bound as the
 *  acceleration: update() subtracts the bias from the acceleration, so a larger bound gives the same
 *  arithmetic again. */
FUZZ_TEST(DvAccumulationFuzz, testDvAccumulationFuzz)
    .WithDomains(fuzztest::InRange<float>(1e-3F, 10.0F),
                 fuzztest::VectorOf(xmera::fuzz::Vector3fInRange(-100.0F, 100.0F)).WithMaxSize(128U),
                 xmera::fuzz::Vector3fInRange(-100.0F, 100.0F));
