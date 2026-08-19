#include "dvAccumulationTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"

#include <fuzztest/fuzztest.h>

/*! Fuzz domain: a control period and a sequence of body-frame accelerations driving a sequence of
 *  update() calls. Accels are bounded to ±100 m/s² and the control period to (0, 10] s to keep the
 *  accumulated Delta-V finite. */
FUZZ_TEST(DvAccumulationFuzz, testDvAccumulationFuzz)
    .WithDomains(fuzztest::InRange<float>(1e-3F, 10.0F),
                 fuzztest::VectorOf(xmera::fuzz::Vector3fInRange(-100.0F, 100.0F)).WithMaxSize(128U));
