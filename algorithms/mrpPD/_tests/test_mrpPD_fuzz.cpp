#include "architecture/testUtilities/eigenFuzzDomains.hpp"
#include "mrpPDTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

FUZZ_TEST(MrpPDAlgorithmFuzz, regressionTestMrpPD)
    .WithDomains(fuzztest::InRange(0.0F, 1e6F),
                 fuzztest::InRange(0.0F, 1e6F),
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F),
                 xmera::fuzz::Vector3fInRange(-1e6F, 1e6F));
