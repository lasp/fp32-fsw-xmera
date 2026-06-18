#include "inertial3DTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

FUZZ_TEST(Inertial3DAlgorithmFuzz, testInertial3D).WithDomains(xmera::fuzz::Vector3fInRange(-1e9f, 1e9f));
