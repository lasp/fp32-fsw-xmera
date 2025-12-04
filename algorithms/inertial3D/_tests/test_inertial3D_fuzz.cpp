#include "inertial3DTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

FUZZ_TEST(Inertial3DAlgorithmFuzz, testInertial3D)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-1e9f, 1e9f)).WithMinSize(3).WithMaxSize(3));
