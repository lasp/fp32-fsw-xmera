#include "inertial3DTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(Inertial3DTest, ReferenceTest) { testInertial3D(std::vector<float>{10000.03, -0.00002, 120310133.2}); }
