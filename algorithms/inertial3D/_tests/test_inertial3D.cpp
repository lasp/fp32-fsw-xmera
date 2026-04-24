#include "inertial3DTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(Inertial3DTest, ReferenceTest) { testInertial3D(Eigen::Vector3f{10000.03F, -0.00002F, 120310133.2F}); }
