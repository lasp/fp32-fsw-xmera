#include "mrpSteeringTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(MrpSteeringTest, ReferenceTest) { testMrpSteering(std::vector<float>{0.4, 0.1, -0.3}, 1.0, 2.0, 3.0, false); }

TEST(MrpSteeringTest, SetupTest) { testMrpSteeringSetup(); }
