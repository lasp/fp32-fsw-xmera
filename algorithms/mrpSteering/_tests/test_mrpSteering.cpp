#include "mrpSteeringTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(MrpSteeringTest, ReferenceTest) {
    testMrpSteering(Eigen::Vector3f{0.4, 0.1, -0.3},
                    1.0,
                    2.0,
                    3.0,
                    false,
                    0.4,
                    0.1,
                    1.1,
                    Eigen::Vector3f{0.1, -0.2, 0.3},
                    Eigen::Vector3f{-0.4, 0.5, -0.6},
                    Eigen::Vector3f{0.7, -0.8, 0.9},
                    Eigen::Vector3f{-1.0, 1.1, -1.2},
                    std::vector<float>{1.9, -2.0, 2.1, -2.2},
                    std::vector<bool>{false, true, false, false},
                    2,
                    std::vector<float>{2.7, -2.8, 2.9, -3.0},
                    std::vector<float>{0.4, 0.1, -0.3, 1.2, 0.4, 0.1, -0.3, 1.2, 0.4, 0.1, -0.3, 1.2},
                    std::vector<float>{0.4, 0.1, -0.3, 0.4, 0.1, -0.3, 0.4, 0.1, -0.3},
                    false,
                    0.1);
}

TEST(MrpSteeringTest, SetupTest) { testMrpSteeringSetup(); }
