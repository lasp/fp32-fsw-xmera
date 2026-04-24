#include "rwMotorTorqueTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(RwMotorTorqueTest, ReferenceTest) {
    testRwMotorTorque(Eigen::Vector3f{0.1, 0.2, 0.3},
                      Eigen::Vector3f{0.2, -0.4, 0.7},
                      std::vector<bool>{false, false, true, false},
                      true,
                      true,
                      4,
                      std::vector<float>{0.4, 0.1, -0.3, 1.2, 0.4, 0.1, -0.3, 1.2, 0.4, 0.1, -0.3, 1.2},
                      3);
}

TEST(RwMotorTorqueTest, SetupTest) { testRwMotorTorqueSetup(); }
