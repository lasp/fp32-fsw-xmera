#include "rwMotorTorqueTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(RwMotorTorqueTest, ReferenceTest) {
    testRwMotorTorque(Eigen::Vector3f{0.1F, 0.2F, 0.3F},
                      Eigen::Vector3f{0.2F, -0.4F, 0.7F},
                      std::vector<bool>{false, false, true, false},
                      true,
                      true,
                      4,
                      std::vector<float>{0.4F, 0.1F, -0.3F, 1.2F, 0.4F, 0.1F, -0.3F, 1.2F, 0.4F, 0.1F, -0.3F, 1.2F},
                      3);
}

TEST(RwMotorTorqueTest, SetupTest) { testRwMotorTorqueSetup(); }
