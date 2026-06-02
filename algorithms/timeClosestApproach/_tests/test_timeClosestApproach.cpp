#include "timeClosestApproachTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(TimeClosestApproachTest, ReferenceTest) {
    testTimeClosestApproach(Eigen::Vector3f{5.0F, 5.0F, 5.0F},  // r
                            Eigen::Vector3f{1.0F, 0.0F, 0.0F},  // v
                            Eigen::MatrixXf::Identity(6, 6));
}
