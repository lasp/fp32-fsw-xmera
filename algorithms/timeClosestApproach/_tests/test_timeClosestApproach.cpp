#include "timeClosestApproachTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(TimeClosestApproachTest, ReferenceTest) {
    testTimeClosestApproach(Eigen::Vector3f{5.0F, 5.0F, 5.0F},  // r
                            Eigen::Vector3f{1.0F, 0.0F, 0.0F},  // v
                            Eigen::MatrixXf::Identity(6, 6));
}

TEST(TimeClosestApproachTest, OrthogonalTest) {
    TimeClosestApproachAlgorithm alg;
    TimeClosestApproachOutput out = alg.update(
        Eigen::Vector3f{0.0F, 1.0F, 0.0F}, Eigen::Vector3f{1.0F, 0.0F, 0.0F}, Eigen::MatrixXf::Identity(6, 6));
    EXPECT_FLOAT_EQ(out.tCA, 0.0F);
}

TEST(TimeClosestApproachTest, ApproachingPositiveTca) {
    // r · v < 0: spacecraft closing on target → tCA > 0
    TimeClosestApproachAlgorithm alg;
    TimeClosestApproachOutput out = alg.update(
        Eigen::Vector3f{-5e7F, 0.0F, 0.0F}, Eigen::Vector3f{1e4F, 0.0F, 0.0F}, Eigen::MatrixXf::Identity(3, 3));
    EXPECT_GT(out.tCA, 0.0F);
}

TEST(TimeClosestApproachTest, recedingNegativeTca) {
    // r · v > 0: spacecraft moving away from target → tCA < 0
    TimeClosestApproachAlgorithm alg;
    TimeClosestApproachOutput out = alg.update(
        Eigen::Vector3f{5e7F, 0.0F, 0.0F}, Eigen::Vector3f{1e4F, 0.0F, 0.0F}, Eigen::MatrixXf::Identity(3, 3));
    EXPECT_LT(out.tCA, 0.0F);
}
