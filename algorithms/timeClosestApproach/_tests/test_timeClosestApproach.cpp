#include "timeClosestApproachTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(TimeClosestApproachTest, ReferenceTest) {
    testTimeClosestApproach(Eigen::Vector3d{5.0F, 5.0F, 5.0F},  // r
                            Eigen::Vector3d{1.0F, 0.0F, 0.0F},  // v
                            Eigen::MatrixXf::Identity(6, 6));
}

TEST(TimeClosestApproachTest, OrthogonalTest) {
    TimeClosestApproachAlgorithm alg(TimeClosestApproachConfig::create());
    TimeClosestApproachOutput out =
        alg.update(Eigen::Vector3d{0.0, 1.0, 0.0}, Eigen::Vector3d{1.0, 0.0, 0.0}, Eigen::MatrixXf::Identity(6, 6));
    EXPECT_FLOAT_EQ(out.tCA, 0.0F);
}

TEST(TimeClosestApproachTest, ApproachingPositiveTca) {
    // r · v < 0: spacecraft closing on target → tCA > 0
    TimeClosestApproachAlgorithm alg(TimeClosestApproachConfig::create());
    TimeClosestApproachOutput out =
        alg.update(Eigen::Vector3d{-5e7, 0.0, 0.0}, Eigen::Vector3d{1e4, 0.0, 0.0}, Eigen::MatrixXf::Identity(3, 3));
    EXPECT_GT(out.tCA, 0.0F);
}

TEST(TimeClosestApproachTest, RecedingNegativeTca) {
    // r · v > 0: spacecraft moving away from target → tCA < 0
    TimeClosestApproachAlgorithm alg(TimeClosestApproachConfig::create());
    TimeClosestApproachOutput out =
        alg.update(Eigen::Vector3d{5e7, 0.0, 0.0}, Eigen::Vector3d{1e4, 0.0, 0.0}, Eigen::MatrixXf::Identity(3, 3));
    EXPECT_LT(out.tCA, 0.0F);
}

TEST(TimeClosestApproachTest, InvariantTca) {
    // r · v > 0: spacecraft moving away from target → tCA < 0
    TimeClosestApproachAlgorithm alg(TimeClosestApproachConfig::create());
    TimeClosestApproachOutput out1 =
        alg.update(Eigen::Vector3d{5e7, 0.0, 0.0}, Eigen::Vector3d{1e4, 0.0, 0.0}, Eigen::MatrixXf::Identity(3, 3));
    TimeClosestApproachOutput out2 =
        alg.update(Eigen::Vector3d{5e7, 0.0, 0.0}, Eigen::Vector3d{1e4, 0.0, 0.0}, Eigen::MatrixXf::Identity(6, 6));
    EXPECT_FLOAT_EQ(out1.tCA, out2.tCA);
}

TEST(TimeClosestApproachConfigTest, ConfigValidCreation) { EXPECT_NO_THROW(TimeClosestApproachConfig::create()); }

TEST(TimeClosestApproachConfigTest, AlgorithmSetConfig) {
    auto config1 = TimeClosestApproachConfig::create();
    TimeClosestApproachAlgorithm alg(config1);

    auto config2 = TimeClosestApproachConfig::create();
    EXPECT_NO_THROW(TimeClosestApproachAlgorithm alg(config2));
}
