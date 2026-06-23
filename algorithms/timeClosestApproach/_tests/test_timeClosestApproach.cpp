#include "timeClosestApproachTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(TimeClosestApproachTest, ReferenceTest) {
    testTimeClosestApproach(Eigen::Vector3d{5.0F, 5.0F, 5.0F},  // r
                            Eigen::Vector3d{1.0F, 0.0F, 0.0F},  // v
                            Eigen::Matrix<double, 6, 6>::Identity());
}

TEST(TimeClosestApproachTest, OrthogonalZeroTcaHasNonZeroSigma) {
    // r ⊥ v → tCA = 0 (at closest approach now), but sigma_tCA is non-zero.
    // With P = I₆: covariance_map = [v_hat/‖r‖, r_hat/‖v‖] = [1,0,0, 0,1,0],
    // mapped covariance = 1² + 1² = 2, ratio = 1 → sigmaTca = sqrt(2).
    TimeClosestApproachAlgorithm alg(TimeClosestApproachConfig::create());
    TimeClosestApproachOutput out = alg.update(
        Eigen::Vector3d{0.0, 1.0, 0.0}, Eigen::Vector3d{1.0, 0.0, 0.0}, Eigen::Matrix<double, 6, 6>::Identity());
    EXPECT_FLOAT_EQ(out.tCA, 0.0F);
    EXPECT_NEAR(out.sigmaTca, std::sqrt(2.0F), 1e-6F);
}

TEST(TimeClosestApproachTest, OrthogonalZeroTcaZeroSigma) {
    // r ⊥ v → tCA = 0 (at closest approach now), with zero covariance inputs --> sigma_tCA is also zero.
    TimeClosestApproachAlgorithm alg(TimeClosestApproachConfig::create());
    TimeClosestApproachOutput out =
        alg.update(Eigen::Vector3d{0.0, 1.0, 0.0}, Eigen::Vector3d{1.0, 0.0, 0.0}, Eigen::Matrix<double, 6, 6>::Zero());
    EXPECT_FLOAT_EQ(out.tCA, 0.0F);
    EXPECT_FLOAT_EQ(out.sigmaTca, 0.0F);
}

TEST(TimeClosestApproachTest, ApproachingPositiveTca) {
    // r · v < 0: spacecraft closing on target → tCA > 0
    TimeClosestApproachAlgorithm alg(TimeClosestApproachConfig::create());
    TimeClosestApproachOutput out = alg.update(
        Eigen::Vector3d{-5e7, 0.0, 0.0}, Eigen::Vector3d{1e4, 0.0, 0.0}, Eigen::Matrix<double, 6, 6>::Identity());
    EXPECT_GT(out.tCA, 0.0F);
}

TEST(TimeClosestApproachTest, RecedingNegativeTca) {
    // r · v > 0: spacecraft moving away from target → tCA < 0
    TimeClosestApproachAlgorithm alg(TimeClosestApproachConfig::create());
    TimeClosestApproachOutput out = alg.update(
        Eigen::Vector3d{5e7, 0.0, 0.0}, Eigen::Vector3d{1e4, 0.0, 0.0}, Eigen::Matrix<double, 6, 6>::Identity());
    EXPECT_LT(out.tCA, 0.0F);
}

TEST(TimeClosestApproachTest, BelowThresholdRNormReturnsZero) {
    // r_BN_N norm below kMinVectorNorm → guard fails, output is zero-initialized
    TimeClosestApproachAlgorithm alg(TimeClosestApproachConfig::create());
    TimeClosestApproachOutput out = alg.update(Eigen::Vector3d{kMinVectorNorm * 0.5, 0.0, 0.0},
                                               Eigen::Vector3d{1.0, 0.0, 0.0},
                                               Eigen::Matrix<double, 6, 6>::Identity());
    EXPECT_FLOAT_EQ(out.tCA, 0.0F);
    EXPECT_FLOAT_EQ(out.sigmaTca, 0.0F);
}

TEST(TimeClosestApproachTest, BelowThresholdVNormReturnsZero) {
    // v_BN_N norm below kMinVectorNorm → guard fails, output is zero-initialized
    TimeClosestApproachAlgorithm alg(TimeClosestApproachConfig::create());
    TimeClosestApproachOutput out = alg.update(Eigen::Vector3d{1.0, 0.0, 0.0},
                                               Eigen::Vector3d{kMinVectorNorm * 0.5, 0.0, 0.0},
                                               Eigen::Matrix<double, 6, 6>::Identity());
    EXPECT_FLOAT_EQ(out.tCA, 0.0F);
    EXPECT_FLOAT_EQ(out.sigmaTca, 0.0F);
}

TEST(TimeClosestApproachTest, AtThresholdComputesNormally) {
    // both norms exactly at kMinVectorNorm — the >= guard passes, algorithm runs
    // parallel vectors (r‖v)
    TimeClosestApproachAlgorithm alg(TimeClosestApproachConfig::create());
    TimeClosestApproachOutput out = alg.update(Eigen::Vector3d{kMinVectorNorm, 0.0, 0.0},
                                               Eigen::Vector3d{-kMinVectorNorm, 0.0, 0.0},
                                               Eigen::Matrix<double, 6, 6>::Identity());
    EXPECT_FLOAT_EQ(out.tCA, 1.0F);
    EXPECT_GT(out.sigmaTca, 0.0F);
}

TEST(TimeClosestApproachTest, HigherCovarianceResultsInHigherUncertainty) {
    // Scaling P by k scales sigmaTca by sqrt(k): sigma(k*P) = sqrt(k) * sigma(P).
    // With r ⊥ v and P = I₆: sigmaTca = sqrt(2) (from OrthogonalZeroTcaHasNonZeroSigma).
    // With P = 4*I₆: sigmaTca = sqrt(4) * sqrt(2) = 2 * sqrt(2).
    // tCA is unaffected by the covariance.
    TimeClosestApproachAlgorithm alg(TimeClosestApproachConfig::create());
    const Eigen::Vector3d r{0.0, 1.0, 0.0};
    const Eigen::Vector3d v{1.0, 0.0, 0.0};

    const TimeClosestApproachOutput out_low = alg.update(r, v, Eigen::Matrix<double, 6, 6>::Identity());
    const TimeClosestApproachOutput out_high = alg.update(r, v, 4.0 * Eigen::Matrix<double, 6, 6>::Identity());

    EXPECT_FLOAT_EQ(out_high.tCA, out_low.tCA);
    EXPECT_GT(out_high.sigmaTca, out_low.sigmaTca);
    EXPECT_NEAR(out_high.sigmaTca, 2.0F * out_low.sigmaTca, 1e-5F);
}

TEST(TimeClosestApproachConfigTest, ConfigValidCreation) { EXPECT_NO_THROW(TimeClosestApproachConfig::create()); }

TEST(TimeClosestApproachConfigTest, AlgorithmSetConfig) {
    auto config1 = TimeClosestApproachConfig::create();
    TimeClosestApproachAlgorithm alg(config1);

    auto config2 = TimeClosestApproachConfig::create();
    EXPECT_NO_THROW(TimeClosestApproachAlgorithm alg(config2));
}
