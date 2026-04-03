#include "mimuMajorityVoteTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(MimuMajorityVoteTest, RegressionTest) {
    regressionTestMimuMajorityVote(1.0F,
                                   1U,
                                   1U,
                                   std::vector<float>{-0.1F, 0.25F, 0.3F},
                                   std::vector<float>{-0.09F, 0.24F, 0.305F},
                                   std::vector<float>{-0.105F, 0.26F, 0.295F});
}

TEST(MimuMajorityVoteTest, RegressionTestOffNominal) {
    regressionTestMimuMajorityVote(0.05F,
                                   1U,
                                   1U,
                                   std::vector<float>{-0.1F, 0.25F, 0.3F},
                                   std::vector<float>{1.9F, 2.25F, 2.3F},
                                   std::vector<float>{-0.1F, 0.25F, 0.3F});
}

TEST(MimuMajorityVoteTest, PropertyTestNominal) {
    // When all IMUs agree within threshold, output should be simple average with no fault
    MimuMajorityVoteAlgorithm alg{};
    float threshold = 1.0F;
    alg.setOmegaThreshold(threshold);

    Eigen::Vector3f baseRate(-0.1F, 0.25F, 0.3F);

    std::array<MimuInput, kMimuCount> imuInputs{};
    imuInputs.at(0).omega_BN_B = baseRate;
    imuInputs.at(1).omega_BN_B = baseRate + Eigen::Vector3f(0.01F, -0.01F, 0.005F);
    imuInputs.at(2).omega_BN_B = baseRate + Eigen::Vector3f(-0.005F, 0.01F, -0.01F);

    Eigen::Vector3f expectedAvg =
        (imuInputs.at(0).omega_BN_B + imuInputs.at(1).omega_BN_B + imuInputs.at(2).omega_BN_B) / 3.0F;

    auto out = alg.update(imuInputs);

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.avgOmega_BN_B[i], expectedAvg[i], 1e-6);
    }
    EXPECT_FALSE(out.faultDetected);
    for (size_t i = 0U; i < 3U; ++i) {
        EXPECT_TRUE(out.validImus.at(i));
    }
}

TEST(MimuMajorityVoteTest, PropertyTestOffNominal) {
    // When one IMU is far off, it should be detected as faulted and excluded from average
    MimuMajorityVoteAlgorithm alg{};
    float threshold = 0.05F;
    alg.setOmegaThreshold(threshold);

    Eigen::Vector3f baseRate(-0.1F, 0.25F, 0.3F);
    Eigen::Vector3f outlierRate = baseRate + Eigen::Vector3f(2.0F, 2.0F, 2.0F);

    std::array<MimuInput, kMimuCount> imuInputs{};
    imuInputs.at(0).omega_BN_B = baseRate;
    imuInputs.at(1).omega_BN_B = outlierRate;  // The outlier
    imuInputs.at(2).omega_BN_B = baseRate;

    // Expected: outlier excluded, average of remaining two
    Eigen::Vector3f expectedAvg = (imuInputs.at(0).omega_BN_B + imuInputs.at(2).omega_BN_B) / 2.0F;

    auto out = alg.update(imuInputs);

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.avgOmega_BN_B[i], expectedAvg[i], 1e-6);
    }
    EXPECT_TRUE(out.faultDetected);
    EXPECT_TRUE(out.validImus.at(0));
    EXPECT_FALSE(out.validImus.at(1));
    EXPECT_TRUE(out.validImus.at(2));
}

TEST(MimuMajorityVoteTest, SetupTest) {
    MimuMajorityVoteAlgorithm alg{};

    // --- Test expected exceptions for omegaThreshold ---

    // Zero or negative omegaThreshold
    EXPECT_THROW(alg.setOmegaThreshold(0.0F), fs::invalid_argument);
    EXPECT_THROW(alg.setOmegaThreshold(-0.1F), fs::invalid_argument);

    // Valid threshold
    float threshold = 0.5F;
    EXPECT_NO_THROW(alg.setOmegaThreshold(threshold));
    EXPECT_NEAR(alg.getOmegaThreshold(), threshold, 1e-6);
}
