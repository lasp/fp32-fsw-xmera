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

TEST(MimuMajorityVoteTest, PersistenceFaultAndRecovery) {
    MimuMajorityVoteAlgorithm alg{};
    alg.setOmegaThreshold(0.05F);
    alg.setFaultPersistenceLimit(3U);

    Eigen::Vector3f baseRate(-0.1F, 0.25F, 0.3F);
    Eigen::Vector3f outlierRate = baseRate + Eigen::Vector3f(2.0F, 2.0F, 2.0F);
    Eigen::Vector3f mildOutlierRate = baseRate + Eigen::Vector3f(0.5F, 0.5F, 0.5F);

    std::array<MimuInput, kMimuCount> imuInputs{};
    imuInputs.at(0).omega_BN_B = baseRate;
    imuInputs.at(1).omega_BN_B = outlierRate;
    imuInputs.at(2).omega_BN_B = baseRate;

    Eigen::Vector3f fullAvg =
        (imuInputs.at(0).omega_BN_B + imuInputs.at(1).omega_BN_B + imuInputs.at(2).omega_BN_B) / 3.0F;
    Eigen::Vector3f faultedAvg = (imuInputs.at(0).omega_BN_B + imuInputs.at(2).omega_BN_B) / 2.0F;

    // Calls 1 and 2: counter building, no fault — full average returned
    for (uint32_t call = 0U; call < 2U; ++call) {
        auto out = alg.update(imuInputs);
        EXPECT_FALSE(out.faultDetected);
        for (size_t i = 0U; i < kMimuCount; ++i) {
            EXPECT_TRUE(out.validImus.at(i));
        }
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(out.avgOmega_BN_B[i], fullAvg[i], 1e-6);
        }
    }

    // Call 3: persistence limit reached, fault triggers — outlier excluded
    auto out = alg.update(imuInputs);
    EXPECT_TRUE(out.faultDetected);
    EXPECT_TRUE(out.validImus.at(0));
    EXPECT_FALSE(out.validImus.at(1));
    EXPECT_TRUE(out.validImus.at(2));
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.avgOmega_BN_B[i], faultedAvg[i], 1e-6);
    }

    // Call 4: IMU 2 recovers to baseRate, IMU 3 becomes mild outlier —
    // persistence resets for IMU 2, increments for IMU 3, no fault yet
    imuInputs.at(0).omega_BN_B = baseRate;
    imuInputs.at(1).omega_BN_B = baseRate;
    imuInputs.at(2).omega_BN_B = mildOutlierRate;

    Eigen::Vector3f call4Avg =
        (imuInputs.at(0).omega_BN_B + imuInputs.at(1).omega_BN_B + imuInputs.at(2).omega_BN_B) / 3.0F;

    out = alg.update(imuInputs);
    EXPECT_FALSE(out.faultDetected);
    for (size_t i = 0U; i < kMimuCount; ++i) {
        EXPECT_TRUE(out.validImus.at(i));
    }
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.avgOmega_BN_B[i], call4Avg[i], 1e-6);
    }

    // Call 5: IMU 3 still the outlier, persistence count increments but no fault yet
    out = alg.update(imuInputs);
    EXPECT_FALSE(out.faultDetected);
    for (size_t i = 0U; i < kMimuCount; ++i) {
        EXPECT_TRUE(out.validImus.at(i));
    }
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.avgOmega_BN_B[i], call4Avg[i], 1e-6);
    }

    // Call 6: IMU 3 persistence limit reached, fault triggers — IMU 3 excluded
    Eigen::Vector3f call6FaultedAvg = (imuInputs.at(0).omega_BN_B + imuInputs.at(1).omega_BN_B) / 2.0F;

    out = alg.update(imuInputs);
    EXPECT_TRUE(out.faultDetected);
    EXPECT_TRUE(out.validImus.at(0));
    EXPECT_TRUE(out.validImus.at(1));
    EXPECT_FALSE(out.validImus.at(2));
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.avgOmega_BN_B[i], call6FaultedAvg[i], 1e-6);
    }
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
