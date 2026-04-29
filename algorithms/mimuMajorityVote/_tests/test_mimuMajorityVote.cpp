#include "mimuMajorityVoteTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(MimuMajorityVoteTest, RegressionTest) {
    regressionTestMimuMajorityVote(1.0F,
                                   1U,
                                   1U,
                                   Eigen::Vector3f{-0.1F, 0.25F, 0.3F},
                                   Eigen::Vector3f{-0.09F, 0.24F, 0.305F},
                                   Eigen::Vector3f{-0.105F, 0.26F, 0.295F});
}

TEST(MimuMajorityVoteTest, RegressionTestOffNominal) {
    regressionTestMimuMajorityVote(0.05F,
                                   1U,
                                   1U,
                                   Eigen::Vector3f{-0.1F, 0.25F, 0.3F},
                                   Eigen::Vector3f{1.9F, 2.25F, 2.3F},
                                   Eigen::Vector3f{-0.1F, 0.25F, 0.3F});
}

TEST(MimuMajorityVoteTest, PropertyTestNominal) {
    // When all IMUs agree within threshold, output should be simple average with no fault
    MimuMajorityVoteAlgorithm alg{};
    float threshold = 1.0F;
    alg.setOmegaThreshold(threshold);

    Eigen::Vector3f baseRate(-0.1F, 0.25F, 0.3F);

    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B{};
    imuOmegas_BN_B.at(0) = baseRate;
    imuOmegas_BN_B.at(1) = baseRate + Eigen::Vector3f(0.01F, -0.01F, 0.005F);
    imuOmegas_BN_B.at(2) = baseRate + Eigen::Vector3f(-0.005F, 0.01F, -0.01F);

    Eigen::Vector3f expectedAvg = (imuOmegas_BN_B.at(0) + imuOmegas_BN_B.at(1) + imuOmegas_BN_B.at(2)) / 3.0F;

    auto out = alg.update(imuOmegas_BN_B);

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

    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B{};
    imuOmegas_BN_B.at(0) = baseRate;
    imuOmegas_BN_B.at(1) = outlierRate;  // The outlier
    imuOmegas_BN_B.at(2) = baseRate;

    // Expected: outlier excluded, average of remaining two
    Eigen::Vector3f expectedAvg = (imuOmegas_BN_B.at(0) + imuOmegas_BN_B.at(2)) / 2.0F;

    auto out = alg.update(imuOmegas_BN_B);

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

    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B{};
    imuOmegas_BN_B.at(0) = baseRate;
    imuOmegas_BN_B.at(1) = outlierRate;
    imuOmegas_BN_B.at(2) = baseRate;

    Eigen::Vector3f fullAvg = (imuOmegas_BN_B.at(0) + imuOmegas_BN_B.at(1) + imuOmegas_BN_B.at(2)) / 3.0F;
    Eigen::Vector3f faultedAvg = (imuOmegas_BN_B.at(0) + imuOmegas_BN_B.at(2)) / 2.0F;

    // Calls 1 and 2: counter building, no fault — full average returned
    for (uint32_t call = 0U; call < 2U; ++call) {
        auto out = alg.update(imuOmegas_BN_B);
        EXPECT_FALSE(out.faultDetected);
        for (size_t i = 0U; i < kMimuCount; ++i) {
            EXPECT_TRUE(out.validImus.at(i));
        }
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(out.avgOmega_BN_B[i], fullAvg[i], 1e-6);
        }
    }

    // Call 3: persistence limit reached, fault triggers — outlier excluded
    auto out = alg.update(imuOmegas_BN_B);
    EXPECT_TRUE(out.faultDetected);
    EXPECT_TRUE(out.validImus.at(0));
    EXPECT_FALSE(out.validImus.at(1));
    EXPECT_TRUE(out.validImus.at(2));
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.avgOmega_BN_B[i], faultedAvg[i], 1e-6);
    }

    // Call 4: IMU 2 recovers to baseRate, IMU 3 becomes mild outlier —
    // persistence resets for IMU 2, increments for IMU 3, no fault yet
    imuOmegas_BN_B.at(0) = baseRate;
    imuOmegas_BN_B.at(1) = baseRate;
    imuOmegas_BN_B.at(2) = mildOutlierRate;

    Eigen::Vector3f call4Avg = (imuOmegas_BN_B.at(0) + imuOmegas_BN_B.at(1) + imuOmegas_BN_B.at(2)) / 3.0F;

    out = alg.update(imuOmegas_BN_B);
    EXPECT_FALSE(out.faultDetected);
    for (size_t i = 0U; i < kMimuCount; ++i) {
        EXPECT_TRUE(out.validImus.at(i));
    }
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.avgOmega_BN_B[i], call4Avg[i], 1e-6);
    }

    // Call 5: IMU 3 still the outlier, persistence count increments but no fault yet
    out = alg.update(imuOmegas_BN_B);
    EXPECT_FALSE(out.faultDetected);
    for (size_t i = 0U; i < kMimuCount; ++i) {
        EXPECT_TRUE(out.validImus.at(i));
    }
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.avgOmega_BN_B[i], call4Avg[i], 1e-6);
    }

    // Call 6: IMU 3 persistence limit reached, fault triggers — IMU 3 excluded
    Eigen::Vector3f call6FaultedAvg = (imuOmegas_BN_B.at(0) + imuOmegas_BN_B.at(1)) / 2.0F;

    out = alg.update(imuOmegas_BN_B);
    EXPECT_TRUE(out.faultDetected);
    EXPECT_TRUE(out.validImus.at(0));
    EXPECT_TRUE(out.validImus.at(1));
    EXPECT_FALSE(out.validImus.at(2));
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.avgOmega_BN_B[i], call6FaultedAvg[i], 1e-6);
    }
}

TEST(MimuMajorityVoteTest, ResetClearsPersistence) {
    MimuMajorityVoteAlgorithm alg{};
    alg.setOmegaThreshold(0.05F);
    alg.setFaultPersistenceLimit(3U);

    Eigen::Vector3f baseRate(-0.1F, 0.25F, 0.3F);
    Eigen::Vector3f outlierRate = baseRate + Eigen::Vector3f(2.0F, 2.0F, 2.0F);

    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B{};
    imuOmegas_BN_B.at(0) = baseRate;
    imuOmegas_BN_B.at(1) = outlierRate;
    imuOmegas_BN_B.at(2) = baseRate;

    Eigen::Vector3f fullAvg = (imuOmegas_BN_B.at(0) + imuOmegas_BN_B.at(1) + imuOmegas_BN_B.at(2)) / 3.0F;
    Eigen::Vector3f faultedAvg = (imuOmegas_BN_B.at(0) + imuOmegas_BN_B.at(2)) / 2.0F;

    // Call twice to build up persistence (limit is 3, so no fault yet)
    for (uint32_t call = 0U; call < 2U; ++call) {
        auto out = alg.update(imuOmegas_BN_B);
        EXPECT_FALSE(out.faultDetected);
        for (size_t i = 0U; i < kMimuCount; ++i) {
            EXPECT_TRUE(out.validImus.at(i));
        }
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(out.avgOmega_BN_B[i], fullAvg[i], 1e-6);
        }
    }

    // Reset clears persistence counters
    alg.reset();

    // Same outlier inputs — counter starts from zero again, no fault
    for (uint32_t call = 0U; call < 2U; ++call) {
        auto out = alg.update(imuOmegas_BN_B);
        EXPECT_FALSE(out.faultDetected);
        for (size_t i = 0U; i < kMimuCount; ++i) {
            EXPECT_TRUE(out.validImus.at(i));
        }
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(out.avgOmega_BN_B[i], fullAvg[i], 1e-6);
        }
    }

    // Third call after reset — now fault triggers
    auto out = alg.update(imuOmegas_BN_B);
    EXPECT_TRUE(out.faultDetected);
    EXPECT_TRUE(out.validImus.at(0));
    EXPECT_FALSE(out.validImus.at(1));
    EXPECT_TRUE(out.validImus.at(2));
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.avgOmega_BN_B[i], faultedAvg[i], 1e-6);
    }
}

TEST(MimuMajorityVoteTest, SetupTest) {
    MimuMajorityVoteAlgorithm alg{};

    // --- Test expected exceptions for omegaThreshold ---

    // Zero or negative omegaThreshold
    EXPECT_THROW(alg.setOmegaThreshold(0.0F), fsw::invalid_argument);
    EXPECT_THROW(alg.setOmegaThreshold(-0.1F), fsw::invalid_argument);

    // Valid threshold
    float threshold = 0.5F;
    EXPECT_NO_THROW(alg.setOmegaThreshold(threshold));
    EXPECT_NEAR(alg.getOmegaThreshold(), threshold, 1e-6);
}
