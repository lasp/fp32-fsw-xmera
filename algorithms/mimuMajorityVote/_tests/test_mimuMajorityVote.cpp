#include "mimuMajorityVoteTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(MimuMajorityVoteTest, RegressionTest) {
    // Nominal: gyro and accel inputs both agree within their thresholds — no fault on either vote.
    regressionTestMimuMajorityVote(/* omegaThreshold */ 1.0F,
                                   /* gyroFaultPersistenceLimit */ 1U,
                                   /* accelThreshold */ 1.0F,
                                   /* accelFaultPersistenceLimit */ 1U,
                                   /* algCallCount */ 1U,
                                   Eigen::Vector3f{-0.1F, 0.25F, 0.3F},
                                   Eigen::Vector3f{-0.09F, 0.24F, 0.305F},
                                   Eigen::Vector3f{-0.105F, 0.26F, 0.295F},
                                   Eigen::Vector3f{0.0F, 0.0F, 9.8F},
                                   Eigen::Vector3f{0.01F, -0.02F, 9.81F},
                                   Eigen::Vector3f{-0.01F, 0.02F, 9.79F});
}

TEST(MimuMajorityVoteTest, RegressionTestOffNominal) {
    // Independent off-nominal: gyro IMU 1 is an outlier (gyro fault) while the accel inputs agree
    // (no accel fault) — the regression helper verifies both votes against the reference.
    regressionTestMimuMajorityVote(/* omegaThreshold */ 0.05F,
                                   /* gyroFaultPersistenceLimit */ 1U,
                                   /* accelThreshold */ 0.05F,
                                   /* accelFaultPersistenceLimit */ 1U,
                                   /* algCallCount */ 1U,
                                   Eigen::Vector3f{-0.1F, 0.25F, 0.3F},
                                   Eigen::Vector3f{1.9F, 2.25F, 2.3F},
                                   Eigen::Vector3f{-0.1F, 0.25F, 0.3F},
                                   Eigen::Vector3f{0.0F, 0.0F, 9.8F},
                                   Eigen::Vector3f{0.0F, 0.0F, 9.8F},
                                   Eigen::Vector3f{0.0F, 0.0F, 9.8F});
}

TEST(MimuMajorityVoteTest, PropertyTestNominal) {
    // When all IMUs agree within threshold, output should be simple average with no fault
    const float threshold = 1.0F;
    MimuMajorityVoteAlgorithm alg{MimuMajorityVoteConfig::create(threshold, 1U, threshold, 1U)};

    Eigen::Vector3f baseRate(-0.1F, 0.25F, 0.3F);

    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B{};
    imuOmegas_BN_B.at(0) = baseRate;
    imuOmegas_BN_B.at(1) = baseRate + Eigen::Vector3f(0.01F, -0.01F, 0.005F);
    imuOmegas_BN_B.at(2) = baseRate + Eigen::Vector3f(-0.005F, 0.01F, -0.01F);
    std::array<Eigen::Vector3f, kMimuCount> imuAccels_B{};  // all zero: no accel fault

    Eigen::Vector3f expectedAvg = (imuOmegas_BN_B.at(0) + imuOmegas_BN_B.at(1) + imuOmegas_BN_B.at(2)) / 3.0F;

    auto out = alg.update(imuOmegas_BN_B, imuAccels_B);

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.gyro.average[i], expectedAvg[i], 1e-6);
    }
    EXPECT_FALSE(out.gyro.faultDetected);
    for (size_t i = 0U; i < 3U; ++i) {
        EXPECT_TRUE(out.gyro.imuValid.at(i));
    }
}

TEST(MimuMajorityVoteTest, PropertyTestOffNominal) {
    // When one IMU is far off, it should be detected as faulted and excluded from average
    const float threshold = 0.05F;
    MimuMajorityVoteAlgorithm alg{MimuMajorityVoteConfig::create(threshold, 1U, threshold, 1U)};

    Eigen::Vector3f baseRate(-0.1F, 0.25F, 0.3F);
    Eigen::Vector3f outlierRate = baseRate + Eigen::Vector3f(2.0F, 2.0F, 2.0F);

    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B{};
    imuOmegas_BN_B.at(0) = baseRate;
    imuOmegas_BN_B.at(1) = outlierRate;  // The outlier
    imuOmegas_BN_B.at(2) = baseRate;
    std::array<Eigen::Vector3f, kMimuCount> imuAccels_B{};  // all zero: no accel fault

    // Expected: outlier excluded, average of remaining two
    Eigen::Vector3f expectedAvg = (imuOmegas_BN_B.at(0) + imuOmegas_BN_B.at(2)) / 2.0F;

    auto out = alg.update(imuOmegas_BN_B, imuAccels_B);

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.gyro.average[i], expectedAvg[i], 1e-6);
    }
    EXPECT_TRUE(out.gyro.faultDetected);
    EXPECT_TRUE(out.gyro.imuValid.at(0));
    EXPECT_FALSE(out.gyro.imuValid.at(1));
    EXPECT_TRUE(out.gyro.imuValid.at(2));
}

TEST(MimuMajorityVoteTest, PersistenceFaultAndRecovery) {
    MimuMajorityVoteAlgorithm alg{MimuMajorityVoteConfig::create(0.05F, 3U, 1.0F, 1U)};

    Eigen::Vector3f baseRate(-0.1F, 0.25F, 0.3F);
    Eigen::Vector3f outlierRate = baseRate + Eigen::Vector3f(2.0F, 2.0F, 2.0F);
    Eigen::Vector3f mildOutlierRate = baseRate + Eigen::Vector3f(0.5F, 0.5F, 0.5F);

    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B{};
    imuOmegas_BN_B.at(0) = baseRate;
    imuOmegas_BN_B.at(1) = outlierRate;
    imuOmegas_BN_B.at(2) = baseRate;
    std::array<Eigen::Vector3f, kMimuCount> imuAccels_B{};  // all zero: no accel fault

    Eigen::Vector3f fullAvg = (imuOmegas_BN_B.at(0) + imuOmegas_BN_B.at(1) + imuOmegas_BN_B.at(2)) / 3.0F;
    Eigen::Vector3f faultedAvg = (imuOmegas_BN_B.at(0) + imuOmegas_BN_B.at(2)) / 2.0F;

    // Calls 1 and 2: counter building, no fault — full average returned
    for (uint32_t call = 0U; call < 2U; ++call) {
        auto out = alg.update(imuOmegas_BN_B, imuAccels_B);
        EXPECT_FALSE(out.gyro.faultDetected);
        for (size_t i = 0U; i < kMimuCount; ++i) {
            EXPECT_TRUE(out.gyro.imuValid.at(i));
        }
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(out.gyro.average[i], fullAvg[i], 1e-6);
        }
    }

    // Call 3: persistence limit reached, fault triggers — outlier excluded
    auto out = alg.update(imuOmegas_BN_B, imuAccels_B);
    EXPECT_TRUE(out.gyro.faultDetected);
    EXPECT_TRUE(out.gyro.imuValid.at(0));
    EXPECT_FALSE(out.gyro.imuValid.at(1));
    EXPECT_TRUE(out.gyro.imuValid.at(2));
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.gyro.average[i], faultedAvg[i], 1e-6);
    }

    // Call 4: IMU 2 recovers to baseRate, IMU 3 becomes mild outlier —
    // persistence resets for IMU 2, increments for IMU 3, no fault yet
    imuOmegas_BN_B.at(0) = baseRate;
    imuOmegas_BN_B.at(1) = baseRate;
    imuOmegas_BN_B.at(2) = mildOutlierRate;

    Eigen::Vector3f call4Avg = (imuOmegas_BN_B.at(0) + imuOmegas_BN_B.at(1) + imuOmegas_BN_B.at(2)) / 3.0F;

    out = alg.update(imuOmegas_BN_B, imuAccels_B);
    EXPECT_FALSE(out.gyro.faultDetected);
    for (size_t i = 0U; i < kMimuCount; ++i) {
        EXPECT_TRUE(out.gyro.imuValid.at(i));
    }
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.gyro.average[i], call4Avg[i], 1e-6);
    }

    // Call 5: IMU 3 still the outlier, persistence count increments but no fault yet
    out = alg.update(imuOmegas_BN_B, imuAccels_B);
    EXPECT_FALSE(out.gyro.faultDetected);
    for (size_t i = 0U; i < kMimuCount; ++i) {
        EXPECT_TRUE(out.gyro.imuValid.at(i));
    }
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.gyro.average[i], call4Avg[i], 1e-6);
    }

    // Call 6: IMU 3 persistence limit reached, fault triggers — IMU 3 excluded
    Eigen::Vector3f call6FaultedAvg = (imuOmegas_BN_B.at(0) + imuOmegas_BN_B.at(1)) / 2.0F;

    out = alg.update(imuOmegas_BN_B, imuAccels_B);
    EXPECT_TRUE(out.gyro.faultDetected);
    EXPECT_TRUE(out.gyro.imuValid.at(0));
    EXPECT_TRUE(out.gyro.imuValid.at(1));
    EXPECT_FALSE(out.gyro.imuValid.at(2));
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.gyro.average[i], call6FaultedAvg[i], 1e-6);
    }
}

TEST(MimuMajorityVoteTest, ReInitializeClearsPersistence) {
    MimuMajorityVoteAlgorithm alg{MimuMajorityVoteConfig::create(0.05F, 3U, 1.0F, 1U)};

    Eigen::Vector3f baseRate(-0.1F, 0.25F, 0.3F);
    Eigen::Vector3f outlierRate = baseRate + Eigen::Vector3f(2.0F, 2.0F, 2.0F);

    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B{};
    imuOmegas_BN_B.at(0) = baseRate;
    imuOmegas_BN_B.at(1) = outlierRate;
    imuOmegas_BN_B.at(2) = baseRate;
    std::array<Eigen::Vector3f, kMimuCount> imuAccels_B{};  // all zero: no accel fault

    Eigen::Vector3f fullAvg = (imuOmegas_BN_B.at(0) + imuOmegas_BN_B.at(1) + imuOmegas_BN_B.at(2)) / 3.0F;
    Eigen::Vector3f faultedAvg = (imuOmegas_BN_B.at(0) + imuOmegas_BN_B.at(2)) / 2.0F;

    // Call twice to build up persistence (limit is 3, so no fault yet)
    for (uint32_t call = 0U; call < 2U; ++call) {
        auto out = alg.update(imuOmegas_BN_B, imuAccels_B);
        EXPECT_FALSE(out.gyro.faultDetected);
        for (size_t i = 0U; i < kMimuCount; ++i) {
            EXPECT_TRUE(out.gyro.imuValid.at(i));
        }
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(out.gyro.average[i], fullAvg[i], 1e-6);
        }
    }

    // reInitialize clears persistence counters
    alg.reInitialize();

    // Same outlier inputs — counter starts from zero again, no fault
    for (uint32_t call = 0U; call < 2U; ++call) {
        auto out = alg.update(imuOmegas_BN_B, imuAccels_B);
        EXPECT_FALSE(out.gyro.faultDetected);
        for (size_t i = 0U; i < kMimuCount; ++i) {
            EXPECT_TRUE(out.gyro.imuValid.at(i));
        }
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(out.gyro.average[i], fullAvg[i], 1e-6);
        }
    }

    // Third call after reInitialize — now fault triggers
    auto out = alg.update(imuOmegas_BN_B, imuAccels_B);
    EXPECT_TRUE(out.gyro.faultDetected);
    EXPECT_TRUE(out.gyro.imuValid.at(0));
    EXPECT_FALSE(out.gyro.imuValid.at(1));
    EXPECT_TRUE(out.gyro.imuValid.at(2));
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.gyro.average[i], faultedAvg[i], 1e-6);
    }
}

TEST(MimuMajorityVoteTest, SetupTest) {
    // Config validation rejects a zero/negative threshold and a zero persistence limit, for both
    // the gyro and accel parameters.
    EXPECT_THROW((void)MimuMajorityVoteConfig::create(0.0F, 1U, 1.0F, 1U), fsw::invalid_argument);
    EXPECT_THROW((void)MimuMajorityVoteConfig::create(-0.1F, 1U, 1.0F, 1U), fsw::invalid_argument);
    EXPECT_THROW((void)MimuMajorityVoteConfig::create(1.0F, 0U, 1.0F, 1U), fsw::invalid_argument);
    EXPECT_THROW((void)MimuMajorityVoteConfig::create(1.0F, 1U, 0.0F, 1U), fsw::invalid_argument);
    EXPECT_THROW((void)MimuMajorityVoteConfig::create(1.0F, 1U, -0.1F, 1U), fsw::invalid_argument);
    EXPECT_THROW((void)MimuMajorityVoteConfig::create(1.0F, 1U, 1.0F, 0U), fsw::invalid_argument);

    // A valid config exposes exactly the supplied parameters.
    const float omegaThreshold = 0.5F;
    const float accelThreshold = 0.25F;
    const MimuMajorityVoteConfig cfg = MimuMajorityVoteConfig::create(omegaThreshold, 2U, accelThreshold, 3U);
    EXPECT_NEAR(cfg.getOmegaThreshold(), omegaThreshold, 1e-6);
    EXPECT_EQ(cfg.getGyroFaultPersistenceLimit(), 2U);
    EXPECT_NEAR(cfg.getAccelThreshold(), accelThreshold, 1e-6);
    EXPECT_EQ(cfg.getAccelFaultPersistenceLimit(), 3U);
}
