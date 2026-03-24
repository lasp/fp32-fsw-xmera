#include "bodyRateMiscompareTestHelpers.hpp"
#include "freestandingInvalidArgument.h"
#include <gtest/gtest.h>
#include <array>

static void runRegressionCase(float threshold,
                              uint32_t faultPersistenceLimit,
                              const std::array<float, 3>& imuVec,
                              const std::array<float, 3>& stVec) {
    BodyRateMiscompareAlgorithm alg{};
    alg.setBodyRateThreshold(threshold);
    alg.setFaultPersistenceLimit(faultPersistenceLimit);

    const Eigen::Vector3f imu = toEigenVector(imuVec);
    const Eigen::Vector3f st = toEigenVector(stVec);

    bool refFaultDetected = false;
    uint32_t refFaultPersistenceCount = 0;

    int numSteps = 5;
    for (int step = 0; step < numSteps; ++step) {
        BodyRateMiscompareOutput out{};
        BodyRateMiscompareOutput ref{};

        EXPECT_NO_THROW(out = alg.update(imu, st));
        EXPECT_NO_THROW(
            ref = referenceUpdate(threshold, faultPersistenceLimit, refFaultDetected, refFaultPersistenceCount, imu, st));

        for (int i = 0; i < 3; ++i) {
            EXPECT_FLOAT_EQ(out.omega_BN_B[i], ref.omega_BN_B[i]);
            EXPECT_TRUE(std::isfinite(out.omega_BN_B[i]));
        }
        EXPECT_EQ(out.bodyRateFaultDetected, ref.bodyRateFaultDetected);

        if (out.bodyRateFaultDetected) {
            for (int i = 0; i < 3; ++i) {
                EXPECT_FLOAT_EQ(out.omega_BN_B[i], imu[i]);
            }
        } else {
            for (int i = 0; i < 3; ++i) {
                EXPECT_FLOAT_EQ(out.omega_BN_B[i], st[i]);
            }
        }
    }
}

TEST(BodyRateMiscompareTest, RegressionTestNoFault) {
    runRegressionCase(1.0F, 1, {0.1F, -0.2F, 0.3F}, {0.2F, -0.1F, 0.1F});
}

TEST(BodyRateMiscompareTest, RegressionTestFault) {
    runRegressionCase(0.5F, 1, {0.1F, -0.2F, 0.3F}, {1.2F, -0.1F, 0.1F});
}

TEST(BodyRateMiscompareTest, RegressionTestFaultWithPersistence) {
    runRegressionCase(0.5F, 3, {0.1F, -0.2F, 0.3F}, {1.2F, -0.1F, 0.1F});
}

TEST(BodyRateMiscompareTest, RegressionTestNoFaultWithPersistence) {
    runRegressionCase(1.0F, 3, {0.1F, -0.2F, 0.3F}, {0.2F, -0.1F, 0.1F});
}

TEST(BodyRateMiscompareTest, SetupTest) {
    BodyRateMiscompareAlgorithm alg{};
    alg.setBodyRateThreshold(0.25F);
    EXPECT_NEAR(alg.getBodyRateThreshold(), 0.25F, 1e-6);

    alg.setFaultPersistenceLimit(5);
    EXPECT_EQ(alg.getFaultPersistenceLimit(), 5u);

    EXPECT_THROW(alg.setBodyRateThreshold(0), fs::invalid_argument);
    EXPECT_THROW(alg.setBodyRateThreshold(-0.1), fs::invalid_argument);

    EXPECT_THROW(alg.setFaultPersistenceLimit(0), fs::invalid_argument);
}
