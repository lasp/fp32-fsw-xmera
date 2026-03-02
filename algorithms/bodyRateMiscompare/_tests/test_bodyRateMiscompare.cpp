#include "bodyRateMiscompareTestHelpers.hpp"
#include <gtest/gtest.h>
#include <array>

static void runRegressionCase(float threshold, const std::array<float, 3>& imuVec, const std::array<float, 3>& stVec) {
    BodyRateMiscompareAlgorithm alg{};
    alg.setBodyRateThreshold(threshold);

    const Eigen::Vector3f imu = toEigenVector(imuVec);
    const Eigen::Vector3f st = toEigenVector(stVec);

    BodyRateMiscompareOutput out{};
    BodyRateMiscompareOutput ref{};

    EXPECT_NO_THROW(out = alg.update(imu, st));
    EXPECT_NO_THROW(ref = referenceUpdate(threshold, imu, st));

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

TEST(BodyRateMiscompareTest, RegressionTestNoFault) {
    runRegressionCase(1.0F, {0.1F, -0.2F, 0.3F}, {0.2F, -0.1F, 0.1F});
}

TEST(BodyRateMiscompareTest, RegressionTestFault) { runRegressionCase(0.5F, {0.1F, -0.2F, 0.3F}, {1.2F, -0.1F, 0.1F}); }

TEST(BodyRateMiscompareTest, SetupTest) {
    BodyRateMiscompareAlgorithm alg{};
    alg.setBodyRateThreshold(0.25F);
    EXPECT_NEAR(alg.getBodyRateThreshold(), 0.25F, 1e-6);
}
