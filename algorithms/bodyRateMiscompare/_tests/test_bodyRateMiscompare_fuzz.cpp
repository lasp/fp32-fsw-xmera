#include "bodyRateMiscompareTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

inline void fuzzPropertyBodyRateMiscompare(float threshold,
                                           const std::array<float, 3>& imuVec,
                                           const std::array<float, 3>& stVec) {
    BodyRateMiscompareAlgorithm alg{};
    alg.setBodyRateThreshold(threshold);

    const Eigen::Vector3f imu = toEigenVector(imuVec);
    const Eigen::Vector3f st = toEigenVector(stVec);
    const Eigen::Vector3f diff = st - imu;

    BodyRateMiscompareOutput out{};
    EXPECT_NO_THROW(out = alg.update(imu, st));

    const bool expectedFault = diff.norm() > threshold;
    EXPECT_EQ(out.bodyRateFaultDetected, expectedFault);

    const Eigen::Vector3f expectedOmega = expectedFault ? imu : st;
    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(out.omega_BN_B[i], expectedOmega[i]);
        EXPECT_TRUE(std::isfinite(out.omega_BN_B[i]));
    }
}

FUZZ_TEST(BodyRateMiscompareAlgorithmFuzz, fuzzPropertyBodyRateMiscompare)
    .WithDomains(fuzztest::InRange(1e-6F, 1e6F),
                 fuzztest::ArrayOf<3>(fuzztest::InRange(-1e6F, 1e6F)),
                 fuzztest::ArrayOf<3>(fuzztest::InRange(-1e6F, 1e6F)));
