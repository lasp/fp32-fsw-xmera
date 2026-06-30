#include "mimuMajorityVoteTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

namespace {

constexpr float kMaxAngVel = 1e3F;
constexpr float kMinThreshold = 1e-2F;
constexpr float kMaxThreshold = 1e3F;
constexpr float kCompTol = 1e-3F;

// The property functions below exercise the gyro vote; a zero accelerometer input (with a fixed,
// valid accel threshold) keeps the accel vote benign so it does not affect the gyro assertions.
// The regressionTestMimuMajorityVote fuzz target exercises both votes against the reference.

void propertyOutputAlwaysFinite(const Eigen::Vector3f& angVel1,
                                const Eigen::Vector3f& angVel2,
                                const Eigen::Vector3f& angVel3,
                                float omegaThreshold) {
    MimuMajorityVoteAlgorithm alg{MimuMajorityVoteConfig::create(omegaThreshold, 1U, kMinThreshold, 1U)};

    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B{angVel1, angVel2, angVel3};
    std::array<Eigen::Vector3f, kMimuCount> imuAccels_B{};

    auto const out = alg.update(imuOmegas_BN_B, imuAccels_B);

    for (int i = 0; i < 3; ++i) {
        ASSERT_TRUE(std::isfinite(out.gyro.average[i]));
    }
    for (size_t i = 0U; i < kMimuCount; ++i) {
        ASSERT_GE(out.gyro.imuDifferenceMag.at(i), 0.0F);
        ASSERT_TRUE(std::isfinite(out.gyro.imuDifferenceMag.at(i)));
    }
}

void propertyFaultAndValidConsistency(const Eigen::Vector3f& angVel1,
                                      const Eigen::Vector3f& angVel2,
                                      const Eigen::Vector3f& angVel3,
                                      float omegaThreshold) {
    MimuMajorityVoteAlgorithm alg{MimuMajorityVoteConfig::create(omegaThreshold, 1U, kMinThreshold, 1U)};

    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B{angVel1, angVel2, angVel3};
    std::array<Eigen::Vector3f, kMimuCount> imuAccels_B{};

    auto const out = alg.update(imuOmegas_BN_B, imuAccels_B);

    size_t invalidCount = 0U;
    for (size_t i = 0U; i < kMimuCount; ++i) {
        if (!out.gyro.imuValid.at(i)) {
            ++invalidCount;
        }
    }

    // Invalid count must be 0 (no fault) or 1 (single outlier)
    ASSERT_TRUE(invalidCount == 0U || invalidCount == 1U);
    // gyro.faultDetected must be consistent with gyro.imuValid
    ASSERT_EQ(out.gyro.faultDetected, invalidCount > 0U);
}

void expectNoFaultAverage(const Eigen::Vector3f& average,
                          const std::array<Eigen::Vector3f, kMimuCount>& imuOmegas_BN_B,
                          size_t numImus) {
    Eigen::Vector3f expectedAvg = Eigen::Vector3f::Zero();
    for (size_t i = 0U; i < numImus; ++i) {
        expectedAvg += imuOmegas_BN_B.at(i);
    }
    expectedAvg /= static_cast<float>(numImus);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(average[i], expectedAvg[i], kCompTol);
    }
}

void expectSingleFaultAverage(const Eigen::Vector3f& average,
                              const std::array<bool, kMimuCount>& imuValid,
                              const std::array<Eigen::Vector3f, kMimuCount>& imuOmegas_BN_B,
                              size_t numImus) {
    size_t faultedIndex = numImus;  // sentinel
    for (size_t i = 0U; i < numImus; ++i) {
        if (!imuValid.at(i)) {
            faultedIndex = i;
            break;
        }
    }
    ASSERT_LT(faultedIndex, numImus);

    Eigen::Vector3f expectedAvg = Eigen::Vector3f::Zero();
    size_t count = 0U;
    for (size_t i = 0U; i < numImus; ++i) {
        if (i != faultedIndex) {
            expectedAvg += imuOmegas_BN_B.at(i);
            ++count;
        }
    }
    expectedAvg /= static_cast<float>(count);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(average[i], expectedAvg[i], kCompTol);
    }
}

void propertyAverageIsCorrect(const Eigen::Vector3f& angVel1,
                              const Eigen::Vector3f& angVel2,
                              const Eigen::Vector3f& angVel3,
                              float omegaThreshold) {
    MimuMajorityVoteAlgorithm alg{MimuMajorityVoteConfig::create(omegaThreshold, 1U, kMinThreshold, 1U)};

    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B{angVel1, angVel2, angVel3};
    std::array<Eigen::Vector3f, kMimuCount> imuAccels_B{};

    auto const out = alg.update(imuOmegas_BN_B, imuAccels_B);

    size_t invalidCount = 0U;
    for (size_t i = 0U; i < kMimuCount; ++i) {
        if (!out.gyro.imuValid.at(i)) {
            ++invalidCount;
        }
    }

    if (invalidCount == 0U) {
        expectNoFaultAverage(out.gyro.average, imuOmegas_BN_B, kMimuCount);
    } else if (invalidCount == 1U) {
        // Single outlier excluded: average of the remaining sensors
        expectSingleFaultAverage(out.gyro.average, out.gyro.imuValid, imuOmegas_BN_B, kMimuCount);
    }
}

void propertyIdenticalIMUsNoFault(const Eigen::Vector3f& angVel, float omegaThreshold) {
    MimuMajorityVoteAlgorithm alg{MimuMajorityVoteConfig::create(omegaThreshold, 1U, kMinThreshold, 1U)};

    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B{};
    for (size_t i = 0U; i < kMimuCount; ++i) {
        imuOmegas_BN_B.at(i) = angVel;
    }
    std::array<Eigen::Vector3f, kMimuCount> imuAccels_B{};

    auto const out = alg.update(imuOmegas_BN_B, imuAccels_B);

    ASSERT_FALSE(out.gyro.faultDetected);
    for (size_t i = 0U; i < kMimuCount; ++i) {
        ASSERT_TRUE(out.gyro.imuValid.at(i));
        ASSERT_NEAR(out.gyro.imuDifferenceMag.at(i), 0.0F, kCompTol);
    }
    for (int i = 0; i < 3; ++i) {
        ASSERT_NEAR(out.gyro.average[i], angVel[i], kCompTol);
    }
}

void propertyClearSingleOutlier(const Eigen::Vector3f& baseAngVel, size_t outlierIndex, float omegaThreshold) {
    constexpr float kOutlierFactor = 10.0F;

    MimuMajorityVoteAlgorithm alg{MimuMajorityVoteConfig::create(omegaThreshold, 1U, kMinThreshold, 1U)};

    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B{};
    for (size_t i = 0U; i < kMimuCount; ++i) {
        imuOmegas_BN_B.at(i) = baseAngVel;
    }
    // Make outlier clearly separable: kOutlierFactor × threshold beyond the base pair
    imuOmegas_BN_B.at(outlierIndex) = baseAngVel + kOutlierFactor * omegaThreshold * Eigen::Vector3f::Ones();
    std::array<Eigen::Vector3f, kMimuCount> imuAccels_B{};

    auto const out = alg.update(imuOmegas_BN_B, imuAccels_B);

    ASSERT_TRUE(out.gyro.faultDetected);
    ASSERT_FALSE(out.gyro.imuValid.at(outlierIndex));
    for (size_t i = 0U; i < kMimuCount; ++i) {
        if (i != outlierIndex) {
            ASSERT_TRUE(out.gyro.imuValid.at(i));
        }
    }
}

void propertyCyclicPermutationInvariant(const Eigen::Vector3f& angVel1,
                                        const Eigen::Vector3f& angVel2,
                                        const Eigen::Vector3f& angVel3,
                                        float omegaThreshold) {
    MimuMajorityVoteAlgorithm alg{MimuMajorityVoteConfig::create(omegaThreshold, 1U, kMinThreshold, 1U)};

    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B{angVel1, angVel2, angVel3};
    std::array<Eigen::Vector3f, kMimuCount> imuAccels_B{};

    auto const out0 = alg.update(imuOmegas_BN_B, imuAccels_B);

    // Cyclic permutation: [v2, v3, v1]
    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B1{};
    imuOmegas_BN_B1.at(0) = imuOmegas_BN_B.at(1);
    imuOmegas_BN_B1.at(1) = imuOmegas_BN_B.at(2);
    imuOmegas_BN_B1.at(2) = imuOmegas_BN_B.at(0);

    auto const out1 = alg.update(imuOmegas_BN_B1, imuAccels_B);

    ASSERT_EQ(out0.gyro.faultDetected, out1.gyro.faultDetected);

    size_t invalidCount0 = 0U;
    size_t invalidCount1 = 0U;
    for (size_t i = 0U; i < kMimuCount; ++i) {
        if (!out0.gyro.imuValid.at(i)) {
            ++invalidCount0;
        }
        if (!out1.gyro.imuValid.at(i)) {
            ++invalidCount1;
        }
    }
    ASSERT_EQ(invalidCount0, invalidCount1);
}

}  // namespace

FUZZ_TEST(MimuMajorityVoteAlgorithmFuzz, regressionTestMimuMajorityVote)
    .WithDomains(fuzztest::InRange(1e-6F, 1e3F),              // omegaThreshold
                 fuzztest::InRange<uint32_t>(1U, 200U),       // gyroFaultPersistenceLimit
                 fuzztest::InRange(1e-6F, 1e3F),              // accelThreshold
                 fuzztest::InRange<uint32_t>(1U, 200U),       // accelFaultPersistenceLimit
                 fuzztest::InRange<uint32_t>(1U, 100U),       // algCallCount
                 xmera::fuzz::Vector3fInRange(-1e3F, 1e3F),   // angVel1
                 xmera::fuzz::Vector3fInRange(-1e3F, 1e3F),   // angVel2
                 xmera::fuzz::Vector3fInRange(-1e3F, 1e3F),   // angVel3
                 xmera::fuzz::Vector3fInRange(-1e3F, 1e3F),   // accel1
                 xmera::fuzz::Vector3fInRange(-1e3F, 1e3F),   // accel2
                 xmera::fuzz::Vector3fInRange(-1e3F, 1e3F));  // accel3

FUZZ_TEST(MimuMajorityVoteAlgorithmFuzz, propertyOutputAlwaysFinite)
    .WithDomains(xmera::fuzz::Vector3fInRange(-kMaxAngVel, kMaxAngVel),  // angVel1
                 xmera::fuzz::Vector3fInRange(-kMaxAngVel, kMaxAngVel),  // angVel2
                 xmera::fuzz::Vector3fInRange(-kMaxAngVel, kMaxAngVel),  // angVel3
                 fuzztest::InRange(kMinThreshold, kMaxThreshold));       // omegaThreshold

FUZZ_TEST(MimuMajorityVoteAlgorithmFuzz, propertyFaultAndValidConsistency)
    .WithDomains(xmera::fuzz::Vector3fInRange(-kMaxAngVel, kMaxAngVel),  // angVel1
                 xmera::fuzz::Vector3fInRange(-kMaxAngVel, kMaxAngVel),  // angVel2
                 xmera::fuzz::Vector3fInRange(-kMaxAngVel, kMaxAngVel),  // angVel3
                 fuzztest::InRange(kMinThreshold, kMaxThreshold));       // omegaThreshold

FUZZ_TEST(MimuMajorityVoteAlgorithmFuzz, propertyAverageIsCorrect)
    .WithDomains(xmera::fuzz::Vector3fInRange(-kMaxAngVel, kMaxAngVel),  // angVel1
                 xmera::fuzz::Vector3fInRange(-kMaxAngVel, kMaxAngVel),  // angVel2
                 xmera::fuzz::Vector3fInRange(-kMaxAngVel, kMaxAngVel),  // angVel3
                 fuzztest::InRange(kMinThreshold, kMaxThreshold));       // omegaThreshold

FUZZ_TEST(MimuMajorityVoteAlgorithmFuzz, propertyIdenticalIMUsNoFault)
    .WithDomains(xmera::fuzz::Vector3fInRange(-kMaxAngVel, kMaxAngVel),  // angVel
                 fuzztest::InRange(kMinThreshold, kMaxThreshold));       // omegaThreshold

FUZZ_TEST(MimuMajorityVoteAlgorithmFuzz, propertyClearSingleOutlier)
    .WithDomains(xmera::fuzz::Vector3fInRange(-kMaxAngVel, kMaxAngVel),  // baseAngVel
                 fuzztest::InRange<size_t>(0, 2),                        // outlierIndex
                 fuzztest::InRange(kMinThreshold, kMaxThreshold));       // omegaThreshold

FUZZ_TEST(MimuMajorityVoteAlgorithmFuzz, propertyCyclicPermutationInvariant)
    .WithDomains(xmera::fuzz::Vector3fInRange(-kMaxAngVel, kMaxAngVel),  // angVel1
                 xmera::fuzz::Vector3fInRange(-kMaxAngVel, kMaxAngVel),  // angVel2
                 xmera::fuzz::Vector3fInRange(-kMaxAngVel, kMaxAngVel),  // angVel3
                 fuzztest::InRange(kMinThreshold, kMaxThreshold));       // omegaThreshold
