#include "architecture/testUtilities/eigenFuzzDomains.hpp"
#include "mimuMajorityVoteTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

namespace {

constexpr float kMaxAngVel = 1e3F;
constexpr float kMinThreshold = 1e-2F;
constexpr float kMaxThreshold = 1e3F;
constexpr float kCompTol = 1e-3F;

void propertyOutputAlwaysFinite(const Eigen::Vector3f& angVel1,
                                const Eigen::Vector3f& angVel2,
                                const Eigen::Vector3f& angVel3,
                                float omegaThreshold) {
    MimuMajorityVoteAlgorithm alg{};
    alg.setOmegaThreshold(omegaThreshold);

    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B{};
    imuOmegas_BN_B.at(0) = angVel1;
    imuOmegas_BN_B.at(1) = angVel2;
    imuOmegas_BN_B.at(2) = angVel3;

    auto const out = alg.update(imuOmegas_BN_B);

    for (int i = 0; i < 3; ++i) {
        ASSERT_TRUE(std::isfinite(out.avgOmega_BN_B[i]));
    }
    for (size_t i = 0U; i < kMimuCount; ++i) {
        ASSERT_GE(out.omegaDifferencesMag.at(i), 0.0F);
        ASSERT_TRUE(std::isfinite(out.omegaDifferencesMag.at(i)));
    }
}

void propertyFaultAndValidConsistency(const Eigen::Vector3f& angVel1,
                                      const Eigen::Vector3f& angVel2,
                                      const Eigen::Vector3f& angVel3,
                                      float omegaThreshold) {
    MimuMajorityVoteAlgorithm alg{};
    alg.setOmegaThreshold(omegaThreshold);

    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B{};
    imuOmegas_BN_B.at(0) = angVel1;
    imuOmegas_BN_B.at(1) = angVel2;
    imuOmegas_BN_B.at(2) = angVel3;

    auto const out = alg.update(imuOmegas_BN_B);

    size_t invalidCount = 0U;
    for (size_t i = 0U; i < kMimuCount; ++i) {
        if (!out.validImus.at(i)) {
            ++invalidCount;
        }
    }

    // Invalid count must be 0 (no fault) or 1 (single outlier)
    ASSERT_TRUE(invalidCount == 0U || invalidCount == 1U);
    // faultDetected must be consistent with validImus
    ASSERT_EQ(out.faultDetected, invalidCount > 0U);
}

void expectNoFaultAverage(const MimuMajorityVoteOutput& out,
                          const std::array<Eigen::Vector3f, kMimuCount>& imuOmegas_BN_B,
                          size_t numImus) {
    Eigen::Vector3f expectedAvg = Eigen::Vector3f::Zero();
    for (size_t i = 0U; i < numImus; ++i) {
        expectedAvg += imuOmegas_BN_B.at(i);
    }
    expectedAvg /= static_cast<float>(numImus);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.avgOmega_BN_B[i], expectedAvg[i], kCompTol);
    }
}

void expectSingleFaultAverage(const MimuMajorityVoteOutput& out,
                              const std::array<Eigen::Vector3f, kMimuCount>& imuOmegas_BN_B,
                              size_t numImus) {
    size_t faultedIndex = numImus;  // sentinel
    for (size_t i = 0U; i < numImus; ++i) {
        if (!out.validImus.at(i)) {
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
        EXPECT_NEAR(out.avgOmega_BN_B[i], expectedAvg[i], kCompTol);
    }
}

void propertyAverageIsCorrect(const Eigen::Vector3f& angVel1,
                              const Eigen::Vector3f& angVel2,
                              const Eigen::Vector3f& angVel3,
                              float omegaThreshold) {
    MimuMajorityVoteAlgorithm alg{};
    alg.setOmegaThreshold(omegaThreshold);

    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B{};
    imuOmegas_BN_B.at(0) = angVel1;
    imuOmegas_BN_B.at(1) = angVel2;
    imuOmegas_BN_B.at(2) = angVel3;

    auto const out = alg.update(imuOmegas_BN_B);

    size_t invalidCount = 0U;
    for (size_t i = 0U; i < kMimuCount; ++i) {
        if (!out.validImus.at(i)) {
            ++invalidCount;
        }
    }

    if (invalidCount == 0U) {
        expectNoFaultAverage(out, imuOmegas_BN_B, kMimuCount);
    } else if (invalidCount == 1U) {
        // Single outlier excluded: average of the remaining sensors
        expectSingleFaultAverage(out, imuOmegas_BN_B, kMimuCount);
    }
}

void propertyIdenticalIMUsNoFault(const Eigen::Vector3f& angVel, float omegaThreshold) {
    MimuMajorityVoteAlgorithm alg{};
    alg.setOmegaThreshold(omegaThreshold);

    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B{};
    for (size_t i = 0U; i < kMimuCount; ++i) {
        imuOmegas_BN_B.at(i) = angVel;
    }

    auto const out = alg.update(imuOmegas_BN_B);

    ASSERT_FALSE(out.faultDetected);
    for (size_t i = 0U; i < kMimuCount; ++i) {
        ASSERT_TRUE(out.validImus.at(i));
        ASSERT_NEAR(out.omegaDifferencesMag.at(i), 0.0F, kCompTol);
    }
    for (int i = 0; i < 3; ++i) {
        ASSERT_NEAR(out.avgOmega_BN_B[i], angVel[i], kCompTol);
    }
}

void propertyClearSingleOutlier(const Eigen::Vector3f& baseAngVel, size_t outlierIndex, float omegaThreshold) {
    constexpr float kOutlierFactor = 10.0F;

    MimuMajorityVoteAlgorithm alg{};
    alg.setOmegaThreshold(omegaThreshold);

    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B{};
    for (size_t i = 0U; i < kMimuCount; ++i) {
        imuOmegas_BN_B.at(i) = baseAngVel;
    }
    // Make outlier clearly separable: kOutlierFactor × threshold beyond the base pair
    imuOmegas_BN_B.at(outlierIndex) = baseAngVel + kOutlierFactor * omegaThreshold * Eigen::Vector3f::Ones();

    auto const out = alg.update(imuOmegas_BN_B);

    ASSERT_TRUE(out.faultDetected);
    ASSERT_FALSE(out.validImus.at(outlierIndex));
    for (size_t i = 0U; i < kMimuCount; ++i) {
        if (i != outlierIndex) {
            ASSERT_TRUE(out.validImus.at(i));
        }
    }
}

void propertyCyclicPermutationInvariant(const Eigen::Vector3f& angVel1,
                                        const Eigen::Vector3f& angVel2,
                                        const Eigen::Vector3f& angVel3,
                                        float omegaThreshold) {
    MimuMajorityVoteAlgorithm alg{};
    alg.setOmegaThreshold(omegaThreshold);

    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B{};
    imuOmegas_BN_B.at(0) = angVel1;
    imuOmegas_BN_B.at(1) = angVel2;
    imuOmegas_BN_B.at(2) = angVel3;

    auto const out0 = alg.update(imuOmegas_BN_B);

    // Cyclic permutation: [v2, v3, v1]
    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B1{};
    imuOmegas_BN_B1.at(0) = imuOmegas_BN_B.at(1);
    imuOmegas_BN_B1.at(1) = imuOmegas_BN_B.at(2);
    imuOmegas_BN_B1.at(2) = imuOmegas_BN_B.at(0);

    auto const out1 = alg.update(imuOmegas_BN_B1);

    ASSERT_EQ(out0.faultDetected, out1.faultDetected);

    size_t invalidCount0 = 0U;
    size_t invalidCount1 = 0U;
    for (size_t i = 0U; i < kMimuCount; ++i) {
        if (!out0.validImus.at(i)) {
            ++invalidCount0;
        }
        if (!out1.validImus.at(i)) {
            ++invalidCount1;
        }
    }
    ASSERT_EQ(invalidCount0, invalidCount1);
}

}  // namespace

FUZZ_TEST(MimuMajorityVoteAlgorithmFuzz, regressionTestMimuMajorityVote)
    .WithDomains(fuzztest::InRange(1e-6F, 1e3F),              // omegaThreshold
                 fuzztest::InRange<uint32_t>(1U, 200U),       // persistenceLimit
                 fuzztest::InRange<uint32_t>(1U, 100U),       // algCallCount
                 xmera::fuzz::Vector3fInRange(-1e3F, 1e3F),   // angVel1
                 xmera::fuzz::Vector3fInRange(-1e3F, 1e3F),   // angVel2
                 xmera::fuzz::Vector3fInRange(-1e3F, 1e3F));  // angVel3

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
