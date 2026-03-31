#include "mimuMajorityVoteTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

namespace {

constexpr float kMaxAngVel = 1e3F;
constexpr float kMinThreshold = 1e-2F;
constexpr float kMaxThreshold = 1e3F;
constexpr float kCompTol = 1e-3F;

void propertyOutputAlwaysFinite(const std::vector<float>& angVel1,
                                const std::vector<float>& angVel2,
                                const std::vector<float>& angVel3,
                                float omegaThreshold) {
    MimuMajorityVoteAlgorithm alg{};
    alg.setOmegaThreshold(omegaThreshold);

    std::array<MimuInput, kMimuCount> imuInputs{};
    imuInputs.at(0).angVelBody = Eigen::Map<const Eigen::Vector3f>(angVel1.data());
    imuInputs.at(1).angVelBody = Eigen::Map<const Eigen::Vector3f>(angVel2.data());
    imuInputs.at(2).angVelBody = Eigen::Map<const Eigen::Vector3f>(angVel3.data());

    auto const out = alg.update(imuInputs);

    for (int i = 0; i < 3; ++i) {
        ASSERT_TRUE(std::isfinite(out.avgAngVelBody[i]));
    }
    for (size_t i = 0U; i < kMimuCount; ++i) {
        ASSERT_GE(out.omegaDifferencesMag.at(i), 0.0F);
        ASSERT_TRUE(std::isfinite(out.omegaDifferencesMag.at(i)));
    }
}

void propertyFaultAndValidConsistency(const std::vector<float>& angVel1,
                                      const std::vector<float>& angVel2,
                                      const std::vector<float>& angVel3,
                                      float omegaThreshold) {
    MimuMajorityVoteAlgorithm alg{};
    alg.setOmegaThreshold(omegaThreshold);

    std::array<MimuInput, kMimuCount> imuInputs{};
    imuInputs.at(0).angVelBody = Eigen::Map<const Eigen::Vector3f>(angVel1.data());
    imuInputs.at(1).angVelBody = Eigen::Map<const Eigen::Vector3f>(angVel2.data());
    imuInputs.at(2).angVelBody = Eigen::Map<const Eigen::Vector3f>(angVel3.data());

    auto const out = alg.update(imuInputs);

    size_t invalidCount = 0U;
    for (size_t i = 0U; i < kMimuCount; ++i) {
        if (!out.validImus.at(i)) {
            ++invalidCount;
        }
    }

    // Invalid count must be 0 (no fault), 1 (single outlier), or 3 (all invalid)
    ASSERT_TRUE(invalidCount == 0U || invalidCount == 1U || invalidCount == kMimuCount);
    // faultDetected must be consistent with validImus
    ASSERT_EQ(out.faultDetected, invalidCount > 0U);
}

void expectNoFaultAverage(const MimuMajorityVoteOutput& out,
                          const std::array<MimuInput, kMimuCount>& imuInputs,
                          size_t numImus) {
    Eigen::Vector3f expectedAvg = Eigen::Vector3f::Zero();
    for (size_t i = 0U; i < numImus; ++i) {
        expectedAvg += imuInputs.at(i).angVelBody;
    }
    expectedAvg /= static_cast<float>(numImus);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.avgAngVelBody[i], expectedAvg[i], kCompTol);
    }
}

void expectSingleFaultAverage(const MimuMajorityVoteOutput& out,
                              const std::array<MimuInput, kMimuCount>& imuInputs,
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
            expectedAvg += imuInputs.at(i).angVelBody;
            ++count;
        }
    }
    expectedAvg /= static_cast<float>(count);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.avgAngVelBody[i], expectedAvg[i], kCompTol);
    }
}

void propertyAverageIsCorrect(const std::vector<float>& angVel1,
                              const std::vector<float>& angVel2,
                              const std::vector<float>& angVel3,
                              float omegaThreshold) {
    MimuMajorityVoteAlgorithm alg{};
    alg.setOmegaThreshold(omegaThreshold);

    std::array<MimuInput, kMimuCount> imuInputs{};
    imuInputs.at(0).angVelBody = Eigen::Map<const Eigen::Vector3f>(angVel1.data());
    imuInputs.at(1).angVelBody = Eigen::Map<const Eigen::Vector3f>(angVel2.data());
    imuInputs.at(2).angVelBody = Eigen::Map<const Eigen::Vector3f>(angVel3.data());

    auto const out = alg.update(imuInputs);

    size_t invalidCount = 0U;
    for (size_t i = 0U; i < kMimuCount; ++i) {
        if (!out.validImus.at(i)) {
            ++invalidCount;
        }
    }

    if (invalidCount == 0U) {
        expectNoFaultAverage(out, imuInputs, kMimuCount);
    } else if (invalidCount == 1U) {
        // Single outlier excluded: average of the remaining sensors
        expectSingleFaultAverage(out, imuInputs, kMimuCount);
    }
    // All-invalid: average is ω̄₂ (Stage 1 outlier excluded); correctness covered by regression test
}

void propertyIdenticalIMUsNoFault(const std::vector<float>& angVel, float omegaThreshold) {
    MimuMajorityVoteAlgorithm alg{};
    alg.setOmegaThreshold(omegaThreshold);

    Eigen::Vector3f const v = Eigen::Map<const Eigen::Vector3f>(angVel.data());
    std::array<MimuInput, kMimuCount> imuInputs{};
    for (size_t i = 0U; i < kMimuCount; ++i) {
        imuInputs.at(i).angVelBody = v;
    }

    auto const out = alg.update(imuInputs);

    ASSERT_FALSE(out.faultDetected);
    for (size_t i = 0U; i < kMimuCount; ++i) {
        ASSERT_TRUE(out.validImus.at(i));
        ASSERT_NEAR(out.omegaDifferencesMag.at(i), 0.0F, kCompTol);
    }
    for (int i = 0; i < 3; ++i) {
        ASSERT_NEAR(out.avgAngVelBody[i], v[i], kCompTol);
    }
}

void propertyClearSingleOutlier(const std::vector<float>& baseAngVel, size_t outlierIndex, float omegaThreshold) {
    constexpr float kOutlierFactor = 10.0F;

    MimuMajorityVoteAlgorithm alg{};
    alg.setOmegaThreshold(omegaThreshold);

    Eigen::Vector3f const base = Eigen::Map<const Eigen::Vector3f>(baseAngVel.data());
    std::array<MimuInput, kMimuCount> imuInputs{};
    for (size_t i = 0U; i < kMimuCount; ++i) {
        imuInputs.at(i).angVelBody = base;
    }
    // Make outlier clearly separable: kOutlierFactor × threshold beyond the base pair
    imuInputs.at(outlierIndex).angVelBody = base + kOutlierFactor * omegaThreshold * Eigen::Vector3f::Ones();

    auto const out = alg.update(imuInputs);

    ASSERT_TRUE(out.faultDetected);
    ASSERT_FALSE(out.validImus.at(outlierIndex));
    for (size_t i = 0U; i < kMimuCount; ++i) {
        if (i != outlierIndex) {
            ASSERT_TRUE(out.validImus.at(i));
        }
    }
}

void propertyCyclicPermutationInvariant(const std::vector<float>& angVel1,
                                        const std::vector<float>& angVel2,
                                        const std::vector<float>& angVel3,
                                        float omegaThreshold) {
    MimuMajorityVoteAlgorithm alg{};
    alg.setOmegaThreshold(omegaThreshold);

    std::array<MimuInput, kMimuCount> imuInputs{};
    imuInputs.at(0).angVelBody = Eigen::Map<const Eigen::Vector3f>(angVel1.data());
    imuInputs.at(1).angVelBody = Eigen::Map<const Eigen::Vector3f>(angVel2.data());
    imuInputs.at(2).angVelBody = Eigen::Map<const Eigen::Vector3f>(angVel3.data());

    auto const out0 = alg.update(imuInputs);

    // Cyclic permutation: [v2, v3, v1]
    std::array<MimuInput, kMimuCount> imuInputs1{};
    imuInputs1.at(0).angVelBody = imuInputs.at(1).angVelBody;
    imuInputs1.at(1).angVelBody = imuInputs.at(2).angVelBody;
    imuInputs1.at(2).angVelBody = imuInputs.at(0).angVelBody;

    auto const out1 = alg.update(imuInputs1);

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
    .WithDomains(fuzztest::InRange(1e-6F, 1e3F),                                   // omegaThreshold
                 fuzztest::VectorOf(fuzztest::InRange(-1e3F, 1e3F)).WithSize(3),   // angVel1
                 fuzztest::VectorOf(fuzztest::InRange(-1e3F, 1e3F)).WithSize(3),   // angVel2
                 fuzztest::VectorOf(fuzztest::InRange(-1e3F, 1e3F)).WithSize(3));  // angVel3

FUZZ_TEST(MimuMajorityVoteAlgorithmFuzz, propertyOutputAlwaysFinite)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-kMaxAngVel, kMaxAngVel)).WithSize(3),  // angVel1
                 fuzztest::VectorOf(fuzztest::InRange(-kMaxAngVel, kMaxAngVel)).WithSize(3),  // angVel2
                 fuzztest::VectorOf(fuzztest::InRange(-kMaxAngVel, kMaxAngVel)).WithSize(3),  // angVel3
                 fuzztest::InRange(kMinThreshold, kMaxThreshold));                            // omegaThreshold

FUZZ_TEST(MimuMajorityVoteAlgorithmFuzz, propertyFaultAndValidConsistency)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-kMaxAngVel, kMaxAngVel)).WithSize(3),  // angVel1
                 fuzztest::VectorOf(fuzztest::InRange(-kMaxAngVel, kMaxAngVel)).WithSize(3),  // angVel2
                 fuzztest::VectorOf(fuzztest::InRange(-kMaxAngVel, kMaxAngVel)).WithSize(3),  // angVel3
                 fuzztest::InRange(kMinThreshold, kMaxThreshold));                            // omegaThreshold

FUZZ_TEST(MimuMajorityVoteAlgorithmFuzz, propertyAverageIsCorrect)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-kMaxAngVel, kMaxAngVel)).WithSize(3),  // angVel1
                 fuzztest::VectorOf(fuzztest::InRange(-kMaxAngVel, kMaxAngVel)).WithSize(3),  // angVel2
                 fuzztest::VectorOf(fuzztest::InRange(-kMaxAngVel, kMaxAngVel)).WithSize(3),  // angVel3
                 fuzztest::InRange(kMinThreshold, kMaxThreshold));                            // omegaThreshold

FUZZ_TEST(MimuMajorityVoteAlgorithmFuzz, propertyIdenticalIMUsNoFault)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-kMaxAngVel, kMaxAngVel)).WithSize(3),  // angVel
                 fuzztest::InRange(kMinThreshold, kMaxThreshold));                            // omegaThreshold

FUZZ_TEST(MimuMajorityVoteAlgorithmFuzz, propertyClearSingleOutlier)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-kMaxAngVel, kMaxAngVel)).WithSize(3),  // baseAngVel
                 fuzztest::InRange<size_t>(0, 2),                                             // outlierIndex
                 fuzztest::InRange(kMinThreshold, kMaxThreshold));                            // omegaThreshold

FUZZ_TEST(MimuMajorityVoteAlgorithmFuzz, propertyCyclicPermutationInvariant)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-kMaxAngVel, kMaxAngVel)).WithSize(3),  // angVel1
                 fuzztest::VectorOf(fuzztest::InRange(-kMaxAngVel, kMaxAngVel)).WithSize(3),  // angVel2
                 fuzztest::VectorOf(fuzztest::InRange(-kMaxAngVel, kMaxAngVel)).WithSize(3),  // angVel3
                 fuzztest::InRange(kMinThreshold, kMaxThreshold));                            // omegaThreshold
