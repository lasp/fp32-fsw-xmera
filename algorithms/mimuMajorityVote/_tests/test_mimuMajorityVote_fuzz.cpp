#include "mimuMajorityVoteTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

namespace {

constexpr float kMaxAngVel = 1e3F;
constexpr float kMaxAccel = 1e3F;
constexpr float kMinThreshold = 1e-2F;
constexpr float kMaxThreshold = 1e3F;
constexpr float kCompTol = 1e-3F;

// The property functions below exercise both the gyro and accelerometer votes: each drives a single
// update() with independent fuzzed gyro and accel inputs (and independent thresholds) and asserts
// the same property on both out.gyro and out.accel. The regressionTestMimuMajorityVote fuzz target
// additionally checks both votes against a reference implementation.

// ----- Shared per-vote property checks (applied identically to the gyro and accel results) -----

size_t invalidCount(const MimuVoteResult& vote) {
    size_t count = 0U;
    for (size_t i = 0U; i < kMimuCount; ++i) {
        if (!vote.imuValid.at(i)) {
            ++count;
        }
    }
    return count;
}

void assertResultFinite(const MimuVoteResult& vote) {
    for (int i = 0; i < 3; ++i) {
        ASSERT_TRUE(std::isfinite(vote.average[i]));
    }
    for (size_t i = 0U; i < kMimuCount; ++i) {
        ASSERT_GE(vote.imuDifferenceMag.at(i), 0.0F);
        ASSERT_TRUE(std::isfinite(vote.imuDifferenceMag.at(i)));
    }
}

void assertFaultValidConsistency(const MimuVoteResult& vote) {
    // Invalid count must be 0 (no fault) or 1 (single outlier)
    size_t const invalid = invalidCount(vote);
    ASSERT_TRUE(invalid == 0U || invalid == 1U);
    // faultDetected must be consistent with imuValid
    ASSERT_EQ(vote.faultDetected, invalid > 0U);
}

void expectNoFaultAverage(const Eigen::Vector3f& average,
                          const std::array<Eigen::Vector3f, kMimuCount>& measurements,
                          size_t numImus) {
    Eigen::Vector3f expectedAvg = Eigen::Vector3f::Zero();
    for (size_t i = 0U; i < numImus; ++i) {
        expectedAvg += measurements.at(i);
    }
    expectedAvg /= static_cast<float>(numImus);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(average[i], expectedAvg[i], kCompTol);
    }
}

void expectSingleFaultAverage(const Eigen::Vector3f& average,
                              const std::array<bool, kMimuCount>& imuValid,
                              const std::array<Eigen::Vector3f, kMimuCount>& measurements,
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
            expectedAvg += measurements.at(i);
            ++count;
        }
    }
    expectedAvg /= static_cast<float>(count);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(average[i], expectedAvg[i], kCompTol);
    }
}

void assertAverageCorrect(const MimuVoteResult& vote, const std::array<Eigen::Vector3f, kMimuCount>& measurements) {
    if (invalidCount(vote) == 0U) {
        expectNoFaultAverage(vote.average, measurements, kMimuCount);
    } else {
        // Single outlier excluded: average of the remaining sensors
        expectSingleFaultAverage(vote.average, vote.imuValid, measurements, kMimuCount);
    }
}

void assertIdenticalNoFault(const MimuVoteResult& vote, const Eigen::Vector3f& value) {
    ASSERT_FALSE(vote.faultDetected);
    for (size_t i = 0U; i < kMimuCount; ++i) {
        ASSERT_TRUE(vote.imuValid.at(i));
        ASSERT_NEAR(vote.imuDifferenceMag.at(i), 0.0F, kCompTol);
    }
    for (int i = 0; i < 3; ++i) {
        ASSERT_NEAR(vote.average[i], value[i], kCompTol);
    }
}

void assertClearSingleOutlier(const MimuVoteResult& vote, size_t outlierIndex) {
    ASSERT_TRUE(vote.faultDetected);
    ASSERT_FALSE(vote.imuValid.at(outlierIndex));
    for (size_t i = 0U; i < kMimuCount; ++i) {
        if (i != outlierIndex) {
            ASSERT_TRUE(vote.imuValid.at(i));
        }
    }
}

void assertVoteInvariantUnderPermutation(const MimuVoteResult& voteA, const MimuVoteResult& voteB) {
    ASSERT_EQ(voteA.faultDetected, voteB.faultDetected);
    ASSERT_EQ(invalidCount(voteA), invalidCount(voteB));
}

// ----- Property functions: each drives both votes and asserts the property on each -----

void propertyOutputAlwaysFinite(const Eigen::Vector3f& angVel1,
                                const Eigen::Vector3f& angVel2,
                                const Eigen::Vector3f& angVel3,
                                float omegaThreshold,
                                const Eigen::Vector3f& accel1,
                                const Eigen::Vector3f& accel2,
                                const Eigen::Vector3f& accel3,
                                float accelThreshold) {
    MimuMajorityVoteAlgorithm alg{MimuMajorityVoteConfig::create(omegaThreshold, 1U, accelThreshold, 1U)};

    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B{angVel1, angVel2, angVel3};
    std::array<Eigen::Vector3f, kMimuCount> imuAccels_B{accel1, accel2, accel3};

    auto const out = alg.update(imuOmegas_BN_B, imuAccels_B);

    assertResultFinite(out.gyro);
    assertResultFinite(out.accel);
}

void propertyFaultAndValidConsistency(const Eigen::Vector3f& angVel1,
                                      const Eigen::Vector3f& angVel2,
                                      const Eigen::Vector3f& angVel3,
                                      float omegaThreshold,
                                      const Eigen::Vector3f& accel1,
                                      const Eigen::Vector3f& accel2,
                                      const Eigen::Vector3f& accel3,
                                      float accelThreshold) {
    MimuMajorityVoteAlgorithm alg{MimuMajorityVoteConfig::create(omegaThreshold, 1U, accelThreshold, 1U)};

    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B{angVel1, angVel2, angVel3};
    std::array<Eigen::Vector3f, kMimuCount> imuAccels_B{accel1, accel2, accel3};

    auto const out = alg.update(imuOmegas_BN_B, imuAccels_B);

    assertFaultValidConsistency(out.gyro);
    assertFaultValidConsistency(out.accel);
}

void propertyAverageIsCorrect(const Eigen::Vector3f& angVel1,
                              const Eigen::Vector3f& angVel2,
                              const Eigen::Vector3f& angVel3,
                              float omegaThreshold,
                              const Eigen::Vector3f& accel1,
                              const Eigen::Vector3f& accel2,
                              const Eigen::Vector3f& accel3,
                              float accelThreshold) {
    MimuMajorityVoteAlgorithm alg{MimuMajorityVoteConfig::create(omegaThreshold, 1U, accelThreshold, 1U)};

    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B{angVel1, angVel2, angVel3};
    std::array<Eigen::Vector3f, kMimuCount> imuAccels_B{accel1, accel2, accel3};

    auto const out = alg.update(imuOmegas_BN_B, imuAccels_B);

    assertAverageCorrect(out.gyro, imuOmegas_BN_B);
    assertAverageCorrect(out.accel, imuAccels_B);
}

void propertyIdenticalIMUsNoFault(const Eigen::Vector3f& angVel,
                                  float omegaThreshold,
                                  const Eigen::Vector3f& accel,
                                  float accelThreshold) {
    MimuMajorityVoteAlgorithm alg{MimuMajorityVoteConfig::create(omegaThreshold, 1U, accelThreshold, 1U)};

    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B{};
    std::array<Eigen::Vector3f, kMimuCount> imuAccels_B{};
    for (size_t i = 0U; i < kMimuCount; ++i) {
        imuOmegas_BN_B.at(i) = angVel;
        imuAccels_B.at(i) = accel;
    }

    auto const out = alg.update(imuOmegas_BN_B, imuAccels_B);

    assertIdenticalNoFault(out.gyro, angVel);
    assertIdenticalNoFault(out.accel, accel);
}

void propertyClearSingleOutlier(const Eigen::Vector3f& baseAngVel,
                                size_t gyroOutlierIndex,
                                float omegaThreshold,
                                const Eigen::Vector3f& baseAccel,
                                size_t accelOutlierIndex,
                                float accelThreshold) {
    constexpr float kOutlierFactor = 10.0F;

    MimuMajorityVoteAlgorithm alg{MimuMajorityVoteConfig::create(omegaThreshold, 1U, accelThreshold, 1U)};

    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B{};
    std::array<Eigen::Vector3f, kMimuCount> imuAccels_B{};
    for (size_t i = 0U; i < kMimuCount; ++i) {
        imuOmegas_BN_B.at(i) = baseAngVel;
        imuAccels_B.at(i) = baseAccel;
    }
    // Make each outlier clearly separable: kOutlierFactor × threshold beyond the base pair
    imuOmegas_BN_B.at(gyroOutlierIndex) = baseAngVel + kOutlierFactor * omegaThreshold * Eigen::Vector3f::Ones();
    imuAccels_B.at(accelOutlierIndex) = baseAccel + kOutlierFactor * accelThreshold * Eigen::Vector3f::Ones();

    auto const out = alg.update(imuOmegas_BN_B, imuAccels_B);

    assertClearSingleOutlier(out.gyro, gyroOutlierIndex);
    assertClearSingleOutlier(out.accel, accelOutlierIndex);
}

void propertyCyclicPermutationInvariant(const Eigen::Vector3f& angVel1,
                                        const Eigen::Vector3f& angVel2,
                                        const Eigen::Vector3f& angVel3,
                                        float omegaThreshold,
                                        const Eigen::Vector3f& accel1,
                                        const Eigen::Vector3f& accel2,
                                        const Eigen::Vector3f& accel3,
                                        float accelThreshold) {
    MimuMajorityVoteAlgorithm alg{MimuMajorityVoteConfig::create(omegaThreshold, 1U, accelThreshold, 1U)};

    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B{angVel1, angVel2, angVel3};
    std::array<Eigen::Vector3f, kMimuCount> imuAccels_B{accel1, accel2, accel3};

    auto const out0 = alg.update(imuOmegas_BN_B, imuAccels_B);

    // Cyclic permutation: [v2, v3, v1] applied to both votes
    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B1{};
    std::array<Eigen::Vector3f, kMimuCount> imuAccels_B1{};
    imuOmegas_BN_B1.at(0) = imuOmegas_BN_B.at(1);
    imuOmegas_BN_B1.at(1) = imuOmegas_BN_B.at(2);
    imuOmegas_BN_B1.at(2) = imuOmegas_BN_B.at(0);
    imuAccels_B1.at(0) = imuAccels_B.at(1);
    imuAccels_B1.at(1) = imuAccels_B.at(2);
    imuAccels_B1.at(2) = imuAccels_B.at(0);

    auto const out1 = alg.update(imuOmegas_BN_B1, imuAccels_B1);

    assertVoteInvariantUnderPermutation(out0.gyro, out1.gyro);
    assertVoteInvariantUnderPermutation(out0.accel, out1.accel);
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
                 fuzztest::InRange(kMinThreshold, kMaxThreshold),        // omegaThreshold
                 xmera::fuzz::Vector3fInRange(-kMaxAccel, kMaxAccel),    // accel1
                 xmera::fuzz::Vector3fInRange(-kMaxAccel, kMaxAccel),    // accel2
                 xmera::fuzz::Vector3fInRange(-kMaxAccel, kMaxAccel),    // accel3
                 fuzztest::InRange(kMinThreshold, kMaxThreshold));       // accelThreshold

FUZZ_TEST(MimuMajorityVoteAlgorithmFuzz, propertyFaultAndValidConsistency)
    .WithDomains(xmera::fuzz::Vector3fInRange(-kMaxAngVel, kMaxAngVel),  // angVel1
                 xmera::fuzz::Vector3fInRange(-kMaxAngVel, kMaxAngVel),  // angVel2
                 xmera::fuzz::Vector3fInRange(-kMaxAngVel, kMaxAngVel),  // angVel3
                 fuzztest::InRange(kMinThreshold, kMaxThreshold),        // omegaThreshold
                 xmera::fuzz::Vector3fInRange(-kMaxAccel, kMaxAccel),    // accel1
                 xmera::fuzz::Vector3fInRange(-kMaxAccel, kMaxAccel),    // accel2
                 xmera::fuzz::Vector3fInRange(-kMaxAccel, kMaxAccel),    // accel3
                 fuzztest::InRange(kMinThreshold, kMaxThreshold));       // accelThreshold

FUZZ_TEST(MimuMajorityVoteAlgorithmFuzz, propertyAverageIsCorrect)
    .WithDomains(xmera::fuzz::Vector3fInRange(-kMaxAngVel, kMaxAngVel),  // angVel1
                 xmera::fuzz::Vector3fInRange(-kMaxAngVel, kMaxAngVel),  // angVel2
                 xmera::fuzz::Vector3fInRange(-kMaxAngVel, kMaxAngVel),  // angVel3
                 fuzztest::InRange(kMinThreshold, kMaxThreshold),        // omegaThreshold
                 xmera::fuzz::Vector3fInRange(-kMaxAccel, kMaxAccel),    // accel1
                 xmera::fuzz::Vector3fInRange(-kMaxAccel, kMaxAccel),    // accel2
                 xmera::fuzz::Vector3fInRange(-kMaxAccel, kMaxAccel),    // accel3
                 fuzztest::InRange(kMinThreshold, kMaxThreshold));       // accelThreshold

FUZZ_TEST(MimuMajorityVoteAlgorithmFuzz, propertyIdenticalIMUsNoFault)
    .WithDomains(xmera::fuzz::Vector3fInRange(-kMaxAngVel, kMaxAngVel),  // angVel
                 fuzztest::InRange(kMinThreshold, kMaxThreshold),        // omegaThreshold
                 xmera::fuzz::Vector3fInRange(-kMaxAccel, kMaxAccel),    // accel
                 fuzztest::InRange(kMinThreshold, kMaxThreshold));       // accelThreshold

FUZZ_TEST(MimuMajorityVoteAlgorithmFuzz, propertyClearSingleOutlier)
    .WithDomains(xmera::fuzz::Vector3fInRange(-kMaxAngVel, kMaxAngVel),  // baseAngVel
                 fuzztest::InRange<size_t>(0, 2),                        // gyroOutlierIndex
                 fuzztest::InRange(kMinThreshold, kMaxThreshold),        // omegaThreshold
                 xmera::fuzz::Vector3fInRange(-kMaxAccel, kMaxAccel),    // baseAccel
                 fuzztest::InRange<size_t>(0, 2),                        // accelOutlierIndex
                 fuzztest::InRange(kMinThreshold, kMaxThreshold));       // accelThreshold

FUZZ_TEST(MimuMajorityVoteAlgorithmFuzz, propertyCyclicPermutationInvariant)
    .WithDomains(xmera::fuzz::Vector3fInRange(-kMaxAngVel, kMaxAngVel),  // angVel1
                 xmera::fuzz::Vector3fInRange(-kMaxAngVel, kMaxAngVel),  // angVel2
                 xmera::fuzz::Vector3fInRange(-kMaxAngVel, kMaxAngVel),  // angVel3
                 fuzztest::InRange(kMinThreshold, kMaxThreshold),        // omegaThreshold
                 xmera::fuzz::Vector3fInRange(-kMaxAccel, kMaxAccel),    // accel1
                 xmera::fuzz::Vector3fInRange(-kMaxAccel, kMaxAccel),    // accel2
                 xmera::fuzz::Vector3fInRange(-kMaxAccel, kMaxAccel),    // accel3
                 fuzztest::InRange(kMinThreshold, kMaxThreshold));       // accelThreshold
