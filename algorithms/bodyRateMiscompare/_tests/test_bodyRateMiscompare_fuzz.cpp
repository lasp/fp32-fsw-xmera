#include "bodyRateMiscompareTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

inline void fuzzPropertyBodyRateMiscompare(float threshold,
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
        EXPECT_NO_THROW(out = alg.update(imu, st));

        BodyRateMiscompareOutput ref{};
        EXPECT_NO_THROW(ref = referenceUpdate(
                            threshold, faultPersistenceLimit, refFaultDetected, refFaultPersistenceCount, imu, st));

        EXPECT_EQ(out.bodyRateFaultDetected, ref.bodyRateFaultDetected);

        for (int i = 0; i < 3; ++i) {
            EXPECT_FLOAT_EQ(out.omega_BN_B[i], ref.omega_BN_B[i]);
            EXPECT_TRUE(std::isfinite(out.omega_BN_B[i]));
        }
    }
}

FUZZ_TEST(BodyRateMiscompareAlgorithmFuzz, fuzzPropertyBodyRateMiscompare)
    .WithDomains(fuzztest::InRange(1e-6F, 1e6F),
                 fuzztest::InRange(1U, 10U),
                 fuzztest::ArrayOf<3>(fuzztest::InRange(-1e6F, 1e6F)),
                 fuzztest::ArrayOf<3>(fuzztest::InRange(-1e6F, 1e6F)));

// ---------------------------------------------------------------------------
// Property fuzz tests
// ---------------------------------------------------------------------------

FUZZ_TEST(BodyRateMiscomparePropertyFuzz, propertyOutputIsOneOfInputs)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3),
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3));

FUZZ_TEST(BodyRateMiscomparePropertyFuzz, propertyFaultFlagMatchesSource)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3),
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3));

FUZZ_TEST(BodyRateMiscomparePropertyFuzz, propertyIdenticalInputsNeverFault)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3));

FUZZ_TEST(BodyRateMiscomparePropertyFuzz, propertyFaultIsSticky)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3),
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3));

FUZZ_TEST(BodyRateMiscomparePropertyFuzz, propertyOutputIsFinite)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3),
                 fuzztest::VectorOf(fuzztest::InRange(-1e6F, 1e6F)).WithSize(3));
