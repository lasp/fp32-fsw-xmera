#include "bodyRateMiscompareTestHelpers.hpp"
#include "freestandingInvalidArgument.h"
#include <gtest/gtest.h>
#include <array>

static void runRegressionCase(float threshold,
                              uint32_t faultPersistenceLimit,
                              const std::array<float, 3>& imuVec,
                              const std::array<float, 3>& stVec,
                              bool useImuRates = false) {
    BodyRateMiscompareAlgorithm alg{};
    alg.setBodyRateThreshold(threshold);
    alg.setFaultPersistenceLimit(faultPersistenceLimit);
    alg.setUseImuRates(useImuRates);

    const Eigen::Vector3f imu = toEigenVector(imuVec);
    const Eigen::Vector3f st = toEigenVector(stVec);

    bool refFaultDetected = useImuRates;
    uint32_t refFaultPersistenceCount = 0;

    int numSteps = 5;
    for (int step = 0; step < numSteps; ++step) {
        BodyRateMiscompareOutput out{};
        BodyRateMiscompareOutput ref{};

        EXPECT_NO_THROW(out = alg.update(imu, st));
        EXPECT_NO_THROW(ref = referenceUpdate(
                            threshold, faultPersistenceLimit, refFaultDetected, refFaultPersistenceCount, imu, st));

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

TEST(BodyRateMiscompareTest, RegressionTestUseImuRatesTrue) {
    runRegressionCase(1.0F, 1, {0.1F, -0.2F, 0.3F}, {0.2F, -0.1F, 0.1F}, true);
}

TEST(BodyRateMiscompareTest, RegressionTestUseImuRatesFalse) {
    runRegressionCase(1.0F, 1, {0.1F, -0.2F, 0.3F}, {0.2F, -0.1F, 0.1F}, false);
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

    EXPECT_FALSE(alg.getUseImuRates());
    alg.setUseImuRates(true);
    EXPECT_TRUE(alg.getUseImuRates());
}

// ---------------------------------------------------------------------------
// Property tests
// ---------------------------------------------------------------------------

// Output omega_BN_B is always exactly one of the two inputs (source selection).
TEST(BodyRateMiscompareTest, OutputIsAlwaysOneOfTheInputs) {
    propertyOutputIsOneOfInputs({0.1F, -0.2F, 0.3F}, {1.0F, 0.5F, -0.5F});
}

// Fault flag and source selection are always consistent.
TEST(BodyRateMiscompareTest, FaultFlagMatchesSourceSelection) {
    propertyFaultFlagMatchesSource({0.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F});
}

// When IMU == ST, difference norm is 0 — fault never triggers.
TEST(BodyRateMiscompareTest, IdenticalInputsNeverFault) { propertyIdenticalInputsNeverFault({0.5F, -0.3F, 0.1F}); }

// Once fault triggers, it stays triggered on all subsequent calls.
TEST(BodyRateMiscompareTest, FaultIsSticky) { propertyFaultIsSticky({0.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}); }

// When useImuRates is set, output is always IMU rate.
TEST(BodyRateMiscompareTest, UseImuRatesAlwaysOutputsImu) {
    propertyUseImuRatesAlwaysOutputsImu({0.1F, -0.2F, 0.3F}, {1.0F, 0.5F, -0.5F});
}

// All output components are finite for finite inputs.
TEST(BodyRateMiscompareTest, OutputIsAlwaysFinite) {
    propertyOutputIsFinite({1e10F, -1e10F, 1e10F}, {-1e10F, 1e10F, -1e10F});
}

// ---------------------------------------------------------------------------
// Edge-case tests
// ---------------------------------------------------------------------------

// Both inputs are zero vectors — no fault, output is zero.
TEST(BodyRateMiscompareTest, ZeroInputsBothSources) {
    BodyRateMiscompareAlgorithm alg{};
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();

    auto out = alg.update(zero, zero);
    EXPECT_FALSE(out.bodyRateFaultDetected);
    EXPECT_EQ(out.omega_BN_B, zero);
}

// Difference norm exactly at threshold (strict >) should NOT trigger fault.
TEST(BodyRateMiscompareTest, ExactlyAtThreshold) {
    const float threshold = 1.0F;
    BodyRateMiscompareAlgorithm alg{};
    alg.setBodyRateThreshold(threshold);
    alg.setFaultPersistenceLimit(1);

    // Place difference exactly at threshold along x-axis
    const Eigen::Vector3f imu(0.0F, 0.0F, 0.0F);
    const Eigen::Vector3f st(threshold, 0.0F, 0.0F);

    for (int step = 0; step < 5; ++step) {
        auto out = alg.update(imu, st);
        EXPECT_FALSE(out.bodyRateFaultDetected);
        EXPECT_EQ(out.omega_BN_B, st);
    }
}

// Difference norm just above threshold triggers fault with persistence=1.
TEST(BodyRateMiscompareTest, JustAboveThreshold) {
    const float threshold = 1.0F;
    BodyRateMiscompareAlgorithm alg{};
    alg.setBodyRateThreshold(threshold);
    alg.setFaultPersistenceLimit(1);

    const Eigen::Vector3f imu(0.0F, 0.0F, 0.0F);
    const Eigen::Vector3f st(threshold + 1e-6F, 0.0F, 0.0F);

    auto out = alg.update(imu, st);
    EXPECT_TRUE(out.bodyRateFaultDetected);
    EXPECT_EQ(out.omega_BN_B, imu);
}

// Alternating above/below threshold resets persistence counter, never reaching limit.
TEST(BodyRateMiscompareTest, PersistenceCounterResetsOnGoodStep) {
    BodyRateMiscompareAlgorithm alg{};
    alg.setBodyRateThreshold(0.5F);
    alg.setFaultPersistenceLimit(3);

    const Eigen::Vector3f imu(0.0F, 0.0F, 0.0F);
    const Eigen::Vector3f stFar(1.0F, 0.0F, 0.0F);    // above threshold
    const Eigen::Vector3f stClose(0.1F, 0.0F, 0.0F);  // below threshold

    // Pattern: far, close, far, close, far, close — counter never reaches 3
    for (int i = 0; i < 10; ++i) {
        const Eigen::Vector3f& st = (i % 2 == 0) ? stFar : stClose;
        auto out = alg.update(imu, st);
        EXPECT_FALSE(out.bodyRateFaultDetected);
    }
}

// After fault triggers, even identical inputs still output IMU.
TEST(BodyRateMiscompareTest, StickyFaultIgnoresSubsequentGoodSteps) {
    BodyRateMiscompareAlgorithm alg{};
    alg.setBodyRateThreshold(0.5F);
    alg.setFaultPersistenceLimit(1);

    const Eigen::Vector3f imu(0.1F, 0.2F, 0.3F);
    const Eigen::Vector3f stFar(10.0F, 0.0F, 0.0F);

    // Trigger fault
    auto out1 = alg.update(imu, stFar);
    EXPECT_TRUE(out1.bodyRateFaultDetected);

    // Now feed identical inputs (diff = 0) — fault persists
    for (int step = 0; step < 5; ++step) {
        auto out = alg.update(imu, imu);
        EXPECT_TRUE(out.bodyRateFaultDetected);
        EXPECT_EQ(out.omega_BN_B, imu);
    }
}

// Difference in only one axis — verifies Euclidean norm comparison.
TEST(BodyRateMiscompareTest, SingleAxisDifference) {
    BodyRateMiscompareAlgorithm alg{};
    alg.setBodyRateThreshold(0.5F);
    alg.setFaultPersistenceLimit(1);

    const Eigen::Vector3f imu(1.0F, 2.0F, 3.0F);

    // Difference of 0.6 only in y-axis → norm = 0.6 > 0.5
    const Eigen::Vector3f st(1.0F, 2.6F, 3.0F);

    auto out = alg.update(imu, st);
    EXPECT_TRUE(out.bodyRateFaultDetected);
    EXPECT_EQ(out.omega_BN_B, imu);
}

// Setting useImuRates forces IMU output even when inputs agree.
TEST(BodyRateMiscompareTest, UseImuRatesForceOutput) {
    BodyRateMiscompareAlgorithm alg{};
    alg.setBodyRateThreshold(1.0F);
    alg.setFaultPersistenceLimit(1);
    alg.setUseImuRates(true);

    const Eigen::Vector3f imu(0.1F, 0.2F, 0.3F);
    const Eigen::Vector3f st(0.1F, 0.2F, 0.31F);  // no miscompare

    for (int step = 0; step < 5; ++step) {
        auto out = alg.update(imu, st);
        EXPECT_TRUE(out.bodyRateFaultDetected);
        EXPECT_EQ(out.omega_BN_B, imu);
    }
}

// Reset clears the persistence counter, preventing fault from triggering.
TEST(BodyRateMiscompareTest, ResetClearsPersistenceCounter) {
    BodyRateMiscompareAlgorithm alg{};
    alg.setBodyRateThreshold(0.5F);
    alg.setFaultPersistenceLimit(3);

    const Eigen::Vector3f imu(0.0F, 0.0F, 0.0F);
    const Eigen::Vector3f stFar(1.0F, 0.0F, 0.0F);  // above threshold

    // Accumulate 2 of 3 needed violations
    alg.update(imu, stFar);
    alg.update(imu, stFar);

    // Reset clears the counter
    alg.reset();

    // Now need 3 more consecutive violations to trigger fault
    auto out1 = alg.update(imu, stFar);
    EXPECT_FALSE(out1.bodyRateFaultDetected);
    auto out2 = alg.update(imu, stFar);
    EXPECT_FALSE(out2.bodyRateFaultDetected);
    auto out3 = alg.update(imu, stFar);
    EXPECT_TRUE(out3.bodyRateFaultDetected);
}

// Reset does not clear the internal fault state — a detected fault persists after reset.
TEST(BodyRateMiscompareTest, ResetDoesNotClearInternalFaultState) {
    BodyRateMiscompareAlgorithm alg{};
    alg.setBodyRateThreshold(0.5F);
    alg.setFaultPersistenceLimit(1);

    const Eigen::Vector3f imu(0.1F, 0.2F, 0.3F);
    const Eigen::Vector3f stFar(10.0F, 0.0F, 0.0F);

    // Trigger fault
    auto out1 = alg.update(imu, stFar);
    EXPECT_TRUE(out1.bodyRateFaultDetected);
    EXPECT_FALSE(alg.getUseImuRates());  // settable parameter unchanged by update

    // Reset only clears counter, not useImuRates
    alg.reset();
    EXPECT_FALSE(alg.getUseImuRates());

    // Output still uses IMU (does not reset internal fault state)
    auto out2 = alg.update(imu, imu);
    EXPECT_TRUE(out2.bodyRateFaultDetected);
    EXPECT_EQ(out2.omega_BN_B, imu);
}

// Very large but finite inputs produce finite output.
TEST(BodyRateMiscompareTest, LargeInputValues) {
    BodyRateMiscompareAlgorithm alg{};
    alg.setBodyRateThreshold(1e30F);

    const Eigen::Vector3f imu(1e30F, -1e30F, 1e30F);
    const Eigen::Vector3f st(-1e30F, 1e30F, -1e30F);

    auto out = alg.update(imu, st);
    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(out.omega_BN_B[i]));
    }
    bool matchesImu = (out.omega_BN_B == imu);
    bool matchesSt = (out.omega_BN_B == st);
    EXPECT_TRUE(matchesImu || matchesSt);
}
