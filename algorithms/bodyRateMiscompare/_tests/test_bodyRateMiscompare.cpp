#include "bodyRateMiscompareTestHelpers.hpp"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include <gtest/gtest.h>
#include <array>

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
    // A valid configuration round-trips its values.
    const auto config = BodyRateMiscompareConfig::create(0.25F, 5U, true);
    EXPECT_NEAR(config.getBodyRateThreshold(), 0.25F, 1e-6);
    EXPECT_EQ(config.getFaultPersistenceLimit(), 5U);
    EXPECT_TRUE(config.getUseImuRates());

    // Invalid threshold and persistence limit are rejected at construction.
    EXPECT_THROW(BodyRateMiscompareConfig::create(0.0F, 1U, false), fsw::invalid_argument);
    EXPECT_THROW(BodyRateMiscompareConfig::create(-0.1F, 1U, false), fsw::invalid_argument);
    EXPECT_THROW(BodyRateMiscompareConfig::create(0.25F, 0U, false), fsw::invalid_argument);

    EXPECT_TRUE(BodyRateMiscompareConfig::isValidBodyRateThreshold(0.25F));
    EXPECT_FALSE(BodyRateMiscompareConfig::isValidBodyRateThreshold(0.0F));
    EXPECT_TRUE(BodyRateMiscompareConfig::isValidFaultPersistenceLimit(1U));
    EXPECT_FALSE(BodyRateMiscompareConfig::isValidFaultPersistenceLimit(0U));
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
    BodyRateMiscompareAlgorithm alg{BodyRateMiscompareConfig::create(0.5F, 1, false)};
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();

    auto out = alg.update(zero, zero);
    EXPECT_FALSE(out.bodyRateFaultDetected);
    EXPECT_EQ(out.omega_BN_B, zero);
}

// Difference norm exactly at threshold (strict >) should NOT trigger fault.
TEST(BodyRateMiscompareTest, ExactlyAtThreshold) {
    const float threshold = 1.0F;
    BodyRateMiscompareAlgorithm alg{BodyRateMiscompareConfig::create(threshold, 1, false)};

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
    BodyRateMiscompareAlgorithm alg{BodyRateMiscompareConfig::create(threshold, 1, false)};

    const Eigen::Vector3f imu(0.0F, 0.0F, 0.0F);
    const Eigen::Vector3f st(threshold + 1e-6F, 0.0F, 0.0F);

    auto out = alg.update(imu, st);
    EXPECT_TRUE(out.bodyRateFaultDetected);
    EXPECT_EQ(out.omega_BN_B, imu);
}

// Alternating above/below threshold resets persistence counter, never reaching limit.
TEST(BodyRateMiscompareTest, PersistenceCounterResetsOnGoodStep) {
    BodyRateMiscompareAlgorithm alg{BodyRateMiscompareConfig::create(0.5F, 3, false)};

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
    BodyRateMiscompareAlgorithm alg{BodyRateMiscompareConfig::create(0.5F, 1, false)};

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

// Setting useImuRates forces IMU output even when inputs agree.
TEST(BodyRateMiscompareTest, UseImuRatesForceOutput) {
    BodyRateMiscompareAlgorithm alg{BodyRateMiscompareConfig::create(1.0F, 1, true)};

    const Eigen::Vector3f imu(0.1F, 0.2F, 0.3F);
    const Eigen::Vector3f st(0.1F, 0.2F, 0.31F);  // no miscompare

    for (int step = 0; step < 5; ++step) {
        auto out = alg.update(imu, st);
        EXPECT_TRUE(out.bodyRateFaultDetected);
        EXPECT_EQ(out.omega_BN_B, imu);
    }
}

// reInitialize() clears the persistence counter, preventing fault from triggering.
TEST(BodyRateMiscompareTest, ReInitializeClearsPersistenceCounter) {
    BodyRateMiscompareAlgorithm alg{BodyRateMiscompareConfig::create(0.5F, 3, false)};

    const Eigen::Vector3f imu(0.0F, 0.0F, 0.0F);
    const Eigen::Vector3f stFar(1.0F, 0.0F, 0.0F);  // above threshold

    // Accumulate 2 of 3 needed violations
    alg.update(imu, stFar);
    alg.update(imu, stFar);

    // reInitialize clears the counter
    alg.reInitialize();

    // Now need 3 more consecutive violations to trigger fault
    auto out1 = alg.update(imu, stFar);
    EXPECT_FALSE(out1.bodyRateFaultDetected);
    auto out2 = alg.update(imu, stFar);
    EXPECT_FALSE(out2.bodyRateFaultDetected);
    auto out3 = alg.update(imu, stFar);
    EXPECT_TRUE(out3.bodyRateFaultDetected);
}

// reInitialize() does not clear the latched fault state — a detected fault persists.
TEST(BodyRateMiscompareTest, ReInitializePreservesLatchedFault) {
    BodyRateMiscompareAlgorithm alg{BodyRateMiscompareConfig::create(0.5F, 1, false)};

    const Eigen::Vector3f imu(0.1F, 0.2F, 0.3F);
    const Eigen::Vector3f stFar(10.0F, 0.0F, 0.0F);

    // Trigger fault
    auto out1 = alg.update(imu, stFar);
    EXPECT_TRUE(out1.bodyRateFaultDetected);

    // reInitialize only clears the counter, not the latched fault
    alg.reInitialize();

    // Output still uses IMU (does not reset internal fault state)
    auto out2 = alg.update(imu, imu);
    EXPECT_TRUE(out2.bodyRateFaultDetected);
    EXPECT_EQ(out2.omega_BN_B, imu);
}

// reInitializeAll re-arms both the persistence counter and the latched fault (unlike reInitialize).
TEST(BodyRateMiscompareTest, ReInitializeAllReArmsLatchedFault) {
    BodyRateMiscompareAlgorithm alg{BodyRateMiscompareConfig::create(0.5F, 1, false)};

    const Eigen::Vector3f imu(0.1F, 0.2F, 0.3F);
    const Eigen::Vector3f stFar(10.0F, 0.0F, 0.0F);

    EXPECT_TRUE(alg.update(imu, stFar).bodyRateFaultDetected);

    // reInitializeAll clears the latch; agreeing inputs no longer report a fault.
    alg.reInitializeAll();
    auto out = alg.update(imu, imu);
    EXPECT_FALSE(out.bodyRateFaultDetected);
    EXPECT_EQ(out.omega_BN_B, imu);
}

// setConfig() swaps the configuration but preserves the runtime state: it does not clear the latched
// fault or the persistence counter. After a latched fault, setConfig leaves the fault latched.
TEST(BodyRateMiscompareTest, SetConfigPreservesLatchedFault) {
    BodyRateMiscompareAlgorithm alg{BodyRateMiscompareConfig::create(0.5F, 1, false)};

    const Eigen::Vector3f imu(0.1F, 0.2F, 0.3F);
    const Eigen::Vector3f stFar(10.0F, 0.0F, 0.0F);

    EXPECT_TRUE(alg.update(imu, stFar).bodyRateFaultDetected);

    // setConfig only swaps the configuration; the latched fault is preserved.
    alg.setConfig(BodyRateMiscompareConfig::create(0.5F, 1, false));
    const auto out = alg.update(imu, imu);
    EXPECT_TRUE(out.bodyRateFaultDetected);
    EXPECT_EQ(out.omega_BN_B, imu);
}
