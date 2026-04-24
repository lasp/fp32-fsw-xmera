#ifndef TEST_BODY_RATE_MISCOMPARE_H
#define TEST_BODY_RATE_MISCOMPARE_H

#include "bodyRateMiscompareAlgorithm.h"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cmath>

inline BodyRateMiscompareOutput referenceUpdate(const float threshold,
                                                const uint32_t faultPersistenceLimit,
                                                bool& faultDetected,
                                                uint32_t& faultPersistenceCount,
                                                const Eigen::Vector3f& imuOmega_BN_B,
                                                const Eigen::Vector3f& stOmega_BN_B) {
    if (!faultDetected) {
        if (const Eigen::Vector3f diff = stOmega_BN_B - imuOmega_BN_B; diff.norm() > threshold) {
            faultPersistenceCount += 1U;
        } else {
            faultPersistenceCount = 0U;
        }
        faultDetected = faultPersistenceCount >= faultPersistenceLimit;
    }

    BodyRateMiscompareOutput out{};
    if (faultDetected) {
        out.omega_BN_B = imuOmega_BN_B;
        out.bodyRateFaultDetected = true;
    } else {
        out.omega_BN_B = stOmega_BN_B;
        out.bodyRateFaultDetected = false;
    }
    return out;
}

inline void runRegressionCase(float threshold,
                              uint32_t faultPersistenceLimit,
                              const Eigen::Vector3f& imu,
                              const Eigen::Vector3f& st,
                              bool useImuRates = false) {
    BodyRateMiscompareAlgorithm alg{};
    alg.setBodyRateThreshold(threshold);
    alg.setFaultPersistenceLimit(faultPersistenceLimit);
    alg.setUseImuRates(useImuRates);

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

// ---------------------------------------------------------------------------
// Property test helper functions
// ---------------------------------------------------------------------------

// Output omega_BN_B is always exactly one of the two inputs (source selection).
inline void propertyOutputIsOneOfInputs(const Eigen::Vector3f& imu, const Eigen::Vector3f& st) {
    BodyRateMiscompareAlgorithm alg{};
    alg.setBodyRateThreshold(0.5F);
    alg.setFaultPersistenceLimit(2);

    for (int step = 0; step < 5; ++step) {
        auto out = alg.update(imu, st);
        bool matchesImu = (out.omega_BN_B == imu);
        bool matchesSt = (out.omega_BN_B == st);
        EXPECT_TRUE(matchesImu || matchesSt);
    }
}

// Fault flag and source selection are always consistent.
inline void propertyFaultFlagMatchesSource(const Eigen::Vector3f& imu, const Eigen::Vector3f& st) {
    BodyRateMiscompareAlgorithm alg{};
    alg.setBodyRateThreshold(0.5F);
    alg.setFaultPersistenceLimit(1);

    for (int step = 0; step < 5; ++step) {
        auto out = alg.update(imu, st);
        if (out.bodyRateFaultDetected) {
            EXPECT_EQ(out.omega_BN_B, imu);
        } else {
            EXPECT_EQ(out.omega_BN_B, st);
        }
    }
}

// When IMU == ST, difference norm is 0 — fault never triggers.
inline void propertyIdenticalInputsNeverFault(const Eigen::Vector3f& rate) {
    BodyRateMiscompareAlgorithm alg{};
    alg.setBodyRateThreshold(1e-6F);
    alg.setFaultPersistenceLimit(1);

    for (int step = 0; step < 10; ++step) {
        auto out = alg.update(rate, rate);
        EXPECT_FALSE(out.bodyRateFaultDetected);
        EXPECT_EQ(out.omega_BN_B, rate);
    }
}

// Once fault triggers, it stays triggered on all subsequent calls.
inline void propertyFaultIsSticky(const Eigen::Vector3f& imu, const Eigen::Vector3f& st) {
    constexpr float threshold = 0.1F;

    // Skip if the difference is not above threshold (can't trigger fault)
    if ((st - imu).norm() <= threshold) {
        return;
    }

    BodyRateMiscompareAlgorithm alg{};
    alg.setBodyRateThreshold(threshold);
    alg.setFaultPersistenceLimit(1);

    // Trigger fault
    auto out1 = alg.update(imu, st);
    EXPECT_TRUE(out1.bodyRateFaultDetected);

    // Subsequent calls with identical inputs must still report fault
    for (int step = 0; step < 5; ++step) {
        auto out = alg.update(imu, imu);
        EXPECT_TRUE(out.bodyRateFaultDetected);
        EXPECT_EQ(out.omega_BN_B, imu);
    }
}

// All output components are finite for finite inputs.
inline void propertyOutputIsFinite(const Eigen::Vector3f& imu, const Eigen::Vector3f& st) {
    BodyRateMiscompareAlgorithm alg{};
    alg.setBodyRateThreshold(0.01F);
    alg.setFaultPersistenceLimit(3);

    for (int step = 0; step < 5; ++step) {
        auto out = alg.update(imu, st);
        for (int i = 0; i < 3; ++i) {
            EXPECT_TRUE(std::isfinite(out.omega_BN_B[i]));
        }
    }
}

// When useImuRates is set, output is always the IMU rate with fault flag true.
inline void propertyUseImuRatesAlwaysOutputsImu(const Eigen::Vector3f& imu, const Eigen::Vector3f& st) {
    BodyRateMiscompareAlgorithm alg{};
    alg.setBodyRateThreshold(0.5F);
    alg.setFaultPersistenceLimit(1);
    alg.setUseImuRates(true);

    for (int step = 0; step < 5; ++step) {
        auto out = alg.update(imu, st);
        EXPECT_TRUE(out.bodyRateFaultDetected);
        EXPECT_EQ(out.omega_BN_B, imu);
    }
}

#endif  // TEST_BODY_RATE_MISCOMPARE_H
