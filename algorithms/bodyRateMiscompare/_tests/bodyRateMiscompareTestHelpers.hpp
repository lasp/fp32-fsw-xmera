#ifndef TEST_BODY_RATE_MISCOMPARE_H
#define TEST_BODY_RATE_MISCOMPARE_H

#include "bodyRateMiscompareAlgorithm.h"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <array>
#include <cstdint>

inline Eigen::Vector3f toEigenVector(const std::array<float, 3>& v) { return Eigen::Vector3f(v[0], v[1], v[2]); }

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
    }

    BodyRateMiscompareOutput out{};
    if (faultPersistenceCount >= faultPersistenceLimit) {
        out.omega_BN_B = imuOmega_BN_B;
        out.bodyRateFaultDetected = true;
        faultDetected = true;
    } else {
        out.omega_BN_B = stOmega_BN_B;
        out.bodyRateFaultDetected = false;
    }
    return out;
}

#endif  // TEST_BODY_RATE_MISCOMPARE_H
