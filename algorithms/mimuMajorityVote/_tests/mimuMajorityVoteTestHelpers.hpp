#ifndef TEST_MIMU_MAJORITY_VOTE_H
#define TEST_MIMU_MAJORITY_VOTE_H

#include "mimuMajorityVoteAlgorithm.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include <gtest/gtest.h>
#include <math.h>
#include <Eigen/Core>
#include <vector>

/*! @brief Reference result of one majority vote over a single quantity (mirrors the production
 majorityVote helper). */
struct ReferenceVote {
    Eigen::Vector3f average{};
    bool faultDetected{};
    std::array<float, kMimuCount> imuDifferenceMag{};
    std::array<bool, kMimuCount> imuValid{};
};

// Reference computation for a single quantity's vote — must mirror the production logic exactly.
inline ReferenceVote referenceUpdate(float threshold,
                                     uint32_t persistenceLimit,
                                     const std::array<Eigen::Vector3f, kMimuCount>& measurements,
                                     std::array<uint32_t, kMimuCount>& persistenceCount) {
    // Stage 1: Compute average and find differences
    Eigen::Vector3f average = Eigen::Vector3f::Zero();
    for (size_t i = 0U; i < kMimuCount; ++i) {
        average += measurements.at(i);
    }
    average /= static_cast<float>(kMimuCount);

    ReferenceVote out{};
    size_t maxDiffIndex = 0U;
    for (size_t i = 0U; i < kMimuCount; ++i) {
        out.imuDifferenceMag.at(i) = (measurements.at(i) - average).norm();
        if (out.imuDifferenceMag.at(i) > out.imuDifferenceMag.at(maxDiffIndex)) {
            maxDiffIndex = i;
        }
        out.imuValid.at(i) = true;
    }

    // Update persistence counter for the worst outlier
    bool faultDetected = false;
    if (out.imuDifferenceMag.at(maxDiffIndex) >= threshold) {
        ++persistenceCount.at(maxDiffIndex);

        // Determine if the outlier has persisted long enough to be faulted
        if (persistenceCount.at(maxDiffIndex) >= persistenceLimit) {
            faultDetected = true;
            out.imuValid.at(maxDiffIndex) = false;
        }
    } else {
        persistenceCount.at(maxDiffIndex) = 0U;
    }

    // Reset counters for non-outlier IMUs
    for (size_t i = 0U; i < kMimuCount; ++i) {
        if (i != maxDiffIndex) {
            persistenceCount.at(i) = 0U;
        }
    }

    if (!faultDetected) {
        out.average = average;
        return out;
    }

    // Exclude outlier and average the remaining IMUs
    out.faultDetected = true;
    out.average = (average * static_cast<float>(kMimuCount) - measurements.at(maxDiffIndex)) /
                  static_cast<float>(kMimuCount - 1U);
    return out;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
inline void regressionTestMimuMajorityVote(float omegaThreshold,
                                           uint32_t gyroFaultPersistenceLimit,
                                           float accelThreshold,
                                           uint32_t accelFaultPersistenceLimit,
                                           uint32_t algCallCount,
                                           const Eigen::Vector3f& angVel1,
                                           const Eigen::Vector3f& angVel2,
                                           const Eigen::Vector3f& angVel3,
                                           const Eigen::Vector3f& accel1,
                                           const Eigen::Vector3f& accel2,
                                           const Eigen::Vector3f& accel3) {
    MimuMajorityVoteAlgorithm alg{MimuMajorityVoteConfig::create(
        omegaThreshold, gyroFaultPersistenceLimit, accelThreshold, accelFaultPersistenceLimit)};

    std::array<Eigen::Vector3f, kMimuCount> imuOmegas_BN_B{angVel1, angVel2, angVel3};
    std::array<Eigen::Vector3f, kMimuCount> imuAccels_B{accel1, accel2, accel3};

    std::array<uint32_t, kMimuCount> gyroCount{};
    std::array<uint32_t, kMimuCount> accelCount{};
    MimuMajorityVoteOutput out{};
    ReferenceVote gyroRef{};
    ReferenceVote accelRef{};

    for (uint32_t call = 0U; call < algCallCount; ++call) {
        EXPECT_NO_THROW(out = alg.update(imuOmegas_BN_B, imuAccels_B));
        EXPECT_NO_THROW(gyroRef =
                            referenceUpdate(omegaThreshold, gyroFaultPersistenceLimit, imuOmegas_BN_B, gyroCount));
        EXPECT_NO_THROW(accelRef =
                            referenceUpdate(accelThreshold, accelFaultPersistenceLimit, imuAccels_B, accelCount));
    }

    // Gyro vote
    EXPECT_EQ(out.gyro.faultDetected, gyroRef.faultDetected);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.gyro.average[i], gyroRef.average[i], 1e-6);
        EXPECT_TRUE(std::isfinite(out.gyro.average[i]));
    }
    for (size_t i = 0U; i < kMimuCount; ++i) {
        EXPECT_EQ(out.gyro.imuValid.at(i), gyroRef.imuValid.at(i));
        EXPECT_NEAR(out.gyro.imuDifferenceMag.at(i), gyroRef.imuDifferenceMag.at(i), 1e-6);
    }

    // Accel vote (independent)
    EXPECT_EQ(out.accel.faultDetected, accelRef.faultDetected);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.accel.average[i], accelRef.average[i], 1e-6);
        EXPECT_TRUE(std::isfinite(out.accel.average[i]));
    }
    for (size_t i = 0U; i < kMimuCount; ++i) {
        EXPECT_EQ(out.accel.imuValid.at(i), accelRef.imuValid.at(i));
        EXPECT_NEAR(out.accel.imuDifferenceMag.at(i), accelRef.imuDifferenceMag.at(i), 1e-6);
    }
}

#endif  // TEST_MIMU_MAJORITY_VOTE_H
