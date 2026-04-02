#ifndef TEST_MIMU_MAJORITY_VOTE_H
#define TEST_MIMU_MAJORITY_VOTE_H

#include "../freestandingInvalidArgument.h"
#include "mimuMajorityVoteAlgorithm.h"
#include <gtest/gtest.h>
#include <math.h>
#include <Eigen/Core>
#include <vector>

// Reference computation for update — must mirror the production logic exactly
MimuMajorityVoteOutput referenceUpdate(float omegaThreshold,
                                       uint32_t persistenceLimit,
                                       const std::array<MimuInput, kMimuCount>& imuInputs,
                                       std::array<uint32_t, kMimuCount>& persistenceCount) {
    // Stage 1: Compute average and find differences
    Eigen::Vector3f omegaAverage = Eigen::Vector3f::Zero();
    for (size_t i = 0U; i < kMimuCount; ++i) {
        omegaAverage += imuInputs.at(i).omega_BN_B;
    }
    omegaAverage /= static_cast<float>(kMimuCount);

    MimuMajorityVoteOutput out{};
    size_t maxDiffIndex = 0U;
    for (size_t i = 0U; i < kMimuCount; ++i) {
        out.omegaDifferencesMag.at(i) = (imuInputs.at(i).omega_BN_B - omegaAverage).norm();
        if (out.omegaDifferencesMag.at(i) > out.omegaDifferencesMag.at(maxDiffIndex)) {
            maxDiffIndex = i;
        }
        out.validImus.at(i) = true;
    }

    // Update persistence counter for the worst outlier
    bool faultDetected = false;
    if (out.omegaDifferencesMag.at(maxDiffIndex) >= omegaThreshold) {
        ++persistenceCount.at(maxDiffIndex);

        // Determine if the outlier has persisted long enough to be faulted
        if (persistenceCount.at(maxDiffIndex) >= persistenceLimit) {
            faultDetected = true;
            out.validImus.at(maxDiffIndex) = false;
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
        out.avgOmega_BN_B = omegaAverage;
        return out;
    }

    // Exclude outlier and average the remaining IMUs
    out.faultDetected = true;
    out.avgOmega_BN_B = (omegaAverage * static_cast<float>(kMimuCount) - imuInputs.at(maxDiffIndex).omega_BN_B) /
                        static_cast<float>(kMimuCount - 1U);
    return out;
}

inline void regressionTestMimuMajorityVote(float omegaThreshold,
                                           const std::vector<float>& angVel1,
                                           const std::vector<float>& angVel2,
                                           const std::vector<float>& angVel3) {
    MimuMajorityVoteAlgorithm alg{};
    alg.setOmegaThreshold(omegaThreshold);

    std::array<MimuInput, kMimuCount> imuInputs{};
    imuInputs.at(0).omega_BN_B = Eigen::Map<const Eigen::Vector3f>(angVel1.data());
    imuInputs.at(1).omega_BN_B = Eigen::Map<const Eigen::Vector3f>(angVel2.data());
    imuInputs.at(2).omega_BN_B = Eigen::Map<const Eigen::Vector3f>(angVel3.data());

    // Algorithm output
    MimuMajorityVoteOutput out{};
    EXPECT_NO_THROW(out = alg.update(imuInputs));

    // Reference output
    std::array<uint32_t, kMimuCount> persistenceCount{};
    MimuMajorityVoteOutput ref{};
    EXPECT_NO_THROW(ref = referenceUpdate(omegaThreshold, alg.getFaultPersistenceLimit(), imuInputs, persistenceCount));

    // Compare averaged angular velocity
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.avgOmega_BN_B[i], ref.avgOmega_BN_B[i], 1e-6);
        EXPECT_TRUE(std::isfinite(out.avgOmega_BN_B[i]));
    }

    // Compare fault detection
    EXPECT_EQ(out.faultDetected, ref.faultDetected);

    // Compare validImus
    for (size_t i = 0U; i < kMimuCount; ++i) {
        EXPECT_EQ(out.validImus.at(i), ref.validImus.at(i));
    }

    // Compare omegaDifferencesMag
    for (size_t i = 0U; i < kMimuCount; ++i) {
        EXPECT_NEAR(out.omegaDifferencesMag.at(i), ref.omegaDifferencesMag.at(i), 1e-6);
    }
}

#endif  // TEST_MIMU_MAJORITY_VOTE_H
