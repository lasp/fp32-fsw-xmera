#ifndef TEST_MIMU_MAJORITY_VOTE_H
#define TEST_MIMU_MAJORITY_VOTE_H

#include "../freestandingInvalidArgument.h"
#include "mimuMajorityVoteAlgorithm.h"
#include <gtest/gtest.h>
#include <math.h>
#include <Eigen/Core>
#include <vector>

// Reference computation for update — must mirror the production two-stage logic exactly
MimuMajorityVoteOutput referenceUpdate(const MimuMajorityVoteAlgorithm& alg,
                                       const std::array<MimuInput, kMimuCount>& imuInputs) {
    float const omegaThreshold = alg.getOmegaThreshold();

    // Stage 1: Compute average and find differences
    Eigen::Vector3f omegaAverage = Eigen::Vector3f::Zero();
    for (size_t i = 0U; i < kMimuCount; ++i) {
        omegaAverage += imuInputs.at(i).angVelBody;
    }
    omegaAverage /= static_cast<float>(kMimuCount);

    MimuMajorityVoteOutput out{};
    size_t maxDiffIndex = 0U;
    for (size_t i = 0U; i < kMimuCount; ++i) {
        out.omegaDifferencesMag.at(i) = (imuInputs.at(i).angVelBody - omegaAverage).norm();
        if (out.omegaDifferencesMag.at(i) > out.omegaDifferencesMag.at(maxDiffIndex)) {
            maxDiffIndex = i;
        }
        out.validImus.at(i) = true;
    }

    if (out.omegaDifferencesMag.at(maxDiffIndex) < omegaThreshold) {
        out.avgAngVelBody = omegaAverage;
        return out;
    }

    // Stage 2: Exclude outlier and average the remaining IMUs
    out.faultDetected = true;
    out.validImus.at(maxDiffIndex) = false;

    Eigen::Vector3f remainingAverage = Eigen::Vector3f::Zero();
    size_t remainingCount = 0U;
    for (size_t i = 0U; i < kMimuCount; ++i) {
        if (i != maxDiffIndex) {
            remainingAverage += imuInputs.at(i).angVelBody;
            ++remainingCount;
        }
    }
    remainingAverage /= static_cast<float>(remainingCount);

    out.avgAngVelBody = remainingAverage;
    return out;
}

inline void regressionTestMimuMajorityVote(float omegaThreshold,
                                           const std::vector<float>& angVel1,
                                           const std::vector<float>& angVel2,
                                           const std::vector<float>& angVel3) {
    MimuMajorityVoteAlgorithm alg{};
    alg.setOmegaThreshold(omegaThreshold);

    std::array<MimuInput, kMimuCount> imuInputs{};
    imuInputs.at(0).angVelBody = Eigen::Map<const Eigen::Vector3f>(angVel1.data());
    imuInputs.at(1).angVelBody = Eigen::Map<const Eigen::Vector3f>(angVel2.data());
    imuInputs.at(2).angVelBody = Eigen::Map<const Eigen::Vector3f>(angVel3.data());

    // Algorithm output
    MimuMajorityVoteOutput out{};
    EXPECT_NO_THROW(out = alg.update(imuInputs));

    // Reference output
    MimuMajorityVoteOutput ref{};
    EXPECT_NO_THROW(ref = referenceUpdate(alg, imuInputs));

    // Compare averaged angular velocity
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.avgAngVelBody[i], ref.avgAngVelBody[i], 1e-6);
        EXPECT_TRUE(std::isfinite(out.avgAngVelBody[i]));
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
