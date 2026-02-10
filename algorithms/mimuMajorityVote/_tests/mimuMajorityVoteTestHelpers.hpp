#ifndef TEST_MIMU_MAJORITY_VOTE_H
#define TEST_MIMU_MAJORITY_VOTE_H

#include "../freestandingInvalidArgument.h"
#include "mimuMajorityVoteAlgorithm.h"
#include <gtest/gtest.h>
#include <math.h>
#include <Eigen/Core>
#include <vector>

// Reference computation for update
MimuMajorityVoteOutput referenceUpdate(const MimuMajorityVoteAlgorithm& alg,
                                       const std::array<MimuInput, MAX_IMU_VEH_COUNT>& imuInputs,
                                       size_t numberOfImus) {
    float omegaThreshold = alg.getOmegaThreshold();

    // Compute average angular velocity
    Eigen::Vector3f omegaAverage = Eigen::Vector3f::Zero();
    for (size_t i = 0U; i < numberOfImus; ++i) {
        omegaAverage += imuInputs.at(i).angVelBody;
    }
    omegaAverage /= static_cast<float>(numberOfImus);

    // Find differences and detect faults
    bool faultDetected = false;
    size_t maxDiffIndex = 0U;
    std::array<float, MAX_IMU_VEH_COUNT> diffMag{};
    for (size_t i = 0U; i < numberOfImus; ++i) {
        Eigen::Vector3f diff = imuInputs.at(i).angVelBody - omegaAverage;
        diffMag.at(i) = diff.norm();
        if (diffMag.at(i) >= omegaThreshold) {
            faultDetected = true;
        }
        if (diffMag.at(i) > diffMag.at(maxDiffIndex)) {
            maxDiffIndex = i;
        }
    }

    MimuMajorityVoteOutput out{.faultDetected = faultDetected, .mimuIndexFaulted = -1};

    // If fault detected, subtract outlier and recompute average
    if (faultDetected) {
        omegaAverage = (3 * omegaAverage - imuInputs.at(maxDiffIndex).angVelBody) / 2;
        out.mimuIndexFaulted = static_cast<int>(maxDiffIndex);
    }

    out.avgAngVelBody = omegaAverage;
    return out;
}

inline void regressionTestMimuMajorityVote(float omegaThreshold,
                                           const std::vector<float>& angVel1,
                                           const std::vector<float>& angVel2,
                                           const std::vector<float>& angVel3) {
    constexpr size_t kNumImus = 3U;
    MimuMajorityVoteAlgorithm alg{};
    alg.setOmegaThreshold(omegaThreshold);

    std::array<MimuInput, MAX_IMU_VEH_COUNT> imuInputs{};
    imuInputs.at(0).angVelBody = Eigen::Map<Eigen::Vector3f>(angVel1.data());
    imuInputs.at(1).angVelBody = Eigen::Map<Eigen::Vector3f>(angVel2.data());
    imuInputs.at(2).angVelBody = Eigen::Map<Eigen::Vector3f>(angVel3.data());
    size_t numberOfImus = 3U;

    // Algorithm output
    MimuMajorityVoteOutput out{};
    EXPECT_NO_THROW(out = alg.update(imuInputs, numberOfImus));

    // Reference output
    MimuMajorityVoteOutput ref{};
    EXPECT_NO_THROW(ref = referenceUpdate(alg, imuInputs, numberOfImus));

    // Compare averaged angular velocity
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.avgAngVelBody[i], ref.avgAngVelBody[i], 1e-6);
        EXPECT_TRUE(std::isfinite(out.avgAngVelBody[i]));
    }

    // Compare fault detection
    EXPECT_EQ(out.faultDetected, ref.faultDetected);
    EXPECT_EQ(out.mimuIndexFaulted, ref.mimuIndexFaulted);
}

#endif  // TEST_MIMU_MAJORITY_VOTE_H
