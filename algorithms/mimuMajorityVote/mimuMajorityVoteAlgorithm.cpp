#include "mimuMajorityVoteAlgorithm.h"

#include "architecture/utilities/eigenSupport.h"
#include "freestandingInvalidArgument.h"

MimuMajorityVoteOutput MimuMajorityVoteAlgorithm::update(
    const std::array<MimuInput, kMimuCount>& imuInputs) const {
    // Stage 1: Compute average of all IMUs and find differences from average
    Eigen::Vector3f omegaAverage_BN_B = Eigen::Vector3f::Zero();
    for (size_t i = 0U; i < kMimuCount; ++i) {
        omegaAverage_BN_B += imuInputs.at(i).angVelBody;
    }
    omegaAverage_BN_B /= static_cast<float>(kMimuCount);

    MimuMajorityVoteOutput output{};
    size_t maxDiffIndex = 0U;
    for (size_t i = 0U; i < kMimuCount; ++i) {
        output.omegaDifferencesMag.at(i) = (imuInputs.at(i).angVelBody - omegaAverage_BN_B).norm();
        if (output.omegaDifferencesMag.at(i) > output.omegaDifferencesMag.at(maxDiffIndex)) {
            maxDiffIndex = i;
        }
        output.validImus.at(i) = true;
    }

    if (output.omegaDifferencesMag.at(maxDiffIndex) < this->omegaThreshold) {
        // No fault - all IMUs agree within threshold
        output.avgAngVelBody = omegaAverage_BN_B;
        return output;
    }

    // Stage 2: Outlier detected - exclude it and average the remaining IMUs
    output.faultDetected = true;
    output.validImus.at(maxDiffIndex) = false;

    Eigen::Vector3f remainingAverage_BN_B = Eigen::Vector3f::Zero();
    size_t remainingCount = 0U;
    for (size_t i = 0U; i < kMimuCount; ++i) {
        if (i != maxDiffIndex) {
            remainingAverage_BN_B += imuInputs.at(i).angVelBody;
            ++remainingCount;
        }
    }
    remainingAverage_BN_B /= static_cast<float>(remainingCount);

    output.avgAngVelBody = remainingAverage_BN_B;
    return output;
}

void MimuMajorityVoteAlgorithm::setOmegaThreshold(const float omegaThresholdIn) {
    if (omegaThresholdIn <= 0.0F) {
        FS_THROW_INVALID_ARGUMENT("Zero or negative omegaThreshold is not valid");
    }
    this->omegaThreshold = omegaThresholdIn;
}

float MimuMajorityVoteAlgorithm::getOmegaThreshold() const { return this->omegaThreshold; }

