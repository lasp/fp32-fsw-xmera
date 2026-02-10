#include "mimuMajorityVoteAlgorithm.h"

#include "architecture/utilities/eigenSupport.h"
#include "freestandingInvalidArgument.h"

MimuMajorityVoteOutput MimuMajorityVoteAlgorithm::update(const std::array<MimuInput, MAX_IMU_VEH_COUNT> &imuInputs,
                                                         size_t const numberOfImus) {
    // Zero the angular velocity to calculate the average
    Eigen::Vector3f omegaAverage_BN_B{Eigen::Vector3f::Zero()};
    for (size_t index = 0U; index < numberOfImus; ++index) {
        omegaAverage_BN_B += imuInputs.at(index).angVelBody;
    }
    omegaAverage_BN_B /= static_cast<float>(numberOfImus);

    bool faultDetected = false;
    // Find the difference and difference magnitude for each mimu with respect to the average
    size_t maxOmegaDifferenceIndex = 0U;
    for (size_t index = 0U; index < numberOfImus; ++index) {
        Eigen::Vector3f const omegaDifference = imuInputs.at(index).angVelBody - omegaAverage_BN_B;
        this->omegaDifferencesMag.at(index) = omegaDifference.norm();
        if (this->omegaDifferencesMag.at(index) >= this->omegaThreshold) {
            faultDetected = true;
        }
        if (this->omegaDifferencesMag.at(index) > this->omegaDifferencesMag.at(maxOmegaDifferenceIndex)) {
            maxOmegaDifferenceIndex = index;
        }
    }

    MimuMajorityVoteOutput mimuMajorityVoteOutput{.faultDetected = faultDetected, .mimuIndexFaulted = -1};

    // If a fault has been detected, subtract outlier from average and indicate which mimu has been faulted
    if (faultDetected) {
        omegaAverage_BN_B = (3 * omegaAverage_BN_B - imuInputs.at(maxOmegaDifferenceIndex).angVelBody) / 2;
        mimuMajorityVoteOutput.mimuIndexFaulted = static_cast<int>(maxOmegaDifferenceIndex);
    }

    mimuMajorityVoteOutput.avgAngVelBody = omegaAverage_BN_B;

    return mimuMajorityVoteOutput;
}

void MimuMajorityVoteAlgorithm::setOmegaThreshold(const float omegaThresholdIn) {
    if (omegaThresholdIn <= 0.0F) {
        FS_THROW_INVALID_ARGUMENT("Zero or negative omegaThreshold is not valid");
    }
    this->omegaThreshold = omegaThresholdIn;
}

float MimuMajorityVoteAlgorithm::getOmegaThreshold() const { return this->omegaThreshold; }
