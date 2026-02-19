#include "mimuMajorityVoteAlgorithm.h"

#include "architecture/utilities/eigenSupport.h"
#include "freestandingInvalidArgument.h"

MimuMajorityVoteOutput MimuMajorityVoteAlgorithm::update(const std::array<MimuInput, MAX_IMU_VEH_COUNT>& imuInputs) {
    if (this->numberOfImus < 3U) {
        FS_THROW_INVALID_ARGUMENT("numberOfImus is not configured; call setNumberOfImus() with a value >= 3");
    }

    // Zero the angular velocity to calculate the average
    Eigen::Vector3f omegaAverage_BN_B{Eigen::Vector3f::Zero()};
    for (size_t index = 0U; index < this->numberOfImus; ++index) {
        omegaAverage_BN_B += imuInputs.at(index).angVelBody;
    }
    omegaAverage_BN_B /= static_cast<float>(this->numberOfImus);

    bool faultDetected = false;
    // Find the difference and difference magnitude for each mimu with respect to the average
    std::array<float, MAX_IMU_VEH_COUNT> omegaDifferencesMag{};
    size_t maxOmegaDifferenceIndex = 0U;
    for (size_t index = 0U; index < this->numberOfImus; ++index) {
        Eigen::Vector3f const omegaDifference = imuInputs.at(index).angVelBody - omegaAverage_BN_B;
        omegaDifferencesMag.at(index) = omegaDifference.norm();
        if (omegaDifferencesMag.at(index) >= this->omegaThreshold) {
            faultDetected = true;
        }
        if (omegaDifferencesMag.at(index) > omegaDifferencesMag.at(maxOmegaDifferenceIndex)) {
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

void MimuMajorityVoteAlgorithm::setNumberOfImus(const size_t numberOfImusIn) {
    if (numberOfImusIn < 3U || numberOfImusIn > static_cast<size_t>(MAX_IMU_VEH_COUNT)) {
        FS_THROW_INVALID_ARGUMENT("numberOfImus must be between 3 and MAX_IMU_VEH_COUNT (inclusive)");
    }
    this->numberOfImus = numberOfImusIn;
}

size_t MimuMajorityVoteAlgorithm::getNumberOfImus() const { return this->numberOfImus; }
