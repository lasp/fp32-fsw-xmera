#include "mimuMajorityVoteAlgorithm.h"

#include "architecture/utilities/eigenSupport.h"
#include "freestandingInvalidArgument.h"

MimuMajorityVoteOutput MimuMajorityVoteAlgorithm::update(
    std::array<IMUSensorBodyMsgF32Payload, MAX_IMU_VEH_COUNT> imuPayloads,
    size_t const numberOfImus) {
    // Zero the angular velocity to calculate the average
    Eigen::Vector3f omegaAverage_BN_B{Eigen::Vector3f::Zero()};
    for (size_t index = 0U; index < numberOfImus; ++index) {
        omegaAverage_BN_B += Eigen::Map<const Eigen::Vector3f>(imuPayloads.at(index).AngVelBody);
    }
    omegaAverage_BN_B /= static_cast<float>(numberOfImus);

    this->faultDetected = false;
    // Find the difference and difference magnitude for each mimu with respect to the average
    size_t maxOmegaDifferenceIndex = 0U;
    for (size_t index = 0U; index < numberOfImus; ++index) {
        Eigen::Vector3f const omegaDifference =
            Eigen::Map<const Eigen::Vector3f>(imuPayloads.at(index).AngVelBody) - omegaAverage_BN_B;
        this->omegaDifferencesMag.at(index) = omegaDifference.norm();
        if (this->omegaDifferencesMag.at(index) >= this->omegaThreshold) {
            this->faultDetected = true;
        }
        if (this->omegaDifferencesMag.at(index) > this->omegaDifferencesMag.at(maxOmegaDifferenceIndex)) {
            maxOmegaDifferenceIndex = index;
        }
    }

    MimuMajorityVoteOutput mimuMajorityVoteOutput{};
    mimuMajorityVoteOutput.mimuFaultMsgPayload.faultDetected = this->faultDetected;
    mimuMajorityVoteOutput.mimuFaultMsgPayload.mimuIndexFaulted = -1;

    // If a fault has been detected, subtract outlier from average and indicate which mimu has been faulted
    if (this->faultDetected) {
        omegaAverage_BN_B = (3 * omegaAverage_BN_B -
                             Eigen::Map<const Eigen::Vector3f>(imuPayloads.at(maxOmegaDifferenceIndex).AngVelBody)) /
                            2;
        mimuMajorityVoteOutput.mimuFaultMsgPayload.mimuIndexFaulted = static_cast<int>(maxOmegaDifferenceIndex);
    }

    eigenVectorToCArray(omegaAverage_BN_B, mimuMajorityVoteOutput.imuSensorBodyMsgF32Payload.AngVelBody);

    return mimuMajorityVoteOutput;
}

void MimuMajorityVoteAlgorithm::setOmegaThreshold(const float omegaThresholdIn) {
    if (omegaThresholdIn <= 0.0F) {
        FS_THROW_INVALID_ARGUMENT("Zero or negative omegaThreshold is not valid");
    }
    this->omegaThreshold = omegaThresholdIn;
}

float MimuMajorityVoteAlgorithm::getOmegaThreshold() const { return this->omegaThreshold; }
