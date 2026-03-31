#include "mimuMajorityVote.h"

#include "architecture/utilities/eigenSupport.h"

void MimuMajorityVote::reset(uint64_t const callTime) {
    if (this->actualNumberOfImus != kMimuCount) {
        throw std::invalid_argument(
            "Number of connected IMU messages must equal kMimuCount (3); call addImuInput() exactly 3 times.");
    }
}

void MimuMajorityVote::updateState(uint64_t const callTime) {
    // Convert message payloads to algorithm input type
    std::array<MimuInput, kMimuCount> imuInputs = {};
    for (size_t index = 0U; index < kMimuCount; ++index) {
        auto payload = this->imuMessages.at(index).imuSensorBodyInMsg();
        imuInputs.at(index).angVelBody = cArrayToEigenVector(payload.AngVelBody);
    }

    MimuMajorityVoteOutput output = this->algorithm.update(imuInputs);

    // Convert algorithm output to message payloads
    IMUSensorBodyMsgF32Payload imuOutPayload{};
    eigenVectorToCArray(output.avgAngVelBody, imuOutPayload.AngVelBody);

    MimuFaultMsgPayload faultPayload{};
    faultPayload.faultDetected = output.faultDetected;
    for (size_t i = 0U; i < kMimuCount; ++i) {
        faultPayload.validImus[i] = output.validImus.at(i);
        faultPayload.omegaDifferencesMag[i] = output.omegaDifferencesMag.at(i);
    }

    this->imuSensorBodyOutMsg.write(&imuOutPayload, this->moduleID, callTime);
    this->mimuFaultMsg.write(&faultPayload, this->moduleID, callTime);
}

// Add imu to the majority vote module
void MimuMajorityVote::addImuInput(const ImuMessage& imu) {
    if (this->actualNumberOfImus >= kMimuCount) {
        throw std::invalid_argument("Cannot add more than kMimuCount (3) IMU inputs");
    }
    this->imuMessages.at(this->actualNumberOfImus) = imu;
    this->actualNumberOfImus++;
}

void MimuMajorityVote::setOmegaThreshold(float const omegaThreshold) {
    this->algorithm.setOmegaThreshold(omegaThreshold);
}

float MimuMajorityVote::getOmegaThreshold() const { return this->algorithm.getOmegaThreshold(); }
