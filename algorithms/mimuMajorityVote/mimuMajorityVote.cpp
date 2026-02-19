#include "mimuMajorityVote.h"

#include "architecture/utilities/eigenSupport.h"

void MimuMajorityVote::reset(uint64_t const callTime) {
    if (this->algorithm.getNumberOfImus() == 0U) {
        throw std::invalid_argument("Expected number of IMUs has not been configured; call setNumberOfImus().");
    }
    if (this->actualNumberOfImus != this->algorithm.getNumberOfImus()) {
        throw std::invalid_argument(
            "Number of connected IMU messages does not match the configured expected number of IMUs.");
    }
}

void MimuMajorityVote::updateState(uint64_t const callTime) {
    size_t const numImus = this->algorithm.getNumberOfImus();

    // Convert message payloads to algorithm input type
    std::array<MimuInput, MAX_IMU_VEH_COUNT> imuInputs = {};
    for (size_t index = 0U; index < numImus; ++index) {
        auto payload = this->imuMessages.at(index).imuSensorBodyInMsg();
        imuInputs.at(index).angVelBody = cArrayToEigenVector(payload.AngVelBody);
    }

    auto output = this->algorithm.update(imuInputs);

    // Convert algorithm output to message payloads
    IMUSensorBodyMsgF32Payload imuOutPayload{};
    eigenVectorToCArray(output.avgAngVelBody, imuOutPayload.AngVelBody);

    MimuFaultMsgPayload faultPayload{.faultDetected = output.faultDetected,
                                     .mimuIndexFaulted = output.mimuIndexFaulted};

    this->imuSensorBodyOutMsg.write(&imuOutPayload, this->moduleID, callTime);
    this->mimuFaultMsg.write(&faultPayload, this->moduleID, callTime);
}

// Add imu to the majority vote module
void MimuMajorityVote::addImuInput(const ImuMessage& imu) {
    this->imuMessages.at(this->actualNumberOfImus) = imu;
    this->actualNumberOfImus++;
}

void MimuMajorityVote::setOmegaThreshold(float const omegaThreshold) {
    this->algorithm.setOmegaThreshold(omegaThreshold);
}

float MimuMajorityVote::getOmegaThreshold() const { return this->algorithm.getOmegaThreshold(); }

void MimuMajorityVote::setNumberOfImus(size_t const numberOfImusIn) { this->algorithm.setNumberOfImus(numberOfImusIn); }

size_t MimuMajorityVote::getNumberOfImus() const { return this->algorithm.getNumberOfImus(); }
