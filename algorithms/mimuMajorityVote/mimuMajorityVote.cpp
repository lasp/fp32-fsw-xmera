#include "mimuMajorityVote.h"

#include "architecture/utilities/eigenSupport.h"

void MimuMajorityVote::reset(uint64_t const callTime) {
    // check if at least 3 imus have been connected
    if (this->numberOfImus < 3U) {
        throw std::invalid_argument("You need at least 3 imus for majority vote.");
    }
}

void MimuMajorityVote::updateState(uint64_t const callTime) {
    // Convert message payloads to algorithm input type
    std::array<MimuInput, MAX_IMU_VEH_COUNT> imuInputs = {};
    for (size_t index = 0U; index < this->numberOfImus; ++index) {
        auto payload = this->imuMessages.at(index).imuSensorBodyInMsg();
        imuInputs.at(index).angVelBody = cArrayToEigenVector(payload.AngVelBody);
    }

    auto [avgAngVelBody, faultDetected, mimuIndexFaulted] = this->algorithm.update(imuInputs, this->numberOfImus);

    // Convert algorithm output to message payloads
    IMUSensorBodyMsgF32Payload imuOutPayload{};
    eigenVectorToCArray(avgAngVelBody, imuOutPayload.AngVelBody);

    MimuFaultMsgPayload faultPayload{.faultDetected = faultDetected, .mimuIndexFaulted = mimuIndexFaulted};

    this->imuSensorBodyOutMsg.write(&imuOutPayload, this->moduleID, callTime);
    this->mimuFaultMsg.write(&faultPayload, this->moduleID, callTime);
}

// Add imu to the majority vote module
void MimuMajorityVote::addImuInput(const ImuMessage& imu) {
    this->imuMessages.at(this->numberOfImus) = imu;
    this->numberOfImus++;
}

void MimuMajorityVote::setOmegaThreshold(float const omegaThreshold) {
    this->algorithm.setOmegaThreshold(omegaThreshold);
}

float MimuMajorityVote::getOmegaThreshold() const { return this->algorithm.getOmegaThreshold(); }
