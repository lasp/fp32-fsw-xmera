#include "mimuMajorityVote.h"

void MimuMajorityVote::reset(uint64_t const callTime) {
    // check if at least 3 imus have been connected
    if (this->numberOfImus < 3U) {
        throw std::invalid_argument("You need at least 3 imus for majority vote.");
    }
}

void MimuMajorityVote::updateState(uint64_t const callTime) {
    for (size_t index = 0U; index < this->numberOfImus; ++index) {
        // Unpack the messages into an array of message payloads
        this->imuPayloads.at(index) = this->imuMessages.at(index).imuSensorBodyInMsg();
    }

    auto [imuSensorBodyMsgF32Payload, mimuFaultMsgPayload] =
        this->algorithm.update(this->imuPayloads, this->numberOfImus);

    this->imuSensorBodyOutMsg.write(&imuSensorBodyMsgF32Payload, this->moduleID, callTime);
    this->mimuFaultMsg.write(&mimuFaultMsgPayload, this->moduleID, callTime);
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
