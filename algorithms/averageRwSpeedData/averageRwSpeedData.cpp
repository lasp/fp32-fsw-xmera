#include "averageRwSpeedData.h"

#include "utilities/xmera/xmeraLifecycleException.h"
#include <utilities/fsw/eigenSupport.h>

AverageRwSpeedDataConfig AverageRwSpeedData::toConfig() const {
    return AverageRwSpeedDataConfig::create(this->rwSpeedAveragingWindow);
}

void AverageRwSpeedData::reset(uint64_t const callTime) {
    // check if the required message has not been connected
    if (!this->rwSpeedInMsg.isLinked()) {
        throw std::invalid_argument("A rwSpeed input message name was not linked and is required for execution");
    }
    this->prevInMsgTime = 0;
    this->algorithm = std::make_unique<AverageRwSpeedDataAlgorithm>(this->toConfig());
}

void AverageRwSpeedData::reconfigure() const {
    if (!this->algorithm) {
        throw XmeraLifecycleException("AverageRwSpeedData reset() has not been called.");
    }
    this->algorithm->setConfig(this->toConfig());
}

void AverageRwSpeedData::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("AverageRwSpeedData reset() has not been called.");
    }
    this->algorithm->reInitialize();
}

void AverageRwSpeedData::updateState(uint64_t const callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("AverageRwSpeedData reset() has not been called.");
    }

    // Skip when the input message has not been updated since the last call.
    const uint64_t writeTime = this->rwSpeedInMsg.timeWritten();
    if (writeTime == this->prevInMsgTime) {
        return;
    }
    this->prevInMsgTime = writeTime;

    const auto [wheelSpeedsIn, wheelThetasIn] = this->rwSpeedInMsg();
    RwSpeedSample wheelData{};
    wheelData.measTime = writeTime;
    wheelData.wheelSpeeds = std::to_array(wheelSpeedsIn);

    const auto wheelSpeedDataOut = this->algorithm->update(wheelData);

    // Copy the input message into the output message and overwrite the averaged wheel speeds from the algorithm
    RWSpeedMsgF32Payload localOutput = this->rwSpeedInMsg();
    std::ranges::copy(wheelSpeedDataOut, localOutput.wheelSpeeds);
    this->rwSpeedOutMsg.write(localOutput, this->moduleID, callTime);
}
