#include "averageMimuData.h"
#include "utilities/xmera/xmeraLifecycleException.h"
#include <utilities/fsw/eigenSupport.h>

AverageMimuDataConfig AverageMimuData::toConfig() const {
    return AverageMimuDataConfig::create(this->gyroAveragingWindow, this->accelAveragingWindow, this->dcm_BC);
}

void AverageMimuData::reset(uint64_t const callTime) {
    // check if the required message has not been connected
    if (!this->mimuPacketInMsg.isLinked()) {
        throw std::invalid_argument("A mimuPacket input message name was not linked and is required for execution");
    }
    this->prevInMsgTime = 0;
    this->algorithm = std::make_unique<AverageMimuDataAlgorithm>(this->toConfig());
}

void AverageMimuData::reconfigure() const {
    if (!this->algorithm) {
        throw XmeraLifecycleException("AverageMimuData reset() has not been called.");
    }
    this->algorithm->setConfig(this->toConfig());
}

void AverageMimuData::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("AverageMimuData reset() has not been called.");
    }
    this->algorithm->reInitialize();
}

void AverageMimuData::updateState(uint64_t const callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("AverageMimuData reset() has not been called.");
    }

    // Skip when the input message has not been updated since the last call.
    const uint64_t writeTime = this->mimuPacketInMsg.timeWritten();
    if (writeTime == this->prevInMsgTime) {
        return;
    }
    this->prevInMsgTime = writeTime;

    const auto [packets, isValid] = this->mimuPacketInMsg();
    InputPktsData in{};
    for (std::size_t p = 0; p < MAX_MIMU_PKT; ++p) {
        in.packets[p].isValid = isValid[p];
        in.packets[p].measTime = packets[p].measTime;
        for (std::size_t s = 0; s < MAX_MIMU_SAMPLES_PER_PKT; ++s) {
            const auto& sample = packets[p].samples[s];
            in.packets[p].samples[s].gyro_C = Eigen::Vector3f(sample.gyro_B[0], sample.gyro_B[1], sample.gyro_B[2]);
            in.packets[p].samples[s].accel_C = Eigen::Vector3f(sample.accel_B[0], sample.accel_B[1], sample.accel_B[2]);
        }
    }
    const auto [accel_B, gyroOmega_B] = this->algorithm->update(in);
    IMUSensorBodyMsgF32Payload localOutput{};
    eigenVectorToCArray(gyroOmega_B, localOutput.AngVelBody);
    eigenVectorToCArray(accel_B, localOutput.AccelBody);

    this->imuOutMsg.write(localOutput, this->moduleID, callTime);
}
