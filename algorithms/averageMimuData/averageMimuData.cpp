#include "averageMimuData.h"
#include "utilities/fsw/eigenSupport.h"

void AverageMimuData::reset(uint64_t const callTime) {
    // check if the required message has not been connected
    if (!this->mimuPacketInMsg.isLinked()) {
        throw std::invalid_argument("A mimuPacket input message name was not linked and is required for execution");
    }
}

void AverageMimuData::updateState(uint64_t const callTime) {
    // Tracking the number of times that you do not receive new acceleration data
    const uint64_t writeTime = this->mimuPacketInMsg.timeWritten();
    if (writeTime == this->prevInMsgTime) {
        this->staleDataCount++;
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
            in.packets[p].samples[s].gyro_P =
                Eigen::Vector3f(sample.gyro_B[0], sample.gyro_B[1], sample.gyro_B[2]);
            in.packets[p].samples[s].accel_P =
                Eigen::Vector3f(sample.accel_B[0], sample.accel_B[1], sample.accel_B[2]);
        }
    }
    const auto [accel_B, gyroOmega_B] = this->algorithm.update(in);
    IMUSensorBodyMsgF32Payload localOutput{};
    eigenVectorToCArray(gyroOmega_B, localOutput.AngVelBody);
    eigenVectorToCArray(accel_B, localOutput.AccelBody);

    this->imuOutMsg.write(&localOutput, this->moduleID, callTime);
}

void AverageMimuData::setGyroAveragingWindow(double const window) { this->algorithm.setGyroAveragingWindow(window); }

double AverageMimuData::getGyroAveragingWindow() const { return this->algorithm.getGyroAveragingWindow(); }

void AverageMimuData::setAccelAveragingWindow(double const window) { this->algorithm.setAccelAveragingWindow(window); }

double AverageMimuData::getAccelAveragingWindow() const { return this->algorithm.getAccelAveragingWindow(); }

void AverageMimuData::setDcmPltfToBdy(Eigen::Matrix3f const& dcm_BP) { this->algorithm.setDcmPltfToBdy(dcm_BP); }

Eigen::Matrix3f AverageMimuData::getDcmPltfToBdy() const { return this->algorithm.getDcmPltfToBdy(); }
