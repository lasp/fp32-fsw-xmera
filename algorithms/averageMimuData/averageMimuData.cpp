#include "averageMimuData.h"
#include "architecture/utilities/eigenSupport.h"

void AverageMimuData::reset(uint64_t const callTime) {
    // check if the required message has not been connected
    if (!this->accDataInMsg.isLinked()) {
        throw std::invalid_argument("An accData input message name was not linked and is required for execution");
    }
}

void AverageMimuData::updateState(uint64_t const callTime) {
    // Tracking the number of times that you do not receive new acceleration data
    const uint64_t writeTime = this->accDataInMsg.timeWritten();
    if (writeTime == this->prevInMsgTime) {
        this->staleDataCount++;
        return;
    }
    this->prevInMsgTime = writeTime;

    const AccDataMsgF32Payload localPkts = this->accDataInMsg();

    InputPktsData in{};
    for (std::size_t i=0; i<MAX_BUF_PKT; i++){
        const auto& p = localPkts.accPkts[i];
        in.measTime[i] = p.measTime;
        in.gyro_P[i]   = Eigen::Vector3f(p.gyro_B[0],  p.gyro_B[1],  p.gyro_B[2]);
        in.accel_P[i]  = Eigen::Vector3f(p.accel_B[0], p.accel_B[1], p.accel_B[2]);
    }
    OutData out = this->algorithm.update(in);
    IMUSensorBodyMsgF32Payload localOutput{};
    eigenVectorToCArray(out.AngVelBody, localOutput.AngVelBody);
    eigenVectorToCArray(out.AccelBody, localOutput.AccelBody);

    this->imuOutMsg.write(&localOutput, this->moduleID, callTime);
}

void AverageMimuData::setTimeDelta(float const timeDelta) { this->algorithm.setTimeDelta(timeDelta); }

float AverageMimuData::getTimeDelta() const { return this->algorithm.getTimeDelta(); }

void AverageMimuData::setDcmPltfToBdy(Eigen::Matrix3f const& dcm_BP) { this->algorithm.setDcmPltfToBdy(dcm_BP); }

Eigen::Matrix3f AverageMimuData::getDcmPltfToBdy() const { return this->algorithm.getDcmPltfToBdy(); }
