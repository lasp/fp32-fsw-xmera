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
    OutData out = this->algorithm.update(localPkts);
    IMUSensorBodyMsgF32Payload localOutput{};
    eigenVectorToCArray(out.AngVelBody, localOutput.AngVelBody);
    eigenVectorToCArray(out.AccelBody, localOutput.AccelBody);

    this->imuOutMsg.write(&localOutput, this->moduleID, callTime);
}

void AverageMimuData::setTimeDelta(float const timeDelta) { this->algorithm.setTimeDelta(timeDelta); }

float AverageMimuData::getTimeDelta() const { return this->algorithm.getTimeDelta(); }

void AverageMimuData::setDcmPltfToBdy(Eigen::Matrix3f const& dcm_BP) { this->algorithm.setDcmPltfToBdy(dcm_BP); }

Eigen::Matrix3f AverageMimuData::getDcmPltfToBdy() const { return this->algorithm.getDcmPltfToBdy(); }
