#include "dvAccumulation.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/timeConstants.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <stdexcept>

void DvAccumulation::reset(const uint64_t /*callTime*/) {
    if (!this->imuInMsg.isLinked()) {
        throw std::invalid_argument("dvAccumulation.imuInMsg wasn't connected.");
    }

    this->algorithm = std::make_unique<DvAccumulationAlgorithm>();
}

void DvAccumulation::reInitialize() {
    if (this->algorithm) {
        this->algorithm->reInitialize();
    }
}

void DvAccumulation::reInitializeExceptPersistentStates() {
    if (this->algorithm) {
        this->algorithm->reInitializeExceptPersistentStates();
    }
}

void DvAccumulation::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("DvAccumulation reset() has not been called.");
    }

    const IMUSensorBodyMsgF32Payload imuData = this->imuInMsg();
    const Eigen::Vector3f rDDotNoGravity_BN_B = cArrayToEigenVector(imuData.AccelBody);
    const Eigen::Vector3f vehAccumDV_B = this->algorithm->update(callTime, rDDotNoGravity_BN_B);

    NavTransMsgF32Payload outputData = NavTransMsgF32Payload();
    /*! - the adapter owns time-tagging: the algorithm returns only the accumulator */
    outputData.timeTag = static_cast<double>(callTime) * kNano2Sec;
    eigenVectorToCArray(vehAccumDV_B, outputData.vehAccumDV);

    this->dvAccumulationOutMsg.write(outputData, this->moduleID, callTime);
}
