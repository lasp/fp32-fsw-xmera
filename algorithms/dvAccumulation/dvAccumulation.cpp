#include "dvAccumulation.h"
#include "architecture/utilities/eigenSupport.h"
#include "utilities/xmeraLifecycleException.h"

#include <stdexcept>

void DvAccumulation::reset(const uint64_t callTime) {
    if (!this->accPktInMsg.isLinked()) {
        throw std::invalid_argument("dvAccumulation.accPktInMsg wasn't connected.");
    }

    auto config = DvAccumulationConfig::create();
    this->algorithm = std::make_unique<DvAccumulationAlgorithm>(config);

    /*! - seed the algorithm's previousTime from the current input buffer so future updates only
     *    integrate truly new packets */
    const AccDataMsgF32Payload inputAccData = this->accPktInMsg();
    this->algorithm->resetState(inputAccData);
}

void DvAccumulation::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("DvAccumulation reset() has not been called.");
    }

    const AccDataMsgF32Payload inputAccData = this->accPktInMsg();
    const DvAccumulationOutput out = this->algorithm->update(inputAccData);

    NavTransMsgF32Payload outputData = NavTransMsgF32Payload();
    outputData.timeTag = out.timeTag;
    eigenVectorToCArray(out.vehAccumDV_B, outputData.vehAccumDV);

    this->dvAcumOutMsg.write(&outputData, this->moduleID, callTime);
}
