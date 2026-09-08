#include "dvGuidance.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <memory>
#include <stdexcept>

void DvGuidance::reset(const uint64_t callTime) {
    if (!this->burnDataInMsg.isLinked()) {
        throw std::invalid_argument("dvGuidance.burnDataInMsg wasn't connected.");
    }
    this->algorithm = std::make_unique<DvGuidanceAlgorithm>();
}

void DvGuidance::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("DvGuidance reset() has not been called.");
    }

    const DvBurnCmdMsgF32Payload burnData = this->burnDataInMsg();
    const Eigen::Vector3f dvInrtlCmd = cArrayToEigenVector3<float>(burnData.dvInrtlCmd);
    const Eigen::Vector3f dvRotVecUnit = cArrayToEigenVector3<float>(burnData.dvRotVecUnit);

    const DvGuidanceOutput out =
        this->algorithm->update(dvInrtlCmd, dvRotVecUnit, burnData.dvRotVecMag, burnData.burnStartTime, callTime);

    AttRefMsgF32Payload attRefOut = AttRefMsgF32Payload();
    eigenVectorToCArray(out.sigma_RN, attRefOut.sigma_RN);
    eigenVectorToCArray(out.omega_RN_N, attRefOut.omega_RN_N);
    eigenVectorToCArray(out.domega_RN_N, attRefOut.domega_RN_N);

    this->attRefOutMsg.write(attRefOut, this->moduleID, callTime);
}
