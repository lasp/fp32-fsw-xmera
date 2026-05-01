// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "dvGuidance.h"
#include "architecture/utilities/eigenSupport.h"
#include "utilities/xmeraLifecycleException.h"

#include <memory>
#include <stdexcept>

void DvGuidance::reset(const uint64_t callTime) {
    if (!this->burnDataInMsg.isLinked()) {
        throw std::invalid_argument("dvGuidance.burnDataInMsg wasn't connected.");
    }
    auto config = DvGuidanceConfig::create();
    this->algorithm = std::make_unique<DvGuidanceAlgorithm>(config);
}

void DvGuidance::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("DvGuidance reset() has not been called.");
    }

    const DvBurnCmdMsgF32Payload localBurnData = this->burnDataInMsg();

    const Eigen::Vector3f dvInrtlCmd = cArrayToEigenVector3<float>(localBurnData.dvInrtlCmd);
    const Eigen::Vector3f dvRotVecUnit = cArrayToEigenVector3<float>(localBurnData.dvRotVecUnit);

    const DvGuidanceOutput out = this->algorithm->update(
        dvInrtlCmd, dvRotVecUnit, localBurnData.dvRotVecMag, localBurnData.burnStartTime, callTime);

    AttRefMsgF32Payload attCmd = {};
    eigenVectorToCArray(out.sigma_RN, attCmd.sigma_RN);
    eigenVectorToCArray(out.omega_RN_N, attCmd.omega_RN_N);
    eigenVectorToCArray(out.domega_RN_N, attCmd.domega_RN_N);

    this->attRefOutMsg.write(&attCmd, this->moduleID, callTime);
}
