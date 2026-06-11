// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "flybyPoint.h"
#include "utilities/xmeraLifecycleException.h"
#include <architecture/utilities/eigenSupport.h>
#include <stdexcept>

void FlybyPoint::reset(uint64_t currentSimNanos) {
    if (!this->filterInMsg.isLinked()) {
        throw std::invalid_argument("flybyPoint.filterInMsg wasn't connected.");
    }

    auto config = FlybyPointConfig::create(this->timeBetweenFilterData,
                                           this->toleranceForCollinearity,
                                           this->signOfOrbitNormalFrameVector,
                                           this->maxRateThreshold,
                                           this->maxAccelerationThreshold,
                                           this->positionKnowledgeSigma);
    this->algorithm = std::make_unique<FlybyPointAlgorithm>(config);
    this->algorithm->reset();
}

void FlybyPoint::updateState(uint64_t currentSimNanos) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("FlybyPoint reset() has not been called.");
    }

    auto [r_BN_N, v_BN_N] = this->readRelativeState();
    FlybyPointOutput algo_output = this->algorithm->updateState(currentSimNanos, r_BN_N, v_BN_N);

    // Create local container for output message
    AttRefMsgF32Payload attMsgBuffer{};

    // Convert algorithm outputs from eigen vectors to C arrays
    eigenVectorToCArray(algo_output.sigma_RN, attMsgBuffer.sigma_RN);
    eigenVectorToCArray(algo_output.omega_RN_N, attMsgBuffer.omega_RN_N);
    eigenVectorToCArray(algo_output.domega_RN_N, attMsgBuffer.domega_RN_N);

    FlybyDiagnosticMsgPayload flybyDiagnosticMsgBuffer{};
    flybyDiagnosticMsgBuffer.collinearityTrigger = algo_output.collinearityTrigger;
    flybyDiagnosticMsgBuffer.maxRateTrigger = algo_output.maxRateTrigger;
    flybyDiagnosticMsgBuffer.maxAccelerationTrigger = algo_output.maxAccelerationTrigger;
    flybyDiagnosticMsgBuffer.positionKnowledgeExceedTrigger = algo_output.positionKnowledgeExceedTrigger;

    this->attRefOutMsg.write(&attMsgBuffer, this->moduleID, currentSimNanos);
    this->flybyDiagnosticOutMsg.write(&flybyDiagnosticMsgBuffer, this->moduleID, currentSimNanos);
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d> FlybyPoint::readRelativeState() {
    NavTransMsgF32Payload relativeState = this->filterInMsg();

    Eigen::Vector3d r_BN_N(relativeState.r_BN_N[0], relativeState.r_BN_N[1], relativeState.r_BN_N[2]);
    Eigen::Vector3d v_BN_N(relativeState.v_BN_N[0], relativeState.v_BN_N[1], relativeState.v_BN_N[2]);

    return {r_BN_N, v_BN_N};
}
