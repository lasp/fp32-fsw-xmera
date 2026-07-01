#include "flybyPoint.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"
#include <stdexcept>

void FlybyPoint::reset(uint64_t currentSimNanos) {
    if (!this->filterInMsg.isLinked()) {
        throw std::runtime_error("flybyPoint.filterInMsg wasn't connected.");
    }
    auto config = FlybyPointConfig::create(this->timeBetweenFilterData,
                                           this->toleranceForCollinearity,
                                           this->signOfOrbitNormalFrameVector,
                                           this->maximumRateThreshold,
                                           this->maximumAccelerationThreshold,
                                           this->positionKnowledgeSigma);
    this->algorithm = std::make_unique<FlybyPointAlgorithm>(config);
}

void FlybyPoint::updateState(uint64_t currentSimNanos) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("FlybyPoint reset() has not been called.");
    }
    auto [r_BN_N, v_BN_N] = this->readRelativeState();
    auto algo_output = this->algorithm->updateState(currentSimNanos, r_BN_N, v_BN_N);
    AttRefMsgF32Payload attMsgBuffer{};
    eigenVectorToCArray(algo_output.sigma_RN, attMsgBuffer.sigma_RN);
    eigenVectorToCArray(algo_output.omega_RN_N, attMsgBuffer.omega_RN_N);
    eigenVectorToCArray(algo_output.domega_RN_N, attMsgBuffer.domega_RN_N);
    this->attRefOutMsg.write(&attMsgBuffer, this->moduleID, currentSimNanos);
    FlybyDiagnosticMsgF32Payload flybyDiagnosticMsgBuffer{};
    flybyDiagnosticMsgBuffer.collinearityTrigger = algo_output.collinearityTrigger;
    flybyDiagnosticMsgBuffer.maxRateTrigger = algo_output.maxRateTrigger;
    flybyDiagnosticMsgBuffer.maxAccelerationTrigger = algo_output.maxAccelerationTrigger;
    flybyDiagnosticMsgBuffer.positionKnowledgeExceedTrigger = algo_output.positionKnowledgeExceedTrigger;
    this->flybyDiagnosticOutMsg.write(&flybyDiagnosticMsgBuffer, this->moduleID, currentSimNanos);
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d> FlybyPoint::readRelativeState() {
    const NavTransMsgF32Payload relativeState = this->filterInMsg();

    Eigen::Vector3d r_BN_N(relativeState.r_BN_N[0], relativeState.r_BN_N[1], relativeState.r_BN_N[2]);
    Eigen::Vector3d v_BN_N(relativeState.v_BN_N[0], relativeState.v_BN_N[1], relativeState.v_BN_N[2]);

    return {r_BN_N, v_BN_N};
}
