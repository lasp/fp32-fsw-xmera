#include "flybyPoint.h"
#include "utilities/fsw/eigenSupport.h"

FlybyPoint::FlybyPoint() { this->algorithm = FlybyPointAlgorithm(); }

void FlybyPoint::reset(uint64_t currentSimNanos) {
    if (!this->filterInMsg.isLinked()) {
        throw std::runtime_error("flybyPoint.filterInMsg wasn't connected.");
    }
    this->algorithm.reset();
}

void FlybyPoint::updateState(uint64_t currentSimNanos) {
    auto [r_BN_N, v_BN_N] = this->readRelativeState();
    auto algo_output = this->algorithm.updateState(currentSimNanos, r_BN_N, v_BN_N);
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

void FlybyPoint::setTimeBetweenFilterData(double timeBetweenFilterData) {
    this->algorithm.setTimeBetweenFilterData(timeBetweenFilterData);
}

void FlybyPoint::setToleranceForCollinearity(float toleranceForCollinearity) {
    this->algorithm.setToleranceForCollinearity(toleranceForCollinearity);
}

void FlybyPoint::setSignOfOrbitNormalFrameVector(int signOfOrbitNormalFrameVector) {
    this->algorithm.setSignOfOrbitNormalFrameVector(signOfOrbitNormalFrameVector);
}

void FlybyPoint::setMaximumRateThreshold(float maximumRateThreshold) {
    this->algorithm.setMaximumRateThreshold(maximumRateThreshold);
}

void FlybyPoint::setMaximumAccelerationThreshold(float maximumAccelerationThreshold) {
    this->algorithm.setMaximumAccelerationThreshold(maximumAccelerationThreshold);
}

void FlybyPoint::setPositionKnowledgeSigma(float positionKnowledgeStd) {
    this->algorithm.setPositionKnowledgeSigma(positionKnowledgeStd);
}

double FlybyPoint::getTimeBetweenFilterData() const { return this->algorithm.getTimeBetweenFilterData(); }

float FlybyPoint::getToleranceForCollinearity() const { return this->algorithm.getToleranceForCollinearity(); }

int FlybyPoint::getSignOfOrbitNormalFrameVector() const { return this->algorithm.getSignOfOrbitNormalFrameVector(); }

float FlybyPoint::getMaximumAccelerationThreshold() const { return this->algorithm.getMaximumAccelerationThreshold(); }

float FlybyPoint::getMaximumRateThreshold() const { return this->algorithm.getMaximumRateThreshold(); }

float FlybyPoint::getPositionKnowledgeSigma() const { return this->algorithm.getPositionKnowledgeSigma(); }