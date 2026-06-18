#include "sunSearch.h"

#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <stdexcept>

void SunSearch::reset(const uint64_t callTime) {
    if (!this->attNavInMsg.isLinked()) {
        throw std::invalid_argument("SunSearch.attNavInMsg wasn't connected.");
    }
    auto config = SunSearchConfig::create(this->rotations);
    this->algorithm = std::make_unique<SunSearchAlgorithm>(config);
}

void SunSearch::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("SunSearch reset() has not been called.");
    }

    NavAttMsgF32Payload navAttIn = this->attNavInMsg();
    const Eigen::Vector3f omega_BN_B = cArrayToEigenVector(navAttIn.omega_BN_B);

    SunSearchOutput output = this->algorithm->update(callTime, omega_BN_B);

    AttGuidMsgF32Payload attGuidOut{};
    eigenVectorToCArray(output.omega_RN_B, attGuidOut.omega_RN_B);
    eigenVectorToCArray(output.omega_BR_B, attGuidOut.omega_BR_B);

    this->attGuidOutMsg.write(attGuidOut, this->moduleID, callTime);
}

void SunSearch::setRotation(const uint32_t index, const RotationProperties& rotation) {
    this->rotations.at(index) = rotation;
}

RotationProperties SunSearch::getRotation(const uint32_t index) const { return this->rotations.at(index); }
