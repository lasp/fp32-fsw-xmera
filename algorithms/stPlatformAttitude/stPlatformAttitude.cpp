#include "stPlatformAttitude.h"

#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"

void StPlatformAttitude::reset(uint64_t callTime) {
    if (!this->stSensorInMsg.isLinked()) {
        throw std::invalid_argument("stPlatformAttitude.stSensorInMsg wasn't connected.");
    }
    auto config = StPlatformAttitudeConfig::create(this->sigma_PB);
    this->algorithm = std::make_unique<StPlatformAttitudeAlgorithm>(config);
}

StPlatformAttitudeConfig StPlatformAttitude::toConfig() const {
    return StPlatformAttitudeConfig::create(this->sigma_PB);
}

void StPlatformAttitude::reconfigure() const {
    if (!this->algorithm) {
        throw XmeraLifecycleException("StPlatformAttitude reset() has not been called.");
    }
    this->algorithm->setConfig(this->toConfig());
}

void StPlatformAttitude::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("StPlatformAttitude reset() has not been called.");
    }

    Eigen::Vector4f q_PN = Eigen::Vector4f::Zero();
    Eigen::Vector4f rateQuaternion_PN = Eigen::Vector4f::Zero();
    double timeTagSeconds = 0.0;
    if (this->stSensorInMsg.isWritten()) {
        const StPlatformAttitudeMsgF32Payload stPayload = this->stSensorInMsg();
        timeTagSeconds = stPayload.solutionTimestamp;

        // The platform solution carries q_PN and rateQuaternion_PN scalar-last, as the algorithm takes them.
        q_PN = cArrayToEigenVector(stPayload.platformAttitude);
        rateQuaternion_PN = cArrayToEigenVector(stPayload.platformAngularVelocity);
    }

    const auto [sigma_BN, omega_BN_B] = this->algorithm->update(q_PN, rateQuaternion_PN);

    STAttMsgF32Payload attOutMsg{};
    attOutMsg.timeTag = timeTagSeconds;
    eigenVectorToCArray(sigma_BN, attOutMsg.MRP_BdyInrtl);
    eigenVectorToCArray(omega_BN_B, attOutMsg.omega_BN_B);

    this->stAttOutMsg.write(attOutMsg, this->moduleID, callTime);
}
