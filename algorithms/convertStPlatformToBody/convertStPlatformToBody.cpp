#include "convertStPlatformToBody.h"

#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"

void ConvertStPlatformToBody::reset(uint64_t callTime) {
    if (!this->stSensorInMsg.isLinked()) {
        throw std::invalid_argument("convertStPlatformToBody.stSensorInMsg wasn't connected.");
    }
    auto config = ConvertStPlatformToBodyConfig::create(this->sigma_CB);
    this->algorithm = std::make_unique<ConvertStPlatformToBodyAlgorithm>(config);
}

ConvertStPlatformToBodyConfig ConvertStPlatformToBody::toConfig() const {
    return ConvertStPlatformToBodyConfig::create(this->sigma_CB);
}

void ConvertStPlatformToBody::reconfigure() const {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ConvertStPlatformToBody reset() has not been called.");
    }
    this->algorithm->setConfig(this->toConfig());
}

void ConvertStPlatformToBody::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("ConvertStPlatformToBody reset() has not been called.");
    }

    Eigen::Vector4f q_CN = Eigen::Vector4f::Zero();
    Eigen::Vector4f dq_CN = Eigen::Vector4f::Zero();
    double timeTagSeconds = 0.0;
    if (this->stSensorInMsg.isWritten()) {
        const AscPlatformAttitudeTmMsgF32Payload stPayload = this->stSensorInMsg();
        timeTagSeconds = stPayload.solutionTimestamp;

        // The platform solution carries q_CN and dq_CN scalar-last, as the algorithm takes them.
        q_CN = cArrayToEigenVector(stPayload.platformAttitude);
        dq_CN = cArrayToEigenVector(stPayload.platformAngularVelocity);
    }

    const auto [sigma_BN, omega_BN_B] = this->algorithm->update(q_CN, dq_CN);

    STAttMsgF32Payload attOutMsg{};
    attOutMsg.timeTag = timeTagSeconds;
    eigenVectorToCArray(sigma_BN, attOutMsg.MRP_BdyInrtl);
    eigenVectorToCArray(omega_BN_B, attOutMsg.omega_BN_B);

    this->stAttOutMsg.write(attOutMsg, this->moduleID, callTime);
}
