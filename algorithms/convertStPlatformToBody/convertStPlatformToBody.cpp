#include "convertStPlatformToBody.h"

#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/timeConstants.h"
#include "utilities/xmera/xmeraLifecycleException.h"

void ConvertStPlatformToBody::reset(uint64_t callTime) {
    if (!this->stSensorInMsg.isLinked()) {
        throw std::invalid_argument("convertStPlatformToBody.stSensorInMsg wasn't connected.");
    }
    auto config = ConvertStPlatformToBodyConfig::create(this->dcm_CB);
    this->algorithm = std::make_unique<ConvertStPlatformToBodyAlgorithm>(config);
}

ConvertStPlatformToBodyConfig ConvertStPlatformToBody::toConfig() const {
    return ConvertStPlatformToBodyConfig::create(this->dcm_CB);
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
    uint64_t timeTagNs = 0U;
    if (this->stSensorInMsg.isWritten()) {
        const auto [timeTag, qInrtl2Case, omega_CN_C] = this->stSensorInMsg();
        timeTagNs = static_cast<uint64_t>(timeTag * kSec2Nano);

        q_CN = cArrayToEigenVector(qInrtl2Case);

        // This is temporary given the module that feeds this algorithm
        // is still producing omega and not delta quaternions.
        // When it becomes delta quaternions this will become a pass
        // through.
        //
        // Build a unit delta quaternion dq_CN = [sin(θ/2)·axis, cos(θ/2)] with unit
        // axis = ω/‖ω‖ and θ = ‖ω‖. The algorithm's atan2-based recovery requires a
        // unit δq on input.
        const Eigen::Vector3f omegaVec = cArrayToEigenVector(omega_CN_C);
        const float angle = omegaVec.stableNorm();
        const float halfSin = std::sin(angle / 2.0F);
        const float halfCos = std::cos(angle / 2.0F);
        const Eigen::Vector3f axis = (angle > 0.0F) ? (omegaVec / angle).eval() : Eigen::Vector3f::Zero();
        dq_CN.head<3>() = halfSin * axis;
        dq_CN[3] = halfCos;
    }

    const auto [sigma_BN, omega_BN_B] = this->algorithm->update(q_CN, dq_CN);

    STAttMsgF32Payload attOutMsg{};
    attOutMsg.timeTag = static_cast<double>(timeTagNs);
    eigenVectorToCArray(sigma_BN, attOutMsg.MRP_BdyInrtl);
    eigenVectorToCArray(omega_BN_B, attOutMsg.omega_BN_B);

    this->stAttOutMsg.write(attOutMsg, this->moduleID, callTime);
}
