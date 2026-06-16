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

        for (int i = 0; i < 4; ++i) {
            q_CN[i] = static_cast<float>(qInrtl2Case[i]);
        }

        // This is temporary given the module that feeds this algorithm
        // is still producing omega and not delta quaternions.
        // When it becomes delta quaternions this will become a pass
        // through.
        //
        // Build a unit delta quaternion dq_CN = [sin(θ/2)·axis, cos(θ/2)] with unit
        // axis = ω/‖ω‖ and θ = ‖ω‖. The algorithm's atan2-based recovery requires a
        // unit δq on input.
        const Eigen::Vector3d omegaVec = cArrayToEigenVector3(omega_CN_C);
        const double angle = omegaVec.norm();
        const double halfSin = std::sin(angle / 2.0);
        const double halfCos = std::cos(angle / 2.0);
        const Eigen::Vector3d axis = (angle > 0.0) ? (omegaVec / angle).eval() : Eigen::Vector3d::Zero();
        dq_CN[0] = static_cast<float>(axis[0] * halfSin);
        dq_CN[1] = static_cast<float>(axis[1] * halfSin);
        dq_CN[2] = static_cast<float>(axis[2] * halfSin);
        dq_CN[3] = static_cast<float>(halfCos);
    }

    const auto [sigma_BN, omega_BN_B] = this->algorithm->update(q_CN, dq_CN);

    STAttMsgPayload attOutMsg{};
    attOutMsg.timeTag = static_cast<double>(timeTagNs);
    for (int i = 0; i < 3; ++i) {
        attOutMsg.MRP_BdyInrtl[i] = static_cast<double>(sigma_BN[i]);
        attOutMsg.omega_BN_B[i] = static_cast<double>(omega_BN_B[i]);
    }

    this->stAttOutMsg.write(attOutMsg, this->moduleID, callTime);
}
