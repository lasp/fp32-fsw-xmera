#include "convertStPlatformToBody.h"

#include "architecture/utilities/eigenSupport.h"
#include "utilities/timeConstants.h"

void ConvertStPlatformToBody::reset(uint64_t callTime) {
    if (!this->stSensorInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: convertStPlatformToBody.stSensorInMsg wasn't connected.");
    }
}

void ConvertStPlatformToBody::updateState(const uint64_t callTime) {
    PlatformAttitude attitude{};
    PlatformAngularVelocity angularVelocity{};
    if (this->stSensorInMsg.isWritten()) {
        const auto [timeTag, qInrtl2Case, omega_CN_C] = this->stSensorInMsg();
        attitude.timeTag = static_cast<uint64_t>(timeTag * kSec2Nano);
        angularVelocity.timeTag = static_cast<uint64_t>(timeTag * kSec2Nano);

        for (int i = 0; i < 4; ++i) {
            attitude.q_CN[i] = static_cast<float>(qInrtl2Case[i]);
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
        angularVelocity.dq_CN[0] = static_cast<float>(axis[0] * halfSin);
        angularVelocity.dq_CN[1] = static_cast<float>(axis[1] * halfSin);
        angularVelocity.dq_CN[2] = static_cast<float>(axis[2] * halfSin);
        angularVelocity.dq_CN[3] = static_cast<float>(halfCos);
    }

    const auto [timeTag, sigma_BN, omega_BN_B] = this->algorithm.update(attitude, angularVelocity);

    STAttMsgPayload attOutMsg{};
    attOutMsg.timeTag = static_cast<double>(timeTag);
    for (int i = 0; i < 3; ++i) {
        attOutMsg.MRP_BdyInrtl[i] = static_cast<double>(sigma_BN[i]);
        attOutMsg.omega_BN_B[i] = static_cast<double>(omega_BN_B[i]);
    }

    this->stAttOutMsg.write(&attOutMsg, this->moduleID, callTime);
}

void ConvertStPlatformToBody::setDcmCB(const Eigen::Matrix3d& dcm_CB) {
    this->algorithm.setDcmCB(dcm_CB.cast<float>());
}

Eigen::Matrix3d ConvertStPlatformToBody::getDcmCB() const { return this->algorithm.getDcmCB().cast<double>(); }
