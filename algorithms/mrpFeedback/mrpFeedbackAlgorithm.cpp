#include "mrpFeedbackAlgorithm.h"
#include "architecture/utilities/eigenSupport.h"
#include "utilities/timeConstants.h"

#include <math.h>
#include <utility>

MrpFeedbackAlgorithm::MrpFeedbackAlgorithm(MrpFeedbackConfig config) : cfg(std::move(config)) {}

void MrpFeedbackAlgorithm::setConfig(const MrpFeedbackConfig& config) { this->cfg = config; }

/*! Reset the algorithm: clear the integral state. The spacecraft inertia and the RW array
    configuration are part of the immutable config (MrpFeedbackConfig). */
void MrpFeedbackAlgorithm::reset() {
    this->int_sigma = Eigen::Vector3f::Zero();
    // priorTime == 0 signals first-call: no time delta is taken on the first update.
    this->priorTime = 0U;
}

/*! Compute the required control torque Lr from the attitude/rate tracking error and (optional)
    RW state. */
MrpFeedbackOutput MrpFeedbackAlgorithm::update(uint64_t callTime,
                                               const MrpFeedbackGuidInput& guid,
                                               const Eigen::Vector<float, RW_EFF_CNT>& wheelSpeeds,
                                               const std::array<bool, RW_EFF_CNT>& wheelAvailability) {
    float dt{};
    if (this->priorTime == 0U) {
        dt = 0.0F;
    } else {
        dt = static_cast<float>(static_cast<double>(callTime - this->priorTime) * kNano2Sec);
    }
    this->priorTime = callTime;

    const Eigen::Matrix3f ISCPntB_B = this->cfg.getISCPntB_B();

    const Eigen::Vector3f& sigma_BR = guid.sigma_BR;
    const Eigen::Vector3f& omega_BR_B = guid.omega_BR_B;
    const Eigen::Vector3f& omega_RN_B = guid.omega_RN_B;
    const Eigen::Vector3f& domega_RN_B = guid.domega_RN_B;

    const Eigen::Vector3f omega_BN_B = omega_BR_B + omega_RN_B;

    Eigen::Vector3f z{Eigen::Vector3f::Zero()};
    if (this->cfg.getKi() > 0.0F) {
        this->int_sigma += this->cfg.getK() * dt * sigma_BR;

        // Anti-windup clamp on the integral state.
        const float integralLimit = this->cfg.getIntegralLimit();
        for (Eigen::Index i = 0; i < 3; ++i) {
            const float intCheck = fabsf(this->int_sigma[i]);
            if (intCheck > integralLimit) {
                this->int_sigma[i] *= integralLimit / intCheck;
            }
        }
        z = this->int_sigma + ISCPntB_B * omega_BR_B;
    }

    const Eigen::Matrix<float, 3, RW_EFF_CNT>& G_s_B = this->cfg.getGs_B();
    const std::array<float, RW_EFF_CNT>& JsList = this->cfg.getJsList();

    Eigen::Vector3f H_B = ISCPntB_B * omega_BN_B;
    for (Eigen::Index i = 0; i < this->cfg.getNumRW(); ++i) {
        if (wheelAvailability[i]) {
            const Eigen::Vector3f G_s_B_i = G_s_B.col(i);
            const Eigen::Vector3f h_s_i = JsList[i] * (omega_BN_B.dot(G_s_B_i) + wheelSpeeds[i]) * G_s_B_i;
            H_B += h_s_i;
        }
    }

    Eigen::Vector3f momentumContribution{};
    if (this->cfg.getControlLawType() == ControlLawType::NORMAL) {
        momentumContribution = (omega_RN_B + this->cfg.getKi() * z).cross(H_B);
    } else {
        momentumContribution = omega_BN_B.cross(H_B);
    }

    const Eigen::Vector3f Lc = this->cfg.getK() * sigma_BR + this->cfg.getP() * omega_BR_B +
                               this->cfg.getP() * this->cfg.getKi() * z - momentumContribution +
                               ISCPntB_B * (omega_BN_B.cross(omega_RN_B) - domega_RN_B) +
                               this->cfg.getKnownTorquePntB_B();

    const Eigen::Vector3f Lr = -Lc;
    const Eigen::Vector3f Li = -(this->cfg.getP() * this->cfg.getKi() * z);

    MrpFeedbackOutput out{};
    eigenVectorToCArray(Lr, out.controlOut.torqueRequestBody);
    eigenVectorToCArray(Li, out.intFeedbackOut.torqueRequestBody);
    return out;
}
