#include "mrpFeedbackAlgorithm.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/timeConstants.h"

#include <math.h>
#include <utility>

MrpFeedbackAlgorithm::MrpFeedbackAlgorithm(MrpFeedbackConfig config) : cfg(std::move(config)) {}

void MrpFeedbackAlgorithm::setConfig(const MrpFeedbackConfig& config) { this->cfg = config; }

/*! Reset the algorithm: snapshot the spacecraft inertia and (optional) RW configuration, and
    clear the integral state. */
void MrpFeedbackAlgorithm::reset(VehicleConfigMsgF32Payload vehConfigMsg,
                                 const RWArrayConfigMsgF32Payload& rwConfigMsg,
                                 const bool rwIsLinked) {
    this->ISCPntB_B = cArrayToEigenMatrix3(vehConfigMsg.ISCPntB_B);

    this->rwConfigParams.numRW = 0;
    if (rwIsLinked) {
        this->rwConfigParams = rwConfigMsg;
    }

    this->int_sigma = Eigen::Vector3f::Zero();
    // priorTime == 0 signals first-call: no time delta is taken on the first update.
    this->priorTime = 0U;
}

/*! Compute the required control torque Lr from the attitude/rate tracking error and (optional)
    RW state. */
MrpFeedbackOutput MrpFeedbackAlgorithm::update(uint64_t callTime,
                                               const AttGuidMsgF32Payload& guidCmd,
                                               const RWSpeedMsgF32Payload& wheelSpeeds,
                                               const RWAvailabilityMsgPayload& wheelsAvailability) {
    float dt{};
    if (this->priorTime == 0U) {
        dt = 0.0F;
    } else {
        dt = static_cast<float>(static_cast<double>(callTime - this->priorTime) * kNano2Sec);
    }
    this->priorTime = callTime;

    const Eigen::Vector3f sigma_BR = cArrayToEigenVector(guidCmd.sigma_BR);
    const Eigen::Vector3f omega_BR_B = cArrayToEigenVector(guidCmd.omega_BR_B);
    const Eigen::Vector3f omega_RN_B = cArrayToEigenVector(guidCmd.omega_RN_B);
    const Eigen::Vector3f domega_RN_B = cArrayToEigenVector(guidCmd.domega_RN_B);

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
        z = this->int_sigma + this->ISCPntB_B * omega_BR_B;
    }

    const Eigen::Matrix<float, 3, RW_EFF_CNT> G_s_B =
        cArrayToEigenMatrix<float, 3, RW_EFF_CNT>(this->rwConfigParams.GsMatrix_B);

    Eigen::Vector3f H_B = this->ISCPntB_B * omega_BN_B;
    for (Eigen::Index i = 0; i < this->rwConfigParams.numRW; ++i) {
        if (wheelsAvailability.wheelAvailability[i] == AVAILABLE) {
            const Eigen::Vector3f G_s_B_i = G_s_B.col(i);
            const Eigen::Vector3f h_s_i =
                this->rwConfigParams.JsList[i] * (omega_BN_B.dot(G_s_B_i) + wheelSpeeds.wheelSpeeds[i]) * G_s_B_i;
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
                               this->ISCPntB_B * (omega_BN_B.cross(omega_RN_B) - domega_RN_B) +
                               this->cfg.getKnownTorquePntB_B();

    const Eigen::Vector3f Lr = -Lc;
    const Eigen::Vector3f Li = -(this->cfg.getP() * this->cfg.getKi() * z);

    MrpFeedbackOutput out{};
    eigenVectorToCArray(Lr, out.controlOut.torqueRequestBody);
    eigenVectorToCArray(Li, out.intFeedbackOut.torqueRequestBody);
    return out;
}
