#include "mrpFeedbackAlgorithm.h"

#include <math.h>
#include <optional>

MrpFeedbackAlgorithm::MrpFeedbackAlgorithm(const MrpFeedbackConfig& config) : cfg(config) {
    setConfig(config);
    reInitialize();
}

void MrpFeedbackAlgorithm::setConfig(const MrpFeedbackConfig& config) { this->cfg = config; }

/*! Reset the integrating runtime state by zeroing the integral of the MRP tracking error. */
void MrpFeedbackAlgorithm::reInitialize() { this->int_sigma = Eigen::Vector3f::Zero(); }

/*! Compute the required control torque Lr from the attitude/rate tracking error and (optional)
    RW state. The MRP error is integrated with the fixed configured control period. */
MrpFeedbackOutput MrpFeedbackAlgorithm::update(const MrpFeedbackInputGuidance& attGuidInput,
                                               const std::array<float, kMaxNumRw>& wheelSpeeds) {
    const MrpFeedbackControlParameters& params = this->cfg.getControlParameters();
    const Eigen::Matrix3f& ISCPntB_B = this->cfg.getSpacecraftInertia();

    const Eigen::Vector3f sigma_BR = attGuidInput.sigma_BR;
    const Eigen::Vector3f omega_BR_B = attGuidInput.omega_BR_B;
    const Eigen::Vector3f omega_RN_B = attGuidInput.omega_RN_B;
    const Eigen::Vector3f domega_RN_B = attGuidInput.domega_RN_B;

    const Eigen::Vector3f omega_BN_B = omega_BR_B + omega_RN_B;

    Eigen::Vector3f z{Eigen::Vector3f::Zero()};
    if (params.Ki > 0.0F) {
        this->int_sigma += params.K * params.controlPeriod * sigma_BR;

        // Anti-windup clamp on the integral state.
        for (Eigen::Index i = 0; i < 3; ++i) {
            const float intCheck = fabsf(this->int_sigma[i]);
            if (intCheck > params.integralLimit) {
                this->int_sigma[i] *= params.integralLimit / intCheck;
            }
        }
        z = this->int_sigma + ISCPntB_B * omega_BR_B;
    }

    Eigen::Vector3f H_B = ISCPntB_B * omega_BN_B;
    const std::optional<MrpFeedbackInputRwData>& rwConfiguration = this->cfg.getRwConfiguration();
    if (rwConfiguration.has_value()) {
        const MrpFeedbackInputRwData& rwConfigParams = *rwConfiguration;
        for (uint32_t i = 0U; i < rwConfigParams.numRW; ++i) {
            if (rwConfigParams.wheelAvailability.at(i) == fsw::DeviceAvailability::Available) {
                const Eigen::Vector3f G_s_B_i = rwConfigParams.GsMatrix_B.col(static_cast<int>(i));
                const Eigen::Vector3f h_s_i =
                    rwConfigParams.JsList.at(i) * (omega_BN_B.dot(G_s_B_i) + wheelSpeeds.at(i)) * G_s_B_i;
                H_B += h_s_i;
            }
        }
    }

    Eigen::Vector3f momentumContribution{};
    if (params.controlLawType == ControlLawType::NORMAL) {
        momentumContribution = (omega_RN_B + params.Ki * z).cross(H_B);
    } else {
        momentumContribution = omega_BN_B.cross(H_B);
    }

    const Eigen::Vector3f Lc = params.K * sigma_BR + params.P * omega_BR_B + params.P * params.Ki * z -
                               momentumContribution + ISCPntB_B * (omega_BN_B.cross(omega_RN_B) - domega_RN_B) +
                               this->cfg.getKnownTorquePntB_B();

    MrpFeedbackOutput out{};
    out.controlTorque = -Lc;
    out.integralFeedbackTorque = -(params.P * params.Ki * z);
    return out;
}
