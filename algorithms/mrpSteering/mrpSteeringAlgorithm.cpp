#include "mrpSteeringAlgorithm.h"
#include "../utilities/fsw/safeMath.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include <fswAlgorithms/fswUtilities/fswDefinitions.h>
#include <math.h>
#include <Eigen/Core>
#include <numbers>
#include <optional>

/*! @brief Construct the algorithm with a validated configuration. The integral state of the rate
 tracking error starts at zero.
 @param config Validated configuration (control gains, known torque, inertia, RW configuration).
 */
MrpSteeringAlgorithm::MrpSteeringAlgorithm(const MrpSteeringConfig& config) : cfg(config) {
    setConfig(config);
    reInitialize();
}

/*! @brief Replace the algorithm's stored configuration at runtime. The integral state is preserved so
 the controller keeps integrating across a reconfiguration.
 @param config New validated configuration to apply.
 */
void MrpSteeringAlgorithm::setConfig(const MrpSteeringConfig& config) { this->cfg = config; }

/*! @brief Reset the integrating runtime state by zeroing the integral of the rate tracking error. */
void MrpSteeringAlgorithm::reInitialize() { this->z = Eigen::Vector3f::Zero(); }

/*! @brief Take the attitude and rate errors relative to the reference frame, together with the
 reference-frame rates and acceleration, and compute the required control torque Lr.
 @param attGuidInput Attitude guidance input (sigma_BR, omega_BR_B, omega_RN_B, domega_RN_B).
 @param wheelSpeeds Reaction wheel speeds.
 @return Eigen::Vector3f Commanded control torque Lr in body-frame components.
 */
Eigen::Vector3f MrpSteeringAlgorithm::update(const InputGuidanceData& attGuidInput,
                                             const std::array<float, RW_EFF_CNT>& wheelSpeeds) {
    const MrpSteeringControlParameters& params = this->cfg.getControlParameters();
    const Eigen::Matrix3f& ISCPntB_B = this->cfg.getSpacecraftInertia();

    const Eigen::Vector3f sigma_BR = attGuidInput.sigma_BR;

    Eigen::Vector3f omega_BastR_B{};
    Eigen::Vector3f omegap_BastR_B{Eigen::Vector3f::Zero()};

    constexpr auto kPiOver2 = static_cast<float>(std::numbers::pi / 2.0F);

    for (Eigen::Index i = 0; i < 3; ++i) {
        const float sigma_i = sigma_BR[i];
        const float f_i =
            safeAtanf(kPiOver2 / params.omegaMax * ((params.K1 * sigma_i) + (params.K3 * powf(sigma_i, 3.0F)))) /
            kPiOver2 * params.omegaMax;
        omega_BastR_B[i] = -f_i;
    }
    if (!params.ignoreOuterLoopFeedforward) {
        const Eigen::Vector3f sigmaDot_BR = dmrp(sigma_BR, omega_BastR_B);

        for (Eigen::Index i = 0; i < 3; ++i) {
            const float sigma_i = sigma_BR[i];
            const float f_i =
                ((3.0F * params.K3 * powf(sigma_i, 2.0F)) + params.K1) /
                (powf(kPiOver2 / params.omegaMax * ((params.K1 * sigma_i) + (params.K3 * powf(sigma_i, 3.0F))), 2.0F) +
                 1.0F);
            omegap_BastR_B[i] = -f_i * sigmaDot_BR[i];
        }
    }

    const Eigen::Vector3f omega_BR_B = attGuidInput.omega_BR_B;
    const Eigen::Vector3f omega_RN_B = attGuidInput.omega_RN_B;
    const Eigen::Vector3f domega_RN_B = attGuidInput.domega_RN_B;

    /*! - compute body rate */
    const Eigen::Vector3f omega_BN_B = omega_BR_B + omega_RN_B;

    /*! - compute the rate tracking error */
    const Eigen::Vector3f omega_BastN_B = omega_BastR_B + omega_RN_B;
    const Eigen::Vector3f omega_BBast_B = omega_BN_B - omega_BastN_B;

    /*! - integrate rate tracking error  */
    if (params.Ki > 0.0F) { /* check if integral feedback is turned on  */
        this->z += omega_BBast_B * params.controlPeriod;
        for (Eigen::Index i = 0; i < 3; ++i) {
            const float intLimCheck = fabsf(this->z[i]);
            if (intLimCheck > params.integralLimit) {
                this->z[i] *= params.integralLimit / intLimCheck;
            }
        }
    } else {
        /* integral feedback is turned off through a negative gain setting */
        this->z = Eigen::Vector3f::Zero();
    }

    /*! - compute momentum  */
    Eigen::Vector3f H_B = ISCPntB_B * omega_BN_B;

    const std::optional<InputRwData>& rwConfiguration = this->cfg.getRwConfiguration();
    if (rwConfiguration.has_value()) {
        const InputRwData& rwConfigParams = *rwConfiguration;
        for (uint32_t i = 0U; i < rwConfigParams.numRW; ++i) {
            if (rwConfigParams.wheelAvailability.at(i) == AVAILABLE) { /* check if wheel is available */
                const Eigen::Vector3f G_s_B_i = rwConfigParams.GsMatrix_B.col(static_cast<int>(i));
                const Eigen::Vector3f h_s_i =
                    rwConfigParams.JsList.at(i) * (omega_BN_B.dot(G_s_B_i) + wheelSpeeds.at(i)) * G_s_B_i;
                H_B += h_s_i;
            }
        }
    }

    /*! - evaluate required attitude control torque Lc */
    const Eigen::Vector3f Lc = params.P * omega_BBast_B + params.Ki * this->z - omega_BastN_B.cross(H_B) -
                               ISCPntB_B * (omegap_BastR_B + domega_RN_B - omega_BN_B.cross(omega_RN_B)) +
                               this->cfg.getKnownTorquePntB_B();

    /* Change sign to compute the net positive control torque onto the spacecraft */
    const Eigen::Vector3f Lr = -Lc;

    return Lr;
}
