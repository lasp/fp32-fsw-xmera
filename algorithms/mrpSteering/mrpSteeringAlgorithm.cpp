/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "mrpSteeringAlgorithm.h"
#include "architecture/utilities/eigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"
#include <Eigen/Core>
#include <numbers>

MrpSteeringAlgorithm::MrpSteeringAlgorithm(const MrpSteeringConfig& config)
    : cfg(config) {
}

void MrpSteeringAlgorithm::setConfig(const MrpSteeringConfig& config) {
    // Internal state that needs to be reconciled with the offered
    // config would need to be modified here.
    this->cfg = config;
}

/*! This method takes the attitude and rate errors relative to the Reference frame, as well as
    the reference frame angular rates and acceleration
 @return RateCmdMsgF32Payload
 @param guidInMsg attitude guidance input message
 */
RateCmdMsgF32Payload MrpSteeringAlgorithm::update(AttGuidMsgF32Payload& guidInMsg) const {
    const Eigen::Vector3f sigma_BR = cArrayToEigenVector(guidInMsg.sigma_BR);

    Eigen::Vector3f omega_ast{};
    Eigen::Vector3f omega_ast_p{Eigen::Vector3f::Zero()};

    constexpr auto kPiOver2 = static_cast<float>(std::numbers::pi / 2.0F);

    for (Eigen::Index i = 0; i < 3; ++i) {
        const float sigma_i = sigma_BR[i];
        const float f_i = atanf(kPiOver2 / this->cfg.getOmegaMax() * (this->cfg.getK1() * sigma_i + this->cfg.getK3() * powf(sigma_i, 3.0F))) /
                          kPiOver2 * this->cfg.getOmegaMax();
        omega_ast[i] = -f_i;
    }
    if (!this->cfg.getIgnoreOuterLoopFeedforward()) {
        const Eigen::Matrix3f B = bmatMrp(sigma_BR);
        const Eigen::Vector3f sigmaDot_BR = 0.25 * B * omega_ast;

        for (Eigen::Index i = 0; i < 3; ++i) {
            const float sigma_i = sigma_BR[i];
            const float f_i =
                (3.0F * this->cfg.getK3() * powf(sigma_i, 2.0F) + this->cfg.getK1()) /
                (powf(kPiOver2 / this->cfg.getOmegaMax() * (this->cfg.getK1() * sigma_i + this->cfg.getK3() * powf(sigma_i, 3.0F)), 2.0F) + 1.0F);
            omega_ast_p[i] = -f_i * sigmaDot_BR[i];
        }
    }
    RateCmdMsgF32Payload outMsg{};

    eigenVectorToCArray(omega_ast, outMsg.omega_BastR_B);
    eigenVectorToCArray(omega_ast_p, outMsg.omegap_BastR_B);

    return outMsg;
}
