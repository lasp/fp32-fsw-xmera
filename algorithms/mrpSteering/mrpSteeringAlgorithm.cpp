/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "mrpSteeringAlgorithm.h"
#include "../freestandingInvalidArgument.h"
#include "architecture/utilities/eigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"
#include <Eigen/Core>
#include <numbers>

/*! This method takes the attitude and rate errors relative to the Reference frame, as well as
    the reference frame angular rates and acceleration1
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
        const float f_i = atanf(kPiOver2 / this->omegaMax * (this->K1 * sigma_i + this->K3 * powf(sigma_i, 3.0F))) /
                          kPiOver2 * this->omegaMax;
        omega_ast[i] = -f_i;
    }
    if (!this->ignoreOuterLoopFeedforward) {
        const Eigen::Matrix3f B = bmatMrp(sigma_BR);
        const Eigen::Vector3f sigmaDot_BR = 0.25 * B * omega_ast;

        for (Eigen::Index i = 0; i < 3; ++i) {
            const float sigma_i = sigma_BR[i];
            const float f_i =
                (3.0F * this->K3 * powf(sigma_i, 2.0F) + this->K1) /
                (powf(kPiOver2 / this->omegaMax * (this->K1 * sigma_i + this->K3 * powf(sigma_i, 3.0F)), 2.0F) + 1.0F);
            omega_ast_p[i] = -f_i * sigmaDot_BR[i];
        }
    }
    RateCmdMsgF32Payload outMsg{};

    eigenVectorToCArray(omega_ast, outMsg.omega_BastR_B);
    eigenVectorToCArray(omega_ast_p, outMsg.omegap_BastR_B);

    return outMsg;
}

/*! Set the linear feedback gain K1
 @return void
 @param gain [-] linear feedback gain K1
*/
void MrpSteeringAlgorithm::setK1(const float gain) {
    if (gain < 0.0) {
        FS_THROW_INVALID_ARGUMENT("mrpSteering feedback gain K1 must not be negative");
    }
    this->K1 = gain;
}

/*! Get the linear feedback gain K1
 @return float
*/
float MrpSteeringAlgorithm::getK1() const { return this->K1; }

/*! Set the cubic feedback gain K3
 @return void
 @param gain [-] cubic feedback gain K3
*/
void MrpSteeringAlgorithm::setK3(const float gain) {
    if (gain < 0.0) {
        FS_THROW_INVALID_ARGUMENT("mrpSteering feedback gain K3 must not be negative");
    }
    this->K3 = gain;
}

/*! Get the cubic feedback gain K3
 @return float
*/
float MrpSteeringAlgorithm::getK3() const { return this->K3; }

/*! Set the maximum rate command of steering control
 @return void
 @param omega [-] maximum rate command of steering control
*/
void MrpSteeringAlgorithm::setOmegaMax(const float omega) {
    if (omega <= 0.0) {
        FS_THROW_INVALID_ARGUMENT("mrpSteering maximum rate omegaMax must be positive");
    }
    this->omegaMax = omega;
}

/*! Get the maximum rate command of steering control
 @return float
*/
float MrpSteeringAlgorithm::getOmegaMax() const { return this->omegaMax; }

/*! Set whether the outer loop feed-forward is ignored
 @return void
 @param ignore boolean whether the outer loop feed-forward should be ignored
*/
void MrpSteeringAlgorithm::setIgnoreFeedforward(const bool ignore) { this->ignoreOuterLoopFeedforward = ignore; }

/*! Get whether the outer loop feed-forward is ignored
 @return bool
*/
bool MrpSteeringAlgorithm::getIgnoreFeedforward() const { return this->ignoreOuterLoopFeedforward; }
