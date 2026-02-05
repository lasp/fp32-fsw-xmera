/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "mrpSteeringAlgorithm.h"
#include "../freestandingInvalidArgument.h"
#include "architecture/utilities/eigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"
#include <architecture/utilities/macroDefinitions.h>
#include <fswAlgorithms/fswUtilities/fswDefinitions.h>
#include <Eigen/Core>
#include <numbers>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param vehConfigMsg vehicle config message
 @param rwConfigMsg reaction wheel config message
 @param rwIsConfigured boolean indicating whether reaction wheels are configured through the rwConfigMsg
 */
void MrpSteeringAlgorithm::reset(VehicleConfigMsgF32Payload vehConfigMsg,
                                 const RWArrayConfigMsgF32Payload& rwConfigMsg,
                                 const bool rwIsConfigured) {
    this->ISCPntB_B = cArrayToEigenMatrix3(vehConfigMsg.ISCPntB_B);

    if (rwIsConfigured) {
        this->rwConfigParams = rwConfigMsg;
        this->rwIsConfigured = rwIsConfigured;
    }

    /* Reset the integral measure of the rate tracking error */
    this->z = Eigen::Vector3f::Zero();

    /* Reset the prior time flag state.
     If zero, control time step not evaluated on the first function call */
    this->priorTime = 0U;
}

/*! This method takes and rate errors relative to the Reference frame, as well as
    the reference frame angular rates and acceleration, and computes the required control torque Lr.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 @param guidCmd Attitude tracking error message
 @param wheelSpeeds Reaction wheel speed message
 @param wheelsAvailability Reaction wheel availability message
 */
CmdTorqueBodyMsgF32Payload MrpSteeringAlgorithm::update(const uint64_t callTime,
                                                        AttGuidMsgF32Payload guidCmd,
                                                        const RWSpeedMsgF32Payload& wheelSpeeds,
                                                        const RWAvailabilityMsgPayload& wheelsAvailability) {
    const Eigen::Vector3f sigma_BR = cArrayToEigenVector(guidCmd.sigma_BR);

    Eigen::Vector3f omega_BastR_B{};
    Eigen::Vector3f omegap_BastR_B{Eigen::Vector3f::Zero()};

    constexpr auto kPiOver2 = static_cast<float>(std::numbers::pi / 2.0F);

    for (Eigen::Index i = 0; i < 3; ++i) {
        const float sigma_i = sigma_BR[i];
        const float f_i = atanf(kPiOver2 / this->omegaMax * (this->K1 * sigma_i + this->K3 * powf(sigma_i, 3.0F))) /
                          kPiOver2 * this->omegaMax;
        omega_BastR_B[i] = -f_i;
    }
    if (!this->ignoreOuterLoopFeedforward) {
        const Eigen::Matrix3f B = bmatMrp(sigma_BR);
        const Eigen::Vector3f sigmaDot_BR = 0.25 * B * omega_BastR_B;

        for (Eigen::Index i = 0; i < 3; ++i) {
            const float sigma_i = sigma_BR[i];
            const float f_i =
                (3.0F * this->K3 * powf(sigma_i, 2.0F) + this->K1) /
                (powf(kPiOver2 / this->omegaMax * (this->K1 * sigma_i + this->K3 * powf(sigma_i, 3.0F)), 2.0F) + 1.0F);
            omegap_BastR_B[i] = -f_i * sigmaDot_BR[i];
        }
    }

    /*! - compute control update time */
    float dt{}; /* [s] control update period */
    if (this->priorTime == 0U) {
        dt = 0.0F;
    } else {
        dt = static_cast<float>(callTime - this->priorTime) * static_cast<float>(NANO2SEC);
    }
    this->priorTime = callTime;

    const Eigen::Vector3f omega_BR_B = cArrayToEigenVector(guidCmd.omega_BR_B);
    const Eigen::Vector3f omega_RN_B = cArrayToEigenVector(guidCmd.omega_RN_B);
    const Eigen::Vector3f domega_RN_B = cArrayToEigenVector(guidCmd.domega_RN_B);

    /*! - compute body rate */
    const Eigen::Vector3f omega_BN_B = omega_BR_B + omega_RN_B;

    /*! - compute the rate tracking error */
    const Eigen::Vector3f omega_BastN_B = omega_BastR_B + omega_RN_B;
    const Eigen::Vector3f omega_BBast_B = omega_BN_B - omega_BastN_B;

    /*! - integrate rate tracking error  */
    if (this->Ki > 0.0F) { /* check if integral feedback is turned on  */
        this->z += omega_BBast_B * dt;
        for (Eigen::Index i = 0; i < 3; ++i) {
            const float intLimCheck = fabsf(this->z[i]);
            if (intLimCheck > this->integralLimit) {
                this->z[i] *= this->integralLimit / intLimCheck;
            }
        }
    } else {
        /* integral feedback is turned off through a negative gain setting */
        this->z = Eigen::Vector3f::Zero();
    }

    /*! - compute momentum  */
    Eigen::Vector3f H_B = this->ISCPntB_B * omega_BN_B;

    if (this->rwIsConfigured) {
        const Eigen::Matrix<float, 3, RW_EFF_CNT> G_s_B =
            cArrayToEigenMatrix<float, 3, RW_EFF_CNT>(this->rwConfigParams.GsMatrix_B);

        for (Eigen::Index i = 0; i < this->rwConfigParams.numRW; ++i) {
            if (wheelsAvailability.wheelAvailability[i] == AVAILABLE) { /* check if wheel is available */
                const Eigen::Vector3f G_s_B_i = G_s_B.col(i);
                const Eigen::Vector3f h_s_i =
                    this->rwConfigParams.JsList[i] * (omega_BN_B.dot(G_s_B_i) + wheelSpeeds.wheelSpeeds[i]) * G_s_B_i;
                H_B += h_s_i;
            }
        }
    }

    /*! - evaluate required attitude control torque Lc */
    const Eigen::Vector3f Lc = this->P * omega_BBast_B + this->Ki * this->z - omega_BastN_B.cross(H_B) -
                               this->ISCPntB_B * (omegap_BastR_B + domega_RN_B - omega_BN_B.cross(omega_RN_B)) +
                               this->knownTorquePntB_B;

    /* Change sign to compute the net positive control torque onto the spacecraft */
    const Eigen::Vector3f Lr = -Lc;

    CmdTorqueBodyMsgF32Payload controlOut{}; /*!< commanded torque output message */

    /*! - Set output message and pass it to the message bus */
    eigenVectorToCArray(Lr, controlOut.torqueRequestBody);

    return controlOut;
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

/*! Setter method for the gain P.
 @return void
 @param gain [N*m*s] Rate error feedback gain
*/
void MrpSteeringAlgorithm::setP(const float gain) {
    if (gain < 0.0) {
        FS_THROW_INVALID_ARGUMENT("Feedback gain P must not be negative");
    }
    this->P = gain;
}

/*! Getter method for the gain P.
 @return const float
*/
float MrpSteeringAlgorithm::getP() const { return this->P; }

/*! Setter method for the gain Ki.
 @return void
 @param gain [N*m] Integral feedback gain
*/
void MrpSteeringAlgorithm::setKi(const float gain) {
    if (gain < 0.0) {
        FS_THROW_INVALID_ARGUMENT("Integral feedback gain Ki must not be negative");
    }
    this->Ki = gain;
}

/*! Getter method for the gain Ki.
 @return const float
*/
float MrpSteeringAlgorithm::getKi() const { return this->Ki; }

/*! Setter method for the integral limit.
 @return void
 @param limit [N*m*s] Integral limit
*/
void MrpSteeringAlgorithm::setIntegralLimit(const float limit) {
    if (limit < 0.0) {
        FS_THROW_INVALID_ARGUMENT("Integral limit must not be negative");
    }
    this->integralLimit = limit;
}

/*! Getter method for the integral limit.
 @return const float
*/
float MrpSteeringAlgorithm::getIntegralLimit() const { return this->integralLimit; }

/*! Setter method for the known external torque about point B.
 @return void
 @param torque [N*m] Known external torque expressed in body frame components
*/
void MrpSteeringAlgorithm::setKnownTorquePntB_B(const Eigen::Vector3f& torque) { this->knownTorquePntB_B = torque; }

/*! Getter method for the known torque about point B.
 @return const Eigen::Vector3f
*/
Eigen::Vector3f MrpSteeringAlgorithm::getKnownTorquePntB_B() const { return this->knownTorquePntB_B; }
